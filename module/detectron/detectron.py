import os
from os.path import isdir,join
import numpy as np
import json
import pickle
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer, ColorMode
from detectron2 import model_zoo
import time
import cv2
from datetime import datetime
from module.vision.Estimator_Pipe import PointEstimator
from module.vision.Estimator_Giftbox import PointEstimator
from multiprocessing import Process
import copy

# rubber toys
# category_dict = {"pinorange": 0,
#                  "redoctopus": 1,
#                  "purpleseaelephant": 2,
#                  "greenseal": 3,
#                  "yellowduck": 4,
#                  "bluewhale": 5,
#                  "orangedolphin": 6,
#                  "pinkseahorse": 7,
#                  "recharger": 8,
#                  "pinyellow":9,
#                  "pinred": 10,
#                  "pingreen":11,
#                  "04_box":12
#                  }

# small pipes
# category_dict = { "red90_lp":0,
#                   "brasselbow_lp":1,
#                   "brassreducer_lp":2,
#                   "stainlesselbow_lp":3,
#                   "stainlesst_lp":4}

# giftbox
category_dict = { "giftbox":0 }

font = cv2.FONT_HERSHEY_SIMPLEX  # hand-writing style font
fontScale = 0.5

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class Detectron(Process):
    predictor = None
    metadata = None
    dataset_dicts = None
    initFlag = True

    def __init__(self,robotStatus,algoStatus,taskQ,pclQ):
        """
        @param robotStatus: dict for sharing robot module status info
        @param algoStatus: dict for sharing algorithm module status info
        @param taskQ: queue for receiving task
        @param pclQ: queue for delivering point cloud
        """

        time.sleep(3)
        Process.__init__(self)

        self.robotStatus = robotStatus
        self.algoStatus = algoStatus
        self.taskQ = taskQ

        cfg = get_cfg()
        #target = "pipe"
        target = "giftbox"
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.9  # set threshold for this model
        cfg.MODEL.WEIGHTS = join(os.getcwd(), 'module', 'detectron', 'weights', target, 'model_0005999.pth')
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(category_dict)
        MetadataCatalog.get(target + "_val").set(thing_classes=list(category_dict.keys()))

        self.metadata = MetadataCatalog.get(target+"_val")
        self.predictor = DefaultPredictor(cfg)
        self.estimator = None
        self.brightness = [0, -45, -30, -15, 15, 30, 45]
        self.contrast = [0, -100, -50, -10, 10, 50, 100]
        self.blur = [0, 1,3,5]
        self.pclQ = pclQ
        print("DETECTRON MODULE INIT")

    def run(self):
        print("DETECTRON RUNNING")

        while self.algoStatus["status"] == "CONF":
            pass

        self.estimator = PointEstimator(self.robotStatus,self.algoStatus,self.pclQ)
        self.estimator.setAffine()
        self.estimator.setIntrinsics((720,1280))
        self.algoStatus["intrinsics"] = self.estimator.intrinsics

        while True:
            if self.robotStatus["status"] in ( "PLACE","READY","HOLD" ) and self.algoStatus["status"] == "RECOG":
                image = self.algoStatus["color"]
                roi = self.algoStatus["roi"]
            else:
                continue

            mode = self.algoStatus["status"]
            #h, w = 1080,1920
            h, w = 720, 1280
            # set ROI
            img = np.zeros((h,w,3), np.uint8)
            img[roi[0][1]:roi[1][1], roi[0][0]:roi[1][0]] = image[roi[0][1]:roi[1][1], roi[0][0]:roi[1][0]]
            dt = datetime.now().strftime("%H%M%S")
            saveImage = True
            rot = 0
            visImg = None
            num_Instances = 0
            oriImg = copy.deepcopy(img)
            manipulationCount = 0
            # rotate & manipulate image multiple times for better predict results
            while num_Instances <= 0 and rot < 4:
                br = 0
                img = np.rot90(oriImg, k=rot, axes=(0, 1))

                for i, br in enumerate(self.blur):
                    manipulationCount += 1
                    if br != 0:
                        size = br
                        kernel_motion_blur = np.zeros((size, size))
                        kernel_motion_blur[int((size - 1) / 2), :] = np.ones(size)
                        kernel_motion_blur = kernel_motion_blur / size
                        img = cv2.filter2D(img, -1, kernel_motion_blur)

                    result = self.predictor(img)
                    num_Instances = len(result["instances"])
                    if num_Instances > 0:
                        break

                    if br < 5:
                        manipulationCount += 1
                        kernel_sharpen_1 = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])

                        sharpen = kernel_sharpen_1
                        img = cv2.filter2D(img, -1, sharpen)
                        result = self.predictor(img)
                        num_Instances = len(result["instances"])
                        if num_Instances > 0:
                            break
                if not num_Instances:
                    rot += 1

            if num_Instances <= 0:
                self.robotStatus["status"] = "READY"
                self.algoStatus["status"] = "CAP"
                continue

            # logging results
            fn = join(os.getcwd(),'log','images',dt+'_result.png')
            orifn = join(os.getcwd(), 'log', 'images', dt + '_original.png')
            if saveImage and num_Instances >= 1:
                v = Visualizer(img[:, :, ::-1],
                               metadata=self.metadata,
                               scale=1,
                               instance_mode=ColorMode.IMAGE_BW  # remove the colors of unsegmented pixels
                               )
                v = v.draw_instance_predictions(result["instances"].to("cpu"))
                v.save(fn)
                self.algoStatus["predictedimage"] = v.get_image()

                cv2.imwrite(orifn, image)

            # postprocessing predict results
            pred_masks = result["instances"].get("pred_masks").cpu().numpy()
            pred_boxes = result["instances"].get("pred_boxes").tensor.cpu().numpy().astype(int)
            scores = [round(x, 4) for x in result["instances"].get("scores").tolist()]
            pred_classes = result["instances"].get("pred_classes").tolist()

            if rot > 0:
                pred_masks = np.rot90(pred_masks, k=4-rot, axes=(1, 2))
                if rot == 1:
                    pred_boxes = [[w - b[3], b[0], w - b[1], b[2]] for b in pred_boxes]
                elif rot == 2:
                    pred_boxes = [[w - b[3], h - b[2], w - b[1], h - b[0]] for b in pred_boxes]
                elif rot == 3:
                    pred_boxes = [[b[1], h - b[2], b[3], h - b[0]] for b in pred_boxes]

            # shift phase to suction
            self.algoStatus["status"] = "SUCTION"

            stat, result = self.estimator.estimatePoint(pred_masks, pred_boxes, scores, pred_classes, num_Instances)

            if result is not None:
                # 0 xyd, 1 centerNorm, 2 nearbox radius ratio, 3 circular normals similarity,
                # 4 pose_target_on 5 nearbox volume, 6 searchorder index, 7 near_points, 8 near_normals, 9 class
                ft = result[0]
                st = result[2]
                log1 = [ft[1], ft[2], ft[3], ft[5], ft[7], ft[8], ft[9]]
                log2 = [st[1], st[2], st[3], st[5], st[7], st[8], st[9]]

                if not self.taskQ.full():
                    task = {"ORDER": "MOVE"
                        , "DEST_POS": result[0][4]
                        , "VERT_POS": result[1]
                        , "SECOND_POS": result[2][4]
                        , "ESCAPE_POS": result[1]
                        , "CLASS": [ ft[9], st[9] ]
                        , "LOG": [log1, log2]}
                    self.taskQ.put(task)

            while self.robotStatus["status"] not in ("READY", "HOLD"):
                pass
            if stat not in ( "RESET", "INIT" ):
                if mode == "RECOG":
                    stat = "MOVE"
                elif mode == "RETRV":
                    if result is None:
                        stat = "RESET"
                    else:
                        stat = "JIG"

            self.robotStatus["status"] = stat
            if stat == "INIT":
                self.algoStatus["status"] = "CAP"