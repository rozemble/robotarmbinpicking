# if your dataset is in COCO format, this cell can be replaced by the following three lines:
# from detectron2.data.datasets import register_coco_instances
# register_coco_instances("my_dataset_train", {}, "json_annotation_train.json", "path/to/image/dir")
# register_coco_instances("my_dataset_val", {}, "json_annotation_val.json", "path/to/image/dir")

import os
from os.path import isdir, join
import numpy as np
import json
from detectron2.structures import BoxMode
import cv2
import random
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog

category_dict = {"bowlingpin": 0,
                 "octopus": 1,
                 "seaelephant": 2,
                 "seal": 3,
                 "rubberduck": 4,
                 "whale": 5,
                 "dolphin": 6,
                 "seahorse": 7,
                 "recharger": 8}

def get_dicts(img_dir):
    obj, target = img_dir.split('/')
    rootdir = join(os.getcwd( ), "data", obj, target)
    datadir = [join(rootdir, d) for d in os.listdir(rootdir) if isdir(join(rootdir, d))]

    dataset_dicts = []
    for d in datadir:
        json_file = join(d, d.split('/')[-1] + '.json')
        print(json_file)
        with open(json_file) as f:
            imgs_anns = json.load(f)

        for idx, v in enumerate(imgs_anns["_via_img_metadata"].values( )):
            record = {}
            filename = os.path.join(d, v["filename"])
            height, width = cv2.imread(filename).shape[:2]

            record["file_name"] = filename
            record["image_id"] = idx
            record["height"] = height
            record["width"] = width

            annos = v["regions"]
            objs = []
            for anno in annos:
                cate = anno["region_attributes"]["category"]
                anno = anno["shape_attributes"]
                px = anno["all_points_x"]
                py = anno["all_points_y"]
                poly = [(x + 0.5, y + 0.5) for x, y in zip(px, py)]

                poly = [p for x in poly for p in x]
                try:
                    obj = {
                        "bbox": [np.min(px), np.min(py), np.max(px), np.max(py)],
                        "bbox_mode": BoxMode.XYXY_ABS,
                        "segmentation": [poly],
                        #"category_id": 0,
                        "category_id": category_dict[cate],
                        "iscrowd": 0
                    }
                except:
                    pass
                objs.append(obj)

            record["annotations"] = objs
            dataset_dicts.append(record)
    return dataset_dicts



from detectron2.data import DatasetCatalog, MetadataCatalog

objName = 'multiple'
for d in [ "val"]:
    #DatasetCatalog.register(objName+"_" + d, lambda d=d: get_dicts(objName+"/" + d))
    #MetadataCatalog.get(objName+"_" + d).set(thing_classes=[objName])
    MetadataCatalog.get(objName + "_" + d).set(thing_classes=list(category_dict.keys()))
obj_metadata = MetadataCatalog.get(objName+ "_val")
print("OBJ_METADATA:", obj_metadata)
#
# dataset_dicts = get_dicts(objName+"/val")
# for d in random.sample(dataset_dicts, 3):
#     img = cv2.imread(d["file_name"])
#     visualizer = Visualizer(img[:, :, ::-1], metadata=obj_metadata, scale=0.5)
#     vis = visualizer.draw_dataset_dict(d)
#     cv2.imshow("img",vis.get_image()[:, :, ::-1])
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()


from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg

cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
#cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_X_101_32x8d_FPN_3x.yaml"))
cfg.DATASETS.TRAIN = (objName+"_train",)
cfg.DATASETS.TEST = ()
cfg.DATALOADER.NUM_WORKERS = 2
#cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")  # Let training initialize from model zoo
cfg.SOLVER.IMS_PER_BATCH = 16
cfg.SOLVER.BASE_LR = 0.001  # pick a good LR

cfg.SOLVER.MAX_ITER = 300    # 300 iterations seems good enough for this toy dataset; you may need to train longer for a practical dataset
cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, and good enough for this toy dataset (default: 512)
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 9  # only has one class (ballon)

#cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
#cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR,objName, '04201738',"model_final.pth")
#cfg.MODEL.WEIGHTS = os.path.join(os.getcwd(),'output','prop','04230036', "model_final.pth")
#cfg.MODEL.WEIGHTS = os.path.join(os.getcwd(),'output','prop','04261915', "model_final.pth")
cfg.MODEL.WEIGHTS = join(os.getcwd(), 'module', 'detectron', 'weights', 'multiple', 'model_0004999.pth')
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.9   # set the testing threshold for this model
cfg.DATASETS.TEST = (objName+"_val", )
predictor = DefaultPredictor(cfg)
from detectron2.utils.visualizer import ColorMode
#
# dataset_dicts = get_dicts(objName+"/train")
# print(len(dataset_dicts))

resultdir = join(os.getcwd( ), "data", objName,'val')
# print("len ", len(dataset_dicts))
# for d in random.sample(dataset_dicts, 10):
#     print(d["file_name"])
#     img = cv2.imread(d["file_name"])
#     print(d["file_name"])
#     visualizer = Visualizer(img[:, :, ::-1], metadata=obj_metadata, scale=1.0)
#     vis = visualizer.draw_dataset_dict(d)
#     cv2.imshow("datasetimg",vis.get_image()[:, :, ::-1])
#     cv2.imwrite(os.path.join(resultdir, "result_" + d["file_name"].split('/')[-1]), vis.get_image( )[:, :, ::-1])
#     print("SAVED TO : ", os.path.join(resultdir, "result_" + d["file_name"].split('/')[-1]))
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

rootdir = join(os.getcwd( ), "data", objName,'val')
#rootdir = join(os.getcwd( ), "data", 'prop','32_Z00200')
datadir = [join(rootdir, d) for d in os.listdir(rootdir) if not isdir(join(rootdir, d)) and not d.endswith(("zip ")) ]
print(datadir)

import time
t = 0.0
#for d in random.sample(dataset_dicts, 4):
for d in datadir:
    #im = cv2.imread(d["file_name"])
    im = cv2.imread(d)
    t0 = time.time()
    outputs = predictor(im)
    tt = time.time() - t0
    print("%s %0.4f" %(d,tt))
    t += tt
    v = Visualizer(im[:, :, ::-1],
                   metadata=obj_metadata,
                   scale=1.0,
                   instance_mode=ColorMode.IMAGE_BW   # remove the colors of unsegmented pixels
    )
    v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    cv2.imshow("img",v.get_image()[:, :, ::-1])
    ff = d.split("/")[-1]
    cv2.imwrite(os.path.join(rootdir,"result_"+ff), v.get_image()[:, :, ::-1])
    cv2.waitKey(0)
    cv2.destroyAllWindows()
print(t,t/len(datadir))