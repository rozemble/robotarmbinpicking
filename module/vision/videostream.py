import open3d as o3d
import os
import numpy as np
import cv2
import multiprocessing
from multiprocessing import Queue
import copy
from util.convertUtils import convert_pix_to_pos, convert_pos_to_pix
import pyrealsense2 as rs
import time
from datetime import datetime
import json

align_depth_to_color = True
config = None
sensor = None
results = None
font = cv2.FONT_HERSHEY_SIMPLEX  # hand-writing style font
fontScale = 0.9
minX, minY = 0, 0
maxX, maxY = 0, 0
floorX, floorY = 0, 0
floorDepth = 0
proceedYN = False
jigroi = [0, 0, 0, 0]
intr = {}


def find_device_that_supports_advanced_mode():
    global intr
    ctx = rs.context()
    ds5_dev = rs.device()
    devices = ctx.query_devices()
    for dev in devices:
        # if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
        if dev.supports(rs.camera_info.name):
            print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
        return dev
    raise Exception("No device that supports advanced mode was found")


class VideoStream():
    def __init__(self):
        self.align_depth_to_color = True
        self.config = None
        self.sensor = None
        self.results = None

        self.dev = find_device_that_supports_advanced_mode()
        print(self.dev.hardware_reset())

        # setup advanced mode
        self.adv_mode = rs.rs400_advanced_mode(self.dev)

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # self.intrinsics = {}
        # self.intrinsics["width"] = rs.intrinsics.width
        # self.intrinsics["height"] = rs.intrinsics.height
        # self.intrinsics["fx"] = rs.intrinsics.fx
        # self.intrinsics["fy"] = rs.intrinsics.fy
        # self.intrinsics["ppx"] = rs.intrinsics.ppx
        # self.intrinsics["ppy"] = rs.intrinsics.ppy

        time.sleep(2)

        # self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.sensor = self.pipeline.start(self.config)

        self.align = rs.align(rs.stream.color)

        # preset_shortrange.json, preset_bodyscan.json, preset_accuracy.json, preset_density.json
        # preset_string = str(json.load(open('preset_shortrange.json'))).replace("'", '\"')
        from os.path import join
        json_path = join(os.getcwd(), "module", "vision")
        # preset_string = str(json.load(open(join(json_path,'preset_accuracy.json')))).replace("'", '\"')
        preset_string = str(json.load(open(join(json_path, 'preset_density.json')))).replace("'", '\"')
        # print(preset_string)
        # preset_string = str(json.load(open('preset_accuracy.json'))).replace("'", '\"')

        self.adv_mode.load_json(preset_string)

        self.color_sensor = self.sensor.get_device().query_sensors()[1]
        self.color_sensor.set_option(rs.option.enable_auto_exposure, True)
        self.color_sensor.set_option(rs.option.enable_auto_white_balance, False)

        self.depth_sensor = self.sensor.get_device().query_sensors()[0]
        self.depth_sensor.set_option(rs.option.depth_units, 0.0001)
        self.depth_sensor.set_option(rs.option.laser_power, 250)

        # post filter 설정
        # reduces temporal noise
        self.temp_filter = rs.temporal_filter()
        self.temp_filter.set_option(rs.option.filter_smooth_alpha, 0.7)
        self.temp_filter.set_option(rs.option.filter_smooth_delta, 80)
        # self.intr = self.profile.as_video_stream_profile().get_intrinsics()
        # print("IINNTTRR:", self.intr)

    def run(self):
        captureFlag = False
        results = []
        # intr = self.profile.as_video_stream_profile().get_intrinsics()
        # intr: {'width': 1280, 'height': 720, 'ppx': 647.222, 'ppy': 365.037, 'fx': 922.15, 'fy': 921.341, 'model': 2,
        #        'coeffs': [0, 0, 0, 0, 0]}

        while not captureFlag:
            frames = self.pipeline.wait_for_frames(3000)
            try:
                frames = self.align.process(frames)
            except:
                continue

            self.profile = frames.get_profile()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # post filtering
            depth_frame = self.temp_filter.process(depth_frame)

            # colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())

            # Convert images to numpy arrays
            depth_image = np.asarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            results = [color_image, depth_image]
            captureFlag = True
        return results


def getStream(pipe):
    vid = VideoStream()
    while True:
        stream = vid.run()        
        if not pipe.full():
            pipe.put(vid.run())

def dispText(img, txt, offsetX, offsetY, fs=fontScale):
    (text_width, text_height) = cv2.getTextSize(txt, font, fontScale=fs, thickness=1)[0]
    text_offset_x = offsetX
    text_offset_y = offsetY
    box_coords = ((text_offset_x, text_offset_y), (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
    cv2.rectangle(img, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
    cv2.putText(img, txt, (text_offset_x, text_offset_y - 5), font, fs, (255, 255, 255), 3)

    return img

def on_mouse_callback(event, x, y, flags, param):
    global minX, minY, maxX, maxY, jigroi, floorDepth, floorX, floorY
    if event == 1:
        if minX == 0 or maxX == 0 or floorDepth == 0:
            if minX == 0:
                minX = x
                minY = y
                print("minXY:", minX, minY)
            elif maxX == 0:
                maxX = x
                maxY = y
                print("maxXY:", maxX, maxY)
            elif floorDepth == 0:
                floorX = x
                floorY = y

def displayImage(pipe, robotStatus, algoStatus, connStatus, taskQ):
    global minX, minY, maxX, maxY, floorDepth, floorY, floorX, proceedYN, intr
    windowName = "A0509"    
    cv2.namedWindow(windowName,cv2.WND_PROP_FULLSCREEN)    
    cv2.setWindowProperty(windowName,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
    font = cv2.FONT_HERSHEY_SIMPLEX  # hand-writing style font
    fontScale = 0.9
    successCount = 0
    failCount = 0
    comboCount = 0
    maxComboCount = 0
    comboTime = None

    initTime = None
    cv2.setMouseCallback(windowName, on_mouse_callback)
    timer = None

    dt = datetime.now().strftime("%H%M%S")

    bfrobotstatus = "INIT"
    pptime = 0.0
    ppCount = 0

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    writer = cv2.VideoWriter(os.path.join(os.getcwd(), "log", "video", dt + ".avi"), fourcc, 14.0, (640, 480), True)

    startFlag = False

    xdirection = 1
    ydirection = 1
    xrot = 2
    yrot = 2

    intr = {
        "width":1280,
        "width" : 1280,
        "height" : 720,
        "fx" : 922.15,
        "fy" : 921.341,
        "ppx" : 647.222,
        "ppy" : 365.037
    }

    while True:
        if not pipe.empty():
            result = pipe.get()
        else:
            continue
        c, d = result

        if robotStatus["status"] == "INIT" and algoStatus["status"] == "CONF":
            if floorX != 0 and floorDepth == 0:
                floorDepth = d[floorY][floorX]
                ceilminxyz = convert_pix_to_pos(minX, minY, floorDepth - 850, intr)
                ceilmaxxyz = convert_pix_to_pos(maxX, maxY, floorDepth - 850, intr)
                floorminxyd = convert_pos_to_pix(ceilminxyz[0], ceilminxyz[1], ceilminxyz[2] + 850, intr)
                floormaxxyd = convert_pos_to_pix(ceilmaxxyz[0], ceilmaxxyz[1], ceilmaxxyz[2] + 850, intr)

            if connStatus["status"] == "OPEN":
                if int(time.time() % 2) == 1:
                    initText = "WAITING FOR ROBOT CONNECTION"
                    h, w, _ = c.shape
                    (text_width, text_height) = cv2.getTextSize(initText, font, fontScale=1, thickness=1)[0]
                    h, w = h // 2, w // 2
                    text_offset_x = w - (text_width // 2)
                    text_offset_y = h - text_height
                    box_coords = (
                    (text_offset_x, text_offset_y), (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
                    cv2.rectangle(c, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
                    cv2.putText(c, initText, (text_offset_x, text_offset_y - 5), font, 1, (255, 255, 255), 4)
                timer = time.time()
            elif minX == 0 or maxX == 0 or floorX == 0 or proceedYN is False:
                requestText = ""
                if minX == 0:
                    requestText = "Please click upper left of BIN ROI."
                elif maxX == 0:
                    requestText = "Please click lower right of BIN ROI."
                elif floorX == 0:
                    requestText = "Please click floor depth of BIN ROI."
                elif proceedYN is False:
                    requestText = "Proceed? (y/n)"

                dispText(c, requestText, 30, 35)
                boxposText = "(" + str(minX) + "," + str(minY) + "),(" + str(maxX) + "," + str(maxY) + ")," + str(
                    floorDepth)
                dispText(c, boxposText, 30, 65)
                if minX != 0:
                    cv2.circle(c, (minX, minY), 1, (0, 0, 255), 2)
                if maxX != 0:
                    cv2.circle(c, (maxX, maxY), 1, (0, 0, 255), 2)
                if floorX != 0:
                    cv2.circle(c, (floorX, floorY), 1, (0, 0, 255), 2)
                timer = time.time()
            else:
                if time.time() - timer < 2.5:
                    initText = "START!"
                    h, w, _ = c.shape
                    if int(time.time() - timer) % 2 == 0:
                        (text_width, text_height) = cv2.getTextSize(initText, font, fontScale=2, thickness=1)[0]
                        h, w = h // 2, w // 2
                        text_offset_x = w - (text_width // 2)
                        text_offset_y = h - text_height
                        box_coords = ((text_offset_x, text_offset_y),
                                      (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
                        cv2.rectangle(c, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
                        cv2.putText(c, initText, (text_offset_x, text_offset_y - 5), font, 2, (255, 255, 255), 4)
                    else:
                        (text_width, text_height) = cv2.getTextSize(initText, font, fontScale=4, thickness=1)[0]
                        h, w = h // 2, w // 2
                        text_offset_x = w - (text_width // 2)
                        text_offset_y = h - text_height
                        box_coords = ((text_offset_x, text_offset_y),
                                      (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
                        cv2.rectangle(c, box_coords[0], box_coords[1], (255, 255, 255), cv2.FILLED)
                        cv2.putText(c, initText, (text_offset_x, text_offset_y - 5), font, 4, (0, 0, 0), 4)

                else:
                    print("detected roi:", [(minX, minY), (maxX, maxY), floorDepth])
                    algoStatus["roi"] = [(minX, minY), (maxX, maxY), floorDepth]
                    algoStatus["binroi"] = [(int(ceilminxyz[0] // 10), int(ceilminxyz[1] // 10)),
                                            (int(ceilmaxxyz[0] // 10), int(ceilmaxxyz[1] // 10))]
                    algoStatus["jigroi"] = [(jigroi[0], jigroi[1]), (jigroi[2], jigroi[3])]
                    algoStatus["status"] = "INIT"
                    startFlag = True
                    initTime = time.time()
                    taskQ.put({"ORDER": "INIT"})
        else:

            if algoStatus["status"] in ("CAP", "RECAP"):
                # print("videostream status",algoStatus["status"])
                tc = copy.deepcopy(c)
                td = copy.deepcopy(d)

                algoStatus["color"] = tc
                algoStatus["depth"] = td
                if algoStatus["status"] == "CAP":
                    result = algoStatus["result"]
                    if result != "NA":
                        if result == "SUCCESS":
                            successCount += 1
                            comboCount += 1
                            if comboCount > maxComboCount:
                                maxComboCount = comboCount
                                comboTime = time.time()
                        elif result == "FAIL":
                            failCount += 1
                            comboCount = 0

                        pptime += algoStatus["elapsed"]
                        ppCount = successCount + failCount

                    algoStatus["result"] = "NA"

                    algoStatus["status"] = "RECOG"
                    robotStatus["status"] = "READY"

                    algoStatus["xdirection"] = xdirection - 1
                    algoStatus["ydirection"] = ydirection - 1
                    algoStatus["xrot"] = (xrot - 2) * 90
                    algoStatus["yrot"] = (yrot - 2) * 90

                else:
                    algoStatus["status"] = "RETRV"

            if robotStatus["status"] != "MOVE":
                cv2.rectangle(c, (minX, minY), (maxX, maxY), (255, 255, 255), thickness=2)
                cv2.rectangle(c, (floorminxyd[0], floorminxyd[1]), (floormaxxyd[0], floormaxxyd[1]), (255, 255, 255),
                              thickness=2)
                cv2.line(c, (minX, minY), (floorminxyd[0], floorminxyd[1]), (255, 255, 255), thickness=2)
                cv2.line(c, (minX, maxY), (floorminxyd[0], floormaxxyd[1]), (255, 255, 255), thickness=2)
                cv2.line(c, (maxX, minY), (floormaxxyd[0], floorminxyd[1]), (255, 255, 255), thickness=2)
                cv2.line(c, (maxX, maxY), (floormaxxyd[0], floormaxxyd[1]), (255, 255, 255), thickness=2)


            statText = "R : " + robotStatus["status"] + " A : " + algoStatus["status"]
            (text_width, text_height) = cv2.getTextSize(statText, font, fontScale=fontScale, thickness=1)[0]
            text_offset_x = 30
            text_offset_y = 35
            box_coords = (
            (text_offset_x, text_offset_y), (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
            cv2.rectangle(c, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
            cv2.putText(c, statText, (30, 30), font, fontScale, (255, 255, 255), 3)

            ratio = 0 if (successCount + failCount) == 0 else round(
                (successCount * 1.0 / (successCount + failCount) * 100.0), 1)
            countText = "T : " + str(successCount + failCount) + " S : " + str(successCount) + " F : " + str(
                failCount) + " R : " + str(ratio) + "%"
            (text_width, text_height) = cv2.getTextSize(countText, font, fontScale=fontScale, thickness=1)[0]
            text_offset_x = 30
            text_offset_y = 65
            box_coords = (
            (text_offset_x, text_offset_y), (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
            cv2.rectangle(c, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
            cv2.putText(c, countText, (30, 60), font, fontScale, (255, 255, 255), 3)

            if bfrobotstatus != "MOVE" and robotStatus["status"] == "MOVE":
                bfrobotstatus = robotStatus["status"]
            if bfrobotstatus == "MOVE" and robotStatus["status"] == "WAIT":
                bfrobotstatus = robotStatus["status"]
                # pptime = str(round(robotStatus["pptime"] / robotStatus["ppcount"],1))
                # ppCount = robotStatus["ppcount"]

            if initTime != None:
                elapsedTime = time.time() - initTime

                if ppCount > 0:
                    ops = str(round(pptime / (successCount + failCount), 1)) + " sec"
                else:
                    ops = "0.0 sec"

                timeText = time.strftime("%H:%M:%S", time.gmtime(elapsedTime)) + " " + ops
                (text_width, text_height) = cv2.getTextSize(timeText, font, fontScale=fontScale, thickness=1)[0]
                text_offset_x = 30
                text_offset_y = 95
                box_coords = (
                (text_offset_x, text_offset_y), (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
                cv2.rectangle(c, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
                cv2.putText(c, timeText, (30, 90), font, fontScale, (255, 255, 255), 3)

            if comboTime != None and time.time() - comboTime < 2.0:
                combotext = str(comboCount) + "COMBO!!! " + " (MAX " + str(maxComboCount) + ")"
                (text_width, text_height) = cv2.getTextSize(combotext, font, fontScale=fontScale, thickness=1)[0]
                text_offset_x = 30
                text_offset_y = 125
                box_coords = (
                    (text_offset_x, text_offset_y), (text_offset_x + text_width + 4, text_offset_y - text_height - 8))
                cv2.rectangle(c, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)
                cv2.putText(c, combotext, (30, 120), font, fontScale, (255, 255, 255), 3)

            if robotStatus["status"] == "MOVE":
                if robotStatus["center"] is not None:
                    centerPoint = robotStatus["center"]

                    cv2.circle(c, (centerPoint[0], centerPoint[1]), 30, (0, 0, 255), 3)
                    if int(time.time() % 2) == 0:
                        cv2.circle(c, (centerPoint[0], centerPoint[1]), 1, (0, 0, 255), 3)
                pi = cv2.resize(algoStatus["predictedimage"], dsize=(0,0), fx=0.4,fy=0.4,interpolation=cv2.INTER_LINEAR)
                sh,sw,sc = c.shape
                dh,dw,dc = pi.shape
                c[sh-dh:sh,sw-dw:sw] = pi
        image = c
        cv2.imshow(windowName, image)

        if startFlag == True:
            writer.write(c)
        key = cv2.waitKey(1)
        if key == 27:  # Key 'esc'
            startFlag = False
            writer.release()
            robotStatus["status"] = "EXIT"
            break
        elif key == ord("s"):
            successCount += 1
            comboCount += 1
            if comboCount > maxComboCount:
                maxComboCount = comboCount
            # if initTime == None:
            #     initTime = time.time()
            comboTime = time.time()
        elif key == ord("f"):
            failCount += 1
            comboCount = 0
            if initTime == None:
                initTime = time.time()
        elif proceedYN is False and (key == ord("Y") or key == ord("y")):
            proceedYN = True
        elif proceedYN is False and (key == ord("N") or key == ord("n")):
            minX, minY, maxX, maxY, floorDepth = 0, 0, 0, 0, 0
        elif robotStatus["status"] == "MOVE" and key == ord("r"):
            robotStatus["retry"] = True

    robotStatus["status"] = "EXIT"
    print("Terminated")

if __name__ == "__main__":
    vs_parent, vs_child = multiprocessing.Pipe()
    camQ = Queue()

    p1 = multiprocessing.Process(target=getStream, args=(camQ,))
    p2 = multiprocessing.Process(target=displayImage, args=(camQ,))
    p1.daemon = True
    p1.start()
    p2.daemon = True
    p2.start()
    p2.join()

cv2.waitKey(0)
cv2.destroyAllWindows()

