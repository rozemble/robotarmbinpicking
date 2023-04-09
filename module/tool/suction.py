import os
from os.path import isdir,join
import numpy as np
import json

import itertools
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer, ColorMode
from detectron2 import model_zoo
#from lib.util.customError import MyError
import time
import cv2
from datetime import datetime
from module.vision.Estimator import PointEstimator
from multiprocessing import Process
from module.tool.u12_suction import KetiSuction

class Suction(Process):

    def __init__(self,robotStatus,algoStatus,toolStatus):
        Process.__init__(self)
        self.robotStatus = robotStatus
        self.algoStatus = algoStatus
        self.toolStatus = toolStatus

    def run(self):
        tool = KetiSuction()
        while True:
            if self.toolStatus["status"] == "HOLD":
                continue
            elif self.toolStatus["status"] == "ON":
                self.toolStatus["status"] = "HOLD"
                tool.suction_on()
            elif self.toolStatus["status"] == "OFF":
                self.toolStatus["status"] = "HOLD"
                tool.suction_off()