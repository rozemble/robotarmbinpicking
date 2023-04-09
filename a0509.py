#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import multiprocessing as mp
from multiprocessing import Queue, Process, Manager
from module.robot.robot import robot
from module.vision.videostream import getStream, displayImage
from module.detectron.detectron import Detectron
from module.comm.tcp_socket import ClientSocket
from module.vision.pclviewer import pclViewer

if __name__ == '__main__':
    mp.set_start_method('spawn')
    os.system('fuser -k -n tcp 7777')
    cfg = {
        'robot_name': 'a0509',
        'gripper_name': '',
        'device_on': False
    }

    manager = Manager()
    robotStatus = manager.dict()
    algoStatus = manager.dict()
    connStatus = manager.dict()

    robotStatus["status"] = "INIT"
    robotStatus["data"] = ""

    algoStatus["status"] = "CONF"
    algoStatus["pcl"] = None
    algoStatus["result"] = "NA"
    algoStatus["elapsed"] = 0.0

    connStatus["status"] = "NONE"

    msgQ, camQ, taskQ, sendQ, pclQ = Queue(), Queue(), Queue(), Queue(), Queue()

    robot = robot(robotStatus,algoStatus,connStatus,taskQ,sendQ,cfg)
    robot.daemon = True
    robot.start()

    p1 = Process(target=getStream, args=(camQ,))
    p2 = Process(target=displayImage, args=(camQ, robotStatus, algoStatus, connStatus, taskQ))
    p3 = Process(target=pclViewer, args=(pclQ,))

    sock = ClientSocket(algoStatus, connStatus, taskQ, sendQ)

    sock.daemon = True
    sock.start()

    detectron = Detectron(robotStatus,algoStatus,taskQ,pclQ)
    detectron.daemon = True
    detectron.start()

    p1.daemon = True
    p1.start()
    p2.daemon = True
    p2.start()
    p3.daemon = True
    p3.start()

    p1.join()
    p2.join()
    p3.join()

    print('Terminated')
    exit(1)