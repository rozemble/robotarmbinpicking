from datetime import datetime
from multiprocessing import Process
import os
from os.path import join
import pickle

class robot(Process):

    #로봇의 생성자.
    ########################################################################################################
    # multiprocessing의 경우 picklable(피클로 만들 수 있는)한 변수만 init에 선언해두고 그이외의 것들은 선언해도 없어짐 T_T
    ########################################################################################################
    def __init__(self, robotStatus, algoStatus, connStatus, taskQ, sendQ, cfg={}):
        """

        @param robotStatus:
        @param algoStatus:
        @param connStatus:
        @param taskQ:
        @param sendQ:
        @param cfg:
        """
        super().__init__()

        self.robotStatus = robotStatus
        self.algoStatus = algoStatus
        self.connStatus = connStatus
        self.taskQ = taskQ
        self.sendQ = sendQ

    def run(self):
        print("ROBOT_MASTER RUN")

        self.checkCamera()
        self.robotStatus["status"] = "READY"
        self.robotStatus["retry"] = False
        self.robotStatus["ppcount"] = 0
        self.robotStatus["pptime"] = 0.0
        self.robotStatus["placetimestamp"] = None

        self.runRoutine()

    def checkCamera(self): #KSH
        print("Waiting for camera init signal...")
        while True:
            if self.algoStatus["status"] != "INIT":
                continue
            else:
                self.connected = True
                print("camera signal received")
                break

    def runRoutine(self):

        print("R:WAIT FOR CONNECTION")
        while self.connStatus["status"] != "INIT":
            continue
        print("R:runROUTINE!")

        log = []
        while self.connected:
            if not self.taskQ.empty():
                task = self.taskQ.get()
            else:
                continue

            try:
                if task["ORDER"] == "INIT":
                    now = datetime.now()
                    self.sendQ.put({"ORDER": "INIT"
                                       , "TIMESTAMP": str(now.ctime())
                                       , "ROI": self.algoStatus["binroi"]})
                elif task["ORDER"] == "PICK":
                    # 로봇에서 현재 위치 받아와야함
                    arg = task["PARA"]
                    self.algoStatus["curpos"] = arg["curpos"]
                    self.algoStatus["result"] = arg["result"]
                    self.algoStatus["elapsed"] = arg["elapsed"]
                    if arg["result"] != "NA":
                        filename = datetime.now().strftime("%m%d_%H%M%S")+'.pkl'
                        with open(join(os.getcwd(),"log", "pickle",filename), 'wb') as f:
                            if arg["dest"] == 1:
                                pickle.dump([1] + log, f, pickle.HIGHEST_PROTOCOL)
                            elif arg["dest"] == 2:
                                pickle.dump([2] + log, f, pickle.HIGHEST_PROTOCOL)
                            elif arg["dest"] == 0:
                                pickle.dump([0] + log, f, pickle.HIGHEST_PROTOCOL)

                        # [센터노멀, 평행점개수, 전체점개수,근접점거리,중앙점거리]
                        # if arg["dest"] == 1:
                        #     f.writelines(str([1] + log)+"\n")
                        # elif arg["dest"] == 2:
                        #     f.writelines(str([2] + log)+"\n")
                        # elif arg["dest"] == 0:
                        #     f.writelines(str([0] + log)+"\n")
                        #

                    self.algoStatus["status"] = "CAP"
                    self.robotStatus["status"] = "READY"

                elif task["ORDER"] == "MOVE":
                    now = datetime.now()
                    if not self.sendQ.full():
                        log = task["LOG"]
                        self.sendQ.put({"ORDER": "MOVE2"
                                        ,"DEST_POS": task["DEST_POS"]
                                        ,"VERT_POS": task["VERT_POS"]
                                        ,"SECOND_POS": task["SECOND_POS"]
                                        ,"ESCAPE_POS": task["ESCAPE_POS"]
                                        ,"CLASS":task["CLASS"]
                                        ,"TIMESTAMP": str(now.ctime())})
                elif task["ORDER"] == "CURPOS":
                    arg = task["PARA"]
                    self.algoStatus["CP"] = arg

            except Exception as ex:
                print(ex)
        f.close()
        print('Terminated')
        exit(1)