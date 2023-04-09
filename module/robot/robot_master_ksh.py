import time
import numpy as np
import math
from datetime import datetime
from module.robot.Parent_Robot import Parent_Robot
from module.robot.robot_irb1200_master import Robot_Master_irb1200
from module.robot import RPP_functions
from multiprocessing import Process, Queue
import os
from os.path import join
import pickle


class robot_instance:
    #로봇의 마스터 코드를 구현시 아래의 함수를 복사하여 구현하면 됨
    move_joint_robot = []   #로봇을 조인트이동하는 명령
    move_pos_robot = []     #로봇을 입력받은 포즈로 이동하게 하는 명령
    move_pos_queue = []     #로봇이 포즈 리스트를 받으면 순차적으로 이동하게 하는 명령어
    move_joint_queue = []   #로봇을 조인트 리스트를 받으면 순차적으로 이동하게 하는 명령어
    move_pos_circle = []    #로봇이 해당 포즈까지 원형으로 이동하게 하는 명령어
    move_jog_robot = []     #로봇이 조그이동을 하게 하는 명령어

    robot_stop = []         #로봇을 정지시키는 명령.

    #아래 코드는 선택적으로 구현
    grip = []               #로봇이 그리퍼를 컨트롤 하는 코드.

    #아래 코드는 구현할 필요 없음. (공통부분)
    #get_robot_pos = []      #로봇의 현재 값을 받아옴
    #get_grip_success = []   #로봇이 그리퍼의 성공여부를 받아오는 코드,

    #인스턴스 클래스의 맴버 변수.
    #복사할 필요 없음.
    m_Robot_Slave = Parent_Robot()


class robot_master(Process):
    #맴버 변수_ 설정

    #맴버함수_변수
    robot_module = [robot_instance()]   #추후에 다수의 로봇이 사용할 경우를 감안하여 리스트로 관리함.

    #로봇 모듈에 맵핑이 될 놈들
    robot_slave = []    #추후에는 로봇 슬래이브를 배열로 만들면 다수의 로봇을 동시에 작용 가능하는게 변경할 수 있음.

    gripper = []

    #로봇의 생성자.
    ########################################################################################################
    # multiprocessing의 경우 picklable(피클로 만들 수 있는)한 변수만 init에 선언해두고 그이외의 것들은 선언해도 없어짐 T_T
    ########################################################################################################
    def __init__(self, robotStatus, algoStatus, connStatus, taskQ, sendQ, cfg={}):

        #robot_name = [], gripper_name = [], device_on = False, HOST="127.0.0.1", PORT=7777

        # self.HOST = cfg['HOST']
        # self.PORT = cfg['PORT']
        # self.HOST = '127.0.0.1'
        # self.PORT = 7777

        #Process.__init__(self)
        super().__init__()

        self.connected = False
        self.message = ''
        self.step = 0
        self.target_move_flag = False
        self.home_flag = False

        self.gripper_alive = False
        #self.device_alive = device_on
        self.device_alive = cfg['device_on']

        self.flip_cancel_action = False
        self.flip_up_action = False

        self.ref = []
        self.cur = []
        self.mwait_flag = False
        self.grip_release_flag = False
        self.dist = 0
        self.robot_name = cfg['robot_name']
        self.gripper_name = cfg['gripper_name']
        #self.msg_queue = msg_queue
        self.robotStatus = robotStatus
        self.algoStatus = algoStatus
        self.connStatus = connStatus
        self.taskQ = taskQ
        self.sendQ = sendQ

        self.alpha = 0.15
        self.vel_value = 2000 * self.alpha  # init: 2000 ,
        self.acc_value = 4000 * self.alpha
        self.vel = [self.vel_value, self.vel_value]  # [2000, 2000]
        self.acc = [10, 10]  # abb
        self.flag = False
        self.normal_offset = 50
        self.placement_flag = False

        #self.joint_init = [0, -15, 15, 0, 90, 0]  # abb
        #self.init_joint = [-2.86, 2.74, 81.49, 0.0, 95.80, -3.08] # A0509
        self.init_joint = [-3.69,-9.36,92.84,0.00,96.56,-3.91]  # A0509
        self.init_pos = [356.82,124.07,446.75,177.20,179.97,176.98]  # A0509

        self.pic_pos = [[604.07, 2.83, 20, 1.34, -179.60, 1.28]]
        self.release_joint = [[-46.4, 34.8, 46.29, 3.91, 87.47, -36.99],
                              [-46.4, 34.8, 46.29, 3.91, 87.47, -36.99]]
        self.place_pos = [[422.05, -414.01, 326.38, 1.72, 170.76, -0.55],
                          [422.05, -414.01, 426.38, 1.72, 170.76, -0.55]]
        self.replace_pos = [[364.07, -402.83, 268.510, 1.34, -179.60, 1.28],
                            [364.07, -402.83, 368.510, 1.34, -179.60, 1.28]]
        self.counterpart_pos = [[175.95, -415.30, 325.49, 178.95, 178.38, 176.70],
                                [175.95, -415.30, 225.49, 178.95, 178.38, 176.70]]

        #self.s = KetiSuction()

    def run(self):
        print("ROBOT_MASTER RUN")
        # if (self.robot_name != []):
        #     self.init_robot(robot_name=self.robot_name)
        #     time.sleep(1)
        # if (self.gripper_name != []):
        #     self.init_gripper(gripper_name=self.gripper_name)
        #     self.gripper_alive = True
        # # print('그리퍼가 따로 설정되지 않으면, ros기본 그리퍼와 연결 시킨다.')
        # # if(self.device_alive != False):
        # #     self.init_device()
        #
        # # self.t1 = threading.Thread(target=self.socket_setting)
        # self.t1 = threading.Thread(target=self.receiveMsg)  # KSH
        # self.t1.daemon = True
        # self.t2 = threading.Thread(target=self.flipper_action)
        # self.t2.daemon = True
        # self.t3 = threading.Thread(target=self.mwait)
        # self.t3.daemon = True
        # self.t4 = threading.Thread(target=self.grip_release)
        # self.t4.daemon = True
        # #print("run pid", os.getpid())
        # #self.checkCamera()
        #
        #
        # current_joint = self.robot_module[0].m_Robot_Slave.robot_state['joint']
        # current_pose = self.robot_module[0].m_Robot_Slave.robot_state['pos']
        # print('Current joint : {0}'.format(current_joint))
        # print('Current pose : {0}'.format(current_pose))
        #
        # self.joint_init = [0, -15, 15, 0, 90, 0]  # abb
        # #self.joint_ready = [-5.01, 39.73, 8.32, 0.0, 39.97, 0.0]
        # self.joint_ready = [0.0, 40.0, -9.99, 0.01, 56.05, 0.0]
        # #self.place_joint = [45.0, 55.0, -35.0, 0.0, 60.0, 0.0]
        # self.place_joint = [40.0, 49.99, -25.02, 5.01, 65.05, 0.0]
        # self.pose_ready = [553.2, 0.0, 600, 0.0, 180.0, 0.0]
        # #self.pose_place = [458.6, 393.3, 432.3, 129.65020328161177, 175.464580592153, 91.8009144495093]
        # #self.pose_place = [458.6, 393.3, 432.3, 45, 180, 0]
        # self.pose_place = [458.6, 393.3, 432.3, 45, 180, 0]
        #
        # self.pose_left = [450.3, -129.6, 597.2, 0.0, 180.0, 0.0]
        # self.pose_right = [450.3, 133.9, 597.2, 0.0, 180.0, 0.0]
        # #self.pose_launcher = [489.1, -265.2, 629.8, -3.1489557223737257, 176.71738171048608, 52.699236860037566]
        # self.pose_launcher = [545.0, -272.4, 647.3, 0.0, 180.0, 0.0]

        #print("Moving to Home position")
        #self.move_joint_robot(self.joint_init, syncType=1, vel=250, acc=50)
        self.home_flag = True
        #print('home positioning complete')

        self.checkCamera()
        self.step = 1
        self.robotStatus["status"] = "READY"
        self.robotStatus["retry"] = False
        self.robotStatus["ppcount"] = 0
        self.robotStatus["pptime"] = 0.0

        movetimestamp = None
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
        #self.taskQ.put({"ORDER": "INIT"})
        log = []
        dt = datetime.now().strftime("%H%M%S")

        #f = open(join(os.getcwd(),"log", "result",dt+".txt" ), mode='wt', encoding='utf-8')
        while self.connected:
            if not self.taskQ.empty():
                task = self.taskQ.get()
                #print("TASK:"+task)
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
                    pos = task["DEST_POS"]
                    #secondpos = task["secondpos"]
                    now = datetime.now()
                    if not self.sendQ.full():
                        log = task["LOG"]
                        self.sendQ.put({"ORDER": "MOVE2"
                                        ,"DEST_POS": pos
                                        ,"VERT_POS": task["VERT_POS"]
                                        ,"SECOND_POS": task["SECOND_POS"]
                                        ,"ESCAPE_POS": task["ESCAPE_POS"]
                                        ,"TIMESTAMP": str(now.ctime())})
                elif task["ORDER"] == "CURPOS":
                    arg = task["PARA"]
                    print("ARGGGGGGGGGGGGGGGG:"+ arg)
                    self.algoStatus["CP"] = arg



            except Exception as ex:
                print(ex)
        f.close()
        print('Good bye !!!')
        exit(1)

    def init_robot(self, robot_name = ''):
        if(robot_name == ''):
            print('로봇 이름이 명시되지 않았습니다.')
            return
        if(robot_name == 'irb1200'):
            self.robot_module[0] = Robot_Master_irb1200()
        print('적절한 로봇 모듈로 초기화 시킨다.')

    def init_gripper(self, gripper_name):
        print('적절한 그리퍼 모듈로 초기화 시킨다.')
#        print('하지만 구현은 안했다.')
        if gripper_name == 'schunk_gripper':
            self.gripper = KetiSchunk()
            self.gripper.init()
        #0319
        if gripper_name == 'zimmer_gripper':
            self.gripper = KetiZimmer()
            self.gripper.init()

    def init_device(self):
        self.m_device_master = device_master()
        self.m_device_master.init_device(device_dict = ['mover', 'flipper'])

    #명령어 코드
    #명령어에 사용되는 좌표는 두산 로보틱스M기준의 좌표로 전달된다.
    #해당 명령어는 메인마스터에서 호출된다.
    def move_joint_robot(self,posj, vel=100, acc = 100, time = 0, moveType = 'Joint', syncType = 0, index_robot = 0):
        #self.robot_module[0].m_Robot_Slave.move_pos_robot(posj,vel=vel,acc=acc, moveType = 'Joint', syncType = 0)
        self.robot_module[index_robot].move_joint_robot(posj, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)
        # self.robot_module[index_robot].move_joint_robot(3, vel=vel, acc=acc, time=time, moveType=moveType,
        #                                                 syncType=syncType)
        #joint_init = [0, -15, 15, 0, 90, 0]  # abb

    def move_pos_robot(self, posx, time = 0.0, vel=[10,10], acc= [50,50], moveType = 'Line', syncType = 0, index_robot = 0):
        self.robot_module[index_robot].move_pos_robot(posx, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

    def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0, index_robot = 0):
        self.robot_module[index_robot].move_pos_queue(pos_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)

    def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0, index_robot = 0):
        self.robot_module[index_robot].move_joint_queue(joint_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)

    def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0, index_robot = 0):
        self.robot_module[index_robot].move_pos_circle(waypos, destpos, vel=vel, acc=acc, syncType=syncType)

    def move_jog_robot(self, JOG_AXIS, REF, SPEED, index_robot = 0):
        self.robot_module[index_robot].move_jog_robot(JOG_AXIS, REF, SPEED)

    def get_cur_pos(self, index_robot = 0):
        return self.robot_module[index_robot].get_pos()

    def get_cur_joints(self,index_robot = 0):
        return self.robot_module[index_robot].get_joints()

    def grip(self, mode = 'grip', pos = 0):
        #그립 코드는 언제나 최대파워, 최대 스피드로 집게 구현할 것임.
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.')

    def grip_release(self):
        # print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.')
        while self.connected:
            time.sleep(0.1)
            if self.grip_release_flag is True:
                self.gripper.grip_release()
                self.grip_release_flag = False

    def get_grip_success(self, index_robot = 0):
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.')
        return False

    #공통함수.
    def get_line_based_tcp(self, X_term, Y_term, Z_term, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].m_Robot_Slave.robot_state['pos']  # self.state_robot_pos
        else:
            ABC = base_pos
        # print(ABC)
        nor = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        # print('NO',nor)
        for i in range(3):
            ABC[i] = ABC[i] + nor[0][i] * X_term + nor[1][i] * Y_term + nor[2][i] * Z_term
        return ABC

    def get_rotated_pos_based_euler(self,X_term,Y_term,Z_term,base_pos=[0,0,0,0,0,0]):
        pose = base_pos

        #print('Current Pose : {0}'.format(pose))
        sp = [X_term, Y_term, Z_term]

        ang_z1 = pose[3]
        ang_y = pose[4]
        ang_z2 = pose[5]
        Rz1 = ([[math.cos(math.radians(ang_z1)), -math.sin(math.radians(ang_z1)), 0],
                [math.sin(math.radians(ang_z1)), math.cos(math.radians(ang_z1)), 0],
                [0, 0, 1]])
        Ry = ([[math.cos(math.radians(ang_y)), 0, math.sin(math.radians(ang_y))],
               [0, 1, 0],
               [-math.sin(math.radians(ang_y)), 0, math.cos(math.radians(ang_y))]])
        Rz2 = ([[math.cos(math.radians(ang_z2)), -math.sin(math.radians(ang_z2)), 0],
                [math.sin(math.radians(ang_z2)), math.cos(math.radians(ang_z2)), 0],
                [0, 0, 1]])
        R_z1_y = np.matmul(Rz1, Ry)
        R = np.matmul(R_z1_y, Rz2)
        s = np.matmul(R, sp)

        return [pose[0] + s[0], pose[1] + s[1], pose[2] + s[2], pose[3], pose[4], pose[5]]

    # TCP기준으로 움직이는 걸 전제로 한다.
    def move_line_based_tcp(self, X_term, Y_term, Z_term, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0, vel=[100,100], acc=[200, 200]):
        # self.robot_module[index_robot].move_pos_robot(self.get_line_based_tcp(X_term, Y_term, Z_term, base_pos=base_pos,index_robot= index_robot), syncType = 1)
        # Tool 좌표계 기준 이동 - Hajun 작성
        #pose = self.robot_module[0].m_Robot_Slave.robot_state['pos']
        pose = base_pos

        print('Current Pose : {0}'.format(pose))
        sp = [X_term, Y_term, Z_term]

        ang_z1 = pose[3]
        ang_y = pose[4]
        ang_z2 = pose[5]
        Rz1 = ([[math.cos(math.radians(ang_z1)), -math.sin(math.radians(ang_z1)), 0],
                [math.sin(math.radians(ang_z1)), math.cos(math.radians(ang_z1)), 0],
                [0, 0, 1]])
        Ry = ([[math.cos(math.radians(ang_y)), 0, math.sin(math.radians(ang_y))],
               [0, 1, 0],
               [-math.sin(math.radians(ang_y)), 0, math.cos(math.radians(ang_y))]])
        Rz2 = ([[math.cos(math.radians(ang_z2)), -math.sin(math.radians(ang_z2)), 0],
                [math.sin(math.radians(ang_z2)), math.cos(math.radians(ang_z2)), 0],
                [0, 0, 1]])
        R_z1_y = np.matmul(Rz1, Ry)
        R = np.matmul(R_z1_y, Rz2)
        s = np.matmul(R, sp)



        goal_pose = [pose[0] + s[0], pose[1] + s[1], pose[2] + s[2], pose[3], pose[4], pose[5]]
        # print 'Goal Pose : {0}'.format(goal_pose)
        self.move_pos_robot(goal_pose, vel=vel, syncType=1, acc=acc)
        # pose_list = [current_pose, goal_pose]
        # m_robot_master.move_pos_queue(pose_list, syncType=1, time=5)

    def move_rotate_based_tcp(self, rotate, index_robot = 0):
        ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        ABC[5] += rotate
        self.robot_module[index_robot].move_pos_robot(ABC, syncType = 1)

    def move_line_based_tcp_and_rotate(self, X_term, Y_term, Z_term, rotate, base_pos=[0, 0, 0, 0, 0, 0], time=2.0, vel=[100, 100], acc=[100, 100], index_robot = 0):
        temp_ABC = self.get_line_based_tcp_and_rotate(X_term, Y_term, Z_term, rotate, base_pos=base_pos, index_robot= index_robot)
        self.robot_module[index_robot].move_pos_robot(temp_ABC, time=time,  syncType = 1)
        return temp_ABC

    def get_line_based_tcp_and_rotate(self, X_term, Y_term, Z_term, rotate, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos
        # print('Cur_POs is... %s'%str(ABC))
        nor = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        # print('Curr_ Nor is..', nor)
        for i in range(3):
            ABC[i] = ABC[i] + nor[0][i] * X_term + nor[1][i] * Y_term + nor[2][i] * Z_term
        ABC[5] += rotate
        return ABC

    def move_rotate_x_based_tcp(self, xrotate, index_robot = 0):
        rotated_norvec, temp_pos = self.get_rotate_x_based_tcp(xrotate, index_robot= index_robot)
        State_Cur_Pos = self.robot_module[index_robot].get_cur_pos()
        State_Cur_Pos[3] = temp_pos[0]
        State_Cur_Pos[4] = temp_pos[1]
        State_Cur_Pos[5] = temp_pos[2]
        self.robot_module[index_robot].move_pos_robot(State_Cur_Pos, syncType = 1)

    # 보는 위치 기준 이동을 위해
    def get_rotate_x_based_tcp(self, x_rotate, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos
        nor_vec1, nor_vec2, nor_vec3 = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        # nor_vec1 = RPP_functions.get_polar_2_nor([1,0,0], [ABC[3],ABC[4],ABC[5]])
        # nor_vec2 = RPP_functions.get_polar_2_nor([0, 0, 1], [ABC[3], ABC[4], ABC[5]])
        # print('current direction vec is...%s' % str(nor_vec1))
        # 방향벡터를 위기준으로 틀어주는 회전벡터를 구해준다.
        rota_matrix = RPP_functions.get_rotation_matrix(nor_vec1, x_rotate)

        # 회전 벡터를 현재 보는 방향(z축) 벡터를 틀어준다.
        rotated_norvec = RPP_functions.Rotate_Nor(nor_vec3, rota_matrix)
        rotated_norvec2 = rotated_norvec / np.sqrt(np.sum(rotated_norvec ** 2))
        # print('Rotated Norvec == %s (%s)' %(str(rotated_norvec2),str(rotated_norvec)))
        ret_ABC = RPP_functions.calculaion_nor_2_polar(rotated_norvec2)

        ret_ABC[2] = RPP_functions.get_C_using_inv_rotate(nor_vec1, ret_ABC[0], ret_ABC[1])
        return rotated_norvec2, ret_ABC

    def move_grip_move(self, X_term, Y_term, Z_term, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos

        # self.grip()
        time.sleep(0.1)
        nor = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        for i in range(3):
            ABC[i] = ABC[i] + nor[0][i] * X_term + nor[1][i] * Y_term + nor[2][i] * Z_term
        self.robot_module[index_robot].move_pos_robot(ABC, syncType = 0)
        # time.sleep(0.2) #셕션 체크 후, 센서값 대기를 위한.
        start_time = time.time()  ##정지에 의한 조건에 사용된다.
        last_time = time.time()
        l_condition_time = 1.0  # 해당 초만큼 후 부터 충돌 체크.
        l_condition_speed = 0.015 * self.robot_module[index_robot].m_Robot_Slave.robot_config['time_rate']  # 이 속도보다 작다면 정지다.
        while (True):
            debug_bool = self.get_grip_success(index_robot=index_robot)
            # print(debug_bool)
            # 속도에 의한 정지 체크.
            if (last_time - start_time < l_condition_time):
                last_time = time.time()
            else:
                if (self.speed < l_condition_speed):
                    self.robot_module[index_robot].robot_stop()
                    self.robot_module[index_robot].grip_release()
                    print('\t[Tool Signal] [Warning] Suction stopped by Speed]')
                    return
            if (debug_bool):
                self.robot_module[index_robot].robot_stop()
                print('\t[Tool Signal] Suction stopped by Vacuum')
                return


    # def socket_setting(self):
    #     while self.connected:
    #         time.sleep(0.1)
    #         # try:
    #         data = self.client_socket.recv(1024)
    #         print('[RMS]Received from {0}, {1}'.format(self.addr, data))
    #         message = str(data.decode())
    #
    #         if not data:
    #             self.connected = False
    #         else:
    #             if len(message) > 2:
    #                 if message[0] == '0' and message[1] == '_':
    #                     # data_str = 'RECEIVED0'
    #                     # m_robot_master.client_socket.sendall(data_str)  # 메시지 송신
    #                     # print '[RMS]송신:{0}'.format(data_str)
    #                     if self.step == 0 or self.step == 6 or self.step == 8 or self.step == 2:
    #                         self.step = 1
    #                         self.message = message


    def receiveMsg(self):  #KSH
        while True:
            time.sleep(0.1)
            data = {}
            if not self.msg_queue.empty():
                payload = self.msg_queue.get()
                print('[RMS]Received from {0}, {1}'.format(payload))
                if self.step == 0 or self.step == 6 or self.step == 8 or self.step == 2:
                    self.step = 1
                    self.message = payload['data']

    def mwait(self):
        while self.connected:
            time.sleep(0.1)
            if self.mwait_flag is True:
                cur = self.robot_module[0].m_Robot_Slave.robot_state['pos']
                self.dist = math.sqrt(math.pow(cur[0] - self.ref[0], 2) + math.pow(cur[1] - self.ref[1], 2) + math.pow(cur[2] - self.ref[2], 2))
                # print '[MWAIT]Distance : {0}'.format(self.dist)
                if self.dist < 80:
                    self.mwait_flag = False
                    print('Goal reach...')

    def calculaion_nor_2_polar(self,nor_vec):
        # print(nor_vec)
        if (nor_vec[0] == 0):
            nor_vec[0] = 0.00001
        if (nor_vec[1] == 0):
            nor_vec[1] = 0.00001
        if (nor_vec[2] == 0):
            nor_vec[2] = 0.00001

        a = math.acos(nor_vec[0] / math.sqrt(nor_vec[0] * nor_vec[0] + nor_vec[1] * nor_vec[1]))
        b = math.acos(
            nor_vec[2] / math.sqrt(nor_vec[0] * nor_vec[0] + nor_vec[1] * nor_vec[1] + nor_vec[2] * nor_vec[2]))
        # y방향이 양수이면, x는 0~180을 가지면 된다.
        if (nor_vec[1] < 0):
            a = a * -1
        a = a * 180 / math.pi
        b = b * 180 / math.pi

        # if(a > 90):
        #	b = b * -1
        #	a = a -180

        return [a, b, 0]
