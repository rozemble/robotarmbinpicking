#!/usr/bin/env python
# -*- coding: utf-8 -*-


import threading
import os, sys, time
from module.robot.Parent_Robot import Parent_Robot
import socket
from datetime import datetime
from module.robot.abb_example import Robot
import math
import numpy as np
from pyquaternion import Quaternion

ROBOT_MODEL = 'irb1200'
HOST = '192.168.125.1'
MOTION_PORT = 5000
LOGGER_PORT = 5001

JOINT_LIMIT = [[-170, 170], [-100, 135], [-200,  70], [-270, 270], [-130, 130], [-400, 400]]


class Robot_Slave_irb1200(Parent_Robot, Robot):
    def __init__(self):
        self.robot_config['name'] = 'irb1200'

        self.init_Robot()

    def init_Robot(self):
        self.init(ip=HOST, port_motion=MOTION_PORT)
        # self.connect_motion((HOST, MOTION_PORT))
        # self.set_units('millimeters', 'degrees')
        # self.set_tool()
        # self.set_workobject()
        # self.set_speed(speed_value=100)
        # self.set_zone(zone_key='z1`', point_motion=False)

        # now = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        # self.logging_flag = False
        # py_file_path = os.path.dirname(os.path.abspath(__file__))
        # if self.logging_flag is True:
        #     self.logging_file = open(py_file_path + '/abb_logging/' + str(now) + '-logging.csv', 'w')
        self.start_recv_state()

        while (self.robot_state['alive'] == False):
            pass
            # time.sleep(1)
        print('\tRobot Initalization Complete')
        return True

    def thread_subscriber(self):
        self.sock_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_logger.settimeout(None)
        try:
            self.sock_logger.connect((HOST, LOGGER_PORT))
            self.sock_logger.settimeout(None)
            self.logger_connected = True
            print('[{0}]Connected by {1}'.format(ROBOT_MODEL, (HOST, LOGGER_PORT)))
        except socket.error:
            print('[{0}]Disconnected by {1}'.format(ROBOT_MODEL, (HOST, LOGGER_PORT)))
            self.logger_connected = False

        self.get_robot_state()

    def get_robot_state(self):
        # now = datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%3s")
        # flag = 1
        while True:
            # time.sleep(0.1)
            if self.logger_connected is True:
                try:
                    # if flag == 1:
                    #     self.sock_logger.sendall("# ")
                    data = self.sock_logger.recv(1024).decode()
                    data_split = data.split()
                    if data_split[0] == '#' and data_split[1] == '0':
                        flag = 0
                        # pose = []
                        # for i in xrange(0, 7):
                            # pose.append(data_split[i + 5])
                        # self.robot_state['pos'] = pose

                        pose = [float(data_split[5]), float(data_split[6]), float(data_split[7]), float(data_split[8]), float(data_split[9]), float(data_split[10]), float(data_split[11])]

                        qw = pose[3]
                        qx = pose[4]
                        qy = pose[5]
                        qz = pose[6]
                        sqw = qw*qw
                        sqx = qx*qx
                        sqy = qy*qy
                        sqz = qz*qz

                        invs = 1 / (sqx + sqy + sqz + sqw)
                        m00 = (sqx - sqy - sqz + sqw) * invs
                        m11 = (-sqx + sqy - sqz + sqw) * invs
                        m22 = (-sqx - sqy + sqz + sqw) * invs

                        tmp1 = qx * qy
                        tmp2 = qz * qw
                        m10 = 2.0 * (tmp1 + tmp2) * invs
                        m01 = 2.0 * (tmp1 - tmp2) * invs

                        tmp1 = qx * qz
                        tmp2 = qy * qw
                        m20 = 2.0 * (tmp1 - tmp2) * invs
                        m02 = 2.0 * (tmp1 + tmp2) * invs
                        tmp1 = qy * qz
                        tmp2 = qx * qw
                        m21 = 2.0 * (tmp1 + tmp2) * invs
                        m12 = 2.0 * (tmp1 - tmp2) * invs

                        if m11 == 1:
                            euler_Z1 = 0
                            euler_Z2 = 0
                            euler_Y = math.atan2(m02, m00)
                        else:
                            euler_Z1 = math.atan2(m12, m02)
                            euler_Z2 = math.atan2(m21, -m20)
                            euler_Y = math.atan2(m21, m22*math.sin(euler_Z2))

                        # print('euler angle : ', [math.degrees(euler_Z1), math.degrees(euler_Y), math.degrees(euler_Z2)])
                        # print('rotation : {0}\n\t{1}\n\t{2}'.format([m00, m01, m02],[m10, m11, m12],[m20, m21, m22]))

                        self.robot_state['pos'] = [pose[0], pose[1], pose[2], math.degrees(euler_Z1), math.degrees(euler_Y), math.degrees(euler_Z2)]

                        # print('Pose : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' % (float(data_split[5]), float(data_split[6]), float(data_split[7]), float(data_split[8]), float(data_split[9]), float(data_split[10]), float(data_split[11])))
                        # if self.logging_flag is True:
                        #     self.logging_file.write('[' + str(now) + '] Pose:,' + data_split[5] + ',' + data_split[6] + ',' + data_split[7] + ',' + data_split[8] + ',' + data_split[9] + ',' + data_split[10] + ',' + data_split[11] + '\n')
                    elif data_split[0] == '#' and data_split[1] == '1':
                        # joint = []
                        # for i in xrange(0, 6):
                            # joint.append(data_split[i + 5])
                        self.robot_state['joint'] = [float(data_split[5]), float(data_split[6]), float(data_split[7]), float(data_split[8]), float(data_split[9]), float(data_split[10])]
                        flag = 1
                        # print('Joint : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' % (float(data_split[5]), float(data_split[6]), float(data_split[7]), float(data_split[8]), float(data_split[9]), float(data_split[10])))
                        # if self.logging_flag is True:
                        #     self.logging_file.write('[' + str(now) + '] Joint:' + data_split[5] + ',' + data_split[6] + ',' + data_split[7] + ',' + data_split[8] + ',' + data_split[9] + ',' + data_split[10] + '\n')

                    self.robot_state['alive'] = True
                except:
                    # print('Logger socket error %s ' % str(msg))
                    self.logger_connected = False
                    # self.logging_file.close()

    def move_joint_robot(self, posj, vel=100, acc=100, time=0, moveType='Joint', syncType=1):
        limit_flag = False
        for i in range(0, 6):
            if posj[i] < JOINT_LIMIT[i][0]:
                print('Joint out fo range #%d' % (i+1))
                limit_flag = True
            elif posj[i] > JOINT_LIMIT[i][1]:
                print('Joint out fo range #%d' % (i+1))
                limit_flag = True

        if limit_flag is False:
            self.set_speed(vel)
            self.set_joints(posj, syncType)
            # print("BEFORE MOVE JOINT pos state :",self.robot_state['pos'])
            # self.get_robot_state()
            # print("AFTER MOVE JOINT pos state :",self.robot_state['pos'])
            print('\t[command]Move joint %s' % str(posj))

    def convertToQuaternion(self,euler):

        R1 = [[math.cos(math.radians(euler[0])), -math.sin(math.radians(euler[0])), 0],
              [math.sin(math.radians(euler[0])), math.cos(math.radians(euler[0])), 0],
              [0, 0, 1]]
        R2 = [[math.cos(math.radians(euler[1])), 0, math.sin(math.radians(euler[1]))],
              [0, 1, 0],
              [-math.sin(math.radians(euler[1])), 0, math.cos(math.radians(euler[1]))]]
        R3 = [[math.cos(math.radians(euler[2])), -math.sin(math.radians(euler[2])), 0],
              [math.sin(math.radians(euler[2])), math.cos(math.radians(euler[2])), 0],
              [0, 0, 1]]
        R12 = np.matmul(R1, R2)
        R = np.matmul(R12, R3)

        trace = R[0][0] + R[1][1] + R[2][2]
        # print("R :", R)
        # print("TRACE :", trace)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (R[2][1] - R[1][2]) / s
            y = (R[0][2] - R[2][0]) / s
            z = (R[1][0] - R[0][1]) / s
        else:
            if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
                s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
                w = (R[2][1] - R[1][2]) / s
                x = 0.2 * s
                y = (R[0][1] + R[1][0]) / s
                z = (R[0][2] + R[2][0]) / s
            elif R[1][1] > R[2][2]:
                s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
                w = (R[0][2] - R[2][0]) / s
                x = (R[0][1] + R[1][0]) / s
                y = 0.25 * s
                z = (R[1][2] + R[2][1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
                w = (R[1][0] - R[0][1]) / s
                x = (R[0][2] + R[2][0]) / s
                y = (R[1][2] + R[2][1]) / s
                z = 0.25 * s
        q = Quaternion(matrix=R)

        return [q[0], q[1], q[2], q[3]]


    def move_pos_robot(self, posx, time=0.0, vel=[10, 10], acc=[50, 50], moveType='Line', syncType=1):
        # if type(vel) == type(list):
        #     self.set_speed(vel[0])
        # elif type(vel) == type(int):
        #     self.set_speed(vel)
        self.set_speed(vel[0])
        self.set_acceleration(acc)

        if len(posx) == 6:
            euler = posx[3:6]
            R1 = [[math.cos(math.radians(euler[0])), -math.sin(math.radians(euler[0])), 0],
                  [math.sin(math.radians(euler[0])),  math.cos(math.radians(euler[0])), 0],
                  [0, 0, 1]]
            R2 = [[math.cos(math.radians(euler[1])), 0, math.sin(math.radians(euler[1]))],
                  [0, 1, 0],
                  [-math.sin(math.radians(euler[1])), 0, math.cos(math.radians(euler[1]))]]
            R3 = [[math.cos(math.radians(euler[2])), -math.sin(math.radians(euler[2])), 0],
                  [math.sin(math.radians(euler[2])),  math.cos(math.radians(euler[2])), 0],
                  [0, 0, 1]]
            R12 = np.matmul(R1, R2)
            R = np.matmul(R12, R3)

            trace = R[0][0] + R[1][1] + R[2][2]
            # print("R :", R)
            # print("TRACE :", trace)
            if trace > 0:
                s = 0.5/math.sqrt(trace+1.0)
                w = 0.25*s
                x = (R[2][1] - R[1][2])/s
                y = (R[0][2] - R[2][0])/s
                z = (R[1][0] - R[0][1])/s
            else:
                if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
                    s = 2.0*math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
                    w = (R[2][1] - R[1][2])/s
                    x = 0.2*s
                    y = (R[0][1] + R[1][0])/s
                    z = (R[0][2] + R[2][0])/s
                elif R[1][1] > R[2][2]:
                    s = 2.0*math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
                    w = (R[0][2] - R[2][0])/s
                    x = (R[0][1] + R[1][0])/s
                    y = 0.25*s
                    z = (R[1][2] + R[2][1])/s
                else:
                    s = 2.0*math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
                    w = (R[1][0] - R[0][1])/s
                    x = (R[0][2] + R[2][0])/s
                    y = (R[1][2] + R[2][1])/s
                    z = 0.25*s

            mag = math.sqrt(w*w + x*x + y*y + z*z)
            w = w/mag
            x = x/mag
            y = y/mag
            z = z/mag

            q = Quaternion(matrix=R)

            # print(R)
            # print(q)

            posx = [posx[0], posx[1], posx[2], q[0], q[1], q[2], q[3]]

        self.set_cartesian(posx, syncType)

        # print("BEFORE MOVE POS pos state :", self.robot_state['pos'])
        # self.get_robot_state()
        # print("AFTER MOVE POS pos state :", self.robot_state['pos'])
        #print('[command]Move 2 %s' % str(posx))

    def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
        self.set_speed(speed_value=vel)
        self.buffer_set_joint(joint_list)

        if syncType == 1:
            pass
        elif syncType == 0:
            pass

        self.buffer_execute_joint(syncType)
        print('\t[command]Move joint list %s' % str(joint_list))

    def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
        self.set_speed(speed_value=vel)
        self.buffer_set_pose(pos_list)

        if syncType == 1:
            pass
        elif syncType == 0:
            pass

        self.buffer_execute_pose()
        print('\t[command]Move pose list %s' % str(pos_list))

    def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0):
        self.set_speed(vel)
        if len(waypos) == 6:
            q1 = self.convertToQuaternion(waypos[3:6])
            c1 = [waypos[0], waypos[1], waypos[2], q1[0], q1[1], q1[2],q1[3]]  # [749,-22,562,13,151,-175]
        else:
            c1 = [waypos[0], waypos[1], waypos[2], waypos[3], waypos[4], waypos[5]]  # [749,-22,562,13,151,-175]
        
        if len(destpos) == 6:
            q2 = self.convertToQuaternion(destpos[3:6])
            c2 = [destpos[0], destpos[1], destpos[2], q2[0], q2[1], q2[2],q2[3]]  # [749,-22,562,13,151,-175]
        else:
            c2 = [int(destpos[0]), int(destpos[1]), int(destpos[2]), int(destpos[3]), int(destpos[4]),
                  int(destpos[5])]  # [937,-6.43,461.58,32.85,173.22,-161.97]
        self.move_circular(c1, c2)

    def start_recv_state(self):
        self.Thread_subscriber = threading.Thread(target=self.thread_subscriber)
        self.Thread_subscriber.daemon = True
        self.Thread_subscriber.start()

    def robot_stop(self, mode=''):
        print ("shutdown time!")
        self.close()

        return 0

    def get_cur_pos(self):
        return self.get_cartesian()

    def get_cur_joints(self):
        return self.get_joints()

if __name__ == '__main__':
    # robot operating test
    m_robot_slave = Robot_Slave_irb1200()

    joint1 = [0, 0, 0, 0, 90, 0]
    joint2 = [150, 0, 0, 0, 0, 0]
    joint_list = [joint1, joint2]
    time.sleep(1)

    alpha = 0.1
    vel_value = 2000 * alpha
    acc_value = 10
    vel = [vel_value, vel_value]  # [2000, 2000]
    acc = [acc_value, acc_value]  # [1000, 1000]

    m_robot_slave.move_joint_robot(joint1, vel=vel[0], acc=acc[0], syncType=1)

    while True:
        try:
            print(m_robot_slave.robot_state['pos'])
            print(m_robot_slave.robot_state['joint'])

            m_robot_slave.move_joint_robot(joint1, vel=vel[0], acc=acc[0], syncType=1)

            target = [612.487, -25.527, 420.748, 30.0, -180.0, 150.0]

            pose_target_on = [target[0], target[1], target[2] + 50, target[3] / 2, target[4], -target[3] / 2]
            m_robot_slave.move_pos_robot(pose_target_on, syncType=1, vel=vel, acc=acc)
            # time.sleep(2.0)

            # target 가까이로 이동
            print('target 가까이로 이동')
            pose_target_near = [target[0], target[1], target[2], target[3] / 2, target[4], -target[3] / 2]
            m_robot_slave.move_pos_robot(pose_target_near, syncType=1, vel=vel, acc=acc)
            # time.sleep(2.0)

            # target 위로 이동
            print('target 위로 이동')
            pose_target_on = [target[0], target[1], target[2] + 60, target[3] / 2, target[4], -target[3] / 2]
            m_robot_slave.move_pos_robot(pose_target_on, syncType=1, vel=vel, acc=acc)
            # time.sleep(2.0)

            # place 위치로 이동
            print('place 위치로 이동')
            pose_place = [506.520, 399.755, 380.118, target[3] / 2, target[4], -target[3] / 2]
            m_robot_slave.move_pos_robot(pose_place, syncType=1, vel=vel, acc=acc)
            # joint_place = [38.29, 61.46, -37.5, -0.0, 66.03, 8.29]
            # m_robot_slave.move_joint_robot(joint_place, syncType=1, vel=vel[0], acc=acc[0])
            # time.sleep(2.0)

        except KeyboardInterrupt:
            m_robot_slave.robot_stop()
