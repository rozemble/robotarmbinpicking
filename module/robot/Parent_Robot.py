#!/usr/bin/env python
# -*- coding: utf-8 -*-

#이 파이썬 코드는.
#로봇 모듈을 만들기 위한 부모 클래스 역활을 합니다.
#로봇간의 통신은 이 코드에 있는 함수 양식을 사용합니다.
#로봇이 추가될때마다, 아래의 명령어 들을 맵핑시키면 됩니다.

import sys
import copy

class Parent_Robot:
    #로봇의 상태
    robot_state = {}
    robot_state['joint'] = [0,0,0,0,0,0]
    robot_state['pos'] = [0,0,0,0,0,0]
    robot_state['speed'] = 0
    robot_state['stop'] = True
    robot_state['alive'] = False
    robot_state['tool_torque'] = [0,0,0,0,0,0]

    #로봇 설정.
    robot_config = {}
    robot_config['name'] = 'NULL'
    robot_config['time_rate'] = 1.0 #속도 비율. 1이 맥스.

    #공통 함수
    #복사할 필요가 없어영
    def mapping_function(self, src_robot):
        self.move_joint = src_robot.move_joint
        self.move_pos = src_robot.move_pos
        self.move_joint_queue = src_robot.move_joint_queue
        self.move_pos_queue = src_robot.move_pos_queue
        self.move_pos_circle = src_robot.move_pos_circle
        self.move_jog = src_robot.move_jog
        self.start_recv_state = src_robot.start_recv_state
        self.get_robot_pos = src_robot.get_robot_pos
        self.robot_stop = src_robot.robot_stop

        self.get_cur_joints = src_robot.get_joints
        self.get_cur_pos = src_robot.get_pos

        self.mapping_grippper(src_robot)

    def mapping_grippper(self, src_robot):
        self.grip = src_robot.grip
        self.grip_release = src_robot.grip_release
        self.get_grip_success = src_robot.get_grip_success


    def applied_global_speed(self,vel, acc, time):
        try:
            for i in range(len(vel)):
                ret_vel = vel * self.robot_config['time_rate']
                ret_acc = acc * self.robot_config['time_rate']
                if (time != 0):
                    ret_time = time / self.robot_config['time_rate']
        except:
            print('{}에서 오류'.sys._getframe().f_code.co_name)
            print('아마, vel,acc,time이 int로 넘어온게 아닌가 함.')
            ret_vel=[0]

        return ret_vel, ret_acc, ret_time

    def get_cur_pos(self):
        return copy.deepcopy(self.robot_state['pos'])

#로봇 모듈을 추가할때 아래 코드를 복사해 가면 됨
#어떤 로봇이 오더라도, move_joint, move_pos은 반드시 구현할 것.
    def move_joint_robot(self,posj, vel=100, acc = 100, time = 0, moveType = 'Joint', syncType = 0):
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇은 안움직일겁니다.'.sys._getframe().f_code.co_name)
        print(posj)
    def move_pos_robot(self, posx, time = 0.0, vel=[10,10], acc= [50,50], moveType = 'Line', syncType = 0):
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇은 안움직일겁니다.'.sys._getframe().f_code.co_name)
        print(posx)

    def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType = 0):
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇은 안움직일겁니다.'.sys._getframe().f_code.co_name)
        print(pos_list)

    def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType = 0):
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇은 안움직일겁니다.'.sys._getframe().f_code.co_name)
        print(joint_list)

    def move_pos_circle(self, waypos, destpos, vel = [50,50], acc = [30,30], syncType = 0):
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇은 안움직일겁니다.'.sys._getframe().f_code.co_name)
        print(waypos, destpos)

    def move_jog_robot(self, JOG_AXIS, REF, SPEED):
        #두산 로봇 기준 메뉴얼 모드에서만 실행이 되므로, 별도로 구현해야함.
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇은 안움직일겁니다.'.sys._getframe().f_code.co_name)
        print(JOG_AXIS, REF, SPEED)

    #로봇의 상태를 받아오는 함수는 아래에서 구현하세요.
    def start_recv_state(self):
        print('당신은 기본 로봇의 {}를 호출했습니다. 로봇의 상태는 없습니다.'.sys._getframe().f_code.co_name)

    def robot_stop(self, mode = 'force'):
        print('당신은 기본 로봇의 {}를 호출했습니다. 정지할 로봇이 없습니다.'.sys._getframe().f_code.co_name)

    def get_cur_joints (self):
        print('당신은 기본 로봇의 {}를 호출했습니다. 가져올 정보가 없습니다.'.sys._getframe().f_code.co_name)

#아래의 코드는 별도로 그리퍼 모듈이 초기화 되지 않으면 호출되는 기본 그리퍼 함수입니다.
    def grip(self):
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.'.sys._getframe().f_code.co_name)
    def grip_release(self):
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.'.sys._getframe().f_code.co_name)
    def get_grip_success(self):
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.'.sys._getframe().f_code.co_name)