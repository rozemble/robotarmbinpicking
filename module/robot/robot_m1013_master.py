#!/usr/bin/env python
# -*- coding: utf-8 -*-
#이 파이선 코드는
import os, sys

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../")) )


#from robot_master import robot_instance
from robot_m1013_slave import Robot_Slave_m1013

class Robot_Master_m1013():
	def __init__(self):
		self.m_Robot_Slave = Robot_Slave_m1013()

	#명령어 코드
	#각 로봇에 맞춰 적절한 연산을 해주면 된다.
	#적절한 연산의 예: M로봇을 A로봇의 좌표계로 변환후 작동.
	def move_joint_robot(self,posj, vel=100, acc = 100, time = 0, moveType = 'Joint', syncType = 0):
		self.m_Robot_Slave.move_joint_robot(posj, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

	def move_pos_robot(self, posx, time = 0.0, vel=[10,10], acc= [50,50], moveType = 'Line', syncType = 0):
		self.m_Robot_Slave.move_pos_robot(posx, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

	def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		self.m_Robot_Slave.move_pos_queue(pos_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)

	def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		self.m_Robot_Slave.move_joint_queue(joint_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)

	def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0):
		self.m_Robot_Slave.move_pos_circle(waypos, destpos, vel=vel, acc=acc, syncType=syncType)

	def move_jog_robot(self, JOG_AXIS, REF, SPEED):
		self.m_Robot_Slave.move_jog_robot(JOG_AXIS, REF, SPEED)

	def robot_stop(self, mode = 'force'):
		self.m_Robot_Slave.robot_stop(mode = mode)