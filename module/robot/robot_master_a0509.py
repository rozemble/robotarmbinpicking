#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/../tcp_module')
from module.comm.serverSocket import ServerSocket
import json ##데이터 통신용
import numpy as np
import cv2

import threading
#노말 벡터 구하기 용
sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/../robot_module/robot')
#import RPP_functions_1224

class robot_master(ServerSocket):
	OD_SEND = {}
	OD_SEND["ORDER"] = "FIX"
	OD_SEND["PP"] = []
	OD_SEND["GRIP_SIZE"] = 40
	OD_SEND["GRIP_ROTATION"] = 0.0
	OD_SEND["CONFIG_GRIPPER"] = {}      #그리퍼의 설정값을 전달합니다.
	OD_SEND["NOR_VEC"] = [0,180,0]      #두산 회전 좌표계 기준으로 전달.

	OD_RECV = {}
	OD_RECV["ORDER"] = "FIX"

	temp_mask = []
	temp_mask_offset = [[]]

	queue_order= []

	def start_vm(self, predefined_pos = {}, mod_path = {}, config_gripper = {}, config_camera = {}):
		self.predefined_pos = predefined_pos
		self.mod_path = mod_path
		self.config_gripper = config_gripper
		self.config_camera = config_camera

		self.tcp_Init(TCP_PORT=4383, mode='server', socket_type = 'TCP', TCP_IMMORTAL = True, callback_func = self.send_init)
		print('비전 모듈 연결 완료')
		print('비전 모듈 연결 완료')
		self.recv_robot_master()

	def get_order(self):
		if(len(self.queue_order)>0):
			return self.queue_order.pop()
		return False

	def send_init(self):
		OD_SEND = {}
		OD_SEND["ORDER"] = "CONFIG"
		OD_SEND["MODE_PATH"] = self.mod_path
		OD_SEND["PREDEFINED_POS"] = self.predefined_pos
		OD_SEND["CONFIG_GRIPPER"] = self.config_gripper
		OD_SEND["CONFIG_CAMERA"] = self.config_camera
		OD_SEND["CONFIG_BIN"] = [475,-192,99,478,154,101,682,154,105,681 ,-192,103]
		self.send_result(OD_SEND)

	def send_OD(self, order = "FIX", target_pos = [0,0,0], grip_size = 40, rotation = 0.0, zyz = [0,180,0]):
		OD_SEND = {}
		OD_SEND["ORDER"] = order
		OD_SEND["PP"] = target_pos
		OD_SEND["GRIP_SIZE"] = grip_size
		OD_SEND["GRIP_ROTATION"] = rotation
		OD_SEND["RXYZ"] = zyz	#대상 로봇의 방향 백터
		#nor_vec = RPP_functions_1224.calculation_polar_2_nor(zyz[0], zyz[1], zyz[2])
		#OD_SEND["NOR_VEC"] = nor_vec

		print('로봇에게 전달됨.')
		print(OD_SEND)

		#OD_SEND["ROI"] = roi
		self.send_result(OD_SEND)

	def recv_robot_master(self):
		event_str = 'normal'
		def thread_recv_robot():
			while(True):
				try:
					recv_data = self.wait_for_recv()
					self.queue_order.append(recv_data)
				except:
					return

		temp_thread = threading.Thread(target=thread_recv_robot)
		temp_thread.daemon = False
		temp_thread.start()


if __name__ == '__main__':
	aaa = vision_master()
	aaa.start_vm()