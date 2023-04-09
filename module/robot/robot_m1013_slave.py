#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from dsr_msgs.msg import *
from dsr_msgs.srv import *
import threading
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import os, sys

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"/home/keti/catkin_ws/src/doosan-robot/common/imp")) )
from DSR_ROBOT import *

from Parent_Robot import Parent_Robot

class Robot_Slave_m1013(Parent_Robot):
	def __init__(self):
		self.robot_config['name'] = 'm1013'
		self.cnt = 0
		self.robot_state_num = 0
		self.robot_state_str = ''
		self.init_Ros()
		self.gripper_power_on()
		self.robot_state_external_tool_torque = []

	def init_Ros(self):
		rospy.init_node('single_robot_basic_py')
		rospy.on_shutdown(self.robot_stop)

		self.pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=1)
		self.set_tcp = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tcp/set_current_tcp', SetCurrentTcp)
		self.add_tcp = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/tcp/config_create_tcp', ConfigCreateTcp)
		self.move_line = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/move_line', MoveLine)
		self.move_joint = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/move_joint', MoveJoint)

		self.move_circle = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_circle", MoveCircle)
		self.move_spline_joint = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_spline_joint",
													MoveSplineJoint)
		self.move_spline_pos = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_spline_task",
												  MoveSplineTask)
		self.move_blend = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/motion/move_blending", MoveBlending)

		self.digital_out = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/io/set_digital_output',
											  SetCtrlBoxDigitalOutput)
		self.analog_input = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/io/get_analog_input",
											   GetCtrlBoxAnalogInput)
		self.set_mode_analog_input = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + "/io/set_analog_input_type",
														SetCtrlBoxAnalogInputType)

		self.set_mode_analog_input(0, 1)

		self.set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
		self.set_robot_mode(ROBOT_MODE_MANUAL)
		time.sleep(1)
		print('해당 로봇은 ROBOT_MODE_AUTONOMOUS로 작동됩니다.')
		self.set_robot_mode(ROBOT_MODE_AUTONOMOUS)

		self.move_jog = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/motion/jog', Jog)

		self.start_recv_state()
		#로봇 입력 대기
		while (self.robot_state['alive'] == False):
			time.sleep(1)
		print('\tRos Initalization Complete')
		return True

	def thread_subscriber(self):
		rospy.Subscriber('/' + ROBOT_ID + ROBOT_MODEL + '/state', RobotState, self.get_robot_state, queue_size=10)
		rospy.spin()

	def get_robot_state(self, msg):
		self.robot_state['joint'] = [msg.current_posj[0], msg.current_posj[1], msg.current_posj[2], msg.current_posj[3],
								  msg.current_posj[4], msg.current_posj[5]]
		self.robot_state['pos']= [msg.current_posx[0], msg.current_posx[1], msg.current_posx[2], msg.current_posx[3],
								msg.current_posx[4], msg.current_posx[5]]
		self.robot_state['speed'] = abs(msg.current_velx[0]) + abs(msg.current_velx[1]) + abs(msg.current_velx[2]) + abs(
			msg.current_velx[3]) + abs(msg.current_velx[4]) + abs(msg.current_velx[5])
		robot_state_dynamic_torque = msg.dynamic_tor
		robot_state_joint_torque = msg.actual_jts
		robot_state_external_joint_torque = msg.actual_ejt
		self.robot_state_external_tool_torque = msg.actual_ett

		self.robot_state_num = msg.robot_state
		self.robot_state_str = msg.robot_state_str

		self.cnt += 1
		if (self.cnt % 1) == 0:
			# print '\n========================= Robot State ======================'
			# print 'Robot State : {0}'.format(self.robot_state_str)
			# print 'Current Joint : {0}'.format(self.robot_state['joint'])
			# print 'Current Pose : {0}'.format(self.robot_state['pos'])
			# print 'Current Dynamic Torque : {0}'.format(robot_state_dynamic_torque)
			# print 'Current Joint Torque : {0}'.format(robot_state_joint_torque)
			# print 'Current External Joint Torque : {0}'.format(robot_state_external_joint_torque)
			# print 'Current External Tool Torque : {0}'.format(self.robot_state_external_tool_torque)
			# print '============================================================\n'
			pass

		# if (self.robot_state['pos'] < 180):
			# self.pub_stop.publish(stop_mode=1)
		self.robot_state['alive'] = True

#상속 구현.
	def move_joint_robot(self, posj, vel=100, acc=100, time=0, moveType='Joint', syncType=0):
		mode = 0;
		ref = 0;
		radius = 0.0;
		blendType = 0;
		sol = 0
		syncType = 1 - syncType

		print('\t[command]Move joint %s' % str(posj))
		# config
		vel = vel * self.robot_config['time_rate']
		acc = acc * self.robot_config['time_rate']
		if (time != 0):
			time = time / self.robot_config['time_rate']

		self.move_joint(posj, vel, acc, time, radius, mode, blendType, syncType)

	def move_pos_robot(self, posx, time=0.0, vel=[10, 10], acc=[50, 50], moveType='Line', syncType=0):
		mode = 0;
		ref = 0;
		radius = 0.0
		blendType = 1;
		sol = 0
		print('\t[command]Move 2 %s' % str(posx))
		syncType = 1 - syncType

		# config
		vel = [vel[0] * self.robot_config['time_rate'], vel[1] * self.robot_config['time_rate']]
		acc = [acc[0] * self.robot_config['time_rate'], acc[1] * self.robot_config['time_rate']]
		time = time / self.robot_config['time_rate']

		self.move_line(posx, vel, acc, time, radius, ref, mode, blendType, syncType)

	def ros_listToFloat64MultiArray(self, list_src):
		_res = []
		for i in list_src:
			item = Float64MultiArray()
			item.data = i
			_res.append(item)
		return _res

	def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		syncType = 1 - syncType
		joint_list = self.ros_listToFloat64MultiArray(joint_list)
		if (time == None):
			vel = vel * self.robot_config['time_rate']
			acc = acc * self.robot_config['time_rate']
		else:
			time = time / self.robot_config['time_rate']
		self.move_spline_joint(joint_list, len(joint_list), vel, acc, time, mod, syncType)

	def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0):
		syncType = 1 - syncType
		pos_list = self.ros_listToFloat64MultiArray(pos_list)
		if (time == None):
			vel = vel * self.robot_config['time_rate']
			acc = acc * self.robot_config['time_rate']
		else:
			time = time / self.robot_config['time_rate']
		self.move_spline_pos(pos_list, len(pos_list), [vel, vel], [acc, acc], time, ref, mod, vel_opt, syncType)

	def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0):
		c1 = [waypos[0], waypos[1], waypos[2], waypos[3], waypos[4], waypos[5]]  # [749,-22,562,13,151,-175]
		c2 = [int(destpos[0]), int(destpos[1]), int(destpos[2]), int(destpos[3]), int(destpos[4]),
			  int(destpos[5])]  # [937,-6.43,461.58,32.85,173.22,-161.97]
		CirclePos = self.ros_listToFloat64MultiArray([c1, c2])

		l_time = 0.0;
		radius = 50.0;
		ref = 0;
		mod = 0
		angle = [0, 0]
		ra = 0
		syncType = 1 - syncType

		# config
		vel = [vel[0] * self.robot_config['time_rate'], vel[1] * self.robot_config['time_rate']]
		acc = [acc[0] * self.robot_config['time_rate'], acc[1] * self.robot_config['time_rate']]
		l_time = l_time / self.robot_config['time_rate']

		self.move_circle(CirclePos, vel, acc, l_time, mod, ref, angle[0], angle[1], radius, ra, syncType)

	def move_jog_robot(self, JOG_AXIS, REF, SPEED):
		# 두산 로봇 기준 메뉴얼 모드에서만 실행이 되므로, 별도로 구현해야함.
		self.move_jog(JOG_AXIS, REF, SPEED)

	def start_recv_state(self):
		self.Thread_subscriber = threading.Thread(target=self.thread_subscriber)
		self.Thread_subscriber.daemon = True
		self.Thread_subscriber.start()

	def robot_stop(self, mode='force'):
		print ("shutdown time!")
		# self.grip_release()
		self.pub_stop.publish(stop_mode=3)
		return 0

	# 아래의 코드는 별도로 그리퍼 모듈이 초기화 되지 않으면 호출되는 기본 그리퍼 함수입니다.
	def gripper_power_on(self) :
		self.digital_out(1, 1)
		self.digital_out(2, 1)
		
		self.digital_out(9, 0)
		self.digital_out(10, 0)
		self.digital_out(11, 0)
		self.digital_out(12, 0)

	def gripper_power_off(self) :
		self.digital_out(1, 0)
		self.digital_out(2, 0)
		
		self.digital_out(9, 0)
		self.digital_out(10, 0)
		self.digital_out(11, 0)
		self.digital_out(12, 0)

	def grip(self):
		self.digital_out(15, 1)
		print('\t[Gripper] Grip')

	def grip_release(self):
		self.digital_out(15, 0)
		self.digital_out(8, 1)
		time.sleep(0.1)
		self.digital_out(8, 0)
		print('\t[Gripper] Release')

	def get_grip_success(self):
		#아래와 같이 구현한 이유는, 가끔 값이 리턴이 안되기 때문이다.
		try:
			analog_value = self.analog_input(0)
		except:
			# print("\tanalog Error. its not my falut!")
			return self.get_grip_success()
		# print('suction sensor %s' %analog_value.value)
		if (analog_value.value > 0.5):
			print('\t\t[Info] Gripped')
			return True
		else:
			return False

	def cylinder_up(self):
		self.digital_out(12, 1)
		time.sleep(0.5)
		self.digital_out(12, 0)

	def cylinder_down(self):
		self.digital_out(10, 1)
		time.sleep(0.5)
		self.digital_out(10, 0)

	def magnet_on(self):
		self.digital_out(11, 1)
		time.sleep(0.5)
		self.digital_out(11, 0)

	def magnet_off(self):
		self.digital_out(9, 1)
		time.sleep(0.5)
		self.digital_out(9, 0)

	def cylinder_down_and_magnet_on(self):
		self.digital_out(11, 1)
		self.digital_out(10, 1)
		time.sleep(0.5)
		self.digital_out(11, 0)
		self.digital_out(10, 0)

if __name__ == '__main__':
	# robot operating test
	m_robot_slave = Robot_Slave_m1013()
	# m_robot_slave.move_joint_robot([185.36,3.16,-125.24,180.27,60.8,0.48], vel=50, acc=10, syncType=1)
	# m_robot_slave.move_joint_robot([193.27,-21.51,-81.48,180.0,75.3,19.99], vel=50, acc=10, syncType=1)
	while True:
		pass

	# # gripper operating test
	# m_robot_slave.gripper_power_on()
	# time.sleep(0.5)
	# m_robot_slave.gripper_power_off()

