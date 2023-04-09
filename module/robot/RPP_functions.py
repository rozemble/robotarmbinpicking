#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

import numpy.linalg as lin


def Rotated_2D(vec, C):
	Rotation_Angle_Radi = math.radians(C)
	temp_sin = math.sin(Rotation_Angle_Radi)
	temp_cos = math.cos(Rotation_Angle_Radi)

	ret_pos1 = vec[0] * temp_cos + vec[1] * temp_sin
	ret_pos2 = -vec[0] * temp_sin + vec[1] * temp_cos
	return [ret_pos1, ret_pos2, vec[2]]


def Rotation_POS_y(B):
	Rotation_Angle_Radi = math.radians(B)
	temp_sin = math.sin(Rotation_Angle_Radi)
	temp_cos = math.cos(Rotation_Angle_Radi)
	rot_y = [[temp_cos, 0, temp_sin], [0, 1, 0], [-temp_sin, 0, temp_cos]]
	return rot_y


def Rotation_POS_z(A):
	Rotation_Angle_Radi = math.radians(A)
	temp_sin = math.sin(Rotation_Angle_Radi)
	temp_cos = math.cos(Rotation_Angle_Radi)
	rot_z = [[temp_cos, -temp_sin, 0], [temp_sin, temp_cos, 0], [0, 0, 1]]
	return rot_z


# 극좌표가 주어졌을 때 , 각 기본 벡터(카메라 기준)의 방향 벡터 출력.
# 1224 기준 버그가 발견되어 해당 코드는 사용 안합니다.
def get_polar_2_3D_nor(A, B, C):
	# 원래는 카메라 좌표 기준이었으나, 추후 TCP 좌표 기준으로 바뀌었당.
	vec_1 = Rotated_2D([1, 0, 0], -C)  # TCP 기본 위치 기준이다. 이때 머리방향(X)는 반대다.
	vec_2 = Rotated_2D([0, 1, 0], -C)  # C는 왜 부호가 반대인가?
	vec_3 = Rotated_2D([0, 0, 1], -C)

	vec = [vec_1, vec_2, vec_3]
	dest_4 = vec

	dest_1 = Rotation_POS_y(B)
	dest_2 = Rotation_POS_z(A)

	for i in range(3):
		dest_3 = np.matmul(dest_1, vec[i])
		dest_4[i] = np.matmul(dest_2, dest_3)

	# dest_3 = np.matmul(dest_1, vec)
	# print(dest_1)
	# print(dest_3)
	# dest_4 = np.matmul(dest_2, dest_3)

	return dest_4[0], dest_4[1], dest_4[2]


# 극좌표가 주어졌을 때 , 각 기본 벡터(카메라 기준)의 방향 벡터 출력.
def get_polar_2_nor(point, ABC):
	# 여기의 벡터는 카메라 좌표 기준으로 바뀐 것.
	vec_1 = Rotated_2D(point, ABC[2])

	dest_1 = Rotation_POS_y(ABC[1])
	dest_2 = Rotation_POS_z(ABC[1])

	dest_3 = np.matmul(dest_1, vec_1)
	dest_4 = np.matmul(dest_2, dest_3)

	return dest_4[0], dest_4[1], dest_4[2]


# 주어진 방향벡터에 대한 회전 벡터를 구해준다.
def get_rotation_matrix(p_vec, rotation):
	# print(p_vec)
	l_rota = math.radians(rotation)
	l_sin = math.sin(l_rota)
	l_cos = math.cos(l_rota)
	l_one = 1.0 - l_cos
	# print(l_sin, l_cos, l_one)

	l_vec2 = np.array(p_vec) ** 2
	ret_vec = []
	temp_vec = [0, 0, 0]
	temp_vec[0] = l_vec2[0] * l_one + l_cos
	temp_vec[1] = (p_vec[0] * p_vec[1] * l_one) - (p_vec[2] * l_sin)
	temp_vec[2] = (p_vec[1] * p_vec[2] * l_one) + (p_vec[1] * l_sin)
	ret_vec = np.append(ret_vec, temp_vec, axis=0)

	temp_vec[0] = (p_vec[0] * p_vec[1] * l_one) + (p_vec[2] * l_sin)
	temp_vec[1] = l_vec2[1] * l_one + l_cos
	temp_vec[2] = (p_vec[1] * p_vec[2] * l_one) - (p_vec[0] * l_sin)
	ret_vec = np.append(ret_vec, temp_vec, axis=0)

	temp_vec[0] = (p_vec[0] * p_vec[2] * l_one) - (p_vec[1] * l_sin)
	temp_vec[1] = (p_vec[1] * p_vec[2] * l_one) + (p_vec[0] * l_sin)
	temp_vec[2] = l_vec2[2] * l_one + l_cos
	ret_vec = np.append(ret_vec, temp_vec, axis=0)

	ret_vec = ret_vec.reshape(3, 3)
	print('Rotation Vec == \n%s ' % str(ret_vec))
	return ret_vec


def Rotate_Nor(normals, rot_matrix):
	src_mat = np.array(normals).reshape(3, 1)
	# print(rot_matrix, src_mat)
	ret = np.matmul(rot_matrix, src_mat)
	# print(ret)
	return ret


def calculaion_nor_2_polar(nor_vec):
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


def calculation_polar_2_nor(A, B, C):
	A = A * math.pi / 180
	B = B * math.pi / 180

	temp_term = math.sin(B)
	return [temp_term * math.cos(A), temp_term * math.sin(A), math.cos(B)]


# A,B를 이용 정답 C를 구한다.
def get_C_using_inv_rotate(x_norvec, A, B):
	rota_B = Rotation_POS_y(B)
	rota_A = Rotation_POS_z(A)
	inv_B = lin.inv(rota_B)
	inv_A = lin.inv(rota_A)

	# print(x_norvec)

	dest_1 = np.matmul(inv_A, x_norvec)
	dest_2 = np.matmul(inv_B, dest_1)
	# print(dest_2)
	ret = [math.acos(-dest_2[0]) * 180 / math.pi, math.asin(dest_2[1]) * 180 / math.pi]
	if (ret[1] > 0):  # sin이음수라는건. 원하는 각이 음수라는 것.
		ret[0] = ret[0] * -1
	# print(ret)
	return ret[0]










