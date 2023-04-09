import math
import numpy as np
import os
from os.path import join

offsetPath = join(os.getcwd(), "module", "vision", "affine", "camera_offset.npy")
affinePath = join(os.getcwd(), "module", "vision", "affine", "camera_to_robot_error_Rt.npy")
affine = np.load(affinePath)
gamma = -70
T_Matrix = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]

# convert robot coord using affine matrix
def convRealRobot3d(rx,ry,rz,g = gamma):
    Robot_Posx = affine[0][0] * rx + affine[0][1] * ry + \
                 affine[0][2] * rz + affine[0][3]

    Robot_Posy = affine[1][0] * rx + affine[1][1] * ry + \
                 affine[1][2] * rz + affine[1][3]

    Robot_Posz = affine[2][0] * rx + affine[2][1] * ry + \
                 affine[2][2] * rz + affine[2][3]

    Robot_Posz += g

    return Robot_Posx, Robot_Posy, Robot_Posz

# convert camera xyz to robot coord using affine matrix
def conCamToRobot3d(CameraPosx, CameraPosy, CameraDepthZ,g = gamma):
    
    Robot_Posx = affine[0][0] * CameraPosx + affine[0][1] * CameraPosy + \
                 affine[0][2] * CameraDepthZ + affine[0][3]

    Robot_Posy = affine[1][0] * CameraPosx + affine[1][1] * CameraPosy + \
                 affine[1][2] * CameraDepthZ + affine[1][3]

    Robot_Posz = affine[2][0] * CameraPosx + affine[2][1] * CameraPosy + \
                 affine[2][2] * CameraDepthZ + affine[2][3]

    Robot_Posz += g

    return Robot_Posx, Robot_Posy, Robot_Posz

# convert normal vector to polar system
def calNormToPolar(nor_vec):
    if (nor_vec[0] == 0):
        nor_vec[0] = 0.00001
    if (nor_vec[1] == 0):
        nor_vec[1] = 0.00001
    if (nor_vec[2] == 0):
        nor_vec[2] = 0.00001

    a = math.acos(nor_vec[0] / math.sqrt(nor_vec[0] * nor_vec[0] + nor_vec[1] * nor_vec[1])) * ( -1 if nor_vec[1] < 0 else 1 )
    b = math.acos(nor_vec[2] / math.sqrt(nor_vec[0] * nor_vec[0] + nor_vec[1] * nor_vec[1] + nor_vec[2] * nor_vec[2]))
    # y방향이 양수이면, x는 0~180을 가지면 된다.
    a = a * 180 / math.pi
    # if abs(a) > 90:
    #     a = ( 180 - abs(a) ) * ( -1 if a > 0 else 1 )
    b = b * 180 / math.pi
    if abs(a) > 90:
        c = 180
    else:
        c = 0
    return [a, b, a]


def rs2_deproject_pixel_to_point(px,py,pz,intr):
    coeffs = [0,0,0,0,0]
    point=[0,0,0]

    fx = intr["fx"]
    fy = intr["fy"]

    ppx = intr["ppx"]
    ppy = intr["ppy"]

    x = (px - ppx) / fx
    y = (py - ppy) / fy
    r2  = x*x + y*y
    f = 1 + coeffs[0]*r2 + coeffs[1]*r2*r2 + coeffs[4]*r2*r2*r2
    ux = x*f + 2*coeffs[2]*x*y + coeffs[3]*(r2 + 2*x*x)
    uy = y*f + 2*coeffs[3]*x*y + coeffs[2]*(r2 + 2*y*y)
    x = ux
    y = uy
    point[0] = pz * x
    point[1] = pz * y
    point[2] = pz
    return point

def get_camera_refpos2tcp_pos(self, ref_pos, T_Matrix):
    T_Matrix = np.array(T_Matrix)
    ref_pos = np.array(ref_pos).reshape(3, 1)
    if (T_Matrix.shape[0] == 4):
        ref_pos = np.append(ref_pos, [1])
        ref_pos = ref_pos.reshape(4, 1)
    ref_pos = np.dot(T_Matrix, ref_pos)
    return ref_pos

# convert camera pixel position to world coord
def convert_pix_to_pos(x,y,d, intr):
    XYZ = np.zeros((3, 1))
    XYZ[0] = ((x - intr["ppx"]) / intr["fx"] * d)
    XYZ[1] = ((y - intr["ppy"]) / intr["fy"] * d)
    XYZ[2] = d
    return XYZ

# convert world coord to camera pixel pisition
def convert_pos_to_pix(x,y,z,intr):
    XYZ = np.zeros((3, 1))
    print(x,y,z)
    XYZ[0] = int(x * intr["fx"] / z + intr["ppx"])
    XYZ[1] = int(y * intr["fy"] / z + intr["ppy"])
    XYZ[2] = int(1000 * z)
    return XYZ