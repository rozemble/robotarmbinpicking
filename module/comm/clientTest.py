#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import cv2
import numpy as np
import numpy
import sys
import threading
import time
import json
from ctypes import *

mode_OD = 0
mode_PE = 1

#json 덤프용
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class ClientTest:
    class my_dict:
        print('')

    python_version = int(sys.version[0])

    TCP_Config_IP = '127.0.0.1'
    TCP_Config_PORT = 4382
    TCP_Config_CONNECT_TYPE = ''
    TCP_Config_Socket_Type = 'TCP'
    TCP_Config_IMMORTAL = False

    m_Socket_SOCKET= []
    m_Socket_CLIENT_SOCKET= []
    m_Socket_TARGET_ADDR = []

    m_State_CONNECTION = 'nothing is over'
    m_state_Time = int(round(time.time() * 1000))

    m_recv_json = []

    m_visual = True
    m_state_send = False

    m_callback_func = []

    m_send_queue = []
    dead_lock_send = False

    port = 4383
    mode = "client"

    def __init__(self):
        port = 4383
        mode = "client"
        #self.tcp_Init(TCP_PORT=port, mode=mode)

#연결 함수
    def tcp_Init(self, TCP_IP= '127.0.0.1', TCP_PORT = 4382, mode = '', socket_type = 'TCP', state_thread = True, visual = True, TCP_IMMORTAL = False, callback_func = []):
        self.TCP_Config_CONNECT_TYPE = mode
        self.TCP_Config_IP = TCP_IP
        self.TCP_Config_PORT = TCP_PORT
        self.TCP_Config_Socket_Type = socket_type
        self.TCP_Config_IMMORTAL = TCP_IMMORTAL
        if(mode == ''):
            print('서버 또는 클라이언트를 명시하세요 (ex. mode="server"')
            return False
        if(mode == 'server'):
            self.server_Init(TCP_PORT= TCP_PORT, socket_type = socket_type)
        elif(mode == 'client'):
            self.client_Init(TCP_IP=TCP_IP, TCP_PORT=TCP_PORT, socket_type = socket_type)
        if(state_thread == True):
            self.start_community()
        self.m_visual = visual
        #만약 재연결시 실행하야할 함수가 있다면..
        if(callback_func != []):
            self.m_callback_func =callback_func
            self.m_callback_func()

    #아이피 포트 등등 다시 넣기 귀찮을때 사용하는 함수.
    def tcp_reconnect(self):
        self.tcp_Init(TCP_IP= self.TCP_Config_IP , TCP_PORT = self.TCP_Config_PORT , mode = self.TCP_Config_CONNECT_TYPE )


    #클라이언트로서 연결을 합니다.
    def client_Init(self, TCP_IP = '127.0.0.1', TCP_PORT = 4382, socket_type = 'TCP'):
        self.TCP_Config_IP = TCP_IP
        self.TCP_Config_PORT  = TCP_PORT
        print("PORT:",self.TCP_Config_PORT)
        if (socket_type == 'TCP'):
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.m_Socket_SOCKET.connect((self.TCP_Config_IP, self.TCP_Config_PORT))
        else:
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.m_Socket_CLIENT_SOCKET = self.m_Socket_SOCKET
        self.m_State_CONNECTION  = '[state]ready'
        print('client_ready')

    def server_Init(self, TCP_PORT = 4382, socket_type = 'TCP'):
        self.TCP_Config_IP  = ""  #서버니깐 IP는 없어요요
        self.TCP_Config_PORT  = TCP_PORT
        if(socket_type == 'TCP'):
            print("[TCP MODULE] OPEN PORT as SERVER(%d)"%self.TCP_Config_PORT )
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            while(True):
                try:
                    self.m_Socket_SOCKET.bind((self.TCP_Config_IP , self.TCP_Config_PORT ))
                    break
                except Exception as ex:
                    self.TCP_Config_PORT = self.TCP_Config_PORT + 1
                    print('bind 에러', ex) # ex는 발생한 에러의 이름을 받아오는 변수
                    pass
            self.m_Socket_SOCKET.listen(10)    #10초 만큼 연결 대기.
            self.m_Socket_CLIENT_SOCKET, self.m_Socket_TARGET_ADDR  = self.m_Socket_SOCKET.accept()
            print("[TCP MODULE] SERVER CONNECTED 2 (%s)"%str(self.m_Socket_TARGET_ADDR ))
        else:
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.m_Socket_SOCKET.bind((self.TCP_Config_IP, self.TCP_Config_PORT))
            self.m_Socket_CLIENT_SOCKET = self.m_Socket_SOCKET
            print("[UDP MODULE] SERVER READY")

        self.m_State_CONNECTION  = '[state]ready'


    def start_community(self):
        for i in range(5):
            if(self.m_State_CONNECTION  == '[state]ready'):
                break
            print('아직 통신할 준비가 안됐다.')
            print('1초 후에 다시 확인')

        #self.thread_tcp2 = threading.Thread(target=self.send_queue_thread)
        #self.thread_tcp2.daemon = False
        #self.thread_tcp2.start()
        self.thread_tcp = threading.Thread(target=self.recv_community)
        self.thread_tcp.daemon = False
        self.thread_tcp.start()

    #수신 함수.
    #입력이 들어올때까지 대기를 걸어두는 함수.
    def wait_for_recv(self):
        while (self.m_State_CONNECTION  != '[state]ready_to_return'):
            time.sleep(0.1)
            if (self.m_State_CONNECTION  == '[state]doomed'):
                return
        self.m_State_CONNECTION = '[state]ready'
        return self.m_recv_json

    def recv_community(self):
        while True:
            while (self.m_State_CONNECTION != '[state]ready'):
                if (self.m_State_CONNECTION == '[state]doomed'):
                    self.disconnection()
                    return
                time.sleep(0.1)
            try:
                self.m_recv_json = self.recv_json()
                if(self.m_recv_json == False):
                    self.disconnection()
                    return
            except:
                self.disconnection()
                return


            #self.m_State_CONNECTION = '[state]ready_to_return'
            self.m_State_CONNECTION = '[state]ready'

    def recv_json(self):
        try:
            l_recv_data = self.recv_data()
            print("mstate:",self.m_State_CONNECTION)
            print("recvdata:",l_recv_data)
            str_data = []
            if (self.python_version > 2):
                str_data = str(l_recv_data, encoding='UTF-8')
            else:
                str_data = str(l_recv_data).encode('UTF-8')  # ).replace("'","\"")
            self.m_recv_json = json.loads(str_data)
            # print(type(self.m_recv_json))
            return self.m_recv_json
        except:
            print('[TCP ERROR]recv error?', sys.exc_info()[0])
            #print(l_recv_data)
            return False

    def recvall(self, count):
        buf = b''
        while count:
            if (self.TCP_Config_Socket_Type == 'TCP'):
                newbuf = self.m_Socket_CLIENT_SOCKET.recv(count)
            elif(self.TCP_Config_Socket_Type == 'UDP'):
                newbuf, counter_udp = self.m_Socket_CLIENT_SOCKET.recvfrom(count)
                print(count)
            #print('im %s : %s'%(self.TCP_Config_CONNECT_TYPE,newbuf))
            #newbuf = self.m_Socket_CLIENT_SOCKET.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def recv_data(self):
        self.m_Socket_SOCKET.settimeout(999)
        try:
            length = int(self.recvall(16))  # 길이 16의 데이터를 먼저 수신하는 것은 여기에 이미지의 길이를 먼저 받아서 이미지를 받을 때 편리하려고 하는 것이다.
            if(length == 0):
                print('[TCP] %s(%s)에 의한 연결 종료'%(str(self.TCP_Config_IP,self.TCP_Config_PORT)))
                return False
            if (self.m_visual):
                print("[RECVED]Data Size == %d" % length)
            stringData = self.recvall(length)
            if (self.m_visual):
                print('이전 SEND 함수 이후 걸린 시간. (msec) == %d'%(int(round(time.time() * 1000)- self.m_state_Time)))
            self.m_state_Time = int(round(time.time() * 1000))
            return stringData
        except Exception as ex:
            print('[TCP] RECV하는 과정에서 에러 발생. %s(%s)'%(str(self.TCP_Config_IP,self.TCP_Config_PORT)))
            print(ex)
            return False

    #보낼 문자열을 스택에 쌓아놓는다. 사실 스택은 아니다.
    #이렇게 하면 수신 실폐해도 멈추는거 읍다.
    def stack_str(self, stringData):
        self.thread_send = threading.Thread(target=self.send_str, args=(stringData,))
        self.thread_send.daemon = True
        self.thread_send.start()


    def send_queue_thread(self):
        while(True):
            if(self.dead_lock_send == True):
                pass
            elif(len(self.m_send_queue)>0):
                inf = self.m_send_queue.pop()
                print(inf.keys())
                l_data = json.dumps(inf, cls=NumpyEncoder)
                # 이미지를 보내기에는 아래 코드는 느리다.
                # l_data = json.dumps(inf, ensure_ascii=False, cls=NumpyEncoder, sort_keys=True, indent=4)
                l_send_size = len(l_data)
                l_send_data = l_data.encode()
                try:
                    if (self.TCP_Config_Socket_Type == 'TCP'):
                        self.m_Socket_CLIENT_SOCKET.sendall(str(l_send_size).ljust(16).encode())
                        self.m_Socket_CLIENT_SOCKET.sendall(l_send_data)
                    elif (self.TCP_Config_Socket_Type == 'UDP'):
                        packet_size = 10240
                        temp_send_size = l_send_size
                        self.m_Socket_CLIENT_SOCKET.sendto(str(l_send_size).ljust(16).encode(),
                                                           (self.TCP_Config_IP, self.TCP_Config_PORT))
                        while (temp_send_size >= 0):
                            print(temp_send_size)
                            temp_send_size -= packet_size
                            send_size = packet_size
                            if (temp_send_size < 0):
                                send_size += temp_send_size
                            self.m_Socket_CLIENT_SOCKET.sendto(l_send_data[0:packet_size],
                                                               (self.TCP_Config_IP, self.TCP_Config_PORT))
                            l_send_data = l_send_data[packet_size:]
                except:
                    print('[TCP ERROR] SEND')
                    self.m_State_CONNECTION = '[state]doomed'
                    return
                if (self.m_visual):
                    print('[TCP SEND]대상 IP(%s, %s)로 메세지 전달.' % (self.TCP_Config_IP, self.TCP_Config_PORT))
                    print('[Info]Send Size is.. %d' % l_send_size)
                    print('이전 SEND 함수 이후 걸린 시간. (msec) == %d' % (int(round(time.time() * 1000) - self.m_state_Time)))
                self.m_state_Time = int(round(time.time() * 1000))
                self.m_state_send = False
            else:
                time.sleep(0.1)

    # 통신하기 편하게 만든 코드.
    # 기본적으로 보낼려는 데이터 싸이즈를 먼저 보내서.
    # 받는 쪽이 파싱하게 편하게 했다.
    def send_json(self, inf):
        def send_thread():
            while(self.m_state_send == True):
                time.sleep(0.1)
            self.m_state_send = True
            l_data = json.dumps(inf, cls=NumpyEncoder)
            print("sendmstat:",self.m_State_CONNECTION)
            print("send data:", l_data)
            #이미지를 보내기에는 아래 코드는 느리다.
            #l_data = json.dumps(inf, ensure_ascii=False, cls=NumpyEncoder, sort_keys=True, indent=4)
            l_send_size = len(l_data)
            l_send_data = l_data.encode()
            try:
                if (self.TCP_Config_Socket_Type == 'TCP'):
                    self.m_Socket_CLIENT_SOCKET.sendall(str(l_send_size).ljust(16).encode())
                    self.m_Socket_CLIENT_SOCKET.sendall(l_send_data)
                elif (self.TCP_Config_Socket_Type == 'UDP'):
                    packet_size = 10240
                    temp_send_size = l_send_size
                    self.m_Socket_CLIENT_SOCKET.sendto(str(l_send_size).ljust(16).encode(),
                                                       (self.TCP_Config_IP, self.TCP_Config_PORT))
                    while (temp_send_size >= 0):
                        print(temp_send_size)
                        temp_send_size -= packet_size
                        send_size = packet_size
                        if (temp_send_size < 0):
                            send_size += temp_send_size
                        self.m_Socket_CLIENT_SOCKET.sendto(l_send_data[0:packet_size],
                                                           (self.TCP_Config_IP, self.TCP_Config_PORT))
                        l_send_data = l_send_data[packet_size:]
            except:
                print('[TCP ERROR] SEND')
                self.m_State_CONNECTION = '[state]doomed'
                return

            if (self.m_visual):
                print('[TCP SEND]대상 IP(%s, %s)로 메세지 전달.' % (self.TCP_Config_IP , self.TCP_Config_PORT ))
                print('[Info]Send Size is.. %d' % l_send_size)
                print('이전 SEND 함수 이후 걸린 시간. (msec) == %d'%(int(round(time.time() * 1000)- self.m_state_Time)))
            self.m_state_Time = int(round(time.time() * 1000))
            self.m_state_send = False
        self.thread_send = threading.Thread(target=send_thread)
        self.thread_send.daemon = True
        self.thread_send.start()


    #200131일자로 send_json와 통합되었어영
    def send_result(self, inf):
        self.send_json(inf)
        #self.dead_lock_send = True
        #self.m_send_queue.append(inf)
        #self.dead_lock_send = False
    #String 형태로 변환한 이미지를 socket을 통해서 전송
    #200131일자로 send_json와 통합되었어영
    def send_str(self,stringData):
        #self.m_send_queue.append(stringData)
        self.send_json(stringData)

    #이미지만 단독으로 보내야 할 때 사용하시죠.
    def send_img(self, img, p_depth = None):
        self.last_color_img = img
        encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),90]
        result, img_encode = cv2.imencode('.jpg',img,encode_param)
        data = numpy.array(img_encode)
        stringData = data.tostring()
        self.send_str(stringData)
        if(type(p_depth) == numpy.ndarray):
            self.last_depth_img = p_depth
            result, img_encode = cv2.imencode(".png", p_depth)
            data = numpy.array(img_encode)
            #data = p_depth
            stringData = data.tostring()
            self.send_str(stringData)


    def send_img_with_index(self, index, p_color, p_depth = None):
        #print(str(index))
        self.send_str(str(index).encode())  #카메라 번호
        if(type(p_depth) == numpy.ndarray):
            self.send_str(str(2).encode())  #이미지 갯수
            self.send_str(str("type:color_img,dtype:%s"%p_color.dtype).encode())
            self.send_str(str("type:depth_img,dtype:%s"%p_depth.dtype).encode())
            #cv2.imwrite('/home/doosan/Documents/depth.png', p_depth)
        else:
            self.send_str(str(1).encode())
            self.send_str(str("type:color_img,dtype:%s"%p_color.dtype).encode())
        self.send_img(p_color, p_depth)

    def send_detect_module(self,command_type, index, p_color, p_depth = None):
        self.send_str(str(command_type).encode())  # 물체 인식을 하기위한
        if(command_type == mode_OD):
            self.send_img_with_index(index, p_color, p_depth = p_depth)
        if(command_type == mode_PE):
            self.send_str(str(index).encode())

    def get_curImg(self):
        return self.last_color_img

    def get_CurImgs(self):
        return self.last_color_img, self.last_depth_img

    def disconnection(self):
        if (self.m_Socket_SOCKET == []):
            return
        try:
            self.m_Socket_CLIENT_SOCKET.sendall(str(0).ljust(16).encode())
            print('종료 신호 전송')
        except:
            pass
        try:
            self.m_Socket_SOCKET.close()
            self.m_Socket_CLIENT_SOCKET.close()
        except:
            print('소멸하는 과정에서 에러 발생.')
            pass
        if(self.TCP_Config_IMMORTAL== True):
            print('TCP is doomed, but will be return!')
            self.tcp_Init(TCP_IP = self.TCP_Config_IP,TCP_PORT=self.TCP_Config_PORT, mode=self.TCP_Config_CONNECT_TYPE, socket_type=self.TCP_Config_Socket_Type, TCP_IMMORTAL = True, callback_func = self.m_callback_func)
            return
        else:
            print('TCP is doomed')
            self.m_State_CONNECTION = '[state]doomed'

    def __del__(self):
        print('소멸자 가동')
        self.disconnection()



if __name__ == '__main__':
    test_port = 4383

    #TEST_struct = struct_tcp()
    #TEST_DATA = TEST_struct.OD_SEND
    def client_test():
        Module_TCP2 = ClientTest()
        Module_TCP2.tcp_Init(TCP_IP='127.0.0.1', TCP_PORT=test_port, mode = 'client', socket_type = 'TCP')
        Module_TCP2.send_json({"ORDER":"ETC","payload":"123"})
        time.sleep(3)
        Module_TCP2.send_json({"ORDER":"ETC","payload":"123"})
        time.sleep(3)
        Module_TCP2.send_json({"ORDER":"ETC","payload":"123"})
        # Module_TCP2.send_json({"payload": "123"})
        #Module_TCP2.send_json([1,2,3])
        #Module_TCP2.send_result(TEST_DATA)

    thread= threading.Thread(target=client_test())
    thread.daemon = True
    thread.start()
    time.sleep(1)
    #thread2= threading.Thread(target=client_test)
    #thread2.daemon = True
    #thread2.start()

    thread.join()
    #thread2.join()

    #temp_str = Module_TCP.recv_data()
    #print(temp_str)

    #json_data = json.loads(str(temp_str))
    #print(json_data["Wow~"])
    #time.sleep(10)