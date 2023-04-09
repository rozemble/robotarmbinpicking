import socket
from multiprocessing import Process, Queue
import threading
import numpy as np
import json
import time
import sys
from datetime import datetime

#json 덤프용
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):

            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class ClientSocket(Process):
    TCP_Config_Socket_Type = "TCP"

    def __init__(self, algoStatus,connStatus,taskQ,sendQ):
        Process.__init__(self)
        self.algoStatus = algoStatus
        self.connStatus = connStatus
        self.ip = "192.168.0.13"
        self.port = 4383
        self.mode = "client"
        self.taskQ = taskQ
        self.sendQ = sendQ

    def tcp_Init(self, TCP_IP='127.0.0.1', TCP_PORT=4383, mode='', socket_type='TCP', state_thread=True, visual=True,
                 TCP_IMMORTAL=False):
        self.TCP_Config_CONNECT_TYPE = mode
        self.TCP_Config_IP = TCP_IP
        self.TCP_Config_PORT = TCP_PORT
        self.TCP_Config_Socket_Type = socket_type
        self.TCP_Config_IMMORTAL = TCP_IMMORTAL
        if (mode == ''):
            print('서버 또는 클라이언트를 명시하세요 (ex. mode="server"')
            return False
        if (mode == 'server'):
            self.server_Init(TCP_PORT=TCP_PORT, socket_type=socket_type)
            #self.recv()
        elif (mode == 'client'):
            self.client_Init(TCP_IP=TCP_IP, TCP_PORT=TCP_PORT, socket_type=socket_type)

    def run(self):
        self.tcp_Init(TCP_IP= self.ip, TCP_PORT=self.port, mode=self.mode)

        temp_thread = threading.Thread(target=self.recv)
        temp_thread.daemon = False
        temp_thread.start()
        self.send()

    # 클라이언트로서 연결을 합니다.
    def client_Init(self, TCP_IP='127.0.0.1', TCP_PORT=4382, socket_type='TCP'):
        self.TCP_Config_IP = TCP_IP
        self.TCP_Config_PORT = TCP_PORT
        print("HERE:", self.TCP_Config_IP, self.TCP_Config_PORT)
        if (socket_type == 'TCP'):
            print("[TCP MODULE] OPEN PORT as CLIENT(%s,%d)" % (self.TCP_Config_IP,self.TCP_Config_PORT))
            self.connStatus["status"] = "OPEN"
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            trial_count = 1
            while True:
                try:
                    self.m_Socket_SOCKET.connect((self.TCP_Config_IP, self.TCP_Config_PORT))
                    break
                except:
                    trial_count += 1
                    time.sleep(1)
                    print("Retry to connect - ", trial_count)

            print("[TCP MODULE] CLIENT CONNECTED 2 (%s:%s)" % (str(self.TCP_Config_IP), str(self.TCP_Config_PORT)))
            self.connStatus["status"] = "INIT"
        else:
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.m_Socket_CLIENT_SOCKET = self.m_Socket_SOCKET
        self.m_State_CONNECTION = '[state]ready'
        print('client_ready')

    def server_Init(self, TCP_PORT=4382, socket_type='TCP'):
        self.TCP_Config_IP = ""  # 서버니깐 IP는 없어요요
        self.TCP_Config_PORT = TCP_PORT
        if (socket_type == 'TCP'):
            print("[TCP MODULE] OPEN PORT as SERVER(%d)" % self.TCP_Config_PORT)
            self.connStatus["status"] = "OPEN"
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            while (True):
                try:
                    self.m_Socket_SOCKET.bind((self.TCP_Config_IP, self.TCP_Config_PORT))
                    break
                except Exception as ex:
                    self.TCP_Config_PORT = self.TCP_Config_PORT + 1
                    print('bind 에러', ex)  # ex는 발생한 에러의 이름을 받아오는 변수
                    pass
            self.m_Socket_SOCKET.listen(999)  # 10초 만큼 연결 대기.
            self.m_Socket_CLIENT_SOCKET, self.m_Socket_TARGET_ADDR = self.m_Socket_SOCKET.accept()
            print("[TCP MODULE] SERVER CONNECTED 2 (%s:%s)" % (str(self.m_Socket_TARGET_ADDR),str(self.TCP_Config_PORT)))
            self.connStatus["status"] = "INIT"
        else:
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.m_Socket_SOCKET.bind((self.TCP_Config_IP, self.TCP_Config_PORT))
            self.m_Socket_CLIENT_SOCKET = self.m_Socket_SOCKET
            print("[UDP MODULE] SERVER READY")

        self.m_State_CONNECTION = '[state]ready'

    def start_community(self):
        for i in range(5):
            if (self.m_State_CONNECTION == '[state]ready'):
                break
            print('아직 통신할 준비가 안됐다.')
            print('1초 후에 다시 확인')

        # self.thread_tcp2 = threading.Thread(target=self.send_queue_thread)
        # self.thread_tcp2.daemon = False
        # self.thread_tcp2.start()
        self.thread_tcp = threading.Thread(target=self.recv_community)
        self.thread_tcp.daemon = False
        self.thread_tcp.start()

    def recv_data(self):
        self.m_Socket_SOCKET.settimeout(999)
        try:

            recv_data = self.recvall(16)
            if not recv_data: return False
            length = int(recv_data)  # 길이 16의 데이터를 먼저 수신하는 것은 여기에 이미지의 길이를 먼저 받아서 이미지를 받을 때 편리하려고 하는 것이다.

            if (length == 0):
                print('[TCP] %s(%s)에 의한 연결 종료' % (str(self.TCP_Config_IP), str(self.TCP_Config_PORT)))
                return False
            stringData = self.recvall(length)


            self.m_state_Time = int(round(time.time() * 1000))
            return stringData
        except Exception as ex:
            print('[TCP] RECV하는 과정에서 에러 발생. %s(%s)' % (str(self.TCP_Config_IP), str(self.TCP_Config_PORT)))
            print(ex)
            return False

    def recv_json(self):
        try:
            l_recv_data = self.recv_data()
            str_data = []
            if not l_recv_data: return False
            str_data = str(l_recv_data, encoding='UTF-8')
            # if (self.python_version > 2):
            #     #str_data = str(l_recv_data, encoding='UTF-8')
            #     str_data = str(l_recv_data)
            # else:
            #     str_data = str(l_recv_data).encode('UTF-8')  # ).replace("'","\"")
            self.m_recv_json = json.loads(str_data)
            # print(type(self.m_recv_json))
            return self.m_recv_json
        except Exception as ex:
            print('[TCP ERROR]recv error?', sys.exc_info())
            print(ex)
            #print(l_recv_data)
            return False

    def wait_for_recv(self):
        print(self.m_State_CONNECTION)
        # while (self.m_State_CONNECTION != '[state]ready_to_return'):
        #     time.sleep(0.1)
        #     print("1")
        #     if (self.m_State_CONNECTION == '[state]doomed'):
        #         return

        while not self.recv_json():
            #print("?")
            time.sleep(0.1)
        self.m_State_CONNECTION = '[state]ready'
        return self.m_recv_json

    def send(self):
        while True:
            if not self.sendQ.empty():
                send_data = self.sendQ.get()
                #print("SENDDATA:",send_data)
                self.send_json(send_data)
            # if self.algoStatus["SEND"]:
            #     self.algoStatus["SEND"] = False
            #     self.send_json()

    def send_json(self, inf):
        l_data = json.dumps(inf, cls=NumpyEncoder)
        # 이미지를 보내기에는 아래 코드는 느리다.
        # l_data = json.dumps(inf, ensure_ascii=False, cls=NumpyEncoder, sort_keys=True, indent=4)
        l_send_size = len(l_data)
        l_send_data = l_data.encode()
        try:
            if (self.TCP_Config_Socket_Type == 'TCP'):
                self.m_Socket_CLIENT_SOCKET.sendall(str(l_send_size).ljust(16).encode())
                self.m_Socket_CLIENT_SOCKET.sendall(l_send_data)
        except:
            print('[TCP ERROR] SEND')
            return

    # def recv_data(self):
    #     self.m_Socket_SOCKET.settimeout(999)
    #     try:
    #         length = int(self.recvall(16))  # 길이 16의 데이터를 먼저 수신하는 것은 여기에 이미지의 길이를 먼저 받아서 이미지를 받을 때 편리하려고 하는 것이다.
    #         if (length == 0):
    #             print('[TCP] %s(%s)에 의한 연결 종료' % (str(self.TCP_Config_IP, self.TCP_Config_PORT)))
    #             return False
    #         stringData = self.recvall(length)
    #         return stringData
    #     except Exception as ex:
    #         print('[TCP] RECV하는 과정에서 에러 발생. %s(%s)' % (str(self.TCP_Config_IP, self.TCP_Config_PORT)))
    #         print(ex)
    #         return False

    def recvall(self, count):
        buf = b''
        while count:
            if (self.TCP_Config_Socket_Type == 'TCP'):
                newbuf = self.m_Socket_CLIENT_SOCKET.recv(count)
            # print('im %s : %s'%(self.TCP_Config_CONNECT_TYPE,newbuf))
            # newbuf = self.m_Socket_CLIENT_SOCKET.recv(count)
            if not newbuf or newbuf == b'': return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    # def recv(self):
    #     event_str = 'normal'
    #     def thread_recv():
    #         while (True):
    #             try:
    #                 recv_data = self.wait_for_recv()
    #                 self.algoStatus["status"] = recv_data["ORDER"]
    #                 #self.queue_order.append(recv_data)
    #             except:
    #                 return
    #
    #     temp_thread = threading.Thread(target=thread_recv)
    #     temp_thread.daemon = False
    #     temp_thread.start()
    #

    def recv(self):
        while (True):
            try:
                print("WAIT")
                recv_data = self.wait_for_recv()
                #self.algoStatus["status"] = recv_data["ORDER"]
                #print("RECVDATA:"+str(recv_data))
                if not self.taskQ.full():
                    self.taskQ.put(recv_data)
                self.send_json(recv_data)
                #self.send_json(recv_data)
                #self.queue_order.append(recv_data)
            except Exception as ex:
                print('[TCP ERROR]recv error??', sys.exc_info())
                print(ex)
                return

class ServerSocket(Process):
    TCP_Config_Socket_Type = "TCP"
    def __init__(self, algoStatus,connStatus,taskQ,sendQ):
        Process.__init__(self)
        self.algoStatus = algoStatus
        self.connStatus = connStatus
        self.port = 4383
        self.mode = "server"
        self.taskQ = taskQ
        self.sendQ = sendQ

    def tcp_Init(self, TCP_IP='127.0.0.1', TCP_PORT=4382, mode='', socket_type='TCP', state_thread=True, visual=True,
                 TCP_IMMORTAL=False):
        self.TCP_Config_CONNECT_TYPE = mode
        self.TCP_Config_IP = TCP_IP
        self.TCP_Config_PORT = TCP_PORT
        self.TCP_Config_Socket_Type = socket_type
        self.TCP_Config_IMMORTAL = TCP_IMMORTAL
        if (mode == ''):
            print('서버 또는 클라이언트를 명시하세요 (ex. mode="server"')
            return False
        if (mode == 'server'):
            self.server_Init(TCP_PORT=TCP_PORT, socket_type=socket_type)
            #self.recv()
        # elif (mode == 'client'):
        #     self.client_Init(TCP_IP=TCP_IP, TCP_PORT=TCP_PORT, socket_type=socket_type)

    def run(self):
        self.tcp_Init(TCP_PORT=self.port, mode=self.mode)

        temp_thread = threading.Thread(target=self.recv)
        temp_thread.daemon = False
        temp_thread.start()
        self.send()

    def server_Init(self, TCP_PORT=4382, socket_type='TCP'):
        self.TCP_Config_IP = ""  # 서버니깐 IP는 없어요요
        self.TCP_Config_PORT = TCP_PORT
        if (socket_type == 'TCP'):
            print("[TCP MODULE] OPEN PORT as SERVER(%d)" % self.TCP_Config_PORT)
            self.connStatus["status"] = "OPEN"
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            while (True):
                try:
                    self.m_Socket_SOCKET.bind((self.TCP_Config_IP, self.TCP_Config_PORT))
                    break
                except Exception as ex:
                    self.TCP_Config_PORT = self.TCP_Config_PORT + 1
                    print('bind 에러', ex)  # ex는 발생한 에러의 이름을 받아오는 변수
                    pass
            self.m_Socket_SOCKET.listen(999)  # 10초 만큼 연결 대기.
            self.m_Socket_CLIENT_SOCKET, self.m_Socket_TARGET_ADDR = self.m_Socket_SOCKET.accept()
            print("[TCP MODULE] SERVER CONNECTED 2 (%s:%s)" % (str(self.m_Socket_TARGET_ADDR),str(self.TCP_Config_PORT)))
            self.connStatus["status"] = "INIT"
        else:
            self.m_Socket_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.m_Socket_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.m_Socket_SOCKET.bind((self.TCP_Config_IP, self.TCP_Config_PORT))
            self.m_Socket_CLIENT_SOCKET = self.m_Socket_SOCKET
            print("[UDP MODULE] SERVER READY")

        self.m_State_CONNECTION = '[state]ready'

    def recv_data(self):
        self.m_Socket_SOCKET.settimeout(999)
        try:

            recv_data = self.recvall(16)
            if not recv_data: return False
            length = int(recv_data)  # 길이 16의 데이터를 먼저 수신하는 것은 여기에 이미지의 길이를 먼저 받아서 이미지를 받을 때 편리하려고 하는 것이다.

            if (length == 0):
                print('[TCP] %s(%s)에 의한 연결 종료' % (str(self.TCP_Config_IP), str(self.TCP_Config_PORT)))
                return False
            stringData = self.recvall(length)


            self.m_state_Time = int(round(time.time() * 1000))
            return stringData
        except Exception as ex:
            print('[TCP] RECV하는 과정에서 에러 발생. %s(%s)' % (str(self.TCP_Config_IP), str(self.TCP_Config_PORT)))
            print(ex)
            return False

    def recv_json(self):
        try:
            l_recv_data = self.recv_data()
            str_data = []
            if not l_recv_data: return False
            str_data = str(l_recv_data, encoding='UTF-8')
            # if (self.python_version > 2):
            #     #str_data = str(l_recv_data, encoding='UTF-8')
            #     str_data = str(l_recv_data)
            # else:
            #     str_data = str(l_recv_data).encode('UTF-8')  # ).replace("'","\"")
            self.m_recv_json = json.loads(str_data)
            # print(type(self.m_recv_json))
            return self.m_recv_json
        except Exception as ex:
            print('[TCP ERROR]recv error?', sys.exc_info())
            print(ex)
            #print(l_recv_data)
            return False

    def wait_for_recv(self):
        print(self.m_State_CONNECTION)
        # while (self.m_State_CONNECTION != '[state]ready_to_return'):
        #     time.sleep(0.1)
        #     print("1")
        #     if (self.m_State_CONNECTION == '[state]doomed'):
        #         return

        while not self.recv_json():
            #print("?")
            time.sleep(0.1)
        self.m_State_CONNECTION = '[state]ready'
        return self.m_recv_json

    def send(self):
        while True:
            if not self.sendQ.empty():
                send_data = self.sendQ.get()
                print("SENDDATA:",send_data)
                self.send_json(send_data)
            # if self.algoStatus["SEND"]:
            #     self.algoStatus["SEND"] = False
            #     self.send_json()

    def send_json(self, inf):
        l_data = json.dumps(inf, cls=NumpyEncoder)
        # 이미지를 보내기에는 아래 코드는 느리다.
        # l_data = json.dumps(inf, ensure_ascii=False, cls=NumpyEncoder, sort_keys=True, indent=4)
        l_send_size = len(l_data)
        l_send_data = l_data.encode()
        try:
            if (self.TCP_Config_Socket_Type == 'TCP'):
                self.m_Socket_CLIENT_SOCKET.sendall(str(l_send_size).ljust(16).encode())
                self.m_Socket_CLIENT_SOCKET.sendall(l_send_data)
        except:
            print('[TCP ERROR] SEND')
            return

    # def recv_data(self):
    #     self.m_Socket_SOCKET.settimeout(999)
    #     try:
    #         length = int(self.recvall(16))  # 길이 16의 데이터를 먼저 수신하는 것은 여기에 이미지의 길이를 먼저 받아서 이미지를 받을 때 편리하려고 하는 것이다.
    #         if (length == 0):
    #             print('[TCP] %s(%s)에 의한 연결 종료' % (str(self.TCP_Config_IP, self.TCP_Config_PORT)))
    #             return False
    #         stringData = self.recvall(length)
    #         return stringData
    #     except Exception as ex:
    #         print('[TCP] RECV하는 과정에서 에러 발생. %s(%s)' % (str(self.TCP_Config_IP, self.TCP_Config_PORT)))
    #         print(ex)
    #         return False

    def recvall(self, count):
        buf = b''
        while count:
            if (self.TCP_Config_Socket_Type == 'TCP'):
                newbuf = self.m_Socket_CLIENT_SOCKET.recv(count)
            # print('im %s : %s'%(self.TCP_Config_CONNECT_TYPE,newbuf))
            # newbuf = self.m_Socket_CLIENT_SOCKET.recv(count)
            if not newbuf or newbuf == b'': return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    # def recv(self):
    #     event_str = 'normal'
    #     def thread_recv():
    #         while (True):
    #             try:
    #                 recv_data = self.wait_for_recv()
    #                 self.algoStatus["status"] = recv_data["ORDER"]
    #                 #self.queue_order.append(recv_data)
    #             except:
    #                 return
    #
    #     temp_thread = threading.Thread(target=thread_recv)
    #     temp_thread.daemon = False
    #     temp_thread.start()
    #

    def recv(self):
        while (True):
            try:
                print("WAIT")
                recv_data = self.wait_for_recv()
                #self.algoStatus["status"] = recv_data["ORDER"]
                print(recv_data)
                if not self.taskQ.full():
                    self.taskQ.put(recv_data)
                self.send_json(recv_data)
                #self.send_json(recv_data)
                #self.queue_order.append(recv_data)
            except Exception as ex:
                print('[TCP ERROR]recv error??', sys.exc_info())
                print(ex)
                return

