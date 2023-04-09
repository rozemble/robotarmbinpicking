import socket
from multiprocessing import Process, Queue
import threading
import numpy as np
import json
import time
import sys

#to serialize some array-like data for making json
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class ClientSocket(Process):
    TCP_Config_Socket_Type = "TCP"
    def __init__(self, algoStatus,connStatus,taskQ,sendQ, server_ip = "192.168.0.13",server_port = 4383):
        Process.__init__(self)
        self.algoStatus = algoStatus
        self.connStatus = connStatus
        self.ip = server_ip
        self.port = server_port
        self.mode = "client"
        self.taskQ = taskQ
        self.sendQ = sendQ

    def tcp_Init(self, TCP_IP='127.0.0.1', TCP_PORT=4383, mode='', socket_type='TCP', TCP_IMMORTAL=False):
        self.TCP_Config_CONNECT_TYPE = mode
        self.TCP_Config_IP = TCP_IP
        self.TCP_Config_PORT = TCP_PORT
        self.TCP_Config_Socket_Type = socket_type
        self.TCP_Config_IMMORTAL = TCP_IMMORTAL

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

    def start_community(self):
        for i in range(5):
            if (self.m_State_CONNECTION == '[state]ready'):
                break
            print('아직 통신할 준비가 안됐다.')
            print('1초 후에 다시 확인')

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
            if not l_recv_data: return False
            str_data = str(l_recv_data, encoding='UTF-8')
            self.m_recv_json = json.loads(str_data)
            return self.m_recv_json
        except Exception as ex:
            print('[TCP ERROR]recv error?', sys.exc_info())
            print(ex)
            return False

    def wait_for_recv(self):
        print(self.m_State_CONNECTION)
        while not self.recv_json():
            time.sleep(0.1)
        self.m_State_CONNECTION = '[state]ready'
        return self.m_recv_json

    def send(self):
        while True:
            if not self.sendQ.empty():
                send_data = self.sendQ.get()
                #print("SENDDATA:",send_data)
                self.send_json(send_data)

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

    def recv(self):
        while (True):
            try:
                print("WAIT")
                recv_data = self.wait_for_recv()
                if not self.taskQ.full():
                    self.taskQ.put(recv_data)
                self.send_json(recv_data)
            except Exception as ex:
                print('[TCP ERROR]recv error??', sys.exc_info())
                print(ex)
                return