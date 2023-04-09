'''
Michael Dawson-Haggerty

abb.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code module SERVER)


For functions which require targets (XYZ positions with quaternion orientation),
targets can be passed as [[XYZ], [Quats]] OR [XYZ, Quats]

'''

import socket
import time
from datetime import datetime
import inspect
import threading
from collections import deque
import logging

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())


class Robot:
    # def init(self, ip='192.168.101.125', port_motion=5000, port_logger=5001):
    def init(self, ip='192.168.125.1', port_motion=5000, port_logger=5001):
        self.delay = .08
        self.connected = False

        self.connect_motion((ip, port_motion))

        self.set_units('millimeters', 'degrees')
        self.set_tool()
        self.set_workobject()
        self.set_speed(speed_value=100)
        self.set_zone(zone_key='z0', point_motion=True)
        self.set_acceleration([10, 10])
        #
        # self.logger_addr = (ip, port_logger)
        # self.logger_connected = False
        #
        # self.pose = [0, 0, 0, 0, 0, 0, 0]
        # self.joint = [0, 0, 0, 0, 0, 0]
        #
        # now = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        # self.logging_file = open('abb_logging/' + str(now) + '_logging.csv', 'w')
        # self.t_logging = threading.Thread(target=self.connect_logger)
        # self.t_logging.setDaemon(True)
        # self.t_logging.start()

    def connect_motion(self, remote):
        log.info('Attempting to connect to robot motion server at %s', str(remote))
        self.sock_motion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_motion.settimeout(None)
        try:
            self.sock_motion.connect(remote)
            # self.sock_motion.settimeout(2.5)
            log.info('Connected to robot motion server at %s', str(remote))
            self.connected = True
        except socket.error:
            log.info('Disconnected to robot motion server at %s', str(remote))
            self.connected = False

    def connect_logger(self):
        log.info('Attempting to connect to robot logging server at %s', str(self.logger_addr))
        self.sock_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_logger.settimeout(None)
        try:
            self.sock_logger.connect(self.logger_addr)
            # self.sock_logger.settimeout(None)
            self.logger_connected = True
            log.info('Connected to robot logging server at %s', str(self.logger_addr))
        except socket.error:
            log.info('Disconnected to robot logging server at %s', str(self.logger_addr))
            self.logger_connected = False

        now = datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%3s")
        while True:
            time.sleep(0.1)
            if self.logger_connected is True:
                try:
                    data = self.sock_logger.recv(4096)
                    data_split = data.split()
                    if data_split[0] == '#' and data_split[1] == '0':
                        for i in range(0, 7):
                            self.pose[i] = data_split[i+5]
                        log.info('Pose : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f',
                                 float(data_split[5]), float(data_split[6]), float(data_split[7]), float(data_split[8]), float(data_split[9]), float(data_split[10]), float(data_split[11]))
                        self.logging_file.write('[' + str(now) + '] Pose:,' + data_split[5] + ',' + data_split[6] + ',' + data_split[7] + ',' + data_split[8] + ',' + data_split[9] + ',' + data_split[10] + ',' + data_split[11] + '\n')
                    elif data_split[0] == '#' and data_split[1] == '1':
                        for i in range(0, 6):
                            self.joint[i] = data_split[i+5]
                        log.info('Joint : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f',
                                 float(data_split[5]), float(data_split[6]), float(data_split[7]), float(data_split[8]), float(data_split[9]), float(data_split[10]))
                        self.logging_file.write('[' + str(now) + '] Joint:' + data_split[5] + ',' + data_split[6] + ',' + data_split[7] + ',' + data_split[8] + ',' + data_split[9] + ',' + data_split[10] + '\n')
                except:
                    # log.warn('Logger socket error %s ', str(msg))
                    self.logger_connected = False
                    # self.logging_file.close()

    def set_units(self, linear, angular):
        units_l = {'millimeters': 1.0, 'meters': 1000.0, 'inches': 25.4}
        units_a = {'degrees': 1.0, 'radians': 57.2957795}
        self.scale_linear = units_l[linear]
        self.scale_angle  = units_a[angular]

    def set_cartesian(self, pose, syncType=1, acc=[10, 10]):
        '''
        Executes a move immediately from the current pose,
        to 'pose', with units of millimeters.
        '''
        msg = "01 "
        # msg += format(acc[0], "+08.1f") + " "
        # msg += format(acc[1], "+08.1f") + " "
        msg += self.format_pose(pose)
        return self.send(msg)

    def set_joints(self, joints, syncType=1):
        '''
        Executes a move immediately, from current joint angles,
        to 'joints', in degrees. 
        '''
        if len(joints) != 6: return False
        msg = "02 "
        for joint in joints: msg += format(joint*self.scale_angle, "+08.2f") + " " 
        msg += "#" 
        return self.send(msg)

    def get_cartesian(self):
        '''
        Returns the current pose of the robot, in millimeters
        '''
        msg = "03 #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return r[2:9]

    def get_joints(self):
        '''
        Returns the current angles of the robots joints, in degrees. 
        '''
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:8]]

    def get_external_axis(self):
        '''
        If you have an external axis connected to your robot controller
        (such as a FlexLifter 600, google it), this returns the joint angles
        '''
        msg = "05 #"
        data = self.send(msg).split()
        return [float(s) for s in data[2:8]]
       
    def get_robotinfo(self):
        '''
        Returns a robot- unique string, with things such as the
        robot's model number. 
        Example output from and IRB 2400:
        ['24-53243', 'ROBOTWARE_5.12.1021.01', '2400/16 Type B']
        '''
        msg = "98 #"
        data = str(self.send(msg))[5:].split('*')
        log.debug('get_robotinfo result: %s', str(data))
        return data

    def set_tool(self, tool=[0,0,0,1,0,0,0]):
        '''
        Sets the tool centerpoint (TCP) of the robot. 
        When you command a cartesian move, 
        it aligns the TCP frame with the requested frame.
        
        Offsets are from tool0, which is defined at the intersection of the
        tool flange center axis and the flange face.
        '''
        msg = "06 " + self.format_pose(tool)
        self.send(msg)
        self.tool = tool

    # def load_json_tool(self, file_obj):
    #     if file_obj.__class__.__name__ == 'str':
    #         file_obj = open(filename, 'rb');
    #     tool = self.check_coordinates(json.load(file_obj))
    #     self.set_tool(tool)
        
    def get_tool(self): 
        log.debug('get_tool returning: %s', str(self.tool))
        return self.tool

    def set_workobject(self, work_obj=[0,0,0,1,0,0,0]):
        '''
        The workobject is a local coordinate frame you can define on the robot,
        then subsequent cartesian moves will be in this coordinate frame. 
        '''
        msg = "07 " + self.format_pose(work_obj)   
        self.send(msg)

    def set_speed(self, speed_value = 10, manual_speed=[]):
        '''
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        '''

        if len(manual_speed) == 4:
            speed = manual_speed
        else:
            speed = [speed_value, 500, 5000, 1000]

        if len(speed) != 4: return False
        msg = "08 "
        msg += format(speed[0], "+08.1f") + " "
        msg += format(speed[1], "+08.2f") + " "
        msg += format(speed[2], "+08.1f") + " "
        msg += format(speed[3], "+08.2f") + " #"
        self.send(msg)

    def set_acceleration(self, acc=[10, 10]):
        msg = "10 "
        msg += format(acc[0], "+08.1f") + " "
        msg += format(acc[1], "+08.1f") + " #"
        self.send(msg)

    def set_zone(self, zone_key = 'z0', point_motion = False, manual_zone  = []):
        zone_dict = {
            'z0': [0.3, 0.3, 0.3, 0.03, 0.3, 0.03],
            'z1': [1, 1, 1, 0.1, 1, 0.1],
            'z5': [5, 8, 8, 0.8, 8, 0.8],
            'z10': [10, 15, 15, 1.5, 15, 1.5],
            'z15': [15, 23, 23, 2.3, 23, 2.3],
            'z20': [20, 30, 30, 3, 30, 3],
            'z30': [30, 45, 45, 4.5, 45, 4.5],
            'z40': [40, 60, 60, 6, 60, 6],
            'z50': [50, 75, 75, 7.5, 75, 7.5],
            'z60': [60, 90, 90, 9, 90, 9],
            'z80': [80, 120, 120, 12, 120, 12],
            'z100': [100, 150, 150, 15, 150, 15],
            'z150': [150, 225, 225, 23, 225, 23],
            'z200': [200, 300, 300, 30, 300, 30]
        }
        '''
        Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C
        
        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: mm, radius from goal where robot tool centerpoint 
                   is not rigidly constrained
        pzone_ori: mm, radius from goal where robot tool orientation 
                   is not rigidly constrained
        zone_ori: degrees, zone size for the tool reorientation
        '''

        if point_motion: 
            zone = [0, 0, 0, 0, 0, 0]
        elif len(manual_zone) == 3: 
            zone = manual_zone
        elif zone_key in zone_dict.keys(): 
            zone = zone_dict[zone_key]
        else: return False
        
        msg = "09 " 
        msg += str(int(point_motion)) + " "
        msg += format(zone[0], "+08.4f") + " " 
        msg += format(zone[1], "+08.4f") + " " 
        msg += format(zone[2], "+08.4f") + " "
        msg += format(zone[3], "+08.4f") + " "
        msg += format(zone[4], "+08.4f") + " "
        msg += format(zone[5], "+08.4f") + " #"
        self.send(msg)

    def buffer_add_pose(self, pose):
        '''
        Appends single pose to the remote buffer
        Move will execute at current speed (which you can change between buffer_add calls)
        '''
        msg = "30 " + self.format_pose(pose)
        self.send(msg)

    def buffer_set_pose(self, pose_list):
        '''
        Adds every pose in pose_list to the remote buffer
        '''
        self.clear_buffer_pose()
        for pose in pose_list:
            self.buffer_add_pose(pose)
        if self.buffer_len_pose() == len(pose_list):
            log.debug('Successfully added %i poses to remote buffer', len(pose_list))
            return True
        else:
            log.warn('Failed to add poses to remote buffer!')
            self.clear_buffer_pose()

        return False

    def clear_buffer_pose(self):
        msg = "31 #"
        data = self.send(msg)
        if self.buffer_len_pose() != 0:
            log.warn('clear_buffer failed! buffer_len: %i', self.buffer_len_pose())
            raise NameError('clear_buffer failed!')
        return data

    def buffer_len_pose(self):
        '''
        Returns the length (number of poses stored) of the remote buffer
        '''
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_execute_pose(self, syncType=1):
        '''
        Immediately execute linear moves to every pose in the remote buffer.
        '''
        msg = "33 #"
        return self.send(msg)

    def buffer_add_joint(self, joint):
        msg = "37 " + self.format_joint(joint)
        self.send(msg)

    def buffer_set_joint(self, joint_list):
        self.clear_buffer_joint()
        for joint in joint_list:
            self.buffer_add_joint(joint)
        if self.buffer_len_joint() == len(joint_list):
            log.debug('Successfully added %i joints to remote buffer', len(joint_list))
            return True
        else:
            log.warn('Failed to add joints to remote buffer!')
            self.clear_buffer_joint()

        return False

    def clear_buffer_joint(self):
        msg = "38 #"
        data = self.send(msg, True)
        if self.buffer_len_joint() != 0:
            log.warn('clear_buffer failed! buffer_len: %i', self.buffer_len_joint())
            raise NameError('clear_buffer failed!')
        return data

    def buffer_len_joint(self):
        '''
        Returns the length (number of poses stored) of the remote buffer
        '''
        msg = "39 #"
        data = self.send(msg, True).split()
        return int(float(data[2]))

    def buffer_execute_joint(self, syncType=1):
        msg = "40 #"
        return self.send(msg)

    # def set_external_axis(self, axis_unscaled=[-550,0,0,0,0,0]):
    #     if len(axis_values) != 6: return False`
    #     msg = "34 "
    #     for axis in axis_values:
    #         msg += format(axis, "+08.2f") + " "
    #     msg += "#"
    #     return self.send(msg)

    def move_circular(self, pose_onarc, pose_end):
        '''
        Executes a movement in a circular path from current position, 
        through pose_onarc, to pose_end
        '''
        msg_0 = "35 " + self.format_pose(pose_onarc)  
        msg_1 = "36 " + self.format_pose(pose_end)

        print("MSG0 :", msg_0)
        print("MSG1 :", msg_1)
        data = self.send(msg_0).split()
        print("CIRCLE DATA:", data)
        # if data[1] != '1':
        #     log.warn('move_circular incorrect response, bailing!')
        #     print("CIRCLE DATA:", data)
        #     print("ERROR")
        #     return False
        return self.send(msg_1)

    def set_dio(self, value, id=0):
        '''
        A function to set a physical DIO line on the robot.
        For this to work you're going to need to edit the RAPID function
        and fill in the DIO you want this to switch. 
        '''
        msg = '50 ' + str(id) + ' ' + str(value) + ' #'
        # return
        return self.send(msg)
        
    def send(self, message, wait_for_response=True):
        '''
        Send a formatted message to the robot socket.
        if wait_for_response, we wait for the response and return it
        '''
        caller = inspect.stack()[1][3]
        log.debug('%-14s sending: %s', caller, message)
        self.sock_motion.send(message.encode('utf-8'))
        time.sleep(self.delay)
        if not wait_for_response:
            return
        data = self.sock_motion.recv(4096)
        log.debug('%-14s recieved: %s', caller, data)
        return data
        
    def format_pose(self, pose):
        pose = self.check_coordinates(pose)
        msg  = ''
        # for cartesian in pose[0]:
        #     msg += format(cartesian * self.scale_linear,  "+08.1f") + " "
        # for quaternion in pose[1]:
        #     msg += format(quaternion, "+08.5f") + " "
        for i in range(0, 7):
            if i < 3:
                msg += format(pose[i] * self.scale_linear,  "+08.1f") + " "
            else:
                msg += format(pose[i], "+08.5f") + " "
        msg += "#"
        return msg

    def format_joint(self, joint):
        msg = ''
        for i in range(0, 6):
            msg += format(joint[i], "+08.2f") + ' '
        msg += '#'
        return msg
        
    def close(self):
        # self.send("99 #", False)
        self.sock_motion.shutdown(socket.SHUT_RDWR)
        self.sock_motion.close()
        self.connected = False

        self.logger_connected = False
        self.sock_logger.shutdown(socket.SHUT_RDWR)
        self.sock_logger.close()
        # self.logging_file.close()

        log.info('Disconnected from ABB robot.')

    # def stop(self):
        # self.send("97 #", False)

    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        self.close()

    def check_coordinates(self, coordinates):
        if len(coordinates) == 7:
            return coordinates
        elif len(coordinates) == 6:
            q1 = self.convertToQuaternion(coordinates[3:6])
            return [coordinates[0], coordinates[1], coordinates[2], q1[0], q1[1], q1[2], q1[3]]  # [749,-22,562,13,151,-175]
        else:
            log.warn('Recieved malformed coordinate: %s', str(coordinates))
            raise NameError('Malformed coordinate!')

if __name__ == '__main__':
    formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    handler_stream.setLevel(logging.DEBUG)
    log = logging.getLogger('abb')
    log.setLevel(logging.DEBUG)
    log.addHandler(handler_stream)

    robot = Robot()
    #robot.init(ip='192.168.101.125', port_motion=5000, port_logger=5001)
    robot.init(ip='192.168.125.1', port_motion=5000, port_logger=5001)

    robot.set_speed(100)
    import time
    while robot.connected is True:
        print(robot.get_cartesian())
        time.sleep(1)

    # while robot.connected is True:
    #     try:
    #         target_joint = [0, 0, 0, 0, 0, 0]
    #         robot.set_joints(target_joint)
    #         # break
    #
    #         target_joint[0] = 0
    #         target_joint[1] = -30
    #         target_joint[2] = 30
    #         target_joint[3] = 0
    #         target_joint[4] = 90
    #         target_joint[5] = 0
    #         robot.set_joints(target_joint)
    #         # break
    #
    #         pose = robot.get_cartesian()
    #
    #         pose[0] += 100
    #         pose[1] += 100
    #         pose[2] += 100
    #         robot.set_cartesian(pose=pose)
    #
    #         pose[2] -= 200
    #         robot.set_cartesian(pose=pose)
    #
    #         pose[1] -= 200
    #         robot.set_cartesian(pose=pose)
    #
    #         pose[2] += 200
    #         robot.set_cartesian(pose=pose)
    #
    #         pose[1] += 200
    #         robot.set_cartesian(pose=pose)
    #
    #         pose[1] -= 100
    #         pose[2] -= 100
    #         robot.set_cartesian(pose=pose)
    #
    #         # pose = robot.get_cartesian()
    #         # print 'current_pose 3 : %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f' % (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6])
    #         # break
    #     except KeyboardInterrupt:
    #         robot.logging_file.close()
    #         break

    robot.close()

