import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import sys, signal
import time
import threading
from rclpy.qos import ReliabilityPolicy, QoSProfile
from mavros_msgs.msg import Altitude, State, PositionTarget, GlobalPositionTarget 
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Vector3, TwistStamped, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from transforms3d import euler
import numpy as np
import cmath
# import math
import struct
from enum import Enum
import binascii

import xbee_v5 as xbee
from tutorial_interfaces.srv import DroneStatus, DroneMissionPath, ReturnAndDetectInformation
from tutorial_interfaces.msg import Img, Motor

speed_count = []
altitude_count = []
latitude_count = []
longitude_count = []
yaw_delta_count = []

speed_count_str = []
altitude_count_str = []
latitude_count_str = []
longitude_count_str = []
yaw_delta_count_str = [] 

point_count = 0
flag = 0

class DroneSubscribeNode(Node):
    def __init__(self):
        super().__init__('drone_xbee_subscriber')
        # yaw,roll,pitch / angu_x,angu_y,angu_z / acc_x,acc_y,acc_z
        self.imuSub = self.create_subscription(Imu, 'mavros/imu/data', self.IMUcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) 
        # heading 航向
        self.headingSub = self.create_subscription(Float64, 'mavros/global_position/compass_hdg', self.HDcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # volocity 
        self.velocitySub = self.create_subscription(TwistStamped, 'mavros/local_position/velocity_body', self.VELcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # gps_altitude / latitude / longitude
        self.GlobalPositionSub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.GPcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # motor pitch and yaw degree 
        self.motorSub = self.create_subscription(Motor, 'motor', self.MOTORcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Image fps and detect
        self.imgSub = self.create_subscription(Img, 'img', self.IMGcb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angu_x = 0.0
        self.angu_y = 0.0
        self.angu_z = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.velocity = 0.0
        self.gps_altitude = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0 

        self.motor_pitch = 0.0
        self.motor_yaw = 0.0
        self.fps = 0.0
        self.detect_status = False
        self.target_center_status = False

    def MOTORcb(self, Motor):
        self.motor_pitch = Motor.pitch
        self.motor_yaw = Motor.yaw

    def IMGcb(self, Img):
        self.fps = Img.fps
        self.detect_status = Img.detect_status
        self.target_center_status = Img.target_center_status

    def IMUcb(self, msg :Imu):
        ned_euler_data = euler.quat2euler([msg.orientation.w,
                                        msg.orientation.x,
                                        msg.orientation.y,
                                        msg.orientation.z]) 
        
        self.pitch = xbee.radian_conv_degree(ned_euler_data[0])
        self.roll = xbee.radian_conv_degree(ned_euler_data[1])
        self.yaw = xbee.radian_conv_degree(ned_euler_data[2])

        #print(self.yaw)
    
        self.angu_x = msg.angular_velocity.x * 57.3
        self.angu_y = msg.angular_velocity.y * 57.3
        self.angu_z = msg.angular_velocity.z * 57.3
        
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z  
        
    
    def HDcb(self, msg):    # 航向
        self.heading = msg.data
        
    def VELcb(self, msg):   # 機體x軸的速度
        self.velocity = msg.twist.linear.x 

    def GPcb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_altitude = msg.altitude    # Positive is above the WGS 84 ellipsoid
              
class DroneTimerTaskNode(Node):
    def __init__(self):
        super().__init__('drone_xbee_timer_task')
        self.sendDroneInfo_xbee_timer = self.create_timer(1/20,self.sendDroneInfo_xbee)
        self.recv_and_pub_timer = self.create_timer(1/80,self.recv_task_code)
        self.service_client = DroneClientNode()        
        
    # def recv_camera_info(self):
    #     global droneSub
    #     if ((droneSub.detect_status == True) and (droneSub.target_center_status == True)):
    #         xbee.xbee_send_target_point(droneSub.latitude,droneSub.longitude,droneSub.motor_pitch,droneSub.motor_yaw)
    #         print('send camera info...')
    #     elif((droneSub.detect_status == True) and (droneSub.target_center_status == False)):
    #         print('Target not in camera center...')
    #     elif((droneSub.detect_status == False) and (droneSub.target_center_status == True)):
    #         print('Detect Status is %s',droneSub.detect_status)
    
    def recvGS(self, result):   #接收地面站State
        global droneState
        
        if result == xbee.groundControlCommand.DRONE_TAKEOFF.value:
            droneState.droneState = xbee.groundControlCommand.DRONE_TAKEOFF.value        
            status = xbee.groundControlCommand.DRONE_TAKEOFF.value                #寫入takeoff指令給topic中的msg state
            self.get_logger().info('Dronestatus Request: "%s"' % status)
            
        if result == xbee.groundControlCommand.DRONE_MISSION_START.value:
            droneState.droneState = xbee.groundControlCommand.DRONE_MISSION_START.value  
            status = xbee.groundControlCommand.DRONE_MISSION_START.value          #寫入mission_start指令給topic中的msg state
            self.get_logger().info('Dronestatus Request: "%s"' % status)
            
        if result == xbee.groundControlCommand.DRONE_RSEARCH_START.value:
            droneState.droneState = xbee.groundControlCommand.DRONE_RSEARCH_START.value  
            status = xbee.groundControlCommand.DRONE_RSEARCH_START.value          #寫入mission_start指令給topic中的msg state
            self.get_logger().info('Dronestatus Request: "%s"' % status)
                
        response = self.service_client.send_status_request(status)
        print('xbee recv response :',response.check)
        
        while response.check is not True:
            response = self.service_client.send_status_request(status)
            print('xbee recv response :',response.check)
            
        xbee.xbee_send_status_check()
             
    def sendDroneInfo_xbee(self):   #傳送無人機狀態
        # 頭碼及尾碼
        head = bytes('\x30', 'utf-8')
        end = bytes('\x40', 'utf-8')
        drone_id = bytes('\x02', 'utf-8')
        
        battery_temp = 0.0
        
        global droneSub
        xbee.xbee_packet(b'\x30',
                            droneSub.roll,droneSub.pitch,droneSub.yaw,
                            droneSub.angu_x,droneSub.angu_y,droneSub.angu_z,
                            droneSub.acc_x,droneSub.acc_y,droneSub.acc_z,
                            droneSub.heading,
                            droneSub.velocity,
                            droneSub.gps_altitude,
                            droneSub.latitude,
                            droneSub.longitude,
                            int(battery_temp),
                            b'\x02',    # Drone ID
                            b'\x40')
       # print('send drone info...')
      
    def recv_task_code(self):
        global drone_state,droneSub
        global ser,RTK_ser
        global flag,point_count,frame_Details
        
        status = xbee.ser.inWaiting()
        if status >= 4:
            get_frame = xbee.ser.read(4)
            data_len = get_frame[2]
            frame_type = get_frame[3]
            # print('data_len = %s' % hex(data_len))
            # print('frame_type = %s' % hex(frame_type))
            if data_len == frame_Details.Status_len and frame_type == frame_Details.Transmit_Status:
                data_status = xbee.ser.inWaiting()
                while data_status < 7:
                    data_status = xbee.ser.inWaiting()
                # print('data buffer = ',data_status)
                if data_status >= 7:
                    trans_status = xbee.ser.read(7)
                    # print('recv trans_status')
            elif data_len == frame_Details.RTCM_len and frame_type == frame_Details.Receive_Packet:
                RTK_data = xbee.ser.inWaiting()
                while RTK_data < 80:
                    RTK_data = xbee.ser.inWaiting()
                    # self.get_logger().info('RTK_data buffer = %s'% RTK_data)
                # print('RTCM_ser_buf = ',RTK_data)
                if RTK_data >= 80:
                    RTCM_data = xbee.ser.read(80)
                    task = RTCM_data[11:15]
                    decode_task_select = task.decode(encoding='utf-8',errors='replace')
                    if decode_task_select == 'RTCM':
                        print('----------Run RTCM Task----------')
                        decode_RTCM_data = RTCM_data[15:79]
                        xbee.RTK_ser.write(decode_RTCM_data)
                        flag = flag + 1
                        # print('flag: %s' %flag)
                        print('RTCM = ',decode_RTCM_data)
                        # self.get_logger().info('---------------------------------')
            elif data_len == frame_Details.GUAV_len and frame_type == frame_Details.Receive_Packet:
                GUAV_data = xbee.ser.inWaiting()
                while GUAV_data < 52:
                    GUAV_data = xbee.ser.inWaiting()
                    self.get_logger().info('data buffer = %s'% GUAV_data)
                # print('path_ser_buf = ',GUAV_data)
                if GUAV_data >= 52:
                    print('----------Run MissionPath Task----------')
                    mission_path = xbee.ser.read(52)
                    mission_point = mission_path[15:19]
                    len_decode = struct.unpack("1i",mission_point)
                    data_length = len_decode[0]
                    print('剩餘 %s 個路徑點... ' % data_length)
                    if data_length != 0:
                        speed_N = mission_path[19:23]
                        altitude_N = mission_path[23:27]
                        latitude_N = mission_path[27:35]
                        longitude_N = mission_path[35:43]
                        yaw_delta_N = mission_path[43:47]
                        
                        speed = speed_N[::-1]
                        altitude = altitude_N[::-1]
                        latitude = latitude_N[::-1]
                        longitude = longitude_N[::-1]
                        yaw_delta = yaw_delta_N[::-1]
                        
                        print('speed',speed)
                        print('altitude',altitude)
                        print('latitude',latitude)
                        print('longitude',longitude)
                        print('yaw_delta',yaw_delta)
                        
                        speedHex = binascii.b2a_hex(speed)
                        altitudeHex = binascii.b2a_hex(altitude)
                        latitudeHex = binascii.b2a_hex(latitude)
                        longitudeHex = binascii.b2a_hex(longitude)
                        yaw_deltaHex = binascii.b2a_hex(yaw_delta)
                        
                        speedASCII = speedHex.decode('ascii')
                        altitudeASCII = altitudeHex.decode('ascii')
                        latitudeASCII = latitudeHex.decode('ascii')
                        longitudeASCII = longitudeHex.decode('ascii')
                        yaw_deltaASCII = yaw_deltaHex.decode('ascii')
                        
                        speedfloat = struct.unpack('!f', bytes.fromhex(speedASCII))[0]
                        altitudefloat = struct.unpack('!f', bytes.fromhex(altitudeASCII))[0]
                        latitudedouble = struct.unpack('!d', bytes.fromhex(latitudeASCII))[0]
                        longitudedouble = struct.unpack('!d', bytes.fromhex(longitudeASCII))[0]
                        yaw_deltafloat = struct.unpack('!f', bytes.fromhex(yaw_deltaASCII))[0]
                        
                        print('speedfloat',speedfloat)
                        print('altitudefloat',altitudefloat)
                        print('latitudedouble',latitudedouble)
                        print('longitudedouble',longitudedouble)
                        print('yaw_deltafloat',yaw_deltafloat)
                        
                        
                        speed_count.append(speedfloat)
                        altitude_count.append(altitudefloat)
                        latitude_count.append(latitudedouble)
                        longitude_count.append(longitudedouble)
                        yaw_delta_count.append(yaw_deltafloat)
                        
                        # print('type:speed_count:',type(speed_count))
                        # print('speed_count:',speed_count)
                        
                        data_length = data_length - 1
                        point_count = point_count + 1 # 紀錄傳送點數
                        
                        if data_length == 0:
                            point_count_str = str(point_count)
                            self.get_logger().info('ServiceClient_point_count: "%s"' % point_count_str)
                            self.get_logger().info('ServiceClient_speed: "%s"' % speed_count)
                            self.get_logger().info('ServiceClient_altitude: "%s"' % altitude_count)
                            self.get_logger().info('ServiceClient_latitude: "%s"' % latitude_count)
                            self.get_logger().info('ServiceClient_longitude: "%s"' % longitude_count)
                            self.get_logger().info('ServiceClient_yaw_delta: "%s"' % yaw_delta_count)
                            
                            response = self.service_client.send_missionpath_request(speed_count, altitude_count, latitude_count, 
                                                                            longitude_count, yaw_delta_count, point_count_str)
                            print('xbee recv response',response.path_check)
                            
                            while response.path_check is not True:
                                response = self.service_client.send_status_request(speed_count, altitude_count, latitude_count, 
                                                                           longitude_count, yaw_delta_count, point_count_str)
                                print('xbee recv response :',response.path_check)
                                
                            xbee.xbee_send_path_check(point_count)
                            print('OK')
                            print('----------------------------------------')  
                        print('----------------------------------------')     
            elif data_len == frame_Details.CUAV_len and frame_type == frame_Details.Receive_Packet:
                CUAV_data = xbee.ser.inWaiting()
                while CUAV_data < 21:
                    CUAV_data = xbee.ser.inWaiting()
                    # self.get_logger().info('data buffer = %s'% data)
                if CUAV_data >= 21:
                    state = xbee.ser.read(21)
                    cuav_task = state[11:15]
                    decode_cuav_task = cuav_task.decode(encoding='utf-8',errors='replace')
                    if decode_cuav_task == 'CUAV':
                        print('----------Run DroneState Task----------')
                        if state[15] == 0x31:
                            print("drone takeoff...")
                            self.recvGS(xbee.groundControlCommand.DRONE_TAKEOFF.value)
                            print('---------------------------------------')
                        if state[15] == 0x32:
                            print("mission start...")
                            self.recvGS(xbee.groundControlCommand.DRONE_MISSION_START.value)
                            print('---------------------------------------')
                        if state[15] == 0x33:
                            print("research start...")
                            self.recvGS(xbee.groundControlCommand.DRONE_RSEARCH_START.value)
                            print('---------------------------------------')
                            
class DroneClientNode(Node):
    def __init__(self):
        super().__init__('drone_xbee_client')
        self._status = self.create_client(DroneStatus, 'drone_status')
        self._missionpath = self.create_client(DroneMissionPath, 'drone_mission_path')
        while not self._status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service _status not available, waiting again...')
        print('_status service OK')
        while not self._missionpath.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service _missionpath not available, waiting again...')
        print('_missionpath service OK')
        self.status_req = DroneStatus.Request()
        self.path_req = DroneMissionPath.Request()
        
    def send_status_request(self, status):
        self.status_req.status = status
        self.future_status = self._status.call_async(self.status_req)
        rclpy.spin_until_future_complete(self, self.future_status)
        return self.future_status.result()
    
    def send_missionpath_request(self, speed, altitude, latitude, longitude, yaw_delta, point_count):
        self.path_req.speed = speed
        self.path_req.altitude = altitude
        self.path_req.latitude = latitude
        self.path_req.longitude = longitude
        self.path_req.yaw_delta = yaw_delta
        self.path_req.point_count = point_count
        self.future_path = self._missionpath.call_async(self.path_req)
        rclpy.spin_until_future_complete(self, self.future_path)
        return self.future_path.result()
''' 
class DroneServiceNode(Node):
    def __init__(self):
        super().__init__('drone_xbee_service')
        self.srv_RADI = self.create_service(ReturnAndDetectInformation, 'return_and_detect_information', self.hold_recv)

    def hold_recv(self, request, response):
        global droneSub
        if (request.hold_flag == True):
            xbee.xbee_send_target_point(droneSub.latitude,droneSub.longitude,droneSub.motor_pitch,droneSub.motor_yaw)
            print('send camera info...')
            response.information_flag == True
            print('information_flag is %s',response.information_flag)
            return response
        else:
            response.information_flag == False
            return response
'''
##############################TOOL#####################################  
def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    rclpy.shutdown()
    sys.exit(0)      
    
def pub_sub_init(args=None):
    global droneSub,droneTimer
    global droneState,frame_Details
    
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)
    
    droneSub = DroneSubscribeNode()
    droneTimer = DroneTimerTaskNode()
    droneState = xbee.drone_state()
    frame_Details = xbee.frame_details()
    
    executor = MultiThreadedExecutor()
    executor.add_node(droneSub)
    executor.add_node(droneTimer)
    executor.spin()
    
    droneSub.destroy_node()
    droneTimer.destroy_node()

    rclpy.shutdown()

def main():
    xbee.zigbee_init()
    xbee.RTK_init()
    pub_sub_init()

if __name__ == '__main__':
    main()