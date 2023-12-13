import struct
import time
import serial
from enum import Enum
import math

ser = None
RTK_ser = None
xbee_mac = b'\x00\x13\xA2\x00\x40\xDD\xE6\x58' 
# xbee_mac = b'\x00\x00\x00\x00\x00\x00\xFF\xFF' 

def zigbee_init():
    global ser
    ser = serial.Serial()
    ser.port = "/dev/ttyUSB0"
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    ser.timeout = 0.001
    ser.open()

def RTK_init():
    global RTK_ser
    RTK_ser = serial.Serial()
    RTK_ser.port = "/dev/ttyUSB1"
    RTK_ser.baudrate = 115200
    RTK_ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    RTK_ser.parity = serial.PARITY_NONE #set parity check
    RTK_ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    RTK_ser.timeout = 0.02
    RTK_ser.open()
    
# 傳送無人機狀態資訊

def trans_double_2_byte(byte_array):
    byte_array = struct.pack('d', byte_array)
    byte_array_2 = ''.join(['{:02x}'.format(byte) for byte in byte_array])
    result = [byte_array_2[i:i+2] for i in range(0, len(byte_array_2), 2)]
    b = 0
    #print('result_lat',result)
    for i in range(8):
        b += int(result[i], 16)
    #print('sum_lat', b)
    return b

def trans_float_2_byte(byte_array):
    byte_array = struct.pack('f', byte_array)
    byte_array_2 = ''.join(['{:02x}'.format(byte) for byte in byte_array])
    result = [byte_array_2[i:i+2] for i in range(0, len(byte_array_2), 2)]
    b = 0
    #print('result_lat',result)
    for i in range(4):
        b += int(result[i], 16)
    #print('sum_lat', b)
    return b

def int_to_bytes(num):
    # 將整數轉換為4個字節
    byte_array = struct.pack('>I', num)
    return byte_array

def trans_int_2_byte(byte_array):
    p_data = int_to_bytes(byte_array)
    total_sum = sum_bytes(p_data)
    return total_sum

def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum += byte
    return checksum % 256 

def sum_bytes(byte_array):
    # 計算所有字節的總和
    total_sum = sum(byte_array)
    return total_sum

def radian_conv_degree(Radian) : 
    return ((Radian / math.pi) * 180)

def degree_conv_radian(Degree) : 
    return ((Degree / 180) * math.pi)     

def xbee_send_path_check(point):
    global ser,xbee_mac
    mac_sum = 0x00
    ########################################################
    xbee_start      = b'\x7E'                               #1s
    xbee_length     = b'\x00\x18'                           #2s 
    xbee_sendCode   = b'\x10'                               #1s
    xbee_frameID    = b'\x01'                               #1s
    xbee_mac        = xbee_mac                              #8s
    xbee_old_1      = b'\xFF'                               #1s
    xbee_old_2      = b'\xFE'                               #1s
    xbee_unknow     = b'\x00\x00'                           #2s
    check           = b'\x50\x41\x54\x48\x4F\x4B'           #6s
    ########################################################
    p = trans_int_2_byte(point)
    
    for i in range(0,8):
        mac_sum += xbee_mac[i] 
    mac_sum_byte = mac_sum & 0xff
    xbee_sum = 0x0E + mac_sum_byte + 0xC7 + p #+ b#1s
    xbee_sum = xbee_sum & 0xff
    xbee_sum_bytes = 0xFF - xbee_sum
    xbee_sum_bytes = xbee_sum_bytes.to_bytes(1, byteorder='big')
    
    packet = struct.pack('=1s2s1s1s8s1s1s2s6s1i1s',xbee_start,
                                                 xbee_length,
                                                 xbee_sendCode,
                                                 xbee_frameID,
                                                 xbee_mac,
                                                 xbee_old_1,
                                                 xbee_old_2,
                                                 xbee_unknow,
                                                 check,
                                                 point,
                                                 xbee_sum_bytes)
    ser.write(packet)

def xbee_send_status_check():
    global ser,xbee_mac
    mac_sum = 0x00
    ########################################################
    xbee_start      = b'\x7E'                               #1s
    xbee_length     = b'\x00\x16'                           #2s 
    xbee_sendCode   = b'\x10'                               #1s
    xbee_frameID    = b'\x01'                               #1s
    xbee_mac        = xbee_mac                              #8s
    xbee_old_1      = b'\xFF'                               #1s
    xbee_old_2      = b'\xFE'                               #1s
    xbee_unknow     = b'\x00\x00'                           #2s
    check           = b'\x53\x54\x41\x54\x55\x53\x4F\x4B'   #8s
    ########################################################
    for i in range(0,8):
        mac_sum += xbee_mac[i] 
    mac_sum_byte = mac_sum & 0xff
    xbee_sum = 0x0E + mac_sum_byte + 0x7E #+ b#1s
    xbee_sum = xbee_sum & 0xff
    xbee_sum_bytes = 0xFF - xbee_sum
    xbee_sum_bytes = xbee_sum_bytes.to_bytes(1, byteorder='big')
    
    packet = struct.pack('=1s2s1s1s8s1s1s2s8s1s',xbee_start,
                                                 xbee_length,
                                                 xbee_sendCode,
                                                 xbee_frameID,
                                                 xbee_mac,
                                                 xbee_old_1,
                                                 xbee_old_2,
                                                 xbee_unknow,
                                                 check,
                                                 xbee_sum_bytes)
    ser.write(packet)
    
def xbee_packet(head, 
                roll, pitch, yaw,
                angu_x, angu_y, angu_z, 
                acc_x, acc_y, acc_z, 
                heading, velocity, gps_altitude,
                latitude, longitude, battery_temp,
                drone_id, end_code):
    global ser,xbee_mac
    mac_sum = 0x00
     
    ########################################################
    xbee_start      = b'\x7E'                               #1s
    xbee_length     = b'\x00\x55'                           #2s 
    xbee_sendCode   = b'\x10'                               #1s
    xbee_frameID    = b'\x01'                               #1s
    xbee_mac        = xbee_mac                              #8s
    xbee_old_1      = b'\xFF'                               #1s
    xbee_old_2      = b'\xFE'                               #1s
    xbee_unknow     = b'\x00\x00'                           #2s
    ########################################################
    
    for i in range(0,8):
        mac_sum += xbee_mac[i] 
    
    mac_sum_byte = mac_sum & 0xff
       
    a = trans_float_2_byte(roll)
    b = trans_float_2_byte(pitch)
    c = trans_float_2_byte(yaw)
    d = trans_float_2_byte(angu_x)
    e = trans_float_2_byte(angu_y)
    f = trans_float_2_byte(angu_z)
    g = trans_float_2_byte(acc_x)
    h = trans_float_2_byte(acc_y)
    i = trans_float_2_byte(acc_z)
    j = trans_float_2_byte(heading)
    k = trans_float_2_byte(velocity)
    l = trans_float_2_byte(gps_altitude)
    m = trans_double_2_byte(latitude)
    n = trans_double_2_byte(longitude)
    o = trans_int_2_byte(battery_temp)
    
    if ((head == b'\x30') and (end_code == b'\x40')):
        if(drone_id == b'\x00'):
            xbee_sum = 0x0E + mac_sum_byte + 0x30 + a + b + c + d + e + f + g + h + i + j + k + l + m + n + o + 0x00 + 0x40  #+ b#1s
        if(drone_id == b'\x01'):
            xbee_sum = 0x0E + mac_sum_byte + 0x30 + a + b + c + d + e + f + g + h + i + j + k + l + m + n + o + 0x01 + 0x40  #+ b#1s
        if(drone_id == b'\x02'):
            xbee_sum = 0x0E + mac_sum_byte + 0x30 + a + b + c + d + e + f + g + h + i + j + k + l + m + n + o + 0x02 + 0x40  #+ b#1s
        
        xbee_sum = xbee_sum & 0xff
        xbee_sum_bytes = 0xFF - xbee_sum
        #print("sum_check = ",hex(xbee_sum_bytes))
        xbee_sum_bytes = xbee_sum_bytes.to_bytes(1, byteorder='big')
        #print('xbee_sum_bytes',type(xbee_sum_bytes))    
    elif (head != b'\x30'):
        print("head_code error !")
    elif (end_code != b'\x40'):
        print('end_code error !')    
        



    packet = struct.pack('=1s2s1s1s8s1s1s2s1s9f1f1f1f1d1d1i1s1s1s', 
                                                xbee_start,  
                                                xbee_length,                    
                                                xbee_sendCode,                 
                                                xbee_frameID,      
                                                xbee_mac,          
                                                xbee_old_1,        
                                                xbee_old_2,        
                                                xbee_unknow,
                                                head,       
                                                roll,pitch,yaw,                 
                                                angu_x,angu_y,angu_z,           
                                                acc_x,acc_y,acc_z,              
                                                heading,                       
                                                velocity,                       
                                                gps_altitude,                  
                                                latitude,                      
                                                longitude,                     
                                                battery_temp,
                                                drone_id,                       
                                                end_code,
                                                xbee_sum_bytes)                

    ser.write(packet)
    
def xbee_send_target_point(latitude, longitude, pitch_motor, yaw_motor): #當搜索至目標傳送經緯度以及兩軸雲台角度資訊
    global ser,xbee_mac
    mac_sum = 0x00
    ########################################################
    xbee_start      = b'\x7E'                               #1s
    xbee_length     = b'\x00\x26'                           #2s 
    xbee_sendCode   = b'\x10'                               #1s
    xbee_frameID    = b'\x01'                               #1s
    xbee_mac        = xbee_mac                              #8s
    xbee_old_1      = b'\xFF'                               #1s
    xbee_old_2      = b'\xFE'                               #1s
    xbee_unknow     = b'\x00\x00'                           #2s
    ########################################################
    for i in range(0,8):
        mac_sum += xbee_mac[i] 
    mac_sum_byte = mac_sum & 0xff
    a = trans_double_2_byte(latitude)
    b = trans_double_2_byte(longitude)
    c = trans_float_2_byte(pitch_motor)
    d = trans_float_2_byte(yaw_motor)
    data_sum_byte = a + b + c + d
    data_sum_byte = data_sum_byte & 0xFF
    xbee_sum = 0x0E + mac_sum_byte + data_sum_byte
    xbee_sum = xbee_sum & 0xFF
    xbee_sum_bytes = 0xFF - xbee_sum
    xbee_sum_bytes = xbee_sum_bytes.to_bytes(1, byteorder='big')
    packet = struct.pack('=1s2s1s1s8s1s1s2s1d1d1f1f1s',xbee_start,
                                                      xbee_length,
                                                      xbee_sendCode,
                                                      xbee_frameID,
                                                      xbee_mac,
                                                      xbee_old_1,
                                                      xbee_old_2,
                                                      xbee_unknow,
                                                      latitude,
                                                      longitude,
                                                      pitch_motor,
                                                      yaw_motor,
                                                      xbee_sum_bytes)
    
    ser.write(packet)
    

# 模擬無人機端接收地面監控站數據的程式

#####################DATA ARRAY#########################
# head = 'GUAV'                                         #4s
# len_data                                              #1i
# speed                                                 #1f
# altitude                                              #1f
# latitude                                              #1d
# longitude                                             #1d
# yaw_delta                                             #1f
# end_code = 'TUAV'                                     #4s
########################################################

drone_point_xbee = [] #無人機不固定飛行點
drone_point_xbee_str = []#無人機不固定飛行點(string)

class groundControlCommand(Enum):
    DRONE_IDLE = '0'
    DRONE_TAKEOFF = '1'
    DRONE_MISSION_START = '2'
    DRONE_RSEARCH_START = '3'

class drone_state():
    def __init__(self):
        self.droneState = groundControlCommand.DRONE_IDLE.value    

class frame_details():
    def __init__(self):
        self.RTCM_len = 0x50
        self.CUAV_len = 0x15
        self.GUAV_len = 0x34
        self.Status_len = 0x07
        self.Receive_Packet = 0x90
        self.Transmit_Status = 0x8B    
         
def get_rsearch_point_Xbee(): 
    global drone_point_xbee
    global GUAV_flag
    if GUAV_flag == True:
        #GUAV_flag = False
        #print(drone_point)
        return drone_point_xbee
    else:
        print("no drone point")

# def delete_first_rsearch_point_Xbee():
#     global drone_point_xbee
#     del drone_point[0]   

         
# if __name__ == '__main__':
#     #err = error_code()
#     #success = success_code()
#     print("Start send...")
#     zigbee_init()
#     # RTK_init()
#     xbee_send_target_point(25.01883,121.40063,90.6,270.3)
#     while True:
#         # recv_RTCM_data()
#         # drone_ground_control_xbee()
#         # recv_fallow_data()
#         # err.Error_code_count()
#         # success.success_count()
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40163,0,b'\x00',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40174,0,b'\x00',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40185,0,b'\x00',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40196,0,b'\x00',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40207,0,b'\x00',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40218,0,b'\x00',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40229,0,b'\x00',b'\x40')
        
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40153,0,b'\x01',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40164,0,b'\x01',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40175,0,b'\x01',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40186,0,b'\x01',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40197,0,b'\x01',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40208,0,b'\x01',b'\x40')
#         # xbee_packet(b'\x30',0,0,0,0,0,0,0,0,0,0,0,0,25.01903,121.40219,0,b'\x01',b'\x40')
#         # xbee_send_target_point(25.01903,121.40163,90.6,270.3)
#         print("Send End...")
