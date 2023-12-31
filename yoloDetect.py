import rclpy
from rclpy.node import Node
from yolo.visionDetect import YOLO, argument, get_detect
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile
import threading
from sensor_msgs.msg import NavSatFix
from tutorial_interfaces.msg import Motor, Img
from xbee_v5 import *

import sys, signal
sys.path.append('./yolo/')
motor, img = Motor(), Img(),   # msg name


        
class TrackerPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.img_publisher_ = self.create_publisher(Img, 'img', 10)
        self.motor_publisher_ = self.create_publisher(Motor, 'motor', 10)

        self.tracker = None
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global img, motor
        if self.tracker != None:
            img._yolo_init_status ,img.fps, img.detect_status, img.target_center_status, motor.yaw, motor.pitch = get_detect(self.tracker)
            print(f'detect_status: {img.detect_status},target_center_status: {img.target_center_status},\nfps: {img.fps:.1f} yaw: {motor.yaw:.3f} pitch: {motor.pitch:.3f}')

        # publisher.
        self.motor_publisher_.publish(motor)
        self.img_publisher_.publish(img)

        
def YOLO_Init():
    return argument(w_target='best.pt')    
    
def pub_sub_Init(args=None):
    try:
        rclpy.init(args=args)
        
        #publisher settings
        track = YOLO(YOLO_Init())
        tracker_publisher = TrackerPublisher()
        tracker_publisher.tracker = track
        
        rclpy.spin(tracker_publisher)

        tracker_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.shutdown()

def main():
    pub_sub_Init()


if __name__ == '__main__':
    main()
