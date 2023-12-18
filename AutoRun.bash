#! /bin/bash
export DISPLAY=:0

source /home/ubuntu/ros2_ws/install/setup.bash

echo "123456789" | sudo -S chmod 666 /dev/ttyRTK

echo "123456789" | sudo -S chmod 666 /dev/ttyXbee

echo "123456789" | sudo -S chmod 666 /dev/ttyPixhawk

echo "123456789" | sudo -S chmod 666 /dev/ttyTHS0

ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyPixhawk &

sleep 20

python3 /home/ubuntu/Documents/DroneFly/yoloDetect.py

sleep 10

python3 /home/ubuntu/Documents/DroneFly/drone_ROS2.py &

sleep 10

python3 /home/ubuntu/Documents/DroneFly/drone_ROS2_xbee_v7.py &

sleep 100000000

