#!/bin/sh
#export ROS_HOSTNAME=pioneer-raspberry
sudo echo "192.168.100.191 seba-inaut">>/etc/hosts
sudo echo "192.168.0.115 seba-Lenovo">>/etc/hosts
export ROS_HOSTNAME=raspberrypi
#export ROS_IP=192.168.100.178
export ROS_IP=192.168.0.116
export ROS_MASTER_URI=http://192.168.0.115:11311
#roscore &
#rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=500000
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
