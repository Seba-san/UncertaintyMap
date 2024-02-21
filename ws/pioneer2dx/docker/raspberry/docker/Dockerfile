FROM ros:melodic-ros-core-bionic

# install ros package
RUN ["sudo","apt-get","update"]
RUN ["sudo","apt-get","install","-y", "ros-melodic-rosbridge-server"]
RUN ["sudo","apt-get","install","-y", "ros-melodic-tf2-ros"]
RUN ["sudo","apt-get","install","-y", "ros-melodic-rosserial"]
# Install imu package
RUN ["cd","/home/pi"]
RUN ["mkdir","-p","ws/src"]
RUN source /opt/ros/melodic/setup.bash
RUN ["cd","ws"]
RUN ["catkin_make"]
RUN ["cd","src"]
RUN ["git","clone","http://github.com/seba-san/bb_mpu9150"]
RUN ["cd",".."]
RUN ["catkin_make"]


#echo "source ${CATKIN_DIR}/devel/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo "source /home/pi/ws/devel/setup.bash" >> /root/.bashrc
#COPY ["ros_entrypoint.sh","/ros_entrypoint.sh"]
#ENTRYPOINT roslaunch rosbridge_server rosbridge_websocket.launch



