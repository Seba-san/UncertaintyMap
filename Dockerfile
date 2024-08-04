#FROM gazebo:libgazebo11
#RUN apt-get update
#RUN apt-get install curl -y
#RUN rm -f /etc/apt/sources.list.d/ros-latest.list
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
#
##RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
##RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
##RUN apt-get update
##RUN apt-get install curl -y
##RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
#RUN apt-get update
#ENV DEBIAN_FRONTEND noninteractive
#RUN apt install ros-noetic-desktop -y
FROM ros:noetic-robot-focal
RUN apt-get update
RUN apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers  -y
#RUN apt-get install apt install ros-noetic-control* -y
RUN apt-get install ros-noetic-rqt-controller-manager -y # instala todas las librerias necesarias.
RUN apt-get install ros-noetic-rviz -y
#RUN apt install g++ -y
RUN apt install build-essential -y
RUN apt-get install git -y
RUN apt install ros-noetic-rqt-plot -y
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN mkdir -p /root/ws/src
#COPY /home/seba/Dropbox/1_Doctorado/exploration_method/ws/ ~/ws/src/
#ADD ../ws/ /root/ws/src/
COPY ./ws/ /root/ws/src/
COPY ./ws/default.rviz /opt/ros/noetic/share/rviz/
COPY ./models/ /usr/share/gazebo-11/models/
COPY ./models/ /usr/share/gazebo-11/models/
#RUN cd ~/ws && source /opt/ros/noetic/setup.bash && catkin_make 
# Probando rosdep
# RUN apt-get install python3-rosdep && rosdep init && rosdep update
# RUN rosdep install -r --from-paths src -i -y --rosdistro noetic
## Probando rosdep
RUN ["/bin/bash", "-c", "cd /root/ws && source /opt/ros/noetic/setup.bash && catkin_make"]
RUN echo "source /root/ws/devel/setup.bash" >> ~/.bashrc
RUN ln -s /usr/bin/python3 /usr/bin/python
# for polaris_robot:
RUN apt-get install wget

RUN ["/bin/bash", "-c", "cd /tmp  && wget https://bootstrap.pypa.io/pip/3.6/get-pip.py && python get-pip.py"]
RUN pip install scipy
RUN pip install scikit-learn
RUN pip install ruamel.yaml
RUN pip install pynput
RUN pip install matplotlib
# Para agregarle color a las lineas (no se si funciona)
#RUN echo "force_color_prompt=yes" | cat - ~/.bashrc > /tmp/out && mv /tmp/out ~/.bashrc
ENV TERM xterm-256color
# Instalar ssh server, source: https://dev.to/s1ntaxe770r/how-to-setup-ssh-within-a-docker-container-i5i
#RUN apt install  openssh-server sudo -y
#RUN apt install  openssh-server -y
#RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1000 test 
#RUN  echo 'test:test' | chpasswd # password test

# Falta algo, no se que es...
#RUN service ssh start
#EXPOSE 22
COPY ./ws/pioneer2dx/worlds /root/ws/src/pioneer2dx/
COPY ./ws/pioneer2dx/launch /root/ws/src/pioneer2dx/
# no tendira que ser?:
#COPY ./ws/pioneer2dx/worlds/ /root/ws/src/pioneer2dx/worlds/
#COPY ./ws/pioneer2dx/launch/ /root/ws/src/pioneer2dx/launch/

COPY ./models/ /usr/share/gazebo-11/




