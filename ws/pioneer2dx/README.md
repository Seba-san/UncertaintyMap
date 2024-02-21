# Requerimientos
Esta libreria se probó en ROS Melodic.
```sh
$sudo apt-get install ros-$ROS_DISTRO-gazebo11-ros-pkgs  ros-$ROS_DISTRO-gazebo11-ros-control 
$sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-state-publisher
```
# Libreria Pioneer2dx
Esta libreria contiene al Pioneer2dx  
![](images/pioneer2dx.png)
Además cuenta con el siguiente entorno de testeo
![](images/escenario.jpg)

Papra probarlo setear las variables de entorno de ROS y luego hacer

```sh
$ mkdir ws/src
$ cd ws/src
$ git clone https://github.com/Seba-san/pioneer2dx.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ roslaunch pioneer2dx controller.launch
```
Este comando iniciará gazebo server sin la interfaz gráfica, iniciara el mundo test_word.sdf y pone el pioneer2dx en el origen de coordenadas.
Para abrir la interfaz grafica en una terminal poner:
```sh
$gzclient
```

Para visualizar el robot en Rviz, ejecutar en una terminal:
```sh
$ roscd pioneer2dx
$rosrun rviz rviz -d ./config/rviz_config.rviz
```
Además puede ejecutar el controlador para ser teleoperado por teclado mediante:

```sh
$rosrun pioneer2dx controller_node.py
```
luego mediante el uso de las teclas: "w-a-s-d" para moverse, "q" para salir, "r" para hacer un respawn del vehículo y "x" para resetear el modelo en el origen de coordenadas.

Pioneer2dx cuenta con una cámara  frontal 800x600px publicando las imagenes a 15 frames. Todo esto es configurable en el archivo plugins/camera_plug.xacro




License
----

MIT
