#! /bin/sh
# Para usar este script hay que hacer: source ~/iniciar
rosrun xacro xacro model.xacro>model.urdf
gz sdf -p model.urdf >model.sdf
