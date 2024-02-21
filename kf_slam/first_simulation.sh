#!/bin/sh
cd /home/seba/Dropbox/1_Doctorado/exploration_method/limpio_docker/ws/pioneer2dx/launch
docker cp one_cone.launch rosros_exploration:/root/ws/src/pioneer2dx/launch/
docker cp empty.launch rosros_exploration:/root/ws/src/pioneer2dx/launch/
cd ../worlds/
docker cp one_cone.world rosros_exploration:/root/ws/src/pioneer2dx/worlds/