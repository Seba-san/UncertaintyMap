#!/bin/sh
xhost +local:*
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v .:/root/ws/src/pioneer2dx/scripts/kf_slam/ -p 11311:11311  --name rosros_exploration -d $1 tail -f /dev/null #-p 11311:11311  --net=host
docker exec -e "TERM=xterm-color" -it rosros_exploration bash
