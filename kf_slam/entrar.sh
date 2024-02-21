#!/bin/bash
xhost +local:*
docker exec -e "TERM=xterm-color" -it rosros_exploration bash 