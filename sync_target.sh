#!/bin/bash

HEAD_ADDR=rpi
HEAD_USER=ubuntu
rsync -avzPc metal_tank_software/ $HEAD_USER@$HEAD_ADDR:~/catkin_ws/src/metal_tank_software/
