#!/bin/bash

gnome-terminal --tab -e "/home/dfec/camera_stream.sh"

gnome-terminal --tab -e "sshpass -p "dfec3141" ssh -o StrictHostKeyChecking=no -t dfec@192.168.60.101 'camera_stream.sh'"

sleep 5s

gnome-terminal --tab --working-directory=/home/dfec/detector/ -e "npm run webgl"