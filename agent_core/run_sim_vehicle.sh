#!/bin/bash

python ~/SITL/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -L USAFA_01 --sysid 1 --no-mavproxy

# python ~/SITL/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -L USAFA_01 --out=udpin:127.0.0.1:14551 --out=udpin:0.0.0.0:14101 --out=udpin:0.0.0.0:15101
