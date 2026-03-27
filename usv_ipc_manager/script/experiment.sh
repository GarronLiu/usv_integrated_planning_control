#!/bin/bash

gnome-terminal -t "link mavros" -x bash -c "roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp://10.168.1.103:14550@;exec bash"

gnome-terminal -t "raise mavros local_position/odom and rc/out topic rate" -x bash -c "while true; do rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0;sleep 4;done;exec bash"

sleep 5

gnome-terminal -t "record rosbag" -x bash -c "mkdir ~/bagfile;cd ~/bagfile;rosbag record -o name.bag /mavros/local_position/odom /mavros/rc/override /tracker/acceleration_error /manager/trajectory/tracking;exec bash"
