#!/bin/bash
gnome-terminal -t "MAVROS" -x bash -c "cd /mavros_ws;source devel/setup.bash;roslaunch mavros apm.launch;exec bash"

sleep 0.5s
gnome-terminal -t "EKFOrigin_Acquire" -x bash -c "cd /mavros_ws;source devel/setup.bash;rosrun ekf_origin_acquire ekf_origin_acquire_node;exec bash"

sleep 0.5s
gnome-terminal -t "request_message_from_fcu" -x bash -c "rosrun mavros mavcmd long 512 49 0 0 0 0 0 0;exec bash"

# sleep 0.5s
# gnome-terminal -t "px4fsm" -x bash -c "cd ~/Workspace/fly_ws;source devel/setup.bash;roslaunch px4_fsm real_world.launch;exec bash"

#sleep 0.5s
#gnome-terminal -t "record" -x bash -c "cd ~/Workspace/swarm_lio_ws;source devel/setup.bash;rosbag record -a;exec bash"
