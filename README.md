# usv_integrated_planning_control

### Dependencies
ROS1
CUDA
### Quick start
1. Clone and Run:

```bash
mkdir -p usv_ipc_ws/src && cd usv_ipc_ws/src
git clone https://github.com/GarronLiu/usv_integrated_planning_control.git
cd ..
catkin_make -j4
source devel/setup.bash
roslaunch usv_ipc_manager run.launch
```

2. This Example can be combined with VRX gazebo simulation v2.4.1

```bash
mkdir -p vrx_ws/src && cd vrx_ws/src
git clone https://github.com/osrf/vrx.git
git checkout v2.4.1
cd ..
catkin_make -j4
source devel/setup.bash
roslaunch vrx_gazebo vrx.launch
```
