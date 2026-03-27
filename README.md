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

### TODO Lists:
1. 完善依赖安装过程流程描述
2. 规范化软件启动流程
3. Fork VRX gazebo v2.4.1, 搭建多个不同的场景，以验证算法的表现
4. 将DOGM的功能包集成进来

