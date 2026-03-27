#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from mavros_msgs.msg import RCOut
from nav_msgs.msg import Odometry
import numpy as np
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# 打开rosbag文件
bag = rosbag.Bag('/home/garronliu/rosbag/8mUSV/z_maneuver_20deg_2.bag')

# 初始化数据列表
time_rc = []
channel_1 = []
channel_2 = []

time_odom = []
yaw_angles = []

# 读取/mavros/rc/out话题的数据
for topic, msg, t in bag.read_messages(topics=['/mavros/rc/out']):
    time_rc.append(t.to_sec())
    channel_1.append(msg.channels[2])
    channel_2.append(msg.channels[3])

# 读取/mavros/local_position/odom话题的数据
for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/odom']):
    time_odom.append(t.to_sec())
    _, _, yaw = euler_from_quaternion(msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w)
    yaw_angles.append(math.degrees(yaw))

bag.close()

# 将时间戳转换为相对时间
time_rc = np.array(time_rc) - time_rc[0]
time_odom = np.array(time_odom) - time_odom[0]

# # 绘制通道一和通道二的输入
# plt.figure()
# plt.subplot(2, 1, 1)
# plt.plot(time_rc, channel_1, label='Channel 1')
# plt.plot(time_rc, channel_2, label='Channel 2')
# plt.xlabel('Time (s)')
# plt.ylabel('PWM Value')
# plt.legend()
# plt.title('Channel 1 and Channel 2 Inputs')

# 绘制首向角
plt.subplot(2, 1, 2)
plt.plot(time_odom, yaw_angles, label='Yaw Angle')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (degrees)')
plt.legend()
plt.title('Yaw Angle')
# 将角度偏移换算至-10到10度
yaw_angles = np.array(yaw_angles)
yaw_angles = yaw_angles -62

# 创建双Y轴图
fig, ax1 = plt.subplots()

# 绘制yaw角度
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Yaw Angle (degrees)', color='tab:blue')
ax1.plot(time_odom, yaw_angles, label='Yaw Angle', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.set_ylim([-40, 40])

# 创建第二个Y轴
ax2 = ax1.twinx()
ax2.set_ylabel('PWM Value', color='tab:red')
ax2.plot(time_rc, channel_1, label='Throttle Left', color='tab:green')
ax2.plot(time_rc, channel_2, label='Throttle Right', color='tab:orange')
ax2.tick_params(axis='y', labelcolor='tab:red')
ax2.set_ylim([1400, 2000])

fig.tight_layout()
plt.title('Z-Manevering (20 Deg)')

plt.tight_layout()
plt.show()