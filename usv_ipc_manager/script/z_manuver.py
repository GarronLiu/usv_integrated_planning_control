#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from nav_msgs.msg import Odometry
import time
import math


class MotorControlNode:
    def __init__(self):
        rospy.init_node("motor_control_node", anonymous=True)
        self.pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.pwm_mid = 1500
        self.end_value_left = None
        self.duration = 2  # 信号切换过渡时间
        self.angle_threshold = 10  # Z型实验首向角偏转切换角度阈值

        self.initial_yaw = None  # 初始化为0
        self.current_yaw = None  # 初始化为0
        self.left_pwm = None  # 确保1通道是产生正向旋转的力
        self.right_pwm = None  # 确保3通道是产生反向旋转的力
        self.right_pwm_max = None
        self.left_pwm_max = None
        self.increasing_yaw = True  # 标志位，控制yaw角的增加和减少
        self.change_flag = False
        self.step_left = None
        self.step_right = None
        self.current_state = State()

        rospy.Subscriber("/wamv/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mavros/rc/out", RCOut, self.rc_out_callback)
        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def state_callback(self, msg):
        self.current_state = msg
        rospy.loginfo("Armed State: %s", self.current_state.armed)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        if self.initial_yaw is None:
            self.initial_yaw = yaw
        self.current_yaw = yaw

        # rospy.loginfo("Current Yaw: %f degrees", math.degrees(self.current_yaw))
        if self.increasing_yaw:
            dyaw = self.current_yaw - self.initial_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi

            if dyaw >= math.radians(self.angle_threshold - 0.01):
                self.increasing_yaw = False
                self.change_flag = True
        else:
            dyaw = self.initial_yaw - self.current_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi

            if dyaw >= math.radians(self.angle_threshold - 0.01):
                self.increasing_yaw = True
                self.change_flag = True

    def rc_out_callback(self, msg):
        if self.step_left == None:
            self.left_pwm = msg.channels[0]
            self.right_pwm = msg.channels[2]
            self.right_pwm_max = self.right_pwm
            self.left_pwm_max = self.left_pwm
            self.step_left = (self.left_pwm_max - self.pwm_mid) / self.duration / 10
            self.step_right = (self.right_pwm_max - self.pwm_mid) / self.duration / 10

    def euler_from_quaternion(self, x, y, z, w):
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

    def simulate_yaw_change(self, change_rate):
        if self.increasing_yaw:
            self.current_yaw += change_rate
            dyaw = self.current_yaw - self.initial_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi

            if dyaw >= math.radians(self.angle_threshold):
                self.increasing_yaw = False
                self.change_flag = True
        else:
            self.current_yaw -= change_rate
            dyaw = self.initial_yaw - self.current_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi

            if dyaw >= math.radians(self.angle_threshold):
                self.increasing_yaw = True
                self.change_flag = True

    def control_motor(self):

        while not rospy.is_shutdown():
            # 如果飞控状态不是ARMED，不发布override消息
            if not self.current_state.armed:
                msg = OverrideRCIn()
                msg.channels = [0] * 18  # 将所有通道设置为0
                self.pub.publish(msg)
                self.rate.sleep()
                continue

            # # 如果飞控状态不是ARMED，不发布override消息
            if self.left_pwm == None:
                msg = OverrideRCIn()
                msg.channels = [0] * 18  # 将所有通道设置为0
                self.pub.publish(msg)
                self.rate.sleep()
                continue

            if self.increasing_yaw:
                if self.right_pwm < self.right_pwm_max:
                    self.right_pwm += self.step_right
                if self.left_pwm > self.pwm_mid:
                    self.left_pwm -= self.step_left
                msg = OverrideRCIn()
                msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                    0
                ] * 15  # 将所有通道设置为相同的值
                # self.simulate_yaw_change(math.radians(self.angle_threshold)/self.duration/10/2)
                self.pub.publish(msg)
                self.rate.sleep()
            else:
                if self.right_pwm > self.pwm_mid:
                    self.right_pwm -= self.step_right
                if self.left_pwm < self.left_pwm_max:
                    self.left_pwm += self.step_left
                msg = OverrideRCIn()
                msg.channels = [int(self.left_pwm), 0, int(self.right_pwm)] + [
                    0
                ] * 15  # 将所有通道设置为相同的值
                self.pub.publish(msg)
                # self.simulate_yaw_change(math.radians(self.angle_threshold)/self.duration/10/2)
                self.rate.sleep()

        # 停止发布消息
        msg = OverrideRCIn()
        msg.channels = [0] * 18  # 将所有通道设置为0
        self.pub.publish(msg)

    def run(self):
        rospy.sleep(2)  # 等待订阅者初始化
        self.control_motor()
        rospy.spin()


if __name__ == "__main__":
    try:
        node = MotorControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
