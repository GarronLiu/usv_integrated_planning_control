#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
import time

def linear_increase(start_value, end_value, duration, rate):
    step = (end_value - start_value) / (duration * rate)
    current_value = start_value
    while current_value < end_value:
        yield int(current_value)
        current_value += step
        time.sleep(1.0 / rate)
    yield int(end_value)

def main():
    rospy.init_node('motor_control_node', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_value = 1500
    end_value = 1900
    duration = 10  # seconds
    hold_time = 60*15  # seconds

    for value in linear_increase(start_value, end_value, duration, 10):
        msg = OverrideRCIn()
        msg.channels = [0,0,value,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # 将所有通道设置为相同的值
        pub.publish(msg)
        rate.sleep()
    
    end_time = rospy.Time.now() + rospy.Duration(hold_time)
    while rospy.Time.now() < end_time:
        msg = OverrideRCIn()
        msg.channels = [0, 0, end_value, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # 将所有通道设置为相同的值
        pub.publish(msg)
        rate.sleep()
    msg = OverrideRCIn()
    msg.channels = [0] * 18  # 将所有通道设置为0
    pub.publish(msg)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass