#!/usr/bin/env python

import rosbag
import numpy as np

def read_track_error(bag_file):
    bag = rosbag.Bag(bag_file)

    track_errors = []

    for topic, msg, t in bag.read_messages(topics=['/tracker/track_error']):
        # 假设track_error包含两个维度的误差值，例如msg.error_x和msg.error_y
        error_x = msg.data[0]
        error_y = msg.data[1]
        error = np.sqrt(error_x**2 + error_y**2)
        track_errors.append(error)

    bag.close()
    return track_errors

def calculate_average_error(track_errors):
    if len(track_errors) == 0:
        return 0.0
    return np.mean(track_errors)

if __name__ == '__main__':
    bag_file = '/home/garronliu/rosbag/sindy_2024-12-31-14-58-58.bag'  # 替换为你的rosbag文件路径
    track_errors = read_track_error(bag_file)
    average_error = calculate_average_error(track_errors)
    print(f'Average Track Error: {average_error}')