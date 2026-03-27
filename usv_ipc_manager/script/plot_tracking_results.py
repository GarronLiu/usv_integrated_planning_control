import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
import numpy as np
from sklearn.metrics import mean_squared_error
from scipy.interpolate import interp1d


def read_rosbag(bag_file, start_time, end_time):
    bag = rosbag.Bag(bag_file)

    odom_x = []
    odom_y = []
    path_x = []
    path_y = []
    u = []
    v = []
    r = []
    rc1 = []
    rc3 = []
    odom_time_stamps = []
    rc_time_stamps = []

    for topic, msg, t in bag.read_messages(topics=["/mavros/local_position/odom"]):
        odom_x.append(msg.pose.pose.position.x)
        odom_y.append(msg.pose.pose.position.y)
        u.append(msg.twist.twist.linear.x)
        v.append(msg.twist.twist.linear.y)
        r.append(msg.twist.twist.angular.z)
        odom_time_stamps.append(t.to_sec())

    for topic, msg, t in bag.read_messages(topics=["/mavros/rc/override"]):
        rc1.append(msg.channels[0])  # 获取通道数据
        rc3.append(msg.channels[2])  # 获取通道数据
        rc_time_stamps.append(t.to_sec())

    odom_time_stamps = np.array(odom_time_stamps) - odom_time_stamps[0]
    rc_time_stamps = np.array(rc_time_stamps) - rc_time_stamps[0]

    odom_x = filter_by_time_range(odom_time_stamps, odom_x, start_time, end_time)
    odom_y = filter_by_time_range(odom_time_stamps, odom_y, start_time, end_time)
    u = filter_by_time_range(odom_time_stamps, u, start_time, end_time)
    v = filter_by_time_range(odom_time_stamps, v, start_time, end_time)
    r = filter_by_time_range(odom_time_stamps, r, start_time, end_time)
    rc1 = filter_by_time_range(rc_time_stamps, rc1, start_time, end_time)
    rc3 = filter_by_time_range(rc_time_stamps, rc3, start_time, end_time)
    odom_time_stamps = filter_by_time_range(
        odom_time_stamps, odom_time_stamps, start_time, end_time
    )
    odom_time_stamps = odom_time_stamps - odom_time_stamps[0]
    rc_time_stamps = filter_by_time_range(
        rc_time_stamps, rc_time_stamps, start_time, end_time
    )
    rc_time_stamps = rc_time_stamps - rc_time_stamps[0]

    bag.close()
    return (
        odom_x,
        odom_y,
        path_x,
        path_y,
        u,
        v,
        r,
        rc1,
        rc3,
        odom_time_stamps,
        rc_time_stamps,
    )

def filter_by_time_range(odom_time_stamps, data, start_time, end_time):
    filtered_data = [
        d for t, d in zip(odom_time_stamps, data) if start_time <= t <= end_time
    ]
    return filtered_data


if __name__ == "__main__":
    (
        odom_x_gp,
        odom_y_gp,
        path_x_gp,
        path_y_gp,
        u_gp,
        v_gp,
        r_gp,
        rc1_gp,
        rc3_gp,
        odom_time_stamps_gp,
        rc_time_stamps_gp,
    ) = read_rosbag("src/usv_ipc_manager/config/eight_gp.bag",3.58,43.6)
    (
        odom_x_sindy,
        odom_y_sindy,
        path_x_sindy,
        path_y_sindy,
        u_sindy,
        v_sindy,
        r_sindy,
        rc1_sindy,
        rc3_sindy,
        odom_time_stamps_sindy,
        rc_time_stamps_sindy,
    ) = read_rosbag("src/usv_ipc_manager/config/circle_sindy.bag",15,35)
    plt.rcParams["figure.dpi"] = 300
    plt.figure(figsize=(8, 14))
    plt.subplot(4, 1, 1)
    plt.plot(odom_time_stamps_gp, u_gp, label="u_gp (linear velocity x)", linewidth=1.25)
    # plt.plot(odom_time_stamps_sindy, u_sindy, label="u_sindy (linear velocity x)")
    plt.xticks(fontsize=5)
    plt.yticks(fontsize=5)
    plt.gca().xaxis.set_ticklabels([])
    plt.ylabel("$ \mathbf{u \quad [m/s]}$")

    plt.subplot(4, 1, 2)
    plt.plot(odom_time_stamps_gp, v_gp, label="v_gp (linear velocity y)")
    # plt.plot(odom_time_stamps_sindy, v_sindy, label="v_sindy (linear velocity y)")
    plt.xticks(fontsize=5)
    plt.yticks(fontsize=5)
    plt.gca().xaxis.set_ticklabels([])
    plt.ylabel("$ \mathbf{v \quad [m/s]}$")

    plt.subplot(4, 1, 3)
    plt.plot(odom_time_stamps_gp, r_gp, label="r_gp (angular velocity z)")
    # plt.plot(odom_time_stamps_sindy, r_sindy, label="r_sindy (angular velocity z)")

    plt.xticks(fontsize=5)
    plt.yticks(fontsize=5)
    plt.gca().xaxis.set_ticklabels([])
    plt.ylabel("$ \mathbf{r \quad [m/s]}$")

    plt.subplot(4, 1, 4)
    plt.plot(rc_time_stamps_gp, rc1_gp, label="Port")
    plt.plot(rc_time_stamps_gp, rc3_gp, label="Starboard")
    plt.ylabel('Control Signal', fontsize=8, fontweight="bold")
    plt.xticks(fontsize=5)
    plt.yticks(fontsize=5)
    plt.xlabel("Time (s)", fontsize=8, fontweight="bold")
    plt.legend(loc="lower right", fontsize=8, ncol=2)
    
    plt.tight_layout()
    plt.show()

    plt.figure()
    plt.plot(odom_x_gp, odom_y_gp, label="Odometry")

    t = np.linspace(0, 2 * np.pi, 1000)
    x_center, y_center = 5.785, -16.17  # 指定中心点
    a, b = 10, 5  # 8字形轨迹的参数
    x_figure8 = x_center + a * np.sin(t)
    y_figure8 = y_center + b * np.sin(2 * t)
    plt.plot(x_figure8, y_figure8, label="Reference Path")
    
    # Create interpolation functions
    interp_func_x = interp1d(odom_time_stamps_gp, odom_x_gp, kind='linear')
    interp_func_y = interp1d(odom_time_stamps_gp, odom_y_gp, kind='linear')

    # Generate new time stamps for 1000 samples
    new_time_stamps = np.linspace(odom_time_stamps_gp[0], odom_time_stamps_gp[-1], 1000)

    # Interpolate odom_x_gp and odom_y_gp to get 1000 samples
    odom_x_gp = interp_func_x(new_time_stamps)
    odom_y_gp = interp_func_y(new_time_stamps)

    rmse = np.sqrt(mean_squared_error(
        np.vstack((odom_x_gp, odom_y_gp)).T[:len(x_figure8)],
        np.vstack((x_figure8, y_figure8)).T
    ))
    plt.text(0.4, 0.95,f"RMSE: {rmse:.3f}", transform=plt.gca().transAxes, fontsize=12, verticalalignment='top', horizontalalignment='left')
    plt.scatter(x_figure8[0], y_figure8[0], color='red', marker='*', zorder=5)
    plt.text(x_figure8[0], y_figure8[0], 'Start', color='red', fontsize=8, verticalalignment='bottom', horizontalalignment='right')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel("X [m]", fontsize=9, fontweight="bold")
    plt.ylabel("Y [m]", fontsize=9, fontweight="bold")
    plt.gca().xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: ""))
    plt.gca().yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: ""))
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.gca().xaxis.set_major_locator(plt.MultipleLocator(5))
    plt.gca().yaxis.set_major_locator(plt.MultipleLocator(5))
    plt.xticks(fontsize=5)
    plt.yticks(fontsize=5)
    plt.legend(loc="best", fontsize=8)
    plt.show()