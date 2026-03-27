import math
import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from pyproj import Proj, transform

# 定义参考点的经纬度和高度
ref_lat = 22.416178  # Reference latitude
ref_lon = 114.042743  # Reference longitude
ref_alt = 103.278  # Reference altitude

# 定义ENU到LLA的转换函数
def enu_to_geodetic(ref_lat, ref_lon, ref_alt, x, y, z):
    # 定义投影
    proj_lla = Proj(proj='latlong', datum='WGS84')
    proj_ecef = Proj(proj='geocent', datum='WGS84')

    # 将参考点的LLA坐标转换为ECEF坐标
    ref_ecef_x, ref_ecef_y, ref_ecef_z = transform(proj_lla, proj_ecef, ref_lon, ref_lat, ref_alt, radians=False)
    print("ref_ecef_x: ", ref_ecef_x, "ref_ecef_y: ", ref_ecef_y, "ref_ecef_z: ", ref_ecef_z)
    # 将ENU坐标转换为ECEF坐标
    ecef_x = ref_ecef_x - x*math.sin(math.radians(ref_lon)) - y * math.sin(math.radians(ref_lat))*math.cos(math.radians(ref_lon))+z*math.cos(math.radians(ref_lat))*math.cos(math.radians(ref_lon))
    ecef_y = ref_ecef_y + x*math.cos(math.radians(ref_lon)) - y * math.sin(math.radians(ref_lat))* math.sin(math.radians(ref_lon)) +z*math.cos(math.radians(ref_lat))*math.sin(math.radians(ref_lon))
    ecef_z = ref_ecef_z + y*math.cos(math.radians(ref_lat)) + z*math.sin(math.radians(ref_lat))

    # 将ECEF坐标转换为LLA坐标
    lon, lat, alt = transform(proj_ecef, proj_lla, ecef_x, ecef_y, ecef_z, radians=False)

    return lat, lon, alt

# Example usage
l_to_sqrt_2 = 10/math.sqrt(2)
# List of ENU coordinates
usv_enu_coordinates = [
    (0, 0, 0),
    (1*l_to_sqrt_2, 1*l_to_sqrt_2, 0),
    (2*l_to_sqrt_2, 0, 0),
    (3*l_to_sqrt_2, 1*l_to_sqrt_2, 0),
    (4*l_to_sqrt_2, 0, 0),
    (5*l_to_sqrt_2, 1*l_to_sqrt_2, 0)
]

# Convert each ENU coordinate to geodetic coordinate
usv_geodetic_coordinates = [enu_to_geodetic(ref_lat, ref_lon, ref_alt, x, y, z) for x, y, z in usv_enu_coordinates]

obs_enu_coordinates = [
    (1*l_to_sqrt_2, 0, 0),
    (2*l_to_sqrt_2, 1*l_to_sqrt_2, 0),
    (3*l_to_sqrt_2, 0, 0),
    (4*l_to_sqrt_2, 1*l_to_sqrt_2, 0),
    (5*l_to_sqrt_2, 0, 0)
]

obs_geodetic_coordinates = [enu_to_geodetic(ref_lat, ref_lon, ref_alt, x, y, z) for x, y, z in obs_enu_coordinates]
# Save obs_geodetic_coordinates to a text file in the specified format
with open('/home/garronliu/ardupilot/Rover/obs_geodetic_coordinates.txt', 'w') as file:
    file.write("QGC WPL 110\n")
    for i, (lat, lon, alt) in enumerate(usv_geodetic_coordinates):
        file.write(f"{i}\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t{lat:.6f}\t{lon:.6f}\t{alt:.6f}\t1\n")

for coord in usv_geodetic_coordinates:
    print(f"Latitude: {coord[0]}, Longitude: {coord[1]}, Altitude: {coord[2]}")

import matplotlib.pyplot as plt

# Extract latitudes and longitudes
lats_usv = [coord[0] for coord in usv_geodetic_coordinates]
lons_usv = [coord[1] for coord in usv_geodetic_coordinates]

lats_obs = [coord[0] for coord in obs_geodetic_coordinates]
lons_obs = [coord[1] for coord in obs_geodetic_coordinates]

# Plot the coordinates
plt.figure(figsize=(10, 6))
plt.plot(lons_usv, lats_usv, 'bo-', markersize=5)
plt.plot(lons_obs, lats_obs, 'ro-.', markersize=5)
plt.plot(lons_usv[0], lats_usv[0], 'b*', markersize=15, label='Starting Point')
plt.plot(lons_obs[0], lats_obs[0], 'r*', markersize=15, label='Starting Point')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Geodetic Coordinates')
plt.grid(True)
plt.show()
# Mark the starting point with a star


def create_waypoint(lat, lon, alt):
    waypoint = Waypoint()
    waypoint.frame = Waypoint.FRAME_GLOBAL
    # waypoint.command = Waypoint.NAV_WAYPOINT
    waypoint.is_current = False
    waypoint.autocontinue = True
    waypoint.param1 = 0
    waypoint.param2 = 0
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = alt
    return waypoint

def publish_waypoints():
    rospy.init_node('waypoint_publisher', anonymous=True)
    waypoint_pub = rospy.Publisher('/mavros/mission/waypoints', WaypointList, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    waypoint_list = WaypointList()
    waypoint_list.waypoints = [create_waypoint(lat, lon, alt) for lat, lon, alt in usv_geodetic_coordinates]

    while not rospy.is_shutdown():
        waypoint_pub.publish(waypoint_list)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass