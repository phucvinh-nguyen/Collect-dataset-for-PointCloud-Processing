#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os

# Khởi tạo node
rospy.init_node('lidar_data_collector', anonymous=True)

# Đường dẫn thư mục để lưu
lidar_dir = "/home/phucvinh/process_point/data/clouds/"
if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)

# Biến đếm cho tên file
lidar_count = 0
def pad_zeros(val, num_digits=5):
    return f"{val:0{num_digits}d}"

def save_point_cloud(pc_data, filename):
    with open(filename, 'w') as f:
        f.write("VERSION .7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {pc_data.width}\n")
        f.write(f"HEIGHT {pc_data.height}\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {pc_data.width * pc_data.height}\n")
        f.write("DATA ascii\n")
        for p in pc2.read_points(pc_data, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            f.write(f"{' '.join(str(value) for value in p)}\n")

def lidar_callback(lidar_msg):
    global lidar_count
    # Lưu PointCloud2
    pcd_file = lidar_dir + pad_zeros(lidar_count) + ".pcd"
    lidar_path = os.path.join(lidar_dir, pcd_file)
    save_point_cloud(lidar_msg, lidar_path)
    lidar_count += 1

    rospy.loginfo(f"Saved {lidar_path}")

# Định nghĩa subscriber cho LiDAR
lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback, queue_size=3000)

rospy.spin()