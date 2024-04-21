#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import os

rospy.init_node('data_saver', anonymous=True)

lidar_dir = "/home/phucvinh/process_point/data/clouds/"
image_dir = "/home/phucvinh/process_point/data/images/"

if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)

if not os.path.exists(image_dir):
    os.makedirs(image_dir)

bridge = CvBridge()

lidar_count = 0
image_count = 0

lidar_msg = None
image_msg = None

def pad_zeros(val, num_digits=5):
    return f"{val:0{num_digits}d}"

def save_point_cloud(lidar_msg, filename):
    with open(filename, 'w') as f:
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {lidar_msg.width}\n")
        f.write(f"HEIGHT {lidar_msg.height}\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(list(pc2.read_points(lidar_msg, skip_nans=True)))}\n")
        f.write("DATA ascii\n")
        for p in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True):
            f.write(f"{' '.join(str(value) for value in p)}\n")

def save_data():
    global lidar_msg, image_msg, lidar_count, image_count
    if lidar_msg and image_msg:
        # Lưu dữ liệu
        lidar_filename = f"{lidar_count}.pcd"
        
        pcd_color_file = lidar_dir + pad_zeros(lidar_count) + ".pcd"
        save_point_cloud(lidar_msg, pcd_color_file)
        lidar_count += 1

        image_filename = f"{image_count}.png"
        pcd_color_file = image_dir + pad_zeros(image_count) + ".png"
        image_path = os.path.join(image_dir, image_filename)
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv2.imwrite(image_path, cv_image)
        image_count += 1

        rospy.loginfo(f"Saved {lidar_path} and {image_path}")

        lidar_msg = None
        image_msg = None

def handle_lidar_msg(msg):
    global lidar_msg
    lidar_msg = msg

def handle_image_msg(msg):
    global image_msg
    image_msg = msg

# Định nghĩa các subscriber
lidar_sub = rospy.Subscriber('/points_raw', PointCloud2, handle_lidar_msg)
image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, handle_image_msg)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    save_data()
    rate.sleep()
