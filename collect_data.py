#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import os


rospy.init_node('data_collecter', anonymous=True)

lidar_dir = "lidar_data"
image_dir = "image_data"

if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)
    
if not os.path.exists(image_dir):
    os.makedirs(image_dir)

bridge = CvBridge()

lidar_count = 0
image_count = 0

def save_point_cloud(pc_data, filename):
    with open(filename, 'w') as f:
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {pc_data.width}\n")
        f.write(f"HEIGHT {pc_data.height}\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {pc_data.width * pc_data.height}\n")
        f.write("DATA ascii\n")
        for p in pc2.read_points(pc_data, field_names=("x", "y", "z"), skip_nans=True):
            f.write(f"{' '.join(str(value) for value in p)}\n")

def callback(lidar_msg, image_msg):
    global lidar_count, image_count

    # save PointCloud2
    lidar_filename = f"{lidar_count}.pcd"
    lidar_path = os.path.join(lidar_dir, lidar_filename)
    save_point_cloud(lidar_msg, lidar_path)
    lidar_count += 1

    # save image
    image_filename = f"{image_count}.png"
    image_path = os.path.join(image_dir, image_filename)
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    cv2.imwrite(image_path, cv_image)
    image_count += 1

    rospy.loginfo(f"Saved {lidar_path} and {image_path}")


lidar_sub = message_filters.Subscriber('/lidar_points', PointCloud2)
image_sub = message_filters.Subscriber('/camera_image', Image)

# synchronization
ts = message_filters.TimeSynchronizer([lidar_sub, image_sub], 10)
ts.registerCallback(callback)

rospy.spin()
