#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from livox_ros_driver.msg import CustomMsg
from cv_bridge import CvBridge
import cv2
import os

# Khởi tạo node
rospy.init_node('data_saver', anonymous=True)

# Đường dẫn thư mục để lưu
lidar_dir = "/home/phucvinh/process_point/data/clouds/"
image_dir = "/home/phucvinh/process_point/data/images/"

if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)
    
if not os.path.exists(image_dir):
    os.makedirs(image_dir)

bridge = CvBridge()

# Biến đếm cho tên file
lidar_count = 0
image_count = 0

# Biến lưu trữ dữ liệu
lidar_msg = None
image_msg = None

def save_custom_lidar_data(lidar_msg, filename):
    with open(filename, 'w') as f:
        f.write("VERSION .7\n")
        f.write("FIELDS x y z reflectivity\n")
        f.write("SIZE 4 4 4 4\n")  # Cập nhật kích thước của reflectivity
        f.write("TYPE F F F F\n")  # Đổi kiểu dữ liệu thành float
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(lidar_msg.points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(lidar_msg.points)}\n")
        f.write("DATA ascii\n")
        for p in lidar_msg.points:
            f.write(f"{p.x} {p.y} {p.z} {float(p.reflectivity)}\n")

def save_data():
    global lidar_msg, image_msg, lidar_count, image_count
    if lidar_msg and image_msg:
        # Lưu dữ liệu LiDAR
        lidar_filename = f"{lidar_count}.pcd"
        lidar_path = os.path.join(lidar_dir, lidar_filename)
        save_custom_lidar_data(lidar_msg, lidar_path)
        lidar_count += 1

        # Chuyển đổi và lưu Image
        image_filename = f"{image_count}.png"
        image_path = os.path.join(image_dir, image_filename)
        cv_image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv2.imwrite(image_path, cv_image)
        image_count += 1

        rospy.loginfo(f"Saved {lidar_path} and {image_path}")

        # Reset để đợi cặp dữ liệu tiếp theo
        lidar_msg = None
        image_msg = None

def handle_lidar_msg(msg):
    global lidar_msg
    lidar_msg = msg

def handle_image_msg(msg):
    global image_msg
    image_msg = msg

# Định nghĩa các subscriber
lidar_sub = rospy.Subscriber('/livox/lidar', CustomMsg, handle_lidar_msg)
image_sub = rospy.Subscriber('/right_camera/image/compressed', CompressedImage, handle_image_msg)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    save_data()
    rate.sleep()
