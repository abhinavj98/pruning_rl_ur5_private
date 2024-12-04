import os
import cv2
import numpy as np
import rosbag2_py
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
from cv_bridge import CvBridge

def extract_images_from_bag(bag_file_path, output_dir):
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    i = 0

    bridge = CvBridge()
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == "/camera/color/image_raw":
            img_msg = deserialize_message(data, Image)
            cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            timestamp_str = str(timestamp)
            img_filename = os.path.join(output_dir, f"image_{i}.png")
            cv2.imwrite(img_filename, cv_img)
            i+=1

bag_file_folder = '/home/grimmlins/bagfiles/zscan/*/*.db3'
# bag_file_path = '/home/grimmlins/rl_ws/jum1'
output_dir = '/home/grimmlins/zscan/images'
os.makedirs(output_dir, exist_ok=True)
import glob
for bag_file in glob.glob(bag_file_folder):
    print(f"Processing {bag_file}")
    bag_file_path = os.path.join(bag_file_folder, bag_file)
    out_file = os.path.join(output_dir, os.path.basename(bag_file).split('.')[0])
    os.makedirs(out_file, exist_ok=True)
    try:
        extract_images_from_bag(bag_file_path, out_file)
    except Exception as e:
        print(f"Error processing {bag_file}: {e}")
        continue
