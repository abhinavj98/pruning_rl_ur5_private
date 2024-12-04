import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
import rosbag2_py
from rclpy.serialization import deserialize_message

class TFExtractor(Node):
    def __init__(self):
        super().__init__('tf_extractor')
        self.buffer = Buffer()
        self.transforms = []
        self.messages = []

    def read_bag_and_set_tf(self, bag_path):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader.open(storage_options, converter_options)

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic in ['/tf', '/tf_static']:
                tf_msg = deserialize_message(data, TFMessage)
                for msg_tf in tf_msg.transforms:
                    if topic == '/tf_static':
                        self.buffer.set_transform_static(msg_tf, 'default_authority')
                    else:
                        self.buffer.set_transform(msg_tf, 'default_authority')
                self.lookup_and_save_transforms('base_link', 'tool0', 'transforms.txt')
                print(f"Processed {topic} at {t}")
        return self.messages

    def lookup_and_save_transforms(self, base_frame, end_effector_frame, output_file):
        try:
            trans = self.buffer.lookup_transform(base_frame, end_effector_frame, rclpy.time.Time(seconds=0))
            self.transforms.append(trans)
            print(f"Translation {trans.transform.translation}")
            print(f"Timestamp {trans.header})")
            self.messages.append(trans)
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {str(e)}')
            

import pandas as pd
import glob, os
def main(args=None):
    bag_file_folder = '/home/grimmlins/bagfiles/zscan/*/*.db3'

    bag_path = '/home/grimmlins/bagfiles/zscan/2feb1/2feb1_0.db3'  # Update this to the path of your ROS2 bag file
    output_file = 'transforms.txt'  # Output file to save the transforms

    for bag_file in glob.glob(bag_file_folder):
        print(f"Processing {bag_file}")
        
        rclpy.init(args=args)
        node = TFExtractor()
        messages = node.read_bag_and_set_tf(bag_path)
   
        df_data = {
            'Timestamp': [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 for msg in messages],
            'Translation_X': [msg.transform.translation.x for msg in messages],
            'Translation_Y': [msg.transform.translation.y for msg in messages],
            'Translation_Z': [msg.transform.translation.z for msg in messages],
            'Rotation_X': [msg.transform.rotation.x for msg in messages],
            'Rotation_Y': [msg.transform.rotation.y for msg in messages],
            'Rotation_Z': [msg.transform.rotation.z for msg in messages],
            'Rotation_W': [msg.transform.rotation.w for msg in messages]
        }
        df = pd.DataFrame(df_data)
        filename = os.path.basename(bag_file).split('.')[0]
        filename_wo_ext = os.path.splitext(filename)[0]
        output_file = f"ee_pose_{filename_wo_ext}.csv"
        df.to_csv(output_file, index=False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
