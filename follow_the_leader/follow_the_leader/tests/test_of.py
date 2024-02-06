#Write a simple ros node to take in the optical flow and mask
#publish the message as a topic
import numpy as np

from rclpy.node import Node
from follow_the_leader_msgs.msg import OpticalFlowAndMask
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rclpy

bridge = CvBridge()
class TestOf(Node):
    def __init__(self):
        super().__init__('test_of')
        self.publisher_ = self.create_publisher(Image, '/test', 10)
        #subscribe to optical flow and mask
        self.sub = self.create_subscription(
            OpticalFlowAndMask,
            "/optical_flow_and_mask",
            self.of_callback,
            1,
        )

    def of_callback(self, msg: OpticalFlowAndMask):
        flow = bridge.imgmsg_to_cv2(msg.optical_flow, desired_encoding="passthrough")
        mask = bridge.imgmsg_to_cv2(msg.point_mask, desired_encoding="passthrough")
        flow = mask#flow[0]
        print(flow.shape)
        flow = np.moveaxis(flow, 0, -1)
        print(flow.shape)
        flow_msg = bridge.cv2_to_imgmsg(flow, encoding="passthrough")
        self.publisher_.publish(flow_msg)
        # self.get_logger().info('Publishing flow: {}'.format(msg.optical_flow))

def main(args=None):
    rclpy.init(args=args)
    test_of = TestOf()
    rclpy.spin(test_of)
    rclpy.shutdown()

if __name__ == '__main__':
    main()