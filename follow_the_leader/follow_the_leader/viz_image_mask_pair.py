#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from follow_the_leader_msgs.msg import ImageMaskPair
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import matplotlib.pyplot as plt
bridge = CvBridge()
class VizImageMask(Node):
    def __init__(self):
        super().__init__("viz_image_mask")

        self.sub = self.create_subscription(
            ImageMaskPair,
            "image_mask_pair",
            self.image_mask_callback,
            1,
        )
        self.pub_mask = self.create_publisher(Image, "image_mask_only", 10)
        return

    def image_mask_callback(self, msg):
        img_in_cv2 = bridge.imgmsg_to_cv2(
            msg.mask, desired_encoding='passthrough')
        backtorgb = cv2.cvtColor(img_in_cv2, cv2.COLOR_GRAY2RGB)
        # print(img_in_cv2.shape)
        # plt.imshow(img_in_cv2, cmap='gray')
        # plt.draw()
        # plt.pause(0.00001)
        # cv2.imshow("mask", img_in_cv2)
        # cv2.waitKey(100)
        self.pub_mask.publish(bridge.cv2_to_imgmsg(backtorgb, "bgr8"))
        return


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = VizImageMask()
    rclpy.spin(node, executor=executor)
    return


if __name__ == "__main__":
    main()
