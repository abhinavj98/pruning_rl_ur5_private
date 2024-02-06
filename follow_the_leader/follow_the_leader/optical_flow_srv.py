#!/usr/bin/env python3
import numpy as np
import rclpy
from follow_the_leader.utils.ros_utils import TFNode
from follow_the_leader.networks.raft import RAFT
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge
from follow_the_leader_msgs.srv import OpticalFlowAndMask
from rclpy.executors import MultiThreadedExecutor
import cv2
from skimage.draw import disk
bridge = CvBridge()
from threading import Lock
DEBUG = True
class OpticalFlowNode(TFNode):
    def __init__(self):
        super().__init__("optical_flow_node", cam_info_topic="/camera/color/camera_info")
        self.camera_topic_name = self.declare_parameter("camera_topic_name",
                                                        '/camera/color/image_raw')  # TODO: Change to Parameter.Type.STRING
        self.base_frame = self.declare_parameter("base_frame", "base_link")
        self.optical_flow = RAFT()
        self.goal = np.array([0, 0, 0])
        self.cb = MutuallyExclusiveCallbackGroup()#ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            Image,
            self.camera_topic_name.get_parameter_value().string_value,
            self.image_callback,
            1,
            callback_group=self.cb,
        )
        self.goal_sub = self.create_subscription(
            Point,
            "/goal",
            self.goal_callback,
            1,
            callback_group=self.cb,
        )
        self.optical_flow_and_mask_srv = self.create_service(OpticalFlowAndMask, "optical_flow_and_mask", self.optical_flow_and_mask_callback)

        self.last_image_msg = None
        self.image_msg = None

        self.height = 224
        self.width = 224
        self.lock = Lock()
        import time
        time.sleep(1)
        if DEBUG:
            self.of_pub = self.create_publisher(Image, "image_of", 10)
            self.mask_pub = self.create_publisher(Image, "image_mask", 10)

        return

    def goal_callback(self, msg: Point):
        self.goal = np.array([msg.x, msg.y, msg.z])
        return

    def get_images(self, imagemsg_current, imagemsg_previous):
        #RGB images
        image_current = bridge.imgmsg_to_cv2(imagemsg_current, desired_encoding="rgb8")/255.
        image_previous = bridge.imgmsg_to_cv2(imagemsg_previous, desired_encoding="rgb8")/255.
        return image_current, image_previous

    def image_callback(self, msg: Image):

        # self.lookup_transform('world_sim', 'camera_color_optical_frame')
        if self.last_image_msg is None:
            self.last_image_msg = msg
        self.image_msg = msg
        return

    def optical_flow_and_mask_callback(self, request, response):
        with self.lock:
            try:
                if self.last_image_msg is not None and self.image_msg is not None:
                    image_current, image_previous = self.get_images(self.image_msg, self.last_image_msg)
                    optical_flow = self.optical_flow.forward(image_current, image_previous)/10
                    point_mask = self.compute_deprojected_point_mask(self.height, self.width, self.goal)
                    print("Flow stats: min: {}, max: {}, mean: {}".format(np.min(optical_flow), np.max(optical_flow), np.mean(optical_flow)))
                    response.optical_flow = bridge.cv2_to_imgmsg(optical_flow, encoding="passthrough")
                    response.point_mask = bridge.cv2_to_imgmsg(point_mask, encoding="passthrough")
                    self.last_image_msg = self.image_msg

                    if DEBUG:
                        of_msg = bridge.cv2_to_imgmsg(np.moveaxis((optical_flow[1] - np.min(optical_flow[1]))/(np.max(optical_flow[1]) - np.min(optical_flow[1])), 1, -1), encoding="passthrough")
                        self.of_pub.publish(of_msg)
                        point_mask_msg = bridge.cv2_to_imgmsg(np.moveaxis(point_mask, 0, -1), encoding="passthrough")
                        self.mask_pub.publish(point_mask_msg)
                else:
                    self.get_logger().warn("No image received yet")
            except Exception as e:
                self.get_logger().error("Exception: {}".format(e))
        return response

    def compute_deprojected_point_mask(self, height, width, point):

        tf_base_cam = self.lookup_transform(
            self.base_frame.value, self.camera.tf_frame, rclpy.time.Time(), as_matrix=True
        )
        tf_cam_base = np.linalg.inv(tf_base_cam)
        cam_height = (240, 424)
        point_mask = np.zeros((cam_height[0], cam_height[1]), dtype=np.float32)
        if self.goal is not None:
            point = self.goal

            point = self.mul_homog(tf_cam_base, point)
            projection = self.camera.project3dToPixel(point)
            row = int(projection[1])
            row = cam_height[0] - row
            col = int(projection[0])
            col = cam_height[1] - col
            self.get_logger().info("Goal: {}, Projection: {}, Row: {}, Col: {}".format(self.goal, projection, row, col))
            if row < cam_height[0] and row > 0 and col < cam_height[1] and col > 0:
                radius = 5  # TODO: Make this a variable proportional to distance
                # modern scikit uses a tuple for center
                rr, cc = disk((row, col),   radius)
                point_mask[np.clip(0, rr, cam_height[0]-1) - 1, np.clip(0, cc,
                                                           cam_height[1]-1)-1] = 1  # TODO: This is a hack, numbers shouldnt exceed max and min anyways


        else:
            # log warning
            self.get_logger().warn("Goal not set")
        point_mask = cv2.resize(point_mask, dsize=(height, width))

        return np.expand_dims(point_mask,0)
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = OpticalFlowNode()
    rclpy.spin(node, executor=executor)
    return


if __name__ == "__main__":
    main()
