#!/usr/bin/env python3
import rclpy
from aruco.utils.ros_utils import TFNode
from sensor_msgs.msg import Image
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import TransformStamped, Vector3
bridge = CvBridge()
from scipy.spatial.transform import Rotation
from rclpy.executors import MultiThreadedExecutor
from aruco_msgs.srv import ArucoTransform
class ArucoGoalService(TFNode):
    def __init__(self):
        super().__init__("aruco_goal_service", cam_info_topic="/camera/color/camera_info")
        self.camera_topic_name = self.declare_parameter("camera_topic_name",
                                                        '/camera/color/image_raw')
        self.base_frame = self.declare_parameter("base_frame", "base_link")
        self.goal = np.array([0, 0, 0])
        self.cb = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            Image,
            self.camera_topic_name.get_parameter_value().string_value,
            self.image_callback,
            1,
            callback_group=self.cb,
        )
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        self.image = None
        self.aruco_goal_srv = self.create_service(ArucoTransform, "aruco_goal", self.aruco_goal_callback)
        self.cameraMatrixParam = np.load('camera_calibration.npz')
        self.cameraMatrix = self.cameraMatrixParam['mtx']
        self.distCoeffs = self.cameraMatrixParam['dist']
        self.marker_size = 0.08

    def image_callback(self, msg: Image):
        image = bridge.imgmsg_to_cv2(msg, "rgb8")
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        self.image = gray

    def aruco_goal_callback(self, request, response):
        print("aruco_goal_callback")
        if self.image is None:
            response.success = False
            return response
        corners, ids, _ = self.detector.detectMarkers(self.image)
        if ids is None or len(ids) != 2:
            response.success = False
            return response
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.cameraMatrix, self.distCoeffs)
        transformation_matrix = np.eye(4)
        transformation_matrix = np.tile(transformation_matrix, (2, 1, 1))
        print(transformation_matrix.shape)
        for i in range(len(ids)):
            rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
            # Create a 4x4 transformation matrix
            transformation_matrix[i, :3, :3] = rotation_matrix
            transformation_matrix[i, :3, 3] = tvecs[i].flatten()

        print(transformation_matrix)
        goal_transformation_mat_aruco = np.mean(transformation_matrix, axis=0)
        print("fettubg tg")
        tbl_af = self.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time(), as_matrix=True)
        goal_transformation_mat_base_link = tbl_af@goal_transformation_mat_aruco
        response.transform.header.frame_id = "base_link"
        response.transform.child_frame_id = "aruco"

        response.transform.transform.translation.x = goal_transformation_mat_base_link[0, 3]
        response.transform.transform.translation.y = goal_transformation_mat_base_link[1, 3]
        response.transform.transform.translation.z = goal_transformation_mat_base_link[2, 3]
        #convert rotation matrix to quaternion
        r_m = Rotation.from_matrix(goal_transformation_mat_base_link[:3, :3])
        quaternion = r_m.as_quat()
        response.transform.transform.rotation.x = quaternion[0]
        response.transform.transform.rotation.y = quaternion[1]
        response.transform.transform.rotation.z = quaternion[2]
        response.transform.transform.rotation.w = quaternion[3]
        response.success = True
        print("aruco_goal_callback done")
        return response

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ArucoGoalService()
    rclpy.spin(node, executor=executor)
    return


if __name__ == "__main__":
    main()
