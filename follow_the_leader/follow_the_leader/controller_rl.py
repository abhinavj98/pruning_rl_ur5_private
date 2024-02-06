import time
#add pruninb_sb3 to python path
import sys
# sys.path.append("/home/abhinav/PycharmProjects/ur5_pruning_control/src/follow_the_leader/")
import pruning_sb3
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistStamped, Vector3, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
import numpy as np
# from follow_the_leader_msgs.msg import OpticalFlowAndMask
from follow_the_leader_msgs.srv import Jacobian, OpticalFlowAndMask
from follow_the_leader.utils.ros_utils import TFNode
from cv_bridge import CvBridge
# import follow_the_leader.networks.pruning_sb3 as pruning_sb3
import torch as th
from pruning_sb3.algo.PPOLSTMAE.ppo_recurrent_ae import RecurrentPPOAE
from collections import namedtuple
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Lock
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import os
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
# from follow_the_leader.utils.run_experiments import move_to
MODEL_WEIGHTS_PATH = os.path.join(os.path.expanduser("~"), "weights", "rl_controller_weights", "ik_virl_test/" )#"/home/abhinav/Desktop/weights/rl_controller_weights/"
# PRUNING_SB3_PATH = "
Joint = namedtuple("Joint", ["angle", "velocity"])

bridge = CvBridge()

class JointInfo():
    def __init__(self):
        self.control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                               "wrist_2_joint", "wrist_3_joint"]
        self.joint_angle_vel = {joint: Joint(None, None) for joint in self.control_joints}

    def update(self, msg: JointState):
        # print("updating joint info")

        for i, joint in enumerate(msg.name):
            self.joint_angle_vel[joint] = Joint(msg.position[i], msg.velocity[i])

    def get_joint_angles_ordered(self):
        return np.array([self.joint_angle_vel[joint].angle for joint in self.joint_angle_vel.keys()])

    def get_joint_velocities_ordered(self):
        return np.array([self.joint_angle_vel[joint].velocity for joint in self.joint_angle_vel.keys()])


class RLController(TFNode):
    def __init__(self):
        #Needs multi threaded executor
        super().__init__('rl_controller')
        self.goal_sim = np.array([0.0, 0.0, 0.0])
        # self.action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.action_sim = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.action_world = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.model = None
        self.lstm_states = None  # initially LSTM states are none TODO: Set to None with reset
        self.init_pos_sim = np.array([0.0, 0.0, 0.0])
        # self.depth_proxy = None

        self.cb = ReentrantCallbackGroup()
        self.action_scale = 0.3
        self.observation = dict()
        self.joint_states_ordered = JointInfo()
        self.create_subscription(
            Point,
            "/goal",
            self.goal_callback,
            1,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            1,
        )
        # self.create_subscription(
        #     OpticalFlowAndMask,
        #     "/optical_flow_and_mask",
        #     self.of_callback,
        #     1,
        # )
        self.lock = Lock()
        self.sub_node = rclpy.create_node('sub_node_rl')
        self.vel_publisher_ = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 1)
        #arrow marker publisher
        self.marker_pub = self.create_publisher(Marker, "/vel_marker", 1)
        self.endpoint_marker = self.create_publisher(Marker, "/endpoint_marker", 1)
        self.reset_srv = self.create_service(Trigger, 'reset_controller_srv', self.reset_srv, callback_group=self.cb)
        self.set_vel_timer = self.create_timer(50./240., self.set_vel)
        self.endpoint_timer = self.create_timer(1., self.publish_endpoint, callback_group=self.cb)
        self.pub_vel_timer = self.create_timer(1/50., self.publish_vel, callback_group=self.cb)
        self.jacobian_client = self.sub_node.create_client(Jacobian, "/get_jacobian")
        self.of_and_mask_client = self.sub_node.create_client(OpticalFlowAndMask, "/optical_flow_and_mask")
        while not self.jacobian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Jacobian service not available, waiting again...')
        while not self.of_and_mask_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('OF service not available, waiting again...')

        self.init_model(MODEL_WEIGHTS_PATH + "best_model.zip", MODEL_WEIGHTS_PATH + "mean_std.pkl")


    def reset_srv(self, request, response):
        self.reset()
        print("Resetting controller")
        response.success = True
        response.message = "Resetting controller"
        return response
    def reset(self):
        #get ee pose
        tf_ee_sim = self.lookup_transform("world_sim", "endpoint")
        self.lstm_states = None
        sim_init_pos = np.array([tf_ee_sim.transform.translation.x, tf_ee_sim.transform.translation.y, tf_ee_sim.transform.translation.z])
        self.init_pos_sim = sim_init_pos
        with self.lock:
            self.depth_proxy = None
        self.goal_sim = np.array([0.0, 0.0, 0.0])
        self.observation = dict()
    def joint_states_callback(self, msg: JointState):
        #Order of these joitns?
        self.joint_states_ordered.update(msg)
        return

    # def of_callback(self, msg: OpticalFlowAndMask):
    #     flow = bridge.imgmsg_to_cv2(msg.optical_flow, desired_encoding="passthrough")
    #     mask = bridge.imgmsg_to_cv2(msg.point_mask, desired_encoding="passthrough")
    #     # self.get_logger().info("Shape of flow: {}, shape of mask: {}".format(flow.shape, mask.shape))
    #     self.depth_proxy = np.concatenate((flow, mask))

    def get_observation(self):
        """
        Make the state for the RL algorithm
        Use locks
        :return:
        """

        """
        # Actual observation
        self.observation['achieved_goal'] = achieved_pos - init_pos
        self.observation['desired_goal'] = desired_pos - init_pos
        self.observation['relative_distance'] = achieved_pos - desired_pos
        # Convert orientation into 6D form for continuity
        self.observation['achieved_or'] = achieved_or_6d
        self.observation['depth_proxy'] = depth_proxy
        # Convert joint angles to sin and cos
        self.observation['joint_angles'] = encoded_joint_angles
        Action actually achieved

        self.observation['prev_action'] = np.hstack((achieved_vel, achieved_ang_vel))

        """
        # print("Getting observation")
        tf_ee_sim = self.lookup_transform("world_sim", "endpoint", as_matrix=True)

        #Target Source
        # print("got tf")
        # achieved_vel, achieved_ang_vel = self.lookup_twist("base_link", "tool0")
        # The order of keys is important

        achieved_sim = tf_ee_sim[:3, 3].reshape(-1)
        sim_or = tf_ee_sim[:3, :3]
        self.observation['achieved_goal'] = achieved_sim - self.init_pos_sim
        self.observation['achieved_or'] = sim_or[:3, :2].reshape(6, )
        self.observation['desired_goal'] = self.goal_sim - self.init_pos_sim
        joint_angles = self.joint_states_ordered.get_joint_angles_ordered()
        if joint_angles[0] is None:
            return None
        self.observation['joint_angles'] = np.hstack((np.sin(joint_angles), np.cos(joint_angles)))
        ee_vel = self.get_ee_vel()
        print(ee_vel)
        if ee_vel is None:
            return None
        ee_vel = self.action_world_to_sim(ee_vel.reshape(6, ))
        self.observation['prev_action'] = ee_vel
        # print(self.observation['prev_action'])
        # print(self.observation['prev_action'])
        self.observation['relative_distance'] = achieved_sim - self.goal_sim
        print("waitinng for of")
        with self.lock:
            depth_proxy = self.get_depth_proxy()
            print("got of")
            self.observation['depth_proxy'] = depth_proxy
        if self.observation['depth_proxy'] is None:
            return None

        # achieved_vel = np.zeros(3)
        # achieved_ang_vel = np.zeros(3)
        # print(self.init_pos_sim)

        return self.observation

    def get_ee_vel(self):
        #get jacobian
        # print("Getting ee vel")
        request = Jacobian.Request()

        # Call the service
        future = self.jacobian_client.call_async(request)
        rclpy.spin_until_future_complete(self.sub_node, future)

        if future.result() is not None:
            response = future.result()
            # print(response.jacobian)
            # self.get_logger().info('Received response: ', response.jacobian)
        else:
            self.get_logger().error('Service call failed: %r' % (future.exception(),))
        jacobian_mat = np.array(response.jacobian).reshape(6, 6).T #Transpose needed as eigen is column major

        # print(jacobian_mat)
        # get joint velocities
        joint_vel = self.joint_states_ordered.get_joint_velocities_ordered()
        if joint_vel[0] is None:
            return None
        #convert to np array
        joint_vel = np.array(joint_vel).reshape(6, 1)
        end_effector_velocity = np.matmul(jacobian_mat, joint_vel)
        return end_effector_velocity

    def get_depth_proxy(self):
        future = self.of_and_mask_client.call_async(OpticalFlowAndMask.Request())
        rclpy.spin_until_future_complete(self.sub_node, future)
        if future.result() is not None:
            response = future.result()
            # print(response.jacobian)
            # self.get_logger().info('Received response: ', response.jacobian)
        else:
            self.get_logger().error('Service call failed: %r' % (future.exception(),))
        flow = bridge.imgmsg_to_cv2(response.optical_flow, desired_encoding="passthrough")
        mask = bridge.imgmsg_to_cv2(response.point_mask, desired_encoding="passthrough")
        # self.get_logger().info("Shape of flow: {}, shape of mask: {}".format(flow.shape, mask.shape))
        depth_proxy = np.concatenate((flow, mask))
        return depth_proxy

    def init_model(self, load_path_model, load_path_mean_std):
        device = "cuda" if th.cuda.is_available() else "cpu"
        assert load_path_mean_std
        assert load_path_model

        self.model = RecurrentPPOAE.load(load_path_model, print_system_info=True)
        self.model.policy.load_running_mean_std_from_file(load_path_mean_std)
        self.model.policy.to(device)
        print(device)
        print("Model loaded")

    def action_as_twist(self, action):
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear = Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2]))
        twist.twist.angular = Vector3(x=float(action[3]), y=float(action[4]), z=float(action[5]))
        return twist

    def set_vel(self):
        tf_ee_sim = self.lookup_transform("world_sim", "endpoint", as_matrix=True)

        self.model.policy.set_training_mode(False)

        observations = self.get_observation()
        # print("Observations", observations)
        if observations is None:
            self.action_sim = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            print("Observation not set")
            return
        # print(observations)
        if observations['depth_proxy'] is None:
            self.action_sim = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            print("Depth proxy not set")
            return
        # print(self.goal_sim, self.init_pos_sim, tf_ee_sim[:3,3])
        print("making action prediction")
        action_sim, self.lstm_states = self.model.predict(
            observations,  # type: ignore[arg-type]
            state=self.lstm_states,
            episode_start=True,
            deterministic=True,
        )
        print("made")
        print(action_sim)
        self.action_sim = np.array(action_sim) * self.action_scale  # This action is ee velocity
        self.action_world = self.action_sim_to_world(self.action_sim)
        # self.depth_proxy = None
        #Set arrow using Marker
        marker = Marker()
        marker.header.frame_id = 'world_sim'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        start = Point()
        start.x = tf_ee_sim[:3,3][0]
        start.y = tf_ee_sim[:3,3][1]
        start.z = tf_ee_sim[:3,3][2]

        end = Point()
        action = self.action_world
        end.x = tf_ee_sim[:3,3][0] + action[0]
        end.y = tf_ee_sim[:3,3][1] + action[1]
        end.z = tf_ee_sim[:3,3][2] + action[2]
        marker.points = [start, end]
        #add start


        #publish marker
        self.marker_pub.publish(marker)
        # print(self.action_sim, self.goal_sim, tf_ee_sim[:3,3])#, self, self.observation['desired_goal'])

    def action_sim_to_world(self, action_sim):
        scale = 10.
        action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        action[0] = action_sim[0]*scale
        action[1] = action_sim[1]*scale
        action[2] = action_sim[2]*scale
        action[3] = action_sim[3]*scale
        action[4] = action_sim[4]*scale
        action[5] = action_sim[5]*scale
        return action

    def action_world_to_sim(self, action_world):
        scale = 10.
        action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        action[0] = action_world[0]/scale
        action[1] = action_world[1]/scale
        action[2] = action_world[2]/scale
        action[3] = action_world[3]/(scale)
        action[4] = action_world[4]/(scale)
        action[5] = action_world[5]/(scale)
        return action
    def publish_vel(self):
        if self.goal_sim is None:
            self.get_logger().info("Goal not set")
            return
        # action = self.action_sim_to_world(self.action_sim)
        action = self.action_world
        twist = self.action_as_twist(action)
        self.vel_publisher_.publish(twist)
        return

    def publish_endpoint(self):
        tf_ee_sim = self.lookup_transform("base_link", "endpoint", as_matrix=True)
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        start = Point()
        start.x = tf_ee_sim[:3,3][0]
        start.y = tf_ee_sim[:3,3][1]
        start.z = tf_ee_sim[:3,3][2]

        marker.pose.position = start
        self.endpoint_marker.publish(marker)
        return
    def goal_callback(self, msg: Point):
        goal_world = np.array([msg.x, msg.y, msg.z, 1])
        tf_base_sim = self.lookup_transform("world_sim", "base_link", as_matrix=True)
        self.goal_sim = (tf_base_sim@goal_world)[:3]
        return

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RLController()
    rclpy.spin(node, executor=executor)
    return


if __name__ == "__main__":
    main()