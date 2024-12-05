import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Point, TwistStamped
from cv_bridge import CvBridge
import numpy as np
import py_trees
import py_trees_ros
import torch as th
from collections import namedtuple

Joint = namedtuple("Joint", ["angle", "velocity"])
Observation = namedtuple("Observation", ["achieved_goal", "achieved_or", "desired_goal", "joint_angles", 
                                         "prev_action_achieved", "relative_distance", "rgb", "prev_rgb"])
class JointInfo():
    def __init__(self):
        self.control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                               "wrist_2_joint", "wrist_3_joint"]
        self.joint_angle_vel = {joint: Joint(None, None) for joint in self.control_joints}

    def update(self, msg: JointState):
        for i, joint in enumerate(msg.name):
            self.joint_angle_vel[joint] = Joint(msg.position[i], msg.velocity[i])

    def get_joint_angles_ordered(self):
        #Collect the joint angles in the order of the control joints
        return np.array([self.joint_angle_vel[joint].angle for joint in self.joint_angle_vel.keys()])

    def get_joint_velocities_ordered(self):
        #Collect the joint velocities in the order of the control joints
        return np.array([self.joint_angle_vel[joint].velocity for joint in self.joint_angle_vel.keys()])

class DataGatheringNode(Node):
    """Node to handle concurrent data gathering and update the py_trees blackboard.
    Use ROS2 with reentrant callback group to handle concurrent data gathering and
    update the py_trees blackboard."""

    def __init__(self):
        super().__init__('data_gathering_node')

        # Blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        self.joint_states = JointInfo()
        self.blackboard.joint_angles = None
        self.blackboard.camera_image = None
        self.blackboard.goal = None

        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            Image, '/camera/image_raw', self.camera_image_callback, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            Point, '/goal', self.goal_callback, 10, callback_group=self.callback_group
        )

    def joint_states_callback(self, msg: JointState):
        self.joint_states.update(msg)
        self.blackboard.joint_angles = self.joint_states.get_joint_angles_ordered()
        self.blackboard.joint_velocities = self.joint_states.get_joint_velocities_ordered()
        self.get_logger().info("Updated joint_states on blackboard.")

    def camera_image_callback(self, msg: Image):
        self.blackboard.last_camera_image = self.blackboard.camera_image
        self.blackboard.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.get_logger().info("Updated camera_image on blackboard.")
        
    def goal_callback(self, msg: Point):
        self.blackboard.goal = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info("Updated goal on blackboard.")


class AggregateObservation(py_trees.behaviours.Behaviour):
    """Aggregate sensor data into an observation."""

    def __init__(self, name, node, pretend_action_scale=1.0):
        """
        :param node: The ROS2 node to use for tf2 buffer and transform listener.
        :param target_frame: The frame to transform into (e.g., "base_link").
        :param source_frame: The frame to transform from (e.g., "end_effector").
        """
        super().__init__(name)
        self.node = node
        self.tf_buffer = Buffer()  # TF2 Buffer for storing transforms
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.pretend_action_scale = pretend_action_scale

    def lookup_transform(self, target_frame, source_frame, time=None, sync=True):
        """
        Look up a transform between frames, with optional matrix conversion.
        We convert the source frame to the target frame.
        To be even more clear, we are looking at the source frame from the target frame.
        To look at frame B with respect to frame A (T_AB), A is the target frame and B is the source frame.
        """
        if time is None:
            time = rclpy.time.Time()

        if sync:
            future = self.tf_buffer.wait_for_transform_async(target_frame, source_frame, time)
            rclpy.spin_until_future_complete(self.node, future)

        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, time)
        
        # Convert to a 4x4 transformation matrix
        tl = tf.transform.translation
        q = tf.transform.rotation
        mat = np.identity(4)
        mat[:3, 3] = [tl.x, tl.y, tl.z]
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.logger.info(f"Transform from {source_frame} to {target_frame}:\n{mat}")
        return mat

    def get_current_pose(self):
        """
        Get the current position of the robot's end effector.
        Return x, y, z position and orientation as a 3x3 rotation matrix.
        """
        tf_fb_end = self.lookup_transform(target_frame='fake_base', source_frame='endpoint') #Looking at endpoint from fake_base
        pos_fb_end = tf_fb_end[:3, 3]
        orientation_fb_end = tf_fb_end[:3, :3]
        return pos_fb_end, orientation_fb_end
    
    def transform_goal_to_fake_base(self, goal):
        """
        Transform the goal position to the fake base frame.
        """
        tf_fb_goal = self.lookup_transform(target_frame='fake_base', source_frame='base_link') #Looking at base_link from fake_base
        goal_fb = tf_fb_goal @ np.array([goal[0], goal[1], goal[2], 1]) #Transform goal to fake_base
        return goal_fb[:3]
    
    def encode_joint_angles(self, joint_angles):
        """
        Encode joint angles by using sin and cos.
        """
        return np.hstack([np.sin(joint_angles), np.cos(joint_angles)])
    
    def get_tool0_velocity(self):
        #Get jacobian matrix
        request = Jacobian.Request()
        future = self.jacobian_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is None:
            self.logger.warning("Service call failed.")
            return None
        else:
            response = future.result()
        jacobian = np.array(response.jacobian).reshape(6, 6)
        joint_velocities = self.blackboard.joint_velocities.reshape(6, 1)
        tool0_velocity = jacobian @ joint_velocities
        return tool0_velocity
    
    def pretend_action(self, action):
        """To overcome latency issues, we pretend the action has been executed
        slower but compensate for that by executing it longer."""
        return action * self.pretend_action_scale
    
    def actual_action_from_pretend(self, pretend_action):
        """Reverse the pretend action scaling."""
        return pretend_action / self.pretend_action_scale

    def update(self):
        if (
            self.blackboard.joint_states is None
            or self.blackboard.camera_image is None
            or self.blackboard.goal is None
        ):
            return py_trees.common.Status.RUNNING  # Wait for all data to be available, next tick will check again

        # Example observation aggregation logic
        observation = Observation()

        end_position, end_orientation = self.get_current_pose()
        fb_goal = self.transform_goal_to_fake_base(self.blackboard.goal)
        tool0_velocity = self.get_tool0_velocity()

        observation.achieved_goal = end_position - end_position_init
        observation.achieved_or = end_orientation[:3, :2].reshape(6, )
        observation.desired_goal = fb_goal - end_position_init
        observation.joint_angles = self.encode_joint_angles(self.blackboard.joint_states["angles"])
        observation.prev_action_achieved = self.pretend_action(tool0_velocity).reshape(6, )
        observation.relative_distance = end_position - fb_goal
        observation.rgb = self.blackboard.camera_image
        observation.prev_rgb = self.blackboard.last_camera_image
        
        # Update blackboard with aggregated observation
        self.blackboard.observation = observation._asdict()
        self.logger.info("Observation aggregated.")
        return py_trees.common.Status.SUCCESS


class ComputeRLAction(py_trees.behaviours.Behaviour):
    """Compute an action using RL logic."""

    def __init__(self, name, model_path):
        super().__init__(name)
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        self.model = self.init_rl_model(model_path)

    def init_rl_model(self, load_path):
        assert load_path is not None, "Model path not provided."
        custom_objects = {"n_envs": 1}
        self.model = RecurrentPPOAE.load(load_path, custom_objects=custom_objects)
        self.model.to(self.device)


    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        if "observation" not in blackboard:
            return py_trees.common.Status.FAILURE

        observation = blackboard.observation
        # Placeholder: Replace with actual RL model inference logic
        action = np.random.uniform(-1, 1, size=6)  # Mock action
        blackboard.action_world = action
        self.logger.info(f"Computed action: {action}")
        return py_trees.common.Status.SUCCESS


class PublishVelocity(py_trees.behaviours.Behaviour):
    """Publish velocity commands to the robot."""

    def __init__(self, name, publisher):
        super().__init__(name)
        self.publisher = publisher

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        if "action_world" not in blackboard:
            return py_trees.common.Status.FAILURE

        action = blackboard.action_world
        twist = TwistStamped()
        twist.twist.linear.x = action[0]
        twist.twist.linear.y = action[1]
        twist.twist.linear.z = action[2]
        twist.twist.angular.x = action[3]
        twist.twist.angular.y = action[4]
        twist.twist.angular.z = action[5]
        self.publisher.publish(twist)
        self.logger.info("Published velocity command.")
        return py_trees.common.Status.SUCCESS


def create_behavior_tree(publisher, model_path):
    """Construct the behavior tree."""
    root = py_trees.composites.Sequence("Root")

    # Subtree: Aggregate Observation
    aggregate_observation = AggregateObservation(name="Aggregate Observation")

    # Compute Action
    compute_action = ComputeRLAction(name="Compute Action", model_path=model_path)

    # Publish Velocity
    publish_velocity = PublishVelocity(name="Publish Velocity", publisher=publisher)

    # Add to the tree
    root.add_children([aggregate_observation, compute_action, publish_velocity])
    return root


def main():
    rclpy.init()

    # Create Data Gathering Node
    data_gathering_node = DataGatheringNode()

    # Create Behavior Tree
    model_path = "/path/to/your/rl/model.pth"  # Update with your model's path
    behavior_tree = py_trees_ros.trees.BehaviourTree(
        create_behavior_tree(data_gathering_node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10), model_path)
    )
    behavior_tree.setup(timeout=15)

    # Multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(data_gathering_node)
    executor.add_node(behavior_tree.node)

    try:
        behavior_tree.tick_tock(period_ms=100)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
