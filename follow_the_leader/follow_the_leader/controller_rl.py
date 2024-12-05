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


class DataGatheringNode(Node):
    """Node to handle concurrent data gathering and update the py_trees blackboard."""

    def __init__(self):
        super().__init__('data_gathering_node')

        # Blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.joint_states = None
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
        self.blackboard.joint_states = {
            "angles": np.array(msg.position),
            "velocities": np.array(msg.velocity),
        }
        self.get_logger().info("Updated joint_states on blackboard.")

    def camera_image_callback(self, msg: Image):
        try:
            self.blackboard.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.get_logger().info("Updated camera_image on blackboard.")
        except Exception as e:
            self.get_logger().error(f"Failed to process camera image: {e}")

    def goal_callback(self, msg: Point):
        self.blackboard.goal = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info("Updated goal on blackboard.")


class AggregateObservation(py_trees.behaviours.Behaviour):
    """Aggregate sensor data into an observation."""

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        if (
            blackboard.joint_states is None
            or blackboard.camera_image is None
            or blackboard.goal is None
        ):
            return py_trees.common.Status.RUNNING

        # Example observation aggregation logic
        blackboard.observation = {
            "achieved_goal": np.random.random(3),  # Placeholder
            "desired_goal": blackboard.goal,
            "joint_angles": blackboard.joint_states["angles"],
            "rgb": blackboard.camera_image,
        }
        self.logger.info("Observation aggregated.")
        return py_trees.common.Status.SUCCESS


class ComputeRLAction(py_trees.behaviours.Behaviour):
    """Compute an action using RL logic."""

    def __init__(self, name, model_path):
        super().__init__(name)
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        self.model = th.load(model_path, map_location=self.device)  # Load RL model
        self.logger.info(f"Model loaded on {self.device}.")

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
