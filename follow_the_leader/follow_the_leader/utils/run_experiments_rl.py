#!/usr/bin/env python3
import os.path
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from follow_the_leader_msgs.msg import BlenderParams, ControllerParams, States, StateTransition
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from follow_the_leader.utils.ros_utils import TFNode
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation
from functools import partial
import subprocess as sp
import shlex
import shutil
from threading import Lock
import yaml


class ExperimentManagementNode(TFNode):
    def __init__(self):
        super().__init__("experiment_manager_node")

        self.home_joints = (-np.pi / 2, -np.pi * 2 / 3, np.pi * 2 / 3, -np.pi, -np.pi / 2,
                                  np.pi)
        #convert to float
        self.home_joints = [float(i) for i in self.home_joints]
        # self.folder = output_folder

        self.lock = Lock()

        # For real experiments only
        # ROS utilities
        self.cb = ReentrantCallbackGroup()
        self.moveit_planning_client = ActionClient(self, MoveGroup, "move_action")
        self.state_announce_pub = self.create_publisher(States, "state_announcement", 1)
        self.controller_pub = self.create_publisher(ControllerParams, "/controller_params", 1)
        self.transition_sub = self.create_subscription(
            StateTransition, "state_transition", self.handle_state_transition, 1, callback_group=self.cb
        )
        self.joint_state_sub = self.create_subscription(JointState, "/move_joints", partial(self.move_to, None), 1)
        self.joy_action_sub = self.create_subscription(
            Int16, "/joy_action", self.handle_joy_action, 1, callback_group=self.cb
        )


    def handle_joy_action(self, msg):
        action = msg.data
        print(f"Received action {action}")
        if action == 5:
            self.move_home()
            return
        #action to register goal
        #action to starta

    def move_home(self):
        self.move_to(joints=self.home_joints)

    def move_to(self, pose=None, joints=None):
        if not (pose is None) ^ (joints is None):
            if pose is not None:
                raise ValueError("Please fill in only a pose or a joints value, not both")
            else:
                raise ValueError("Please specify a pose or joints value to move to")

        if joints is not None:
            joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]

            if isinstance(joints, JointState):
                joints = joints.position

            if not len(joints):
                joints = self.home_joints

            joint_constraints = [JointConstraint(joint_name=n, position=p) for n, p in zip(joint_names, joints)]
            kwargs = {"joint_constraints": joint_constraints}

        else:
            pos = pose[:3, 3]
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()

            # TODO: This doesn't work - why not?
            kwargs = {
                "position_constraints": [
                    PositionConstraint(link_name="tool0", target_point_offset=Vector3(x=pos[0], y=pos[1], z=pos[2]))
                ],
                "orientation_constraints": [
                    OrientationConstraint(
                        link_name="tool0", orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    ),
                ],
            }

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest(
            group_name="ur_manipulator",
            goal_constraints=[Constraints(**kwargs)],
            allowed_planning_time=5.0,
        )
        goal_msg.planning_options = PlanningOptions(plan_only=False)

        self.moveit_planning_client.wait_for_server()
        future = self.moveit_planning_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_complete)

    def handle_state_transition(self, msg: StateTransition):
        pass
        # if msg.state_end == States.IDLE:
        #     self.end_experiment()

    def goal_complete(self, future):
        rez = future.result()
        if not rez.accepted:
            print("Planning failed!")
            return
        else:
            print("Plan succeeded!")

 

if __name__ == "__main__":
    rclpy.init()
    node = ExperimentManagementNode()
    # rclpy.get_default_context().on_shutdown(node.end_experiment)

    # if len(sys.argv) >= 3 and int(sys.argv[2]):
    #     node.move_home()

    rclpy.spin(node)
    rclpy.shutdown()
