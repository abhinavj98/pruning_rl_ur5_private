This code has been built on work done by Alex You from this repository - https://github.com/OSUrobotics/follow-the-leader

This ROS2 code is used to servo a UR5 arm. Image and proprioception inputs are fed into a controller learned using reinforcement learning. This controller outputs velocity commands required to reach a cut point on a tree.

## To launch:
```
ros2 launch follow_the_leader follow_the_leader.launch.py ur_type:=ur5e use_sim:=false camera_type:=d435 
```

## Set goal value in goal publisher and run 
```
ros2 run follow_the_leader goal_publisher
```
## Xbox key mapping using io_manager.py
 A - Start/Stop servo
 X - Restart RL controller
 LB - Go to home position

## Visualization
Rviz will visualize the end-effector velocity commanded as an arrow along with the cut-point as a sphere. To debug optical flow/image is visualized in Rviz.


The controller works by taking the optical flow input, joint angles, and current joint velocities as input and outputs the 6DOF end effector velocity required to reach the goal. The controller training can be found at https://github.com/abhinavj98/pruning_sb3 and is trained in simulation. 

## Rubriks


Project goals (points) - File related to goal
1. Setting up UR5 and UR5 moveit (10) - follow_the_leader/launch/follow_the_leader.launch.py

2. Enable servoing and how to use moveit servo to publish end effector velocities (10) - follow_the_leader/follow_the_leader/io_manager.py

3. Use image data and convert to optical flow (10) - follow_the_leader/follow_the_leader/optical_flow_srv.py

4. Get jacobian (10) - ftl_move_group_server/src/ftl_move_group_server.cpp

5. Assemble neural network input (10) - follow_the_leader/follow_the_leader/controller_rl.py

6. Use neural network output to servo (5) - follow_the_leader/follow_the_leader/controller_rl.py

7. Setting up services (5) - follow_the_leader/follow_the_leader/optical_flow_srv.py, follow_the_leader_msgs/srv/OpticalFlowAndMask.srv
                            - ftl_move_group_server/src/ftl_move_group_server.cpp, , follow_the_leader_msgs/srv/Jacobian.srv
