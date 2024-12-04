import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


"""
This node publishes the goal to the controller node.
Logic on how to set the goal is implemented here.
"""
# 1	0.3	-0.6	0.793
# 2	0.327	-0.688	0.579
# 3	-0.214	-0.844	0.792
# 4	-0.196	-0.802	0.485
# 5	0.24	-0.635	0.553
# 6	-0.301	-0.872	0.531
# 7	0.079	-0.691	0.557
# 8	-0.285	-0.964	0.56
# 9	-0.218	-0.904	0.411
# 10	-0.252	-0.969	0.22
# 11	0.562	-0.695	0.27
# 12	0.227	-0.715	0.541
# 13	0.411	-0.725	0.489
# 14	0.419	-0.683	0.168
# 15	0.459	-0.476	0.297
# 16	-0.261	-0.89	0.044
# 17	0.115	-0.754	0.192
# 18	0.384	-0.708	0.671
# 19	0.399	-0.459	0.719
# 20	0.493	-0.735	0.709

goal_points = {1: [0.33, -0.6, 0.793], 2: [0.327, -0.688, 0.579], 3: [-0.214, -0.844, 0.792], 4: [-0.196, -0.802, 0.485], 
               5: [0.24, -0.635, 0.553], 6: [-0.301, -0.872, 0.531], 7: [0.079, -0.691, 0.557], 8: [-0.285, -0.964, 0.56],
             9: [-0.218, -0.904, 0.411], 10: [-0.252, -0.969, 0.22], 11: [0.562, -0.695, 0.27], 12: [0.227, -0.715, 0.541], 
             13: [0.411, -0.725, 0.489], 14: [-0.510, -0.900, 0.417], 15: [-0.273, -0.952, 0.362], 16: [0.401, -0.741, 0.513], 
             17: [0.115, -0.754, 0.192], 18: [0.384, -0.708, 0.671], 19: [0.399, -0.459, 0.719], 20: [0.493, -0.735, 0.709]}

id = 7
class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Point, '/goal', 10)
        self.publisher_marker = self.create_publisher(Marker, '/point_marker', 10)
        self.timer_ = self.create_timer(1.0, self.publish_goal)  # Publish goal every 1 second

    def publish_goal(self):
        goal_msg = Point()
        #TODO: Set goal using service
        # Set the goal coordinates (x, y, z) here0-0.021, -0.798, 0.619
        goal_msg.x = goal_points[id][0]
        goal_msg.y = goal_points[id][1]
        goal_msg.z = goal_points[id][2]
        # -0.006, -0.592, 0.601
        self.publisher_.publish(goal_msg)

        marker = Marker()
        marker.header.frame_id = 'base_link'  # Set the frame in which the points are defined
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Set the scale of the points
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1


        # Set the color of the points
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.position.x = goal_msg.x
        marker.pose.position.y = goal_msg.y
        marker.pose.position.z = goal_msg.z


        # Publish the marker
        self.publisher_marker.publish(marker)
        self.get_logger().info('Marker published')
        # self.get_logger().info('Publishing goal: x={}, y={}, z={}'.format(
        #     goal_msg.x, goal_msg.y, goal_msg.z))

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
