import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


"""
This node publishes the goal to the controller node.
Logic on how to set the goal is implemented here.
"""
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
        goal_msg.x = -.354#255
        goal_msg.y = -.667#-0.707#-0.779#
        goal_msg.z = 0.575 #0.680 #
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
