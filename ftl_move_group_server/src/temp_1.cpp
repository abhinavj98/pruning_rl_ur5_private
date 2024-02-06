
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"\
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
/* Create a robot state, subscribe to joint states, set the robot state to the joint states, get the jacobian, subscribe to velocity, and publish the end effector velocity */

class EeVelocityPublisher : public rclcpp::Node
{
  public:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    std::shared_ptr<rclcpp::Node> move_group_node;
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group ;
    rclcpp::Subscription <sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ee_velocity_pub;

    EeVelocityPublisher()
    : Node("ee_velocity_publisher")
    {
    //Create a robot model for ur5 arm
      rclcpp::init(argc, argv);
      rclcpp::NodeOptions node_options;
      node_options.automatically_declare_parameters_from_overrides(true);
      std::shared_ptr<rclcpp::Node> move_group_node = rclcpp::Node::make_shared("jacobian_publisher", node_options);

      // We spin up a SingleThreadedExecutor for the current state monitor to get information
      // about the robot's state.
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(move_group_node);
      std::thread([&executor]() { executor.spin(); }).detach();
      //Create a publisher to publish the jacobian
      auto jacobian_publisher = move_group_node->create_publisher<std_msgs::msg::Float32MultiArray>("jacobian", 10);
      //Creat a timer to publish the jacobian
      auto timer = move_group_node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(timer_callback));

      static const std::string PLANNING_GROUP = "ur_manipulator";
          500ms, std::bind(&EeVelocityPublisher::timer_callback, this));
//      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(move_group_node);
      std::thread([&executor]() { executor.spin(); });
      rclcpp::shutdown();
    }


  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
    //print the joint states
      RCLCPP_INFO_STREAM(this->get_logger(), "Joint states: \n" << msg->name[0] << "\n" << msg->position[0] << "\n" << msg->velocity[0] << "\n" << msg->effort[0] << "\n");
    }


};

int main(int argc, char * argv[])
{
EeVelocityPublisher ee_velocity_publisher;
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<EeVelocityPublisher>());
//  rclcpp::shutdown();
  return 0;
}

