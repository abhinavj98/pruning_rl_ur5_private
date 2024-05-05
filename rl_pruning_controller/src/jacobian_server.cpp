
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <rl_pruning_controller_msgs/srv/jacobian.hpp>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("jacobian_server");
//Create a jacobian matrix from the current state of the robot
Eigen::MatrixXd make_jacobian(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group, const moveit::core::JointModelGroup* joint_model_group)
{
  //Get current state of the robot to calculate jacobian
  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  current_state->getJacobian(joint_model_group,
        current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
          reference_point_position, jacobian);
  return jacobian;
}

void get_jacobian(const std::shared_ptr<follow_the_leader_msgs::srv::Jacobian::Request> request,
  std::shared_ptr<follow_the_leader_msgs::srv::Jacobian::Response> response,
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
  const moveit::core::JointModelGroup* joint_model_group)
{
  Eigen::MatrixXd jacobian = make_jacobian(move_group, joint_model_group);
  //convert the jacobian to a vector
  std::vector<float> vec(jacobian.data(), jacobian.data() + jacobian.rows() * jacobian.cols()); //Remember the conversion from Eigen to std::vector is row-major
  response->jacobian = vec;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::sleep_for(std::chrono::seconds(5));
  auto move_group_node = rclcpp::Node::make_shared("jacobian_server", node_options);
  auto my_callback_group = move_group_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // We spin up a MultiThreadedExecutor for the current state monitor to get information
  // about the robot's state so the services do not block
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread t([&executor]()
    { executor.spin(); });

  RCLCPP_INFO(LOGGER, "Hello!");

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "ur_manipulator";

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
    move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));
    auto jacobian_cb =
    [&](const std::shared_ptr<follow_the_leader_msgs::srv::Jacobian::Request>& request,
      const std::shared_ptr<follow_the_leader_msgs::srv::Jacobian::Response>& response) -> void
    {
      return get_jacobian(request, response, move_group, joint_model_group);
    };

  auto jacobian_server =
    move_group_node->create_service<follow_the_leader_msgs::srv::Jacobian>("get_jacobian", jacobian_cb,
      rmw_qos_profile_services_default, my_callback_group);

  RCLCPP_INFO(LOGGER, "Jacobian Server up");
  t.join();
  RCLCPP_INFO(LOGGER, "Join complete");
  rclcpp::shutdown(0);
  return 0;
}