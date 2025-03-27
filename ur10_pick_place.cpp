// ur10_pick_place_simple.cpp
// ROS2 program for UR10 pick and place operation using MoveIt2 with Cartesian planning

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"

class UR10PickPlace : public rclcpp::Node
{
public:
  UR10PickPlace() : Node("ur10_pick_place")
  {
    // Set up parameters for pick and place positions
    this->declare_parameter("pick_position.x", -0.4);
    this->declare_parameter("pick_position.y", -0.1);
    this->declare_parameter("pick_position.z", 1.2);
    
    this->declare_parameter("place_position.x", 0.5);
    this->declare_parameter("place_position.y", -0.7);
    this->declare_parameter("place_position.z", 0.4);
    
    // Initialize service for executing pick and place
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "execute_pick_place",
      std::bind(&UR10PickPlace::handleExecutePickPlace, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "UR10 Pick and Place node started. "
                "Call the 'execute_pick_place' service to begin the operation.");
  }

  void initialize()
  {
    // Wait for MoveIt to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt to initialize...");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");
    
    // Set planning parameters
    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.3); // Lower velocity for cartesian paths
    move_group_->setMaxAccelerationScalingFactor(0.1); // Lower acceleration for cartesian paths
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01);
    
    // Get end effector link
    end_effector_link_ = move_group_->getEndEffectorLink();
    
    RCLCPP_INFO(this->get_logger(), "MoveIt initialized successfully");
  }

  void handleExecutePickPlace(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!move_group_) {
      initialize();
    }
    
    bool success = executePickAndPlace();
    response->success = success;
    
    if (success) {
      response->message = "Pick and place executed successfully";
    } else {
      response->message = "Failed to execute pick and place operation";
    }
  }

  bool executePickAndPlace()
  {
    RCLCPP_INFO(this->get_logger(), "Starting pick and place operation...");
    
    // Move to home position (using joint space planning)
    RCLCPP_INFO(this->get_logger(), "Moving to home position...");
    move_group_->setNamedTarget("home");
    if (!executePlan()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to home position");
      return false;
    }
    
    // Read pick and place positions from parameters
    auto pick_x = this->get_parameter("pick_position.x").as_double();
    auto pick_y = this->get_parameter("pick_position.y").as_double();
    auto pick_z = this->get_parameter("pick_position.z").as_double();
    
    auto place_x = this->get_parameter("place_position.x").as_double();
    auto place_y = this->get_parameter("place_position.y").as_double();
    auto place_z = this->get_parameter("place_position.z").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Pick position: [%f, %f, %f]", pick_x, pick_y, pick_z);
    RCLCPP_INFO(this->get_logger(), "Place position: [%f, %f, %f]", place_x, place_y, place_z);
    
    // Execute pick
    if (!executePick(pick_x, pick_y, pick_z)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute pick operation");
      return false;
    }
    
    // Execute place
    if (!executePlace(place_x, place_y, place_z)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute place operation");
      return false;
    }
    
    // Return to home
    RCLCPP_INFO(this->get_logger(), "Returning to home position...");
    move_group_->setNamedTarget("home");
    if (!executePlan()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to return to home position");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Pick and place operation completed successfully");
    return true;
  }

  bool executePick(double x, double y, double z)
  {
    RCLCPP_INFO(this->get_logger(), "Executing pick operation...");
    
    // Move directly to the pick position
    geometry_msgs::msg::Pose pick_pose = createPose(x, y, z);
    move_group_->setPoseTarget(pick_pose);
    if (!executePlan()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pick position");
      return false;
    }
    
    // Close gripper (simulated)
    RCLCPP_INFO(this->get_logger(), "Closing gripper (simulated)");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    return true;
  }

  bool executePlace(double x, double y, double z)
  {
    RCLCPP_INFO(this->get_logger(), "Executing place operation...");
    
    // Move directly to the place position
    geometry_msgs::msg::Pose place_pose = createPose(x, y, z);
    move_group_->setPoseTarget(place_pose);
    if (!executePlan()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place position");
      return false;
    }
    
    // Open gripper (simulated)
    RCLCPP_INFO(this->get_logger(), "Opening gripper (simulated)");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    return true;
  }

  geometry_msgs::msg::Pose createPose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    
    // Set orientation - a typical top-down approach (Z-axis pointing down)
    tf2::Quaternion q;
    q.setRPY(3.14159, 0.0, 0.0);  // Roll, Pitch, Yaw (in radians)
    pose.orientation = tf2::toMsg(q);
    
    return pose;
  }

  bool executePlan()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    
    return success;
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::string end_effector_link_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR10PickPlace>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
