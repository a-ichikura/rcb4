#include "kxr_controller/kxr_robot_hardware.h"

#include <angles/angles.h>
#include <ros/package.h>
#include <urdf/model.h>

namespace kxr_controller {

  bool KXRRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    std::string model_str;
    ros::Rate rate(0.2);
    std::string clean_namespace;
    while (ros::ok()) {
      std::string full_namespace = robot_hw_nh.getNamespace();
      size_t last_slash_pos = full_namespace.find_last_of("/");
      clean_namespace = full_namespace.substr(0, last_slash_pos);
      if (last_slash_pos != std::string::npos) {
        clean_namespace = full_namespace.substr(0, last_slash_pos);
      }

      std::string param_name = clean_namespace.empty() ? "/robot_description" : clean_namespace + "/robot_description";

      if (robot_hw_nh.getParam(param_name, model_str)) {
        break;
      }

      ROS_WARN_STREAM("Failed to get model from " << param_name << ". Could you set " << param_name << " ros param? You can check it with `rosparam get " << param_name << "`");
      rate.sleep();
    }

    urdf::Model model;
    if (!model.initString(model_str)) {
      ROS_ERROR("Failed to parse robot_description.");
      return false;
    }

    double control_loop_rate;
    if (!root_nh.getParam("control_loop_rate", control_loop_rate) ||
        control_loop_rate <= 0.0) {
      control_loop_rate = 20.0;
    }
    control_loop_period_ = ros::Duration(1.0 / control_loop_rate);

    joint_position_command_.resize(model.joints_.size());
    joint_velocity_command_.resize(model.joints_.size());
    joint_state_position_.resize(model.joints_.size());
    joint_state_velocity_.resize(model.joints_.size());
    joint_state_effort_.resize(model.joints_.size());

    size_t i = 0;
    std::map<std::string, urdf::JointSharedPtr> joints = model.joints_;
    for (const auto& joint_pair : joints) {
      const urdf::JointSharedPtr& joint = joint_pair.second;
      if (joint->type == urdf::Joint::FIXED) {
        continue;
      }
      const std::string jointname = joint_pair.first;
      jointname_to_id_[jointname] = i;

      hardware_interface::JointStateHandle state_handle(jointname, &joint_state_position_[i], &joint_state_velocity_[i], &joint_state_effort_[i]);
      joint_state_interface.registerHandle(state_handle);
      hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(jointname), &joint_position_command_[i]);
      joint_position_interface.registerHandle(pos_handle);
      hardware_interface::JointHandle vel_handle(joint_state_interface.getHandle(jointname), &joint_velocity_command_[i]);
      joint_velocity_interface.registerHandle(vel_handle);

      ++i;
    }

    registerInterface(&joint_state_interface);
    registerInterface(&joint_position_interface);
    registerInterface(&joint_velocity_interface);

    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(clean_namespace + "/current_joint_states", 1, &KXRRobotHW::jointStateCallback, this);
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>(clean_namespace + "/command_joint_state", 1);
    joint_velocity_command_pub_ = nh_.advertise<sensor_msgs::JointState>(clean_namespace + "/velocity_command_joint_state", 1);

    ROS_INFO("Waiting for action server to start.");
    servo_on_off_action_server_ = std::make_shared<ServoOnOffActionServer>(robot_hw_nh, clean_namespace + "/kxr_fullbody_controller/servo_on_off",
                                                                           boost::bind(&KXRRobotHW::execute_servo_on_off_callback, this, _1), false);
    servo_on_off_action_client_ = std::make_shared<ServoOnOffClient>(clean_namespace + "/kxr_fullbody_controller/servo_on_off_real_interface", true);
    servo_on_off_action_client_->waitForServer();
    servo_on_off_action_server_->start();
    ROS_INFO("Action server started, client can send goal.");
    return true;
  }

  void KXRRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    if (!joint_state_received_) {
      return;
    }
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
      size_t idx = jointname_to_id_[current_joint_state_.name[i]];
      if (i < current_joint_state_.position.size())
        joint_state_position_[idx] = current_joint_state_.position[i];
      if (i < current_joint_state_.velocity.size())
        joint_state_velocity_[idx] = current_joint_state_.velocity[i];
      if (i < current_joint_state_.effort.size())
        joint_state_effort_[idx] = current_joint_state_.effort[i];
    }
  }

  void KXRRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    boost::mutex::scoped_lock lock(mutex_);
    sensor_msgs::JointState joint_state_msg;
    sensor_msgs::JointState joint_velocity_state_msg;
    joint_state_msg.header.stamp = getTime();
    joint_velocity_state_msg.header.stamp = getTime();
    for (const auto& pair : jointname_to_id_) {
      const std::string& joint_name = pair.first;
      unsigned int joint_id = pair.second;
      joint_state_msg.position.push_back(joint_position_command_[joint_id]);
      joint_state_msg.name.push_back(joint_name);

      joint_velocity_state_msg.position.push_back(joint_velocity_command_[joint_id]);
      joint_velocity_state_msg.name.push_back(joint_name);
    }
    joint_command_pub_.publish(joint_state_msg);
    joint_velocity_command_pub_.publish(joint_velocity_state_msg);
  }

  void KXRRobotHW::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    current_joint_state_ = *msg;
    joint_state_received_ = true;
  }

  void KXRRobotHW::execute_servo_on_off_callback(const kxr_controller::ServoOnOffGoalConstPtr &goal)
  {
    boost::mutex::scoped_lock lock(mutex_);
    servo_on_off_action_client_->sendGoal(*goal);
    kxr_controller::ServoOnOffResult result;
    servo_on_off_action_server_->setSucceeded(result);
  }

}  // namespace kxr_controller
