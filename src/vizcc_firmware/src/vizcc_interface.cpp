/**
 * @file vizcc_interface.cpp
 * @author Gonzalo G. Fernandez
 * @brief Vizcc hardware interface definition.
 */

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "vizcc_firmware/vizcc_interface.hpp"

namespace vizcc_firmware {

VizccInterface::VizccInterface() {}

VizccInterface::~VizccInterface() {
  if (exec_) {
    exec_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

CallbackReturn
VizccInterface::on_init(const hardware_interface::HardwareInfo &hardware_info) {
  CallbackReturn result =
      hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }
  info_ = hardware_info;

  velocity_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);

  node_ = rclcpp::Node::make_shared("vizcc_diff_drive");
  cmd_topic_ = info_.hardware_parameters.at("cmd_topic");
  state_topic_ = info_.hardware_parameters.at("state_topic");
  cmd_pub_ =
      node_->create_publisher<std_msgs::msg::Float32MultiArray>(cmd_topic_, 10);
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < info_.joints.size(); i++) {
          if (msg->name[i] == "left_wheel_joint") {
            position_states_[0] = msg->position[i];
            velocity_states_[0] = msg->velocity[i];
          } else if (msg->name[i] == "right_wheel_joint") {
            position_states_[1] = msg->position[i];
            velocity_states_[1] = msg->velocity[i];
          }
        }
      });
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([this] { exec_->spin(); });

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
VizccInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocity_states_[i]));
  }
  return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
VizccInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocity_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn VizccInterface::on_activate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

CallbackReturn VizccInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type VizccInterface::read(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  // done in the subscriber callback
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
VizccInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
  std_msgs::msg::Float32MultiArray msg;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    msg.data[i] = static_cast<float>(velocity_commands_[i]);
  }
  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

} // namespace vizcc_firmware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(vizcc_firmware::VizccInterface,
                       hardware_interface::SystemInterface)
