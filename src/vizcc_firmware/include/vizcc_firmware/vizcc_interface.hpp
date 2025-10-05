#ifndef VIZCC_INTERFACE_HPP
#define VIZCC_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>

namespace vizcc_firmware {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VizccInterface : public hardware_interface::SystemInterface {
public:
  VizccInterface();
  virtual ~VizccInterface();

  virtual CallbackReturn
  on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  virtual CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  virtual std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  virtual std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  virtual hardware_interface::return_type
  read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  virtual hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  hardware_interface::HardwareInfo info_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  std::string cmd_topic_;
  std::string state_topic_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;
  rclcpp::QoS cmd_qos_{rclcpp::KeepLast(1)};
};

} // namespace vizcc_firmware

#endif // __VIZCC_INTERFACE_HPP__
