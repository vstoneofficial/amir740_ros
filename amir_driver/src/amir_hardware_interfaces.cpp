#include <limits>
#include <vector>

#include "amir_driver/amir_hardware_interfaces.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG_TO_MRAD(x) ((x)*(17.453292519943295769236907684886))

namespace amir_driver
{

static rclcpp::Logger LOGGER = rclcpp::get_logger("Amir740Hardware");
const int initial_position[6] = {2970, 2360, -2790, 1310, -2760, 260};  // mrad

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=amir_ros2_control_topic_" + info_.name });

  node_ = rclcpp::Node::make_shared("_", options);

  amir_joint_states_subscriber_ = node_->create_subscription<amir_interfaces::msg::AmirSensor>(
      "encoder_pub", rclcpp::SensorDataQoS(),
      [this](const amir_interfaces::msg::AmirSensor::SharedPtr amir_state) { latest_joint_state_ = *amir_state; });
  amir_joint_commands_publisher_ = node_->create_publisher<amir_interfaces::msg::AmirCmd>(
      "motor_sub", rclcpp::QoS(1));

  RCLCPP_INFO(LOGGER, "Successfully init!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AmirHardwareInterfaces::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AmirHardwareInterfaces::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating ...please wait...");

  for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
    if (std::isnan(hw_states_velocities_[i])) hw_states_velocities_[i] = 0;

    // command and state should be equal when starting
    hw_commands_positions_[i] = hw_states_positions_[i];
    hw_commands_velocities_[i] = hw_states_velocities_[i];
  }

  RCLCPP_INFO(LOGGER, "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmirHardwareInterfaces::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating ...please wait...");
  // TODO: prepare the robot to stop receiving commands

  RCLCPP_INFO(LOGGER, "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type AmirHardwareInterfaces::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */)
{
  rclcpp::spin_some(node_);

  for (size_t i = 0; i < hw_states_velocities_.size(); i++) {
    hw_states_positions_[i] = (latest_joint_state_.angle[i] + initial_position[i]) / 1000.0; // rad
    hw_states_velocities_[i] = (latest_joint_state_.vel[i]) / 1000.0; // rad/s
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmirHardwareInterfaces::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /* period */)
{
  static const float tolerance = DEG_TO_MRAD(0.500);
  static const float velocity_buffer = 300.0f;

  amir_interfaces::msg::AmirCmd amir_cmd;

  for (size_t i = 0; i < hw_commands_velocities_.size(); i++) {
    amir_cmd.angle[i] = static_cast<float>((hw_commands_positions_[i] * 1000.0f) - initial_position[i]); // in mrad
    amir_cmd.vel[i]   = static_cast<float>(hw_commands_velocities_[i]) * 1000.0f; // in mrad/s 
  
    // ! Workaround for issue with amir_cmd.vel[i] when tolerance is not reached
    if (fabs(latest_joint_state_.angle[i] - amir_cmd.angle[i]) >= tolerance) {
      if (amir_cmd.vel[i] >= 0.0f && amir_cmd.vel[i] < velocity_buffer) {
        amir_cmd.vel[i] = velocity_buffer;
      }
      if (amir_cmd.vel[i] <= 0.0f && amir_cmd.vel[i] >= -velocity_buffer) {
        amir_cmd.vel[i] = -velocity_buffer;
      }
    }
  }

  amir_joint_commands_publisher_->publish(amir_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace amir_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  amir_driver::AmirHardwareInterfaces, hardware_interface::SystemInterface)
