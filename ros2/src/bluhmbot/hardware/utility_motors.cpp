#define DEBUG true

#include "bluhmbot/utility_motors.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bluhmbot
{

hardware_interface::CallbackReturn UtilityMotorHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read <hardware><param .../> from URDF
  const auto & hp = info_.hardware_parameters;

  // pwm_in1 (required)
  {
    auto it = hp.find("pwm_in1");
    if (it == hp.end()) {
      throw std::runtime_error("Missing parameter: pwm_in1");
    }
    this->config_.pwm_in1 = std::stoi((*it).second);  // using (*it).second to avoid '->'
  }

  // pwm_in2 (required)
  {
    auto it = hp.find("pwm_in2");
    if (it == hp.end()) {
      throw std::runtime_error("Missing parameter: pwm_in2");
    }
    this->config_.pwm_in2 = std::stoi((*it).second);  // using (*it).second to avoid '->'
  }

  // frequency (required)
  {
    auto it = hp.find("frequency");
    if (it == hp.end()) {
      throw std::runtime_error("Missing parameter: frequency");
    }
    this->config_.frequency = std::stoi((*it).second);  // using (*it).second to avoid '->'
  }

  // invert_dir (optional)
  {
    auto it = hp.find("invert_dir");
    this->config_.invert_dir = (*it).second=="true"?true:false;  // using (*it).second to avoid '->'
  }

  // min_effort (optional)
  {
    auto it = hp.find("min_effort");
    this->config_.min_effort = std::stof((*it).second);  // using (*it).second to avoid '->'
  }

  // max_effort (optional)
  {
    auto it = hp.find("max_effort");
    this->config_.max_effort = std::stof((*it).second);  // using (*it).second to avoid '->'
  }

  const hardware_interface::ComponentInfo & joint = info_.joints[0];
  // RRBotActuatorWithoutFeedback has exactly one command interface and one joint
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
  {
    RCLCPP_FATAL(
      get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
      joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
      hardware_interface::HW_IF_EFFORT);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------- on_configure -------------------------
hardware_interface::CallbackReturn UtilityMotorHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  if( DEBUG ) std::cout << "[DEBUG] UTILITY MOTOR ZK-BM1 " << info_.name.c_str() << " - Connecting with PIN" << this->config_.pwm_in1 << " AND " << this->config_.pwm_in2 << std::endl;
  if (motor_driver_.connect(this->config_.pwm_in1, this->config_.pwm_in2, this->config_.frequency) < 0)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  motor_driver_.set_ramp(/*accel*/ 60.0, /*decel*/ 120.0, /*dwell*/ 0.5); // %/s
  motor_driver_.set_min_speed(this->config_.min_effort); // ignore tiny commands under 8%

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------- on_cleanup -------------------------
hardware_interface::CallbackReturn UtilityMotorHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
  motor_driver_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------- on_shutdown -------------------------
hardware_interface::CallbackReturn UtilityMotorHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

// ------------------------- on_activate -------------------------
hardware_interface::CallbackReturn UtilityMotorHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // set some default values for joints
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    speed = 0.0;
    set_command(name, speed);
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------- on_deactivate -------------------------
hardware_interface::CallbackReturn UtilityMotorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  auto name = info_.joints[0].name + "/" + hardware_interface::HW_IF_EFFORT;
  speed = 0.0;
  set_command(name, speed);
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------- read -------------------------
hardware_interface::return_type UtilityMotorHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto name = info_.joints[0].name + "/" + hardware_interface::HW_IF_EFFORT;
  set_state(name, speed);
  return hardware_interface::return_type::OK;
}

// ------------------------- write -------------------------
hardware_interface::return_type bluhmbot::UtilityMotorHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::stringstream ss;
  std::ostringstream data;

  ss << "Writing..." << std::endl;
  ss << std::fixed << std::setprecision(2);

  auto name = info_.joints[0].name + "/" + hardware_interface::HW_IF_EFFORT;
  speed = get_command(name);

  ss << "Writing command: " << speed << std::endl;
  motor_driver_.set_motor_values(speed);

  data << get_command(name);
  ss << "Sending data command: " << data.str() << std::endl;
  RCLCPP_DEBUG(get_logger(), ss.str().c_str());

  motor_driver_.update(period.seconds()); // deterministic, uses measured dt

  return hardware_interface::return_type::OK;
}

}  // namespace bluhmbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  bluhmbot::UtilityMotorHardware, hardware_interface::SystemInterface)
