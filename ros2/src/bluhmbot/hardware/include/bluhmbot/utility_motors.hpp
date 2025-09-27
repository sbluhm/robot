#ifndef BLUHMBOT__UTILITY_MOTORS_HPP_
#define BLUHMBOT__UTILITY_MOTORS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "bluhmbot/zk_bm1_driver.hpp"


namespace bluhmbot
{

// utility_motors.hpp (add inside namespace bluhmbot)

struct Config {
  int pwm_in1 = 18;
  int pwm_in2 = 14;
  int frequency = 1000;
  bool invert_dir = false;
  float min_effort = 0.0;
  float max_effort = 100.0;
};

class UtilityMotorHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UtilityMotorHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  zk_bm1_Driver motor_driver_;
  Config        config_;        // <-- add this
  double speed = 0;
};

}  // namespace bluhmbot

#endif  // BLUHMBOT__UTILITY_MOTORS_HPP_
