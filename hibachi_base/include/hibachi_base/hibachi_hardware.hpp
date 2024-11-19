#ifndef __HIBACHI_BASE__HIBACHI_HARDWARE_HPP__
#define __HIBACHI_BASE__HIBACHI_HARDWARE_HPP__

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "serial_HDLC_interface.hpp"

#define FRONT_LEFT_JOINT_NAME "front_left_wheel_joint"
#define FRONT_RIGHT_JOINT_NAME "front_right_wheel_joint"
#define REAR_LEFT_JOINT_NAME "rear_left_wheel_joint"
#define REAR_RIGHT_JOINT_NAME "rear_right_wheel_joint"

namespace hibachi_base
{
using CallbackReturn = hardware_interface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class HibachiHardware : public hardware_interface::SystemInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HibachiHardware)

    HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

    /* // HARDWARE_INTERFACE_PUBLIC
    // CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state)
    // override;

    // HARDWARE_INTERFACE_PUBLIC
    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state)
    // override; */

    HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /* // HARDWARE_INTERFACE_PUBLIC
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state)
    // override;

    // HARDWARE_INTERFACE_PUBLIC
    // CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override; */

    HARDWARE_INTERFACE_PUBLIC
    std::vector<StateInterface> export_state_interfaces() override;

    HARDWARE_INTERFACE_PUBLIC
    std::vector<CommandInterface> export_command_interfaces() override;

    HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time& time, const
    rclcpp::Duration& period) override;

    HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time& time, const
    rclcpp::Duration& period) override;

  protected:
    bool writeCommandsToHardware(void);  // true = error;
    bool updateJointsFromHardware(void);  // true = error;
    bool resetOdometry(void); // true = error;
    bool startCommunication(void); // true = error;

    // Store the command for the robot and other joints information
    static const size_t JOINTS_NUMBER = 4;
    struct HWJoint{
      std::string name;
      double command = 0.0; 
      double state_position = 0.0, state_velocity = 0.0;
      size_t joint_idx = -1;
    } hw_joints[JOINTS_NUMBER] = {
      {FRONT_LEFT_JOINT_NAME},
      {FRONT_RIGHT_JOINT_NAME},
      {REAR_LEFT_JOINT_NAME},
      {REAR_RIGHT_JOINT_NAME}
    };

    // ROS Parameters (in URDF)
    std::string serial_port_name;
    uint32_t serial_baud_rate;
    uint8_t serial_flow_control;
    bool serial_parity;
    uint8_t serial_stop_bits;

    // Handle different update rate that ros2_control main node
    float hw_update_rate;
    uint32_t _hw_update_period_nS;
    bool _first_read_pass, _first_write_pass = true;
    rclcpp::Time _last_read_time, _last_write_time;
    
    std::shared_ptr<SerialPort> serial_port;
};

}  // namespace hibachi_base

#endif  // !__HIBACHI_BASE__HIBACHI_HARDWARE_H__