#ifndef __HIBACHI_BASE_HIBACHI_HARDWARE_H__
#define __HIBACHI_BASE_HIBACHI_HARDWARE_H__

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "hibachi_base/serial_port.h"

namespace hibachi_base {
class HibachiHardware : public hardware_interface::RobotHW {
public:
  /**
   * \brief Constructor
   * \param private_nh - Node handle for topics.
   */
  HibachiHardware(ros::NodeHandle &private_nh);

  /** \brief Destructor */
  ~HibachiHardware(void);

  /** \brief Initialize the hardware interface */
  void init(void);

  // void updateJointsFromHardware(const ros::Duration &duration);
  /** \brief Read the state from the robot hardware. */
  void read(ros::Duration &elapsed_time);

  /** \brief Read the state from the robot hardware
   *
   * \note This delegates RobotHW::read() calls to read()
   *
   * \param time The current time, currently unused
   * \param period The time passed since the last call
   */
  void read(const ros::Time & /*time*/, const ros::Duration &period) override {
    ros::Duration elapsed_time = period;
    read(elapsed_time);
  }

  // void writeCommandsToHardware();
  /** \brief Write the command to the robot hardware. */
  void write(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware
   *
   * \note This delegates RobotHW::write() calls to \ref write()
   *
   * \param time The current time, currently unused
   * \param period The time passed since the last call
   */
  void write(const ros::Time & /*time*/, const ros::Duration &period) override {
    ros::Duration elapsed_time = period;
    write(elapsed_time);
  }

  /** \brief Set all members to default values */
  void reset(void);

  /** \brief Enforce limits for all values before writing */
  void enforceLimits(ros::Duration &period);

private:
  ros::NodeHandle _private_nh;

  // Serial interface to Arduino
  std::string serial_port_name;
  hibachi_base::SerialPort serial_port;
  bool openSerial();
  void closeSerial();
  bool setupPIDGains(const double &kP, const double &kI, const double &kD);

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;
  void registerControlInterfaces(void);

  // Configuration
  std::vector<std::string> joint_names;
  std::size_t num_joints;

  // States
  std::vector<double> joint_position;
  std::vector<double> joint_last_position;
  std::vector<double> joint_velocity;
  std::vector<double> joint_effort;

  // Pub commands
  ros::Publisher front_right_pub, front_left_pub, rear_right_pub, rear_left_pub;
  

  // Commands
  std::vector<double> joint_velocity_command;
};  // class
}  // namespace hibachi_base
#endif  // !__HIBACHI_BASE_HIBACHI_HARDWARE_H__