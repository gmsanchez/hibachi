#include "hibachi_base/hibachi_hardware.h"
#include <geometry_msgs/Twist.h>

namespace hibachi_base {
HibachiHardware::HibachiHardware(ros::NodeHandle &private_nh)
    : _private_nh(private_nh) {
  _private_nh.param<std::string>("serial_port", serial_port_name, "/dev/ttyACM0");

  joint_names = {"front_left_wheel",
                 "front_right_wheel",
                 "rear_left_wheel",
                 "rear_right_wheel"};
  num_joints = 4;

  front_right_pub = private_nh.advertise<geometry_msgs::Twist>("front_right_cmd", 10);
  front_left_pub = private_nh.advertise<geometry_msgs::Twist>("front_left_cmd", 10);
  rear_right_pub = private_nh.advertise<geometry_msgs::Twist>("rear_right_cmd", 10);
  rear_left_pub = private_nh.advertise<geometry_msgs::Twist>("rear_left_cmd", 10);
  

  openSerial();
}

HibachiHardware::~HibachiHardware() { closeSerial(); }

void HibachiHardware::init(void) {
  // Status
  joint_position.resize(num_joints, 0.0);
  joint_velocity.resize(num_joints, 0.0);
  joint_effort.resize(num_joints, 0.0);
  joint_last_position.resize(num_joints, 0.0);

  // Command
  joint_velocity_command.resize(num_joints, 0.0);

  registerControlInterfaces();

  // Reset odometry
  reset();

  // PID gains configurable
  double _kP, _kI, _kD;
  _private_nh.param<double>("PID_kP_gain", _kP, 0.0);
  _private_nh.param<double>("PID_kI_gain", _kI, 0.0);
  _private_nh.param<double>("PID_kD_gain", _kD, 0.0);
  if (_kP != 0.0 || _kI != 0.0 || _kD != 0.0) {
    setupPIDGains(_kP, _kI, _kD);
  } 
}

/**
 * Register interfaces with the RobotHW interface manager, allowing ros_control
 * operation
 */
void HibachiHardware::registerControlInterfaces() {
  // Connect and register the joint state and velocity interface
  for (std::size_t joint_id = 0; joint_id < num_joints; ++joint_id) {
    // Create joint state handle
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names[joint_id],
        &joint_position[joint_id],
        &joint_velocity[joint_id],
        &joint_effort[joint_id]);

    // Register in joint state interface
    joint_state_interface.registerHandle(joint_state_handle);

    // Add command interfaces to joints and register it
    hardware_interface::JointHandle joint_handle_velocity(
        joint_state_handle, &joint_velocity_command[joint_id]);
    velocity_joint_interface.registerHandle(joint_handle_velocity);
  }  // end for each joint

  registerInterface(&joint_state_interface);     // From RobotHW base class
  registerInterface(&velocity_joint_interface);  // From RobotHW base class
}

/**
 * Pull latest speed and travel measurements from MCU, and store in joint structure
 * for ros_control
 */
void HibachiHardware::read(ros::Duration &elapsed_time) {
  hibachi_base::GetFourWheelEncoder getFourWheelEncoder;
  serial_port.sendMessage(&getFourWheelEncoder);

  FourWheelEncoderPosition *fourWheelEncoderPosition =
      (FourWheelEncoderPosition *)serial_port.waitMessage(
          FourWheelEncoderPosition::MESSAGE_TYPE, 1.0);
  FourWheelEncoderSpeed *fourWheelEncoderSpeed =
      (FourWheelEncoderSpeed *)serial_port.waitMessage(
          FourWheelEncoderSpeed::MESSAGE_TYPE, 1.0);

  std::vector<double> tmp(4);
  if (fourWheelEncoderPosition != NULL) {
    fourWheelEncoderPosition->getWheelsAngPosition(tmp[0], tmp[1], tmp[2], tmp[3]);
    // Convert 2.Pi range in continuos
    for (std::size_t i = 0; i < num_joints; ++i) {
      double d_position = tmp[i] - joint_last_position[i];
      joint_last_position[i] = tmp[i];

      if (joint_velocity[i] < 0.0) {
        d_position *= -1.0;
      }
      if (d_position < 0.0) {
        d_position += 2 * M_PI;
      }
      if (joint_velocity[i] >= 0.0) {
        joint_position[i] += d_position;
      } else {
        joint_position[i] -= d_position;
      }
    }

    // Print some output
    ROS_INFO_THROTTLE(0.5,
      "Recived pos: %f, %f, %f, %f", tmp[0], tmp[1], tmp[2], tmp[3]);
  } else {
    ROS_ERROR("No valid encoder position received");
  }

  if (fourWheelEncoderSpeed != NULL) {
    fourWheelEncoderSpeed->getWheelsAngSpeed(tmp[0], tmp[1], tmp[2], tmp[3]);
    joint_velocity = tmp;
    ROS_INFO_THROTTLE(0.5,
      "Received speed: %f, %f, %f, %f", tmp[0], tmp[1], tmp[2], tmp[3]);
  } else {
    ROS_ERROR("No valid encoder speed received");
  }
}

/**
 * Get latest velocity commands from ros_control via joint structure, and send to MCU
 */
void HibachiHardware::write(ros::Duration &elapsed_time) {
  ROS_INFO_THROTTLE(0.5, "I am sending %f, %f, %f and %f",
           joint_velocity_command[0],
           joint_velocity_command[1],
           joint_velocity_command[2],
           joint_velocity_command[3]);
  
  geometry_msgs::Twist front_right_msg, front_left_msg,
    rear_right_msg, rear_left_msg;

  front_left_msg.linear.x = joint_velocity_command[0]; // front_left_wheel
  front_right_msg.linear.x = joint_velocity_command[1]; // front_right_wheel
  rear_left_msg.linear.x = joint_velocity_command[2]; // rear_left_wheel
  rear_right_msg.linear.x = joint_velocity_command[3]; // rear_right_wheel

  front_left_pub.publish(front_left_msg);
  front_right_pub.publish(front_right_msg);
  rear_left_pub.publish(rear_left_msg);
  rear_right_pub.publish(rear_right_msg);

  hibachi_base::SetSkidSteerMotorSpeed SetSkidSteerMotorSpeed(
      joint_velocity_command[0],
      joint_velocity_command[1],
      joint_velocity_command[2],
      joint_velocity_command[3]);
  serial_port.sendMessage(&SetSkidSteerMotorSpeed);
}

bool HibachiHardware::openSerial() {
  serial_port.setPort(serial_port_name);
  return serial_port.openConnection();
  return false;
}

void HibachiHardware::closeSerial() { serial_port.closeConnection(); }

void HibachiHardware::reset() {
  // Reset joint limits state, in case of mode switch or e-stop
  ROS_INFO("Reset request");
  hibachi_base::ResetOdometry resetOdometry;
  serial_port.sendMessage(&resetOdometry);

  for (std::size_t i = 0; i < num_joints; ++i) {
    joint_position[i] = 0.0;
    joint_last_position[i] = 0.0;
    joint_velocity[i] = 0.0;
  }
}

 bool HibachiHardware::setupPIDGains(const double &kP, const double &kI, const double &kD) {
  ROS_INFO("Setting up PID gains: P = %f, I = %f, D = %f", kP, kI, kD);

  uint8_t selector = 0; // Set same gains for all wheel
  hibachi_base::SetPIDGains PIDGains(selector, kP, kI, kD);
  
  serial_port.sendMessage(&PIDGains);
  return true;
}
}  // namespace hibachi_base