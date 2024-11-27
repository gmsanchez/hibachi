#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "insens-lib/src/insens.h"

using namespace InSens;

void rotate_rep103_ENU(IMU_Data &data) {
  static const float g_to_ms2 = 9.80665f;
  static const float dps_torads = 0.017453f;

  // Accelerometer
  // Flip Y (ENU)
  data.accel.y() = -data.accel.y();

  // G to m.s^-2
  data.accel.x() *= g_to_ms2;
  data.accel.y() *= g_to_ms2;
  data.accel.z() *= g_to_ms2;

  // Gyro rotation
  // Flip Y (ENU)
  data.gyro.y() = -data.gyro.y();
  
  // dps to rad/s
  data.gyro.x() *= dps_torads;
  data.gyro.y() *= dps_torads;
  data.gyro.z() *= dps_torads;

  // Mag normalization
  // float l2 = data.compass.x()^2 + data.compass.y()^2 + data.compass.z()^2;
  // l2 = sqrt(l2);
  // float l2 = data.compass.norm();
  // data.compass.x() /= l2;
  // data.compass.y() /= l2;
  // data.compass.z() /= l2;

  // Mag rotation
  // Flip X and Y (ENU)
  data.compass.x() = -data.compass.x();
  data.compass.y() = -data.compass.y();

  // gauss to tesla
  data.compass.x() *= 0.0001;
  data.compass.y() *= 0.0001;
  data.compass.z() *= 0.0001;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh, private_nh("~");

  std::string frame_id;
  private_nh.param<std::string>("frame_id", frame_id, "imu_link");

  double sensor_frequency;
  private_nh.param<double>("sensor_frequency", sensor_frequency, 10.0);

  // Settings
  InSens::SettingFile settings;

  // SPI config
  settings.m_SPIBus = 0;
  settings.m_SPISelect = 0;  
  settings.m_SPISpeed = 500000UL;
  settings.m_SPI2Bus = 0;
  settings.m_SPI2Select = 1;
  settings.m_SPI2Speed = 500000UL;

  // Create a dual-slave SPI interface for LSM9DS1
  InSens::DualSlaveSPI interface(settings);

  // Give settings and interface to IMU
  InSens::LSM9DS1 imu(&settings, &interface);

  // Initialize
  imu.init();

  ros::Publisher imu_pub = private_nh.advertise<sensor_msgs::Imu>("data_raw", 10);
  ros::Publisher mag_pub = private_nh.advertise<sensor_msgs::MagneticField>("mag", 10);

  ros::Rate rate(sensor_frequency);
  while (ros::ok()) {

    // ROS_INFO("The time is %f", ros::Time::now());
    ros::Time current_time = ros::Time::now();
    if (imu.read()) {
      // Acc data in G
      // Gyro data in dps
      // Mag data in gauss
      IMU_Data imuData = imu.getData();
      // Convert to ENU, acc data in m.s^-2 and gyro data in rad/s
      rotate_rep103_ENU(imuData);

      ROS_INFO_THROTTLE(1,
                        "Acc: %+2.3f %+2.3f %+2.3f  Gyr: %+1.4f %+1.4f %+1.4f",
                        imuData.accel.x(),
                        imuData.accel.y(),
                        imuData.accel.z(),
                        imuData.gyro.x(),
                        imuData.gyro.y(),
                        imuData.gyro.z());

      // rotate_rep103(ax, ay, az, gx, gy, gz, mx, my, mz);

      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = current_time;
      imu_msg.header.frame_id = frame_id;

      imu_msg.angular_velocity.x = imuData.gyro.x();
      imu_msg.angular_velocity.y = imuData.gyro.y();
      imu_msg.angular_velocity.z = imuData.gyro.z();

      imu_msg.linear_acceleration.x = imuData.accel.x();
      imu_msg.linear_acceleration.y = imuData.accel.y();
      imu_msg.linear_acceleration.z = imuData.accel.z();

      sensor_msgs::MagneticField mag_msg;
      mag_msg.header.stamp = current_time;
      mag_msg.header.frame_id = frame_id;

      mag_msg.magnetic_field.x = imuData.compass.x(); 
      mag_msg.magnetic_field.y = imuData.compass.y();
      mag_msg.magnetic_field.z = imuData.compass.z();

      // Pub & sleep.
      imu_pub.publish(imu_msg);
      mag_pub.publish(mag_msg);
    }
    ros::spinOnce();  // the missing call
    rate.sleep();
  }

  return 0;
}
