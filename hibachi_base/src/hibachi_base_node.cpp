#include "hibachi_base/hibachi_hardware.h"
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <boost/chrono.hpp>
#include <std_srvs/Trigger.h>

/**
 * This hardware_interface node and the corresponding Arduino firmware are heavily
 * influenced by Clearpath Robotics Husky and AMPRU. Thanks to the Open Source
 * community for sharing and this is what "standing in the shoulder of giants" really
 * mean.
 **/

typedef boost::chrono::steady_clock time_source;

/**
 * Control loop for Husky, not realtime safe
 */
void controlLoop(hibachi_base::HibachiHardware &hibachi,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time) {
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  // hibachi.reportLoopDuration(elapsed);
  hibachi.read(ros::Time::now(), elapsed);
  cm.update(ros::Time::now(), elapsed);
  hibachi.write(ros::Time::now(), elapsed);
}

bool resetOdometry(hibachi_base::HibachiHardware &hibachi,
                   std_srvs::Trigger::Request &request,
                   std_srvs::Trigger::Response &response) {
  // ROS_INFO("Reset odometry\n");
  hibachi.reset();
  response.success = true /* hibachi.resetOdometry() */;
  response.message = "Odometry resetted";
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hibachi_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // Initialize robot hardware and link to controller manager
  hibachi_base::HibachiHardware hibachi(private_nh);
  hibachi.init();
  controller_manager::ControllerManager cm(&hibachi, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue hibachi_queue;
  ros::AsyncSpinner hibachi_spinner(1, &hibachi_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
      ros::Duration(1 / control_frequency),
      boost::bind(
          controlLoop, boost::ref(hibachi), boost::ref(cm), boost::ref(last_time)),
      &hibachi_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  // wait for serial port
  ros::Duration(0.5).sleep();
  hibachi_spinner.start();

  // Publish service for odometry reset
  ros::ServiceServer service =
      private_nh
          .advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
              "reset_odometry",
              boost::bind(resetOdometry, boost::ref(hibachi), _1, _2));

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}