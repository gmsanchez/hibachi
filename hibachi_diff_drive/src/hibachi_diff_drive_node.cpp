#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <ros/callback_queue.h>
// #include <boost/chrono.hpp>
// #include <string>

#include <hibachi_diff_drive/diff_drive.h>

// typedef boost::chrono::steady_clock time_source;

/**
 * - Recibe comandos de Twist por /cmd_vel_in (configurable por remap)
 * - Devuelve los comandos limitados por /cmd_vel_out (configurable por remap)
 * - Parámetros que están en el constructor de la clase
 * - Parámetro de frecuencia de publicación
 *
 * Using Class Methods as Callbacks:
 *  - https://raw.githubusercontent.com/ros/ros_tutorials/groovy-devel/roscpp_tutorials/listener_class/listener_class.cpp
 */

class DifferentialDrive_node
{
public:
    DifferentialDrive_node(ros::NodeHandle &private_nh)
        : _private_nh(private_nh)
    {
        double control_frequency;
        private_nh.param<double>("control_frequency", control_frequency, 10.0);

        double wheel_radius = 0.0;
        if (!private_nh.getParam("wheel_radius", wheel_radius))
        {
            ROS_ERROR("Falta el parametro wheel_radius");
            ros::shutdown();
        }

        double wheel_separation = 0.0;
        if (!private_nh.getParam("wheel_separation", wheel_separation))
        {
            ROS_ERROR("Falta el parametro wheel_separation");
            ros::shutdown();
        }

        double v_body_max, w_body_max, w_left_max, w_right_max;
        private_nh.param<double>("v_body_max", v_body_max, std::numeric_limits<double>::infinity());
        private_nh.param<double>("w_body_max", w_body_max, std::numeric_limits<double>::infinity());
        private_nh.param<double>("w_left_max", w_left_max, std::numeric_limits<double>::infinity());
        private_nh.param<double>("w_right_max", w_right_max, std::numeric_limits<double>::infinity());

        // Publisher
        pub = private_nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);

        // Subscriber
        sub = private_nh.subscribe("cmd_vel_in", 10, &DifferentialDrive_node::sub_callback, this);

        differential_drive_class =
            hibachi_diff_drive::DifferentialDrive(wheel_radius, wheel_separation, v_body_max, w_body_max, w_left_max, w_right_max);
    }

private:
    void sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_in)
    {
        // Velocidades de entrada
        double v_body_ref_ = (*cmd_in).linear.x;
        double w_body_ref_ = (*cmd_in).angular.z;

        std::vector<double> sol = differential_drive_class.solve(v_body_ref_, w_body_ref_);

        geometry_msgs::Twist cmd_out;
        cmd_out.linear.x = sol[0];
        cmd_out.angular.z = sol[1];

        pub.publish(cmd_out);
    }

    ros::NodeHandle &_private_nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    hibachi_diff_drive::DifferentialDrive differential_drive_class;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hibachi_diff_drive");
    ros::NodeHandle nh, private_nh("~");

    DifferentialDrive_node node(private_nh);

    // Process remainder of ROS callbacks separately, mainly ControlManager related
    ros::spin();

    return 0;
}