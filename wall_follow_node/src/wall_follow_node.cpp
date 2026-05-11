#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <algorithm>

class WallFollow : public rclcpp::Node {
public:
    WallFollow() : Node("wall_follow_node")
    {
        RCLCPP_INFO(this->get_logger(), "Wall follow node started");

        prev_time_ = this->now();
        
        // PID publishes to drive for wall following
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 1);

        // Laser scans used to get error for PID
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            1,
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
            {
                scan_callback(scan_msg);
            }
        );
    }

private:
    // PID control parameters
    double kp_ = 1.0;
    double kd_ = 0.2;
    double ki_ = 0.01;
    double servo_offset_ = 0.0;
    rclcpp::Time prev_time_; // for dt
    double prev_error_ = 0.0; // for derivative
    double integral_ = 0.0; // for integral
    double desired_dist_ = 1.0;
    double look_ahead_ = 1.5;

    // Topics
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // Returns the corresponding range measurement from a given angle
    double get_range(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {
        // angle = angle_min + i * angle_increment -> i = (angle - angle_min) / angle_increment
        size_t index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;

        if (index >= scan_msg->ranges.size() || !std::isfinite(scan_msg->ranges[index])) {
            return static_cast<double>(scan_msg->range_max);
        }

        return static_cast<double>(scan_msg->ranges[index]);
    }

    // Calculates the error to the wall, following the wall to the left (going counter clockwise in the Levine loop).
    double get_error(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Getting the trigonometry for finding alpha
        double angle_b = M_PI / 2;
        double angle_a = angle_b + 35 * (M_PI / 180.0); // angle b + (0, 70] degrees
        double theta = angle_a - angle_b;

        double range_a = get_range(scan_msg, angle_a);
        double range_b = get_range(scan_msg, angle_b);

        double alpha = std::atan((range_a * std::cos(theta) - range_b) / (range_a * std::sin(theta)));

        // Error from centerline
        double D_t = desired_dist_ - range_b * std::cos(alpha);
        // Total Error: Error from centerline + Error of misalignment (with look ahead)
        double D_t1 = D_t + look_ahead_ * std::sin(alpha);
        
        return -D_t1; // Error is negated for PID to counter adjust
    }

    // Based on the given error, use PID to publish vehicle control
    void pid_control(double error)
    {
        // Calculate steering angle using PID
        rclcpp::Time current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        prev_time_ = current_time;

        double derivative = 0;
        if (dt > 0.0) {
            derivative = (error - prev_error_) / dt;
        }

        integral_ += error * dt;
        integral_ = std::clamp(integral_, -10.0, 10.0); // Clamp integral to avoid risk of windup

        double steering_angle = kp_ * error + kd_ * derivative + ki_ * integral_ + servo_offset_;
        // Clamp steering angle to [-100, 100] degrees
        steering_angle = std::clamp(steering_angle, -1.74532925199, 1.74532925199); // In rad
        prev_error_ = error;

        // Publish PID vehicle control
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;

        // Determine velocity based on steering angle
        double velocity;
        if (std::abs(steering_angle) < 0.174532925) velocity = 1.5; // ~10 degrees
        else if (std::abs(steering_angle) < 0.34906585) velocity = 1; // ~20 degrees
        else velocity = 0.5;

        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;
        drive_pub_->publish(drive_msg); 
    }

    // Callback function for LaserScan messages. Calculates the error and calls PID control with it.
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        pid_control(get_error(scan_msg));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
