// // Copyright 2020 F1TENTH Foundation
// //
// // Redistribution and use in source and binary forms, with or without
// // modification, are permitted provided that the following conditions are met:
// //
// //   * Redistributions of source code must retain the above copyright
// //     notice, this list of conditions and the following disclaimer.
// //
// //   * Redistributions in binary form must reproduce the above copyright
// //     notice, this list of conditions and the following disclaimer in the
// //     documentation and/or other materials provided with the distribution.
// //
// //   * Neither the name of the {copyright_holder} nor the names of its
// //     contributors may be used to endorse or promote products derived from
// //     this software without specific prior written permission.
// //
// // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// // POSSIBILITY OF SUCH DAMAGE.

// // -*- mode:c++; fill-column: 100; -*-

// #ifndef VESC_ACKERMANN__VESC_TO_ODOM_HPP_
// #define VESC_ACKERMANN__VESC_TO_ODOM_HPP_

// #include <nav_msgs/msg/odometry.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <vesc_msgs/msg/vesc_state_stamped.hpp>

// #include <memory>
// #include <string>

// namespace vesc_ackermann
// {

// using nav_msgs::msg::Odometry;
// using std_msgs::msg::Float64;
// using vesc_msgs::msg::VescStateStamped;

// class VescToOdom : public rclcpp::Node
// {
// public:
//   explicit VescToOdom(const rclcpp::NodeOptions & options);

// private:
//   // ROS parameters
//   std::string odom_frame_;
//   std::string base_frame_;
//   /** State message does not report servo position, so use the command instead */
//   bool use_servo_cmd_;
//   // conversion gain and offset
//   double speed_to_erpm_gain_, speed_to_erpm_offset_;
//   double steering_to_servo_gain_, steering_to_servo_offset_;
//   double wheelbase_;
//   bool publish_tf_;

//   // odometry state
//   double x_, y_, yaw_;
//   Float64::SharedPtr last_servo_cmd_;  ///< Last servo position commanded value
//   VescStateStamped::SharedPtr last_state_;  ///< Last received state message

//   // ROS services
//   rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
//   rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
//   rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
//   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

//   // ROS callbacks
//   void vescStateCallback(const VescStateStamped::SharedPtr state);
//   void servoCmdCallback(const Float64::SharedPtr servo);
// };

// }  // namespace vesc_ackermann

// #endif  // VESC_ACKERMANN__VESC_TO_ODOM_HPP_


#ifndef EKF_ODOM_HPP
#define EKF_ODOM_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;

namespace vesc_ackermann
{

class VescToOdom : public rclcpp::Node
{
public:
    // 컴포넌트 생성자 (필수)
    explicit VescToOdom(const rclcpp::NodeOptions& options);


private:
    // General Parameters
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;
    bool use_servo_cmd_;
    bool publish_tf_;
    bool update_imu_;

    // Kinematics Parameters
    double speed_to_erpm_gain_;
    double speed_to_erpm_offset_;
    double steering_to_servo_gain_;
    double steering_to_servo_offset_;
    double wheelbase_;
    double servo_min_;
    double servo_max_;

    // EKF Parameters
    double ekf_timer_period_;
    double q_x_, q_y_, q_yaw_, q_yaw_rate_, q_vx_, q_vy_; // process noise variances

    // State
    Vector6d x_; // state vector [x, y, yaw, yaw_rate]
    Matrix6d P_; // state covariance [x, y, yaw, yaw_rate]
    Matrix4d R_; // measurement noise variances [yaw_angle, yaw_rate]
    rclcpp::Time last_time_;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pred_cov_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_yaw_rate_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_yaw_angle_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_cmd_sub_;
    rclcpp::TimerBase::SharedPtr ekf_timer_;
    
    // Last received messages
    std::mutex data_mutex_;
    vesc_msgs::msg::VescStateStamped::SharedPtr last_state_;
    vesc_msgs::msg::VescImuStamped::SharedPtr last_imu_state_;
    std_msgs::msg::Float64::SharedPtr last_servo_cmd_;

    // TF Listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Initial IMU yaw for relative angle calculation
    double initial_imu_yaw_ = std::numeric_limits<double>::quiet_NaN();

    // Methods
    void setParams();
    void ekfTimerCallback();

    void vescCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state_msg) {last_state_ = state_msg;}
    void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu_msg) {last_imu_state_ = imu_msg;}
    void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo_msg) {last_servo_cmd_ = servo_msg;}
    
    void predict(double dt);
    void updateIMU(double measured_yaw_angle, double measured_yaw_rate, double v_linear);

    void publishOdometry(const rclcpp::Time& stamp);
    double normalize_angle(double angle);
};

}  // namespace vesc_ackermann

#endif // VESC_TO_ODOM_HPP