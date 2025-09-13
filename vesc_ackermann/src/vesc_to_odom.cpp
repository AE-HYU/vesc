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

// #include "vesc_ackermann/vesc_to_odom.hpp"

// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <vesc_msgs/msg/vesc_state_stamped.hpp>

// #include <cmath>
// #include <string>

// namespace vesc_ackermann
// {

// using geometry_msgs::msg::TransformStamped;
// using nav_msgs::msg::Odometry;
// using std::placeholders::_1;
// using std_msgs::msg::Float64;
// using vesc_msgs::msg::VescStateStamped;

// VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
// : Node("vesc_to_odom_node", options),
//   odom_frame_("odom"),
//   base_frame_("base_link"),
//   use_servo_cmd_(true),
//   publish_tf_(false),
//   x_(0.0),
//   y_(0.0),
//   yaw_(0.0)
// {
//   // get ROS parameters
//   odom_frame_ = declare_parameter("odom_frame", odom_frame_);
//   base_frame_ = declare_parameter("base_frame", base_frame_);
//   use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  
//   declare_parameter<double>("speed_to_erpm_gain", 0.0);
//   declare_parameter<double>("speed_to_erpm_offset", 0.0);

//   speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
//   speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();

//   if (use_servo_cmd_) {
//     declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
//     declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
//     declare_parameter<double>("wheelbase", 0.0);
    
//     steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
//     steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
//     wheelbase_ = get_parameter("wheelbase").get_value<double>();
//   }

//   publish_tf_ = declare_parameter("publish_tf", publish_tf_);

//   // create odom publisher
//   odom_pub_ = create_publisher<Odometry>("odom", 10);

//   // create tf broadcaster
//   if (publish_tf_) {
//     tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
//   }

//   // subscribe to vesc state and. optionally, servo command
//   vesc_state_sub_ = create_subscription<VescStateStamped>(
//     "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

//   if (use_servo_cmd_) {
//     servo_sub_ = create_subscription<Float64>(
//       "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
//   }
// }

// void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
// {
//   // check that we have a last servo command if we are depending on it for angular velocity
//   if (use_servo_cmd_ && !last_servo_cmd_) {
//     return;
//   }

//   // convert to engineering units
//   double current_speed = -(state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
//   if (std::fabs(current_speed) < 0.05) {
//     current_speed = 0.0;
//   }
//   double current_steering_angle(0.0), current_angular_velocity(0.0);
//   if (use_servo_cmd_) {
//     current_steering_angle =
//       (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
//     current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
//   }

//   // use current state as last state if this is our first time here
//   if (!last_state_) {
//     last_state_ = state;
//   }

//   // calc elapsed time
//   auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

//   /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

//   // propigate odometry
//   double x_dot = current_speed * cos(yaw_);
//   double y_dot = current_speed * sin(yaw_);
//   x_ += x_dot * dt.seconds();
//   y_ += y_dot * dt.seconds();
//   if (use_servo_cmd_) {
//     yaw_ += current_angular_velocity * dt.seconds();
//   }

//   // save state for next time
//   last_state_ = state;

//   // publish odometry message
//   Odometry odom;
//   odom.header.frame_id = odom_frame_;
//   odom.header.stamp = state->header.stamp;
//   odom.child_frame_id = base_frame_;

//   // Position
//   odom.pose.pose.position.x = x_;
//   odom.pose.pose.position.y = y_;
//   odom.pose.pose.orientation.x = 0.0;
//   odom.pose.pose.orientation.y = 0.0;
//   odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
//   odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

//   // Position uncertainty
//   /** @todo Think about position uncertainty, perhaps get from parameters? */
//   odom.pose.covariance[0] = 0.2;   ///< x
//   odom.pose.covariance[7] = 0.2;   ///< y
//   odom.pose.covariance[35] = 0.4;  ///< yaw

//   // Velocity ("in the coordinate frame given by the child_frame_id")
//   odom.twist.twist.linear.x = current_speed;
//   odom.twist.twist.linear.y = 0.0;
//   odom.twist.twist.angular.z = current_angular_velocity;

//   // Velocity uncertainty
//   /** @todo Think about velocity uncertainty */

//   if (publish_tf_) {
//     TransformStamped tf;
//     tf.header.frame_id = odom_frame_;
//     tf.child_frame_id = base_frame_;
//     tf.header.stamp = now();
//     tf.transform.translation.x = x_;
//     tf.transform.translation.y = y_;
//     tf.transform.translation.z = 0.0;
//     tf.transform.rotation = odom.pose.pose.orientation;

//     if (rclcpp::ok()) {
//       tf_pub_->sendTransform(tf);
//     }
//   }

//   if (rclcpp::ok()) {
//     odom_pub_->publish(odom);
//   }
// }

// void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
// {
//   last_servo_cmd_ = servo;
// }

// }  // namespace vesc_ackermann

// #include "rclcpp_components/register_node_macro.hpp"  // NOLINT

// RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)




#include <tf2/utils.h>

#include "vesc_ackermann/vesc_to_odom.hpp"
namespace vesc_ackermann
{

VescToOdom::VescToOdom(const rclcpp::NodeOptions& options) : rclcpp::Node("vesc_to_odom_node", options)
{
    // Initialize parameters
    setParams();

    // Subscriptions
    vesc_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
        "sensors/core", 10, std::bind(&VescToOdom::vescCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<vesc_msgs::msg::VescImuStamped>(
        "sensors/imu", 10, std::bind(&VescToOdom::imuCallback, this, std::placeholders::_1));
    if (use_servo_cmd_)
    {
        servo_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, std::placeholders::_1));
    }

    // Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (publish_tf_) tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
    
    // Initialize EKF
    x_.setZero();                   // Initial state vector
    P_.setIdentity(); P_ *= 0.1;    // Initial covariance
    last_time_ = this->now();

    // Initialize TF Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // EKF timer
    ekf_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ekf_timer_period_), std::bind(&VescToOdom::ekfTimerCallback, this));
}

void VescToOdom::setParams()
{
    // General Parameters
    this->declare_parameter("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_);

    this->declare_parameter("base_frame", "base_link");
    this->get_parameter("base_frame", base_frame_);

    this->declare_parameter("imu_frame", "imu_link");
    this->get_parameter("imu_frame", imu_frame_);

    this->declare_parameter("use_servo_cmd", true);
    this->get_parameter("use_servo_cmd", use_servo_cmd_);

    this->declare_parameter("publish_tf", true);
    this->get_parameter("publish_tf", publish_tf_);

    this->declare_parameter("update_imu", true);
    this->get_parameter("update_imu", update_imu_);

    // Kinematic Parameters
    this->declare_parameter("speed_to_erpm_gain", 1.0);
    this->get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);

    this->declare_parameter("speed_to_erpm_offset", 0.0);
    this->get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);

    this->declare_parameter("steering_angle_to_servo_gain", 1.0);
    this->get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);

    this->declare_parameter("steering_angle_to_servo_offset", 0.0);
    this->get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);

    this->declare_parameter("wheelbase", 0.32);
    this->get_parameter("wheelbase", wheelbase_);

    this->declare_parameter("servo_min", 0.15);
    this->get_parameter("servo_min", servo_min_);

    this->declare_parameter("servo_max", 0.85);
    this->get_parameter("servo_max", servo_max_);

    // EKF Parameters
    this->declare_parameter<double>("ekf_timer_period", 50.0);
    this->get_parameter("ekf_timer_period", ekf_timer_period_);

    this->declare_parameter<double>("Q.x", 1e-5);
    this->get_parameter("Q.x", q_x_);

    this->declare_parameter<double>("Q.y", 1e-5);
    this->get_parameter("Q.y", q_y_);

    this->declare_parameter<double>("Q.yaw", 1e-2);
    this->get_parameter("Q.yaw", q_yaw_);

    this->declare_parameter<double>("Q.yaw_rate", 1e-2);
    this->get_parameter("Q.yaw_rate", q_yaw_rate_);

    this->declare_parameter<double>("Q.vx", 1e-3);
    this->get_parameter("Q.vx", q_vx_);

    this->declare_parameter<double>("Q.vy", 1e-3);
    this->get_parameter("Q.vy", q_vy_);

    this->declare_parameter<double>("R.yaw_angle", 0.5);
    this->get_parameter("R.yaw_angle", R_(0,0));

    this->declare_parameter<double>("R.yaw_rate", 0.5);
    this->get_parameter("R.yaw_rate", R_(1,1));

    this->declare_parameter<double>("R.vx", 0.5);
    this->get_parameter("R.vx", R_(2,2));

    R_(0,1) = 0.0; R_(1,0) = 0.0; R_(0,2) = 0.0; R_(2,0) = 0.0; R_(1,2) = 0.0; R_(2,1) = 0.0;
}

void VescToOdom::ekfTimerCallback()
{
    vesc_msgs::msg::VescStateStamped::SharedPtr state_copy;
    vesc_msgs::msg::VescImuStamped::SharedPtr imu_copy;
    std_msgs::msg::Float64::SharedPtr servo_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_copy = last_state_;
        imu_copy = last_imu_state_;
        if (use_servo_cmd_) servo_copy = last_servo_cmd_;
    }

    if (!state_copy || !imu_copy || (use_servo_cmd_ && !servo_copy))
    {
        RCLCPP_WARN(this->get_logger(), "EKF timer called but no data available yet.");
        return;
    }

    rclcpp::Time current_time = now();
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "dt is zero or negative. Skipping EKF update.");
        return;
    }
    last_time_ = current_time;
    // RCLCPP_INFO(this->get_logger(), "EKF timer callback at %f seconds.", dt);

    double v_linear = (state_copy->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(v_linear) < 0.05) v_linear = 0.0;

    double steer = 0.0, kinematic_yaw_rate = 0.0;
    if (use_servo_cmd_)
    {
        double cliped_servo = std::max(servo_min_, std::min(servo_copy->data, servo_max_));
        steer = (cliped_servo - steering_to_servo_offset_) / steering_to_servo_gain_;
        kinematic_yaw_rate = v_linear * tan(steer) / wheelbase_;
    }

    predict(dt, kinematic_yaw_rate);

    double measured_yaw_rate = 0.0;
    double measured_yaw_angle= 0.0;
    // try
    // {
    //     geometry_msgs::msg::TransformStamped imu_to_base_tf =
    //         tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
        
    //     // Transform Angular Velocity
    //     geometry_msgs::msg::Vector3 imu_angular_vel;
    //     imu_angular_vel.x = imu_copy->imu.angular_velocity.x;
    //     imu_angular_vel.y = imu_copy->imu.angular_velocity.y;
    //     imu_angular_vel.z = imu_copy->imu.angular_velocity.z;

    //     geometry_msgs::msg::Vector3 transformed_angular_vel;
    //     tf2::doTransform(imu_angular_vel, transformed_angular_vel, imu_to_base_tf);

    //     if (std::fabs(transformed_angular_vel.z) > 1.0)
    //         measured_yaw_rate = transformed_angular_vel.z * M_PI / 180;

    //     // Transform Orientation
    //     geometry_msgs::msg::Quaternion imu_orientation = imu_copy->imu.orientation;
    //     geometry_msgs::msg::Quaternion transformed_orientation;
    //     tf2::doTransform(imu_orientation, transformed_orientation, imu_to_base_tf);

    //     measured_yaw_angle = tf2::getYaw(transformed_orientation);

    //     if (std::isnan(initial_imu_yaw_))
    //     {
    //         initial_imu_yaw_ = measured_yaw_angle;
    //         RCLCPP_INFO(this->get_logger(), "Initial IMU Yaw Set: %f rad", initial_imu_yaw_);
    //     }
    //     measured_yaw_angle = normalize_angle(measured_yaw_angle - initial_imu_yaw_);
    // }
    // catch (tf2::TransformException &ex)
    // {
    //     RCLCPP_WARN(this->get_logger(), "Failed to get transform from %s to %s: %s",
    //                 imu_frame_.c_str(), base_frame_.c_str(), ex.what());
    //     return;
    // }
    geometry_msgs::msg::TransformStamped imu_to_base_tf;
    imu_to_base_tf.header.frame_id = base_frame_;
    imu_to_base_tf.child_frame_id = imu_frame_;

    imu_to_base_tf.transform.translation.x = 0.20;
    imu_to_base_tf.transform.translation.y = 0.0;
    imu_to_base_tf.transform.translation.z = 0.07; 
    imu_to_base_tf.transform.rotation.x = 0.0;
    imu_to_base_tf.transform.rotation.y = 0.0;
    imu_to_base_tf.transform.rotation.z = sqrt(2) / 2;
    imu_to_base_tf.transform.rotation.w = sqrt(2) / 2;

    geometry_msgs::msg::Vector3 imu_angular_vel;
    imu_angular_vel.x = imu_copy->imu.angular_velocity.x;
    imu_angular_vel.y = imu_copy->imu.angular_velocity.y;
    imu_angular_vel.z = imu_copy->imu.angular_velocity.z;

    geometry_msgs::msg::Vector3 transformed_angular_vel;
    tf2::doTransform(imu_angular_vel, transformed_angular_vel, imu_to_base_tf);

    if (std::fabs(transformed_angular_vel.z) > 1.0)
        measured_yaw_rate = transformed_angular_vel.z * M_PI / 180;

    // Transform Orientation
    geometry_msgs::msg::Quaternion imu_orientation = imu_copy->imu.orientation;
    geometry_msgs::msg::Quaternion transformed_orientation;
    tf2::doTransform(imu_orientation, transformed_orientation, imu_to_base_tf);

    measured_yaw_angle = tf2::getYaw(transformed_orientation);

    if (std::isnan(initial_imu_yaw_))
    {
        initial_imu_yaw_ = measured_yaw_angle;

    }
    measured_yaw_angle = normalize_angle(measured_yaw_angle - initial_imu_yaw_);

    if (!update_imu_)
    {
        // Update EKF with yaw_rate
        updateYawRate(measured_yaw_rate);
    }
    else
    {
        // Update EKF with yaw_angle, yaw_rate
        updateIMU(measured_yaw_angle, measured_yaw_rate, v_linear);
    }

    publishOdometry(current_time);
}

void VescToOdom::predict(double dt, double kinematic_yaw_rate)
{
    double current_yaw = x_(2);
    double current_yaw_rate = x_(3);

    Vector6d x_pred = x_;
    x_pred(0) += (x_(4) * cos(current_yaw) - x_(5) * sin(current_yaw)) * dt; // x_new = x_old + v * cos(yaw) * dt
    x_pred(1) += (x_(4) * sin(current_yaw) + x_(5) * cos(current_yaw)) * dt; // y_new = y_old + v * sin(yaw) * dt
    x_pred(2) += current_yaw_rate * dt;            // yaw_new = yaw_old + yaw_rate * dt
    x_pred(3) = kinematic_yaw_rate;
    x_pred(4) = x_(4);    // v_x
    x_pred(5) = x_(5);    // v_y

    Matrix6d F = Matrix6d::Identity();
    F(0, 2) = -x_(4) * sin(current_yaw) * dt; // d(x_new)/d(yaw_old)
    F(1, 2) =  x_(4) * cos(current_yaw) * dt; // d(y_new)/d(yaw_old)
    F(2, 3) = dt;                             // d(yaw_new)/d(yaw_rate_old)
    F(0, 4) = cos(current_yaw) * dt;          // d(x_new)/d(v_x)
    F(0, 5) = -sin(current_yaw) * dt;
    F(1, 4) = sin(current_yaw) * dt;          // d(y_new)/d(v_x)
    F(1, 5) = cos(current_yaw) * dt;

    Matrix6d Qd = Matrix6d::Zero();
    Qd(0, 0) = q_x_ * dt * dt;        // Noise in X
    Qd(1, 1) = q_y_ * dt * dt;        // Noise in Y
    Qd(2, 2) = q_yaw_ * dt * dt;      // Noise in Yaw
    Qd(3, 3) = q_yaw_rate_ * dt * dt; // Noise in Yaw Rate
    Qd(4, 4) = q_vx_ * dt * dt;      // Noise in Vx
    Qd(5, 5) = q_vy_ * dt * dt;      // Noise in Vy

    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Qd;
}

void VescToOdom::updateYawRate(double measured_yaw_rate)
{
    // Eigen::RowVector4d H;
    // H << 0, 0, 0, 1;

    // Eigen::Matrix<double, 1, 1> z;
    // z << measured_yaw_rate;

    // Eigen::Matrix<double, 1, 1> y = z - H * x_;
    
    // // Kalman Gain
    // Eigen::Matrix<double, 1, 1> R_yaw_rate_ = R_.block<1, 1>(1, 1);

    // Eigen::Matrix<double, 1, 1> S = H * P_ * H.transpose() + R_yaw_rate_;
    // Eigen::Vector4d K = P_ * H.transpose() * S.inverse();

    // // Update state and covariance
    // x_ = x_ + K * y;
    // P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
}

void VescToOdom::updateIMU(double measured_yaw_angle, double measured_yaw_rate, double v_linear)
{
    Eigen::Vector3d z;
    z << measured_yaw_angle, measured_yaw_rate, v_linear;

    Eigen::Vector3d h_x_pred;
    h_x_pred << x_(2), x_(3), x_(4);

    Eigen::Vector3d y = z - h_x_pred;
    y(0) = normalize_angle(y(0)); // Normalize the angle innovation to be within (-PI, PI)

    // Measurement function Jacobian (H) - 2x4 matrix
    Eigen::Matrix<double, 3, 6> H;
    H << 0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0;

    // Innovation covariance
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;

    // Kalman Gain
    Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    // Update state
    x_ = x_ + K * y;
    // Normalize yaw angle after update
    x_(2) = normalize_angle(x_(2));

    // Update covariance
    P_ = (Matrix6d::Identity() - K * H) * P_;
}

void VescToOdom::publishOdometry(const rclcpp::Time& stamp)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_(0);
    odom_msg.pose.pose.position.y = x_(1);
    odom_msg.pose.pose.orientation.z = sin(x_(2) / 2.0);
    odom_msg.pose.pose.orientation.w = cos(x_(2) / 2.0);

    odom_msg.pose.covariance[0] = P_(0,0);   // x variance
    odom_msg.pose.covariance[1] = P_(0,1);   // x-y covariance
    odom_msg.pose.covariance[5] = P_(0,2);   // x-yaw covariance

    odom_msg.pose.covariance[7] = P_(1,1);   // y variance
    odom_msg.pose.covariance[11] = P_(1,2);  // y-yaw covariance

    odom_msg.pose.covariance[35] = P_(2,2);  // vy variance

    odom_msg.twist.twist.linear.x = x_(4);
    odom_msg.twist.twist.linear.y = x_(5);
    odom_msg.twist.twist.angular.z = x_(3);

    odom_msg.twist.covariance[0] = P_(4,4);      // linear x
    odom_msg.twist.covariance[1] = P_(4,5);      // linear x-y
    odom_msg.twist.covariance[5] = P_(4,3);      // linear x-angular z

    odom_msg.twist.covariance[7] = P_(5,5);      // linear y (though always 0 here)
    odom_msg.twist.covariance[11] = P_(5,3);     // linear y-angular z

    odom_msg.twist.covariance[35] = P_(3,3); // angular z

    odom_pub_->publish(odom_msg);

    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.header.stamp = this->now();  // Use current time like original code
        tf.transform.translation.x = x_(0);
        tf.transform.translation.y = x_(1);
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom_msg.pose.pose.orientation;

        if (rclcpp::ok()) {
            tf_pub_->sendTransform(tf);
        }
    }
}

double VescToOdom::normalize_angle(double angle)
{
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

} // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)