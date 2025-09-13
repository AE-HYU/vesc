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
    
    // Initialize EKF state and covariance
    x_.setZero();                   // Initial state vector [x, y, yaw, yaw_rate, vx, vy]
    P_.setIdentity(); P_ *= 0.1;    // Initial covariance matrix
    last_time_ = this->now();

    // Initialize TF Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // EKF update timer
    ekf_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ekf_timer_period_), std::bind(&VescToOdom::ekfTimerCallback, this));
}

void VescToOdom::setParams()
{
    // Frame parameters
    this->declare_parameter("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_);
    this->declare_parameter("base_frame", "base_link");
    this->get_parameter("base_frame", base_frame_);
    this->declare_parameter("imu_frame", "imu_link");
    this->get_parameter("imu_frame", imu_frame_);

    // Control parameters
    this->declare_parameter("use_servo_cmd", true);
    this->get_parameter("use_servo_cmd", use_servo_cmd_);
    this->declare_parameter("publish_tf", true);
    this->get_parameter("publish_tf", publish_tf_);
    this->declare_parameter("update_imu", true);
    this->get_parameter("update_imu", update_imu_);

    // Vehicle kinematic parameters
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

    // EKF timing parameter
    this->declare_parameter<double>("ekf_timer_period", 50.0);
    this->get_parameter("ekf_timer_period", ekf_timer_period_);

    // Process noise parameters (Q matrix)
    this->declare_parameter<double>("Q.x", 1e-5);
    this->get_parameter("Q.x", q_x_);
    this->declare_parameter<double>("Q.y", 1e-5);
    this->get_parameter("Q.y", q_y_);
    this->declare_parameter<double>("Q.yaw", 1e-3);
    this->get_parameter("Q.yaw", q_yaw_);
    this->declare_parameter<double>("Q.yaw_rate", 1e-3);
    this->get_parameter("Q.yaw_rate", q_yaw_rate_);
    this->declare_parameter<double>("Q.vx", 1e-3);
    this->get_parameter("Q.vx", q_vx_);
    this->declare_parameter<double>("Q.vy", 1e-3);
    this->get_parameter("Q.vy", q_vy_);

    // Measurement noise parameters (R matrix)
    this->declare_parameter<double>("R.yaw_angle", 0.001);
    this->get_parameter("R.yaw_angle", R_(0,0));
    this->declare_parameter<double>("R.yaw_rate", 0.001);
    this->get_parameter("R.yaw_rate", R_(1,1));
    this->declare_parameter<double>("R.vx", 0.001);
    this->get_parameter("R.vx", R_(2,2));

    // Initialize off-diagonal elements to zero
    R_(0,1) = 0.0; R_(1,0) = 0.0; R_(0,2) = 0.0;
    R_(2,0) = 0.0; R_(1,2) = 0.0; R_(2,1) = 0.0;
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

    // Convert ERPM to linear velocity and apply deadband
    double v_linear = (state_copy->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(v_linear) < 0.05) v_linear = 0.0;

    // Calculate kinematic yaw rate from steering command
    double steer = 0.0, kinematic_yaw_rate = 0.0;
    if (use_servo_cmd_)
    {
        double clipped_servo = std::max(servo_min_, std::min(servo_copy->data, servo_max_));
        steer = (clipped_servo - steering_to_servo_offset_) / steering_to_servo_gain_;
        kinematic_yaw_rate = v_linear * tan(steer) / wheelbase_;
    }

    // EKF Prediction step
    predict(dt, kinematic_yaw_rate);

    // Process IMU measurements
    double measured_yaw_rate = 0.0;
    double measured_yaw_angle = 0.0;
    // Define static transform from IMU to base frame
    geometry_msgs::msg::TransformStamped imu_to_base_tf;
    imu_to_base_tf.header.frame_id = base_frame_;
    imu_to_base_tf.child_frame_id = imu_frame_;
    imu_to_base_tf.transform.translation.x = 0.20;
    imu_to_base_tf.transform.translation.y = 0.0;
    imu_to_base_tf.transform.translation.z = 0.07;
    imu_to_base_tf.transform.rotation.x = 0.0;
    imu_to_base_tf.transform.rotation.y = 0.0;
    imu_to_base_tf.transform.rotation.z = 0.0;
    imu_to_base_tf.transform.rotation.w = 1.0;

    // Transform IMU angular velocity to base frame
    geometry_msgs::msg::Vector3 imu_angular_vel;
    imu_angular_vel.x = imu_copy->imu.angular_velocity.x;
    imu_angular_vel.y = imu_copy->imu.angular_velocity.y;
    imu_angular_vel.z = imu_copy->imu.angular_velocity.z;

    geometry_msgs::msg::Vector3 transformed_angular_vel;
    tf2::doTransform(imu_angular_vel, transformed_angular_vel, imu_to_base_tf);

    // Use yaw rate measurement if above threshold
    if (std::fabs(transformed_angular_vel.z) > 1.0)
    {
        measured_yaw_rate = transformed_angular_vel.z * M_PI / 180;
    }

    // Transform IMU orientation to base frame
    geometry_msgs::msg::Quaternion imu_orientation = imu_copy->imu.orientation;
    geometry_msgs::msg::Quaternion transformed_orientation;
    tf2::doTransform(imu_orientation, transformed_orientation, imu_to_base_tf);

    measured_yaw_angle = tf2::getYaw(transformed_orientation);

    // Initialize reference yaw angle on first measurement
    if (std::isnan(initial_imu_yaw_))
    {
        initial_imu_yaw_ = measured_yaw_angle;
    }
    measured_yaw_angle = normalize_angle(measured_yaw_angle - initial_imu_yaw_);

    if (!update_imu_)
    {
        // Skip IMU update - only use prediction
    }
    else
    {
        // Update EKF with yaw_angle, yaw_rate
        updateIMU(measured_yaw_angle, measured_yaw_rate, v_linear);
    }

    // Publish the updated odometry
    publishOdometry(current_time);
}

void VescToOdom::predict(double dt, double kinematic_yaw_rate)
{
    double current_yaw = x_(2);
    double current_yaw_rate = x_(3);

    // Predict next state using kinematic model
    Vector6d x_pred = x_;
    x_pred(0) += (x_(4) * cos(current_yaw) - x_(5) * sin(current_yaw)) * dt; // x position
    x_pred(1) += (x_(4) * sin(current_yaw) + x_(5) * cos(current_yaw)) * dt; // y position
    x_pred(2) += current_yaw_rate * dt;            // yaw angle
    x_pred(3) = kinematic_yaw_rate;                // yaw rate from kinematics
    x_pred(4) = x_(4);                             // vx (unchanged)
    x_pred(5) = x_(5);                             // vy (unchanged)

    // Jacobian matrix for state transition
    Matrix6d F = Matrix6d::Identity();
    F(0, 2) = -x_(4) * sin(current_yaw) * dt; // dx/dyaw
    F(1, 2) =  x_(4) * cos(current_yaw) * dt; // dy/dyaw
    F(2, 3) = dt;                             // dyaw/dyaw_rate
    F(0, 4) = cos(current_yaw) * dt;          // dx/dvx
    F(0, 5) = -sin(current_yaw) * dt;         // dx/dvy
    F(1, 4) = sin(current_yaw) * dt;          // dy/dvx
    F(1, 5) = cos(current_yaw) * dt;          // dy/dvy

    // Process noise covariance matrix
    Matrix6d Qd = Matrix6d::Zero();
    Qd(0, 0) = q_x_ * dt * dt;        // Position x noise
    Qd(1, 1) = q_y_ * dt * dt;        // Position y noise
    Qd(2, 2) = q_yaw_ * dt * dt;      // Yaw angle noise
    Qd(3, 3) = q_yaw_rate_ * dt * dt; // Yaw rate noise
    Qd(4, 4) = q_vx_ * dt * dt;       // Velocity x noise
    Qd(5, 5) = q_vy_ * dt * dt;       // Velocity y noise

    // Update state and covariance
    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Qd;
}


void VescToOdom::updateIMU(double measured_yaw_angle, double measured_yaw_rate, double v_linear)
{
    // Measurement vector [yaw_angle, yaw_rate, velocity]
    Eigen::Vector3d z;
    z << measured_yaw_angle, measured_yaw_rate, v_linear;

    // Expected measurement from current state
    Eigen::Vector3d h_x_pred;
    h_x_pred << x_(2), x_(3), x_(4);

    // Innovation (measurement residual)
    Eigen::Vector3d y = z - h_x_pred;
    y(0) = normalize_angle(y(0)); // Normalize angle innovation

    // Measurement Jacobian matrix H (3x6)
    Eigen::Matrix<double, 3, 6> H;
    H << 0, 0, 1, 0, 0, 0,  // yaw angle measurement
         0, 0, 0, 1, 0, 0,  // yaw rate measurement
         0, 0, 0, 0, 1, 0;  // velocity measurement

    // Innovation covariance
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;

    // Kalman gain
    Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    // Update state estimate
    x_ = x_ + K * y;
    x_(2) = normalize_angle(x_(2)); // Normalize yaw angle

    // Update covariance estimate
    P_ = (Matrix6d::Identity() - K * H) * P_;
}

void VescToOdom::publishOdometry(const rclcpp::Time& stamp)
{
    // Create odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Set pose from state estimate
    odom_msg.pose.pose.position.x = x_(0);
    odom_msg.pose.pose.position.y = x_(1);
    odom_msg.pose.pose.orientation.z = sin(x_(2) / 2.0);
    odom_msg.pose.pose.orientation.w = cos(x_(2) / 2.0);

    // Set pose covariance
    odom_msg.pose.covariance[0] = P_(0,0);   // x variance
    odom_msg.pose.covariance[1] = P_(0,1);   // x-y covariance
    odom_msg.pose.covariance[5] = P_(0,2);   // x-yaw covariance
    odom_msg.pose.covariance[7] = P_(1,1);   // y variance
    odom_msg.pose.covariance[11] = P_(1,2);  // y-yaw covariance
    odom_msg.pose.covariance[35] = P_(2,2);  // yaw variance

    // Set twist from state estimate
    odom_msg.twist.twist.linear.x = x_(4);
    odom_msg.twist.twist.linear.y = x_(5);
    odom_msg.twist.twist.angular.z = x_(3);

    // Set twist covariance
    odom_msg.twist.covariance[0] = P_(4,4);   // vx variance
    odom_msg.twist.covariance[1] = P_(4,5);   // vx-vy covariance
    odom_msg.twist.covariance[5] = P_(4,3);   // vx-angular covariance
    odom_msg.twist.covariance[7] = P_(5,5);   // vy variance
    odom_msg.twist.covariance[11] = P_(5,3);  // vy-angular covariance
    odom_msg.twist.covariance[35] = P_(3,3);  // angular variance

    // Publish odometry message
    odom_pub_->publish(odom_msg);

    // Publish TF transform if enabled
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.header.stamp = this->now();
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

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)