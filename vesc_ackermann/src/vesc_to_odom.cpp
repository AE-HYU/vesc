// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <string>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  use_imu_(false),
  publish_tf_(false),
  integration_method_("euler"),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  initial_imu_yaw_(0.0),
  imu_initialized_(false),
  imu_angular_velocity_alpha_(0.3),
  filtered_angular_velocity_(0.0),
  angular_velocity_filter_initialized_(false)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  use_imu_ = declare_parameter("use_imu", use_imu_);
  integration_method_ = declare_parameter("integration_method", integration_method_);
  imu_angular_velocity_alpha_ = declare_parameter("imu_angular_velocity_alpha", imu_angular_velocity_alpha_);

  declare_parameter<double>("speed_to_erpm_gain", 0.0);
  declare_parameter<double>("speed_to_erpm_offset", 0.0);

  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();

  if (use_servo_cmd_) {
    declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
    declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
    declare_parameter<double>("wheelbase", 0.0);

    steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
    steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
    wheelbase_ = get_parameter("wheelbase").get_value<double>();
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // Parameter validation
  if (use_imu_ && use_servo_cmd_) {
    RCLCPP_WARN(get_logger(),
      "Both use_imu and use_servo_cmd are true. IMU will be used for angular velocity and yaw.");
  }

  // Validate integration method
  if (integration_method_ != "euler" && integration_method_ != "trapezoidal" &&
      integration_method_ != "analytical") {
    RCLCPP_WARN(get_logger(),
      "Invalid integration_method '%s'. Using 'euler' as default. "
      "Valid options: 'euler', 'trapezoidal', 'analytical'",
      integration_method_.c_str());
    integration_method_ = "euler";
  }

  RCLCPP_INFO(get_logger(), "Using '%s' integration method for odometry",
              integration_method_.c_str());

  // Get odometry publishing rate parameter (default: 100 Hz)
  odom_publish_rate_ = declare_parameter("odom_publish_rate", 100.0);
  RCLCPP_INFO(get_logger(), "Odometry publishing rate: %.1f Hz", odom_publish_rate_);

  // Maximum history size for backward correction (default: 100 entries)
  max_history_size_ = declare_parameter("max_odom_history_size", 100);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create filtered angular velocity publisher
  if (use_imu_) {
    filtered_angular_velocity_pub_ = create_publisher<Float64>("imu/filtered_angular_velocity", 10);
  }

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  if (use_imu_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "sensors/imu/raw", 10, std::bind(&VescToOdom::imuCallback, this, _1));
  }

  // Create timer for consistent odometry publishing
  auto timer_period = std::chrono::duration<double>(1.0 / odom_publish_rate_);
  odom_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&VescToOdom::odomTimerCallback, this));
}


void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // Queue the incoming VESC state data for processing by timer callback
  std::lock_guard<std::mutex> lock(queue_mutex_);
  vesc_state_queue_.push_back(state);

  // Warn if queue is getting too large (indicates timer can't keep up)
  if (vesc_state_queue_.size() > 50) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "VESC state queue size: %zu (timer may not be keeping up!)",
      vesc_state_queue_.size());
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  // Queue the incoming IMU data for processing by timer callback
  std::lock_guard<std::mutex> lock(queue_mutex_);
  imu_queue_.push_back(imu);

  // Warn if queue is getting too large
  if (imu_queue_.size() > 50) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "IMU queue size: %zu (timer may not be keeping up!)",
      imu_queue_.size());
  }
}

void VescToOdom::odomTimerCallback()
{
  // Determine how many data points we can process
  size_t processable_count = 0;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (use_imu_) {
      // Must process equal number from both queues
      processable_count = std::min(vesc_state_queue_.size(), imu_queue_.size());
    } else {
      processable_count = vesc_state_queue_.size();
    }
  }

  // Process all available data points (backward correction if multiple)
  if (processable_count >= 2) {
    RCLCPP_INFO(get_logger(),
      "=== Backward correction START: %zu buffered data points ===", processable_count);

    // Find the first prediction point in history (oldest predicted entry in consecutive sequence)
    size_t correction_start_idx = odom_history_.size();
    for (size_t i = odom_history_.size(); i > 0; --i) {
      if (odom_history_[i-1].was_predicted) {
        correction_start_idx = i - 1;
      } else {
        // Found a non-predicted entry, stop searching
        break;
      }
    }

    RCLCPP_INFO(get_logger(),
      "History size: %zu, correction_start_idx: %zu, predictions to replace: %zu",
      odom_history_.size(), correction_start_idx,
      correction_start_idx < odom_history_.size() ? odom_history_.size() - correction_start_idx : 0);

    // Restore odometry state from history to the point before first prediction
    if (correction_start_idx > 0) {
      // Restore from the last non-predicted entry (entry just before predictions started)
      const auto& restore_point = odom_history_[correction_start_idx - 1];

      RCLCPP_INFO(get_logger(),
        "Restoring to last real data: timestamp=%.9f s, x=%.3f, y=%.3f, yaw=%.3f",
        restore_point.timestamp.seconds(), restore_point.x, restore_point.y, restore_point.yaw);

      // Restore odometry position and orientation
      x_ = restore_point.x;
      y_ = restore_point.y;
      yaw_ = restore_point.yaw;

      // Restore last_state_ with timestamp for dt calculation
      last_state_ = std::make_shared<VescStateStamped>();
      last_state_->header.stamp = restore_point.timestamp;

      // Clear predicted entries that will be replaced with actual data
      size_t entries_to_remove = std::min(processable_count, odom_history_.size() - correction_start_idx);
      odom_history_.erase(
        odom_history_.begin() + correction_start_idx,
        odom_history_.begin() + correction_start_idx + entries_to_remove
      );

      RCLCPP_INFO(get_logger(),
        "Cleared %zu predicted entries from history. New history size: %zu",
        entries_to_remove, odom_history_.size());
    } else {
      // All history is predicted or empty - shouldn't happen normally
      RCLCPP_WARN(get_logger(),
        "Backward correction: no valid restoration point (correction_start_idx=%zu, history_size=%zu)",
        correction_start_idx, odom_history_.size());
    }

    // Log queue timestamps before processing
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      RCLCPP_INFO(get_logger(), "Queue timestamps:");
      for (size_t i = 0; i < std::min(processable_count, vesc_state_queue_.size()); ++i) {
        double vesc_t = rclcpp::Time(vesc_state_queue_[i]->header.stamp).seconds();
        if (use_imu_ && i < imu_queue_.size()) {
          double imu_t = rclcpp::Time(imu_queue_[i]->header.stamp).seconds();
          RCLCPP_INFO(get_logger(), "  [%zu] VESC: %.9f s, IMU: %.9f s", i, vesc_t, imu_t);
        } else {
          RCLCPP_INFO(get_logger(), "  [%zu] VESC: %.9f s", i, vesc_t);
        }
      }
    }

    // Process all queued data points with timestamp verification
    for (size_t i = 0; i < processable_count; ++i) {
      VescStateStamped::SharedPtr state;
      sensor_msgs::msg::Imu::SharedPtr imu;

      {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        if (vesc_state_queue_.empty()) {
          RCLCPP_WARN(get_logger(), "VESC queue empty during backward correction at iteration %zu", i);
          break;
        }

        state = vesc_state_queue_.front();

        if (use_imu_) {
          if (imu_queue_.empty()) {
            RCLCPP_WARN(get_logger(), "IMU queue empty during backward correction at iteration %zu", i);
            break;
          }

          imu = imu_queue_.front();

          // Verify timestamp synchronization
          rclcpp::Time vesc_time(state->header.stamp);
          rclcpp::Time imu_time(imu->header.stamp);
          double time_diff = std::abs((vesc_time - imu_time).seconds());

          if (time_diff > 1e-6) {  // 1 microsecond tolerance
            RCLCPP_ERROR(get_logger(),
              "[%zu/%zu] CRITICAL: Timestamp mismatch! VESC: %.9f s, IMU: %.9f s (diff: %.6f ms). "
              "Skipping odometry update.",
              i+1, processable_count, vesc_time.seconds(), imu_time.seconds(), time_diff * 1000.0);
            return;  // Skip processing to maintain data integrity
          }
        }

        // Pop from queues
        vesc_state_queue_.pop_front();
        if (use_imu_) {
          imu_queue_.pop_front();
        }
      }

      // Verify timestamp ordering before processing
      if (last_state_) {
        rclcpp::Time current_time(state->header.stamp);
        rclcpp::Time last_time(last_state_->header.stamp);

        RCLCPP_INFO(get_logger(),
          "[%zu/%zu] Processing: current_timestamp=%.9f s, last_timestamp=%.9f s, dt=%.6f ms",
          i+1, processable_count, current_time.seconds(), last_time.seconds(),
          (current_time - last_time).seconds() * 1000.0);

        if (current_time <= last_time) {
          RCLCPP_WARN(get_logger(),
            "[%zu/%zu] Skipping out-of-order data during backward correction! "
            "Current: %.9f s, Last: %.9f s (diff: %.6f ms)",
            i+1, processable_count, current_time.seconds(), last_time.seconds(),
            (current_time - last_time).seconds() * 1000.0);
          continue;  // Skip this out-of-order data point
        }
      }

      processDataPoint(state, imu, false);  // false = not predicted
    }

    RCLCPP_INFO(get_logger(), "=== Backward correction END ===");
  } else if (processable_count == 1) {
    // Normal case: process one data point
    VescStateStamped::SharedPtr state;
    sensor_msgs::msg::Imu::SharedPtr imu;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);

      state = vesc_state_queue_.front();

      if (use_imu_) {
        imu = imu_queue_.front();

        // Verify timestamp synchronization
        rclcpp::Time vesc_time(state->header.stamp);
        rclcpp::Time imu_time(imu->header.stamp);
        double time_diff = std::abs((vesc_time - imu_time).seconds());

        if (time_diff > 1e-6) {
          RCLCPP_ERROR(get_logger(),
            "CRITICAL: Timestamp mismatch! VESC: %.6f s, IMU: %.6f s (diff: %.3f ms). "
            "Skipping odometry update.",
            vesc_time.seconds(), imu_time.seconds(), time_diff * 1000.0);
          return;
        }
      }

      vesc_state_queue_.pop_front();
      if (use_imu_) {
        imu_queue_.pop_front();
      }
    }

    // Verify timestamp ordering before processing
    if (last_state_) {
      rclcpp::Time current_time(state->header.stamp);
      rclcpp::Time last_time(last_state_->header.stamp);

      if (current_time <= last_time) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Skipping out-of-order data! Current: %.9f s, Last: %.9f s (diff: %.6f ms)",
          current_time.seconds(), last_time.seconds(),
          (current_time - last_time).seconds() * 1000.0);
        return;  // Skip this cycle
      }
    }

    processDataPoint(state, imu, false);  // false = not predicted
  } else {
    // No data available: prediction mode
    if (!last_state_) {
      // No previous data yet, skip
      return;
    }

    // Predict one step forward based on timer interval
    // Timestamp = last_timestamp + timer_period
    auto dt = rclcpp::Duration::from_seconds(1.0 / odom_publish_rate_);
    rclcpp::Time predicted_timestamp = rclcpp::Time(last_state_->header.stamp) + dt;

    auto predicted_state = std::make_shared<VescStateStamped>(*last_state_);
    predicted_state->header.stamp = predicted_timestamp;

    processDataPoint(predicted_state, last_imu_, true);  // true = predicted
  }

  // Publish the current odometry (use the timestamp from the last processed data)
  if (!odom_history_.empty()) {
    publishOdometry(odom_history_.back().timestamp);
  }
}

void VescToOdom::processDataPoint(const VescStateStamped::SharedPtr& state,
                                   const sensor_msgs::msg::Imu::SharedPtr& imu,
                                   bool is_prediction)
{
  // Process IMU data if available (update filter and last_imu_)
  if (imu && !is_prediction) {
    last_imu_ = imu;

    // Apply low-pass filter to angular velocity
    double raw_angular_velocity = imu->angular_velocity.z;

    if (!angular_velocity_filter_initialized_) {
      // Initialize filter with first value
      filtered_angular_velocity_ = raw_angular_velocity;
      angular_velocity_filter_initialized_ = true;
    } else {
      // Apply exponential moving average (low-pass filter)
      filtered_angular_velocity_ = imu_angular_velocity_alpha_ * raw_angular_velocity +
                                    (1.0 - imu_angular_velocity_alpha_) * filtered_angular_velocity_;
    }

    // Publish filtered angular velocity
    Float64 filtered_msg;
    filtered_msg.data = filtered_angular_velocity_;
    filtered_angular_velocity_pub_->publish(filtered_msg);
  }

  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  // check that we have IMU data if we are using it
  if (use_imu_ && !last_imu_) {
    return;
  }

  // convert to engineering units
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }

  double current_angular_velocity = 0.0;

  if (use_imu_) {
    // Use filtered IMU yaw rate (angular velocity around z-axis)
    current_angular_velocity = filtered_angular_velocity_;
  } else if (use_servo_cmd_) {
    // Calculate from steering angle
    double current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
    return;
  }

  // calc elapsed time using VESC timestamps (which are now synchronized to request times)
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  // Check for abnormal dt (e.g., node restart, message dropout)
  const double MAX_DT = 1.0;  // 1 second
  if (dt.seconds() > MAX_DT || dt.seconds() < 0) {
    RCLCPP_WARN(get_logger(),
      "Abnormal dt detected: %.6f seconds. Skipping odometry update. "
      "Current stamp: %d.%09d, Last stamp: %d.%09d",
      dt.seconds(),
      state->header.stamp.sec, state->header.stamp.nanosec,
      last_state_->header.stamp.sec, last_state_->header.stamp.nanosec);
    last_state_ = state;
    return;
  }

  // Update yaw first (needed for position integration)
  double yaw_start = yaw_;
  double yaw_end = yaw_;

  if (use_imu_) {
    // Extract yaw from IMU quaternion
    tf2::Quaternion q(
      last_imu_->orientation.x,
      last_imu_->orientation.y,
      last_imu_->orientation.z,
      last_imu_->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_imu_yaw;
    m.getRPY(roll, pitch, current_imu_yaw);

    // Initialize IMU yaw offset on first IMU data
    if (!imu_initialized_) {
      initial_imu_yaw_ = current_imu_yaw;
      imu_initialized_ = true;
      RCLCPP_INFO(get_logger(), "IMU initialized with yaw offset: %.3f rad", initial_imu_yaw_);
    }

    // Apply offset to make initial yaw = 0
    yaw_end = current_imu_yaw - initial_imu_yaw_;
    yaw_ = yaw_end;
  } else {
    yaw_end = yaw_ + current_angular_velocity * dt.seconds();
    yaw_ = yaw_end;
  }

  // Integrate odometry
  integrateOdometry(current_speed, current_angular_velocity, dt.seconds(), yaw_start, yaw_end);

  // Save state for next time
  last_state_ = state;

  // Add to history
  OdomHistoryEntry entry;
  entry.timestamp = rclcpp::Time(state->header.stamp);
  entry.x = x_;
  entry.y = y_;
  entry.yaw = yaw_;
  entry.speed = current_speed;
  entry.angular_velocity = current_angular_velocity;
  entry.was_predicted = is_prediction;

  odom_history_.push_back(entry);

  // Limit history size
  while (odom_history_.size() > max_history_size_) {
    odom_history_.pop_front();
  }
}

void VescToOdom::publishOdometry(const rclcpp::Time& current_time)
{
  // Get current velocity from last processed data
  double current_speed = 0.0;
  double current_angular_velocity = 0.0;

  if (!odom_history_.empty()) {
    const auto& last_entry = odom_history_.back();
    current_speed = last_entry.speed;
    current_angular_velocity = last_entry.angular_velocity;
  }

  // Publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = current_time;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  odom.pose.covariance[0] = 0.2;   ///< x
  odom.pose.covariance[7] = 0.2;   ///< y
  odom.pose.covariance[35] = 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = odom.header.stamp;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::integrateOdometry(double current_speed, double current_angular_velocity,
                                    double dt, double yaw_start, double yaw_end)
{
  // Propagate odometry using selected integration method
  if (integration_method_ == "trapezoidal") {
    // Trapezoidal integration: average velocity at start and end of dt
    double x_dot_start = current_speed * cos(yaw_start);
    double y_dot_start = current_speed * sin(yaw_start);
    double x_dot_end = current_speed * cos(yaw_end);
    double y_dot_end = current_speed * sin(yaw_end);

    // Use average of start and end velocities
    x_ += 0.5 * (x_dot_start + x_dot_end) * dt;
    y_ += 0.5 * (y_dot_start + y_dot_end) * dt;

  } else if (integration_method_ == "analytical") {
    // Analytical solution for Ackermann kinematics (circular arc)
    double delta_yaw = yaw_end - yaw_start;

    if (std::fabs(delta_yaw) < 1e-6) {
      // Nearly straight motion: use simple forward integration
      x_ += current_speed * cos(yaw_start) * dt;
      y_ += current_speed * sin(yaw_start) * dt;
    } else {
      // Circular arc motion: exact solution for constant curvature
      // Use actual delta_yaw to calculate turning radius (more accurate than using angular velocity)
      double actual_angular_velocity = delta_yaw / dt;
      double turning_radius = current_speed / actual_angular_velocity;

      // Calculate displacement in vehicle frame (arc geometry)
      double dx_vehicle = turning_radius * sin(delta_yaw);
      double dy_vehicle = turning_radius * (1.0 - cos(delta_yaw));

      // Transform to global frame using start yaw
      x_ += dx_vehicle * cos(yaw_start) - dy_vehicle * sin(yaw_start);
      y_ += dx_vehicle * sin(yaw_start) + dy_vehicle * cos(yaw_start);
    }

  } else {
    // Default: Euler integration (first-order)
    double x_dot = current_speed * cos(yaw_start);
    double y_dot = current_speed * sin(yaw_start);
    x_ += x_dot * dt;
    y_ += y_dot * dt;
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)