// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file subscriber_member_function.cpp
 * @author Tej Kiran (itej89@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <functional>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

/**
 * @brief ROS Node Class processess pose messages and publishes over odometry topic
 *
 */
class PoseToOdom : public rclcpp::Node {
 public:
  PoseToOdom() : Node("PoseToOdom") {

    /**
     * @brief Create a publisher to the topic "topic"
     *
     */
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/laser_odom", 10);


    /**
     * @brief Create the subscription to the "topic"
     *
     */
    subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 10,
            std::bind(&PoseToOdom::pose_callback, this, _1));

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "PoseToOdom node started.");

    /**
     * @brief Create timer to publish the message at periodic intervals
     *
     */
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PoseToOdom::timer_callback, this));
  }

 private:

   /**
   * @brief Timer callback that publishes messages periodically
   *
   */
  void timer_callback() {
    if (_mutex.try_lock()) {
    
        /**
        * @brief Conver pose to odom message type
        * 
        */
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = pose_msg.header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = pose_msg.pose.position.x;
        odom_msg.pose.pose.position.y = pose_msg.pose.position.y;
        odom_msg.pose.pose.position.z = pose_msg.pose.position.z;
        odom_msg.pose.pose.orientation.x = pose_msg.pose.orientation.x;
        odom_msg.pose.pose.orientation.y = pose_msg.pose.orientation.y;
        odom_msg.pose.pose.orientation.z = pose_msg.pose.orientation.z;
        odom_msg.pose.pose.orientation.w = pose_msg.pose.orientation.w;

        /**
         * @brief Publish the odom message
         * 
         */
        publisher_->publish(odom_msg);

        _mutex.unlock();
    }
  }


  /**
   * @brief Create a callback for the topic
   *
   * @param msg
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped& msg) {

    std::lock_guard<std::mutex> m(_mutex);

    pose_msg.header.stamp.sec = msg.header.stamp.sec;
    pose_msg.header.stamp.nanosec = msg.header.stamp.nanosec;
    pose_msg.pose.position.x = msg.pose.position.x;
    pose_msg.pose.position.y = msg.pose.position.y;
    pose_msg.pose.position.z = msg.pose.position.z;
    pose_msg.pose.orientation.x = msg.pose.orientation.x;
    pose_msg.pose.orientation.y = msg.pose.orientation.y;
    pose_msg.pose.orientation.z = msg.pose.orientation.z;
    pose_msg.pose.orientation.w = msg.pose.orientation.w;

  }

  /**
   * @brief Timer parameter
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Pointer for adding subscription
   *
   */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_;

        /**
   * @brief Pointer for the publisher
   *
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr 
    publisher_;


   geometry_msgs::msg::PoseStamped pose_msg;
   std::mutex _mutex;
};

/**
 * @brief Main funciton  for the subscriber node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToOdom>());
  rclcpp::shutdown();
  return 0;
}