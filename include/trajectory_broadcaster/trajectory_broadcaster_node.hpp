// Copyright 2022 Tier IV, Inc.
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

#ifndef TRAJECTORY_BORADCASTER__TRAJECTORY_BORADCASTER_NODE_HPP_
#define TRAJECTORY_BORADCASTER__TRAJECTORY_BORADCASTER_NODE_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"

class TrajectoryBroadcasterNode : public rclcpp::Node
{
public:
  explicit TrajectoryBroadcasterNode(const rclcpp::NodeOptions &options);

private:
  geometry_msgs::msg::TransformStamped transform_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::TimerBase::SharedPtr self_timer_;
  std::string input_frame_;
  std::string output_frame_;
  std::vector<geometry_msgs::msg::Transform> poses_vector_;
  uint pose_counter_;
  rclcpp::Time last_message_time_;
  uint sequence_interval_;

  void timerCallback();
  void onPointCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void expandPoses(const std::string &input);
};

#endif // TRAJECTORY_BORADCASTER__TRAJECTORY_BORADCASTER_NODE_HPP_
