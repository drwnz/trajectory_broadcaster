#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "trajectory_broadcaster/trajectory_broadcaster_node.hpp"

std::vector<std::vector<float>> parsePoses(const std::string &input, std::string &error_return)
{
  std::vector<std::vector<float>> result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
    case EOF:
      break;
    case '{':
      depth++;
      if (depth > 2)
      {
        error_return = "Array depth greater than 2";
        return result;
      }
      input_ss.get();
      current_vector.clear();
      break;
    case '}':
      depth--;
      if (depth < 0)
      {
        error_return = "More close ] than open [";
        return result;
      }
      input_ss.get();
      if (depth == 1)
      {
        result.push_back(current_vector);
      }
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default: // All other characters should be part of the numbers.
      if (depth != 2)
      {
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
        error_return = err_ss.str();
        return result;
      }
      float value;
      input_ss >> value;
      if (!!input_ss)
      {
        current_vector.push_back(value);
      }
      break;
    }
  }
  if (depth != 0)
  {
    error_return = "Unterminated vector string.";
  }
  else
  {
    error_return = "";
  }
  return result;
}

TrajectoryBroadcasterNode::TrajectoryBroadcasterNode(const rclcpp::NodeOptions &options)
    : Node("trajectory_broadcaster_node", options)
{
  input_frame_ = this->declare_parameter<std::string>("input_frame", "base_link");
  output_frame_ = this->declare_parameter<std::string>("output_frame", "flying_link");
  sequence_interval_ = this->declare_parameter<int>("sequence_interval", 10);
  const auto poses_string = this->declare_parameter<std::string>("target_poses", "{0, 0, 0, 0, 0, 0}");

  expandPoses(poses_string);
  // Handle error return later
  pose_counter_ = 0;

  // Initialize the transform broadcaster
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Just for testing (time)
  // using std::placeholders::_1;
  // pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "/pandar_points", rclcpp::SensorDataQoS().keep_last(1),
  //     std::bind(&TrajectoryBroadcasterNode::onPointCloud, this, _1));

  transform_.header.frame_id = input_frame_.c_str();
  transform_.child_frame_id = output_frame_.c_str();
  self_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TrajectoryBroadcasterNode::timerCallback, this));
}

void TrajectoryBroadcasterNode::timerCallback()
{

  // transform_.header.stamp = last_message_time_;
  //  transform_.transform.translation.x += 0.1;
  transform_.transform = poses_vector_[pose_counter_];
  transform_.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(transform_);
  if (pose_counter_ == 0)
  {
    // Keep waiting for the sequence to sync up with system time interval
    if (transform_.header.stamp.sec % sequence_interval_ == 0)
    {
      pose_counter_++;
      RCLCPP_WARN(this->get_logger(), "Sarting to move at %d", transform_.header.stamp.sec);
    }
  }
  else
  {
    pose_counter_++;
  }
  if (pose_counter_ == poses_vector_.size())
  {
    pose_counter_ = 0;
  }
}

void TrajectoryBroadcasterNode::onPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  // last_message_time_ = input_msg->header.stamp;
  // RCLCPP_WARN(this->get_logger(), "Time %d", last_message_time_.seconds);
  transform_.header.stamp = input_msg->header.stamp;
  transform_.header.stamp.sec += 1;

  // transform_.transform.translation.x += 0.1;
  transform_.transform = poses_vector_[pose_counter_];
  tf_broadcaster_->sendTransform(transform_);
  pose_counter_++;
  if (pose_counter_ == poses_vector_.size())
  {
    pose_counter_ = 0;
  }
}

void TrajectoryBroadcasterNode::expandPoses(const std::string &input)
{
  uint num_sub_poses = 100;
  std::string parse_error;
  std::vector<std::vector<float>> key_poses = parsePoses(input, parse_error);
  tf2::Quaternion quaternion;
  geometry_msgs::msg::Transform current_key_pose;

  quaternion.setRPY(key_poses[0][3], key_poses[0][4], key_poses[0][5]);
  quaternion = quaternion.normalize();
  current_key_pose.translation.x = key_poses[0][0];
  current_key_pose.translation.y = key_poses[0][1];
  current_key_pose.translation.z = key_poses[0][2];
  quaternion.setRPY(key_poses[0][3], key_poses[0][4], key_poses[0][5]);
  quaternion = quaternion.normalize();
  current_key_pose.rotation.x = quaternion.x();
  current_key_pose.rotation.y = quaternion.y();
  current_key_pose.rotation.z = quaternion.z();
  current_key_pose.rotation.w = quaternion.w();

  poses_vector_.push_back(current_key_pose);
  float x_step, y_step, z_step, roll_step, pitch_step, yaw_step;
  for (uint i = 1; i < key_poses.size(); i++)
  {
    x_step = (key_poses[i][0] - key_poses[i - 1][0]) / (float)num_sub_poses;
    y_step = (key_poses[i][1] - key_poses[i - 1][1]) / (float)num_sub_poses;
    z_step = (key_poses[i][2] - key_poses[i - 1][2]) / (float)num_sub_poses;
    roll_step = (key_poses[i][3] - key_poses[i - 1][3]) / (float)num_sub_poses;
    pitch_step = (key_poses[i][4] - key_poses[i - 1][4]) / (float)num_sub_poses;
    yaw_step = (key_poses[i][5] - key_poses[i - 1][5]) / (float)num_sub_poses;

    for (uint j = 1; j < num_sub_poses; j++)
    {
      geometry_msgs::msg::Transform sub_pose;
      sub_pose.translation.x = key_poses[i - 1][0] + x_step * j;
      sub_pose.translation.y = key_poses[i - 1][1] + y_step * j;
      sub_pose.translation.z = key_poses[i - 1][2] + z_step * j;
      quaternion.setRPY(key_poses[i - 1][3] + roll_step * j, key_poses[i - 1][4] + pitch_step * j, key_poses[i - 1][5] + yaw_step * j);
      quaternion = quaternion.normalize();
      sub_pose.rotation.x = quaternion.x();
      sub_pose.rotation.y = quaternion.y();
      sub_pose.rotation.z = quaternion.z();
      sub_pose.rotation.w = quaternion.w();
      poses_vector_.push_back(sub_pose);
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryBroadcasterNode)
