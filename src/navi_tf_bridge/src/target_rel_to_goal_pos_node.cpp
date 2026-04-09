#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "auto_aim_common/msg/relative_target.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navi_tf_bridge
{
class TargetRelToGoalPosNode : public rclcpp::Node
{
public:
  TargetRelToGoalPosNode()
  : Node("target_rel_to_goal_pos_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/ly/navi/target_rel");
    output_goal_pos_topic_ =
      this->declare_parameter<std::string>("output_goal_pos_topic", "/ly/navi/goal_pos");
    output_target_map_topic_ =
      this->declare_parameter<std::string>("output_target_map_topic", "/ly/navi/target_map");

    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    fallback_base_frame_ =
      this->declare_parameter<std::string>("fallback_base_frame", "baselink");
    use_msg_frame_id_ = this->declare_parameter<bool>("use_msg_frame_id", true);

    publish_target_map_ = this->declare_parameter<bool>("publish_target_map", true);
    invert_y_axis_ = this->declare_parameter<bool>("invert_y_axis", false);
    y_axis_max_cm_ = this->declare_parameter<int>("y_axis_max_cm", 1500);

    sub_target_rel_ = this->create_subscription<auto_aim_common::msg::RelativeTarget>(
      input_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::targetRelCallback, this, std::placeholders::_1));

    pub_goal_pos_ =
      this->create_publisher<std_msgs::msg::UInt16MultiArray>(output_goal_pos_topic_, 10);

    if (publish_target_map_) {
      pub_target_map_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>(output_target_map_topic_, 10);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Started target_rel -> goal_pos bridge. in=%s out=%s map_frame=%s base_frame=%s "
      "fallback_base_frame=%s invert_y_axis=%s y_axis_max_cm=%d",
      input_topic_.c_str(),
      output_goal_pos_topic_.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str(),
      fallback_base_frame_.c_str(),
      invert_y_axis_ ? "true" : "false",
      y_axis_max_cm_);
  }

private:
  void targetRelCallback(const auto_aim_common::msg::RelativeTarget::SharedPtr msg)
  {
    if (!msg->valid) {
      return;
    }

    const bool explicit_source_frame = use_msg_frame_id_ && !msg->header.frame_id.empty();
    const std::string requested_source_frame =
      explicit_source_frame ? msg->header.frame_id : base_frame_;

    std::vector<std::string> source_candidates;
    source_candidates.reserve(2);
    source_candidates.push_back(requested_source_frame);
    if (!explicit_source_frame && !fallback_base_frame_.empty() &&
      fallback_base_frame_ != requested_source_frame)
    {
      source_candidates.push_back(fallback_base_frame_);
    }

    geometry_msgs::msg::PointStamped point_in;
    point_in.header.stamp = builtin_interfaces::msg::Time();
    point_in.point.x = static_cast<double>(msg->x);
    point_in.point.y = static_cast<double>(msg->y);
    point_in.point.z = static_cast<double>(msg->z);

    geometry_msgs::msg::PointStamped point_map;
    std::string last_tf_error;
    std::string resolved_source_frame;

    bool transformed = false;
    for (const auto & source_frame : source_candidates) {
      point_in.header.frame_id = source_frame;
      try {
        const geometry_msgs::msg::TransformStamped tf_map_source =
          tf_buffer_.lookupTransform(map_frame_, source_frame, tf2::TimePointZero);
        tf2::doTransform(point_in, point_map, tf_map_source);
        resolved_source_frame = source_frame;
        transformed = true;
        break;
      } catch (const tf2::TransformException & ex) {
        last_tf_error = ex.what();
      }
    }

    if (!transformed) {
      std::ostringstream oss;
      for (const auto & frame : source_candidates) {
        oss << frame << " ";
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF transform failed (%s <- [%s]). Last error: %s",
        map_frame_.c_str(),
        oss.str().c_str(),
        last_tf_error.c_str());
      return;
    }

    point_map.header.frame_id = map_frame_;
    point_map.header.stamp = this->now();

    if (publish_target_map_ && pub_target_map_) {
      pub_target_map_->publish(point_map);
    }

    long x_cm = std::lround(point_map.point.x * 100.0);
    long y_cm = std::lround(point_map.point.y * 100.0);

    if (invert_y_axis_) {
      y_cm = static_cast<long>(y_axis_max_cm_) - y_cm;
    }

    const long kU16Min = 0L;
    const long kU16Max = static_cast<long>(std::numeric_limits<std::uint16_t>::max());
    if (x_cm < kU16Min || x_cm > kU16Max || y_cm < kU16Min || y_cm > kU16Max) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Drop transformed point (frame %s): map_xy=(%.3f, %.3f)m -> cm=(%ld, %ld) out of "
        "UInt16 range",
        resolved_source_frame.c_str(),
        point_map.point.x,
        point_map.point.y,
        x_cm,
        y_cm);
      return;
    }

    std_msgs::msg::UInt16MultiArray out;
    out.data = {
      static_cast<std::uint16_t>(x_cm),
      static_cast<std::uint16_t>(y_cm)};
    pub_goal_pos_->publish(out);
  }

  std::string input_topic_;
  std::string output_goal_pos_topic_;
  std::string output_target_map_topic_;

  std::string map_frame_;
  std::string base_frame_;
  std::string fallback_base_frame_;
  bool use_msg_frame_id_{true};

  bool publish_target_map_{true};
  bool invert_y_axis_{false};
  int y_axis_max_cm_{1500};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<auto_aim_common::msg::RelativeTarget>::SharedPtr sub_target_rel_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_goal_pos_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_map_;
};
}  // namespace navi_tf_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navi_tf_bridge::TargetRelToGoalPosNode>());
  rclcpp::shutdown();
  return 0;
}
