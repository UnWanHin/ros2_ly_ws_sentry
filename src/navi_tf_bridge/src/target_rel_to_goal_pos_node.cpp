#include <cmath>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
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
    input_goal_pos_raw_topic_ =
      this->declare_parameter<std::string>("input_goal_pos_raw_topic", "/ly/navi/goal_pos_raw");
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
    preferred_distance_cm_ = this->declare_parameter<int>("preferred_distance_cm", 100);
    distance_deadband_cm_ = this->declare_parameter<int>("distance_deadband_cm", 50);
    stop_when_no_target_ = this->declare_parameter<bool>("stop_when_no_target", true);
    allow_reverse_goal_ = this->declare_parameter<bool>("allow_reverse_goal", false);
    enable_goal_pos_raw_bridge_ =
      this->declare_parameter<bool>("enable_goal_pos_raw_bridge", true);
    goal_pos_raw_frame_ = this->declare_parameter<std::string>("goal_pos_raw_frame", "map");
    debug_export_point_pairs_ = this->declare_parameter<bool>("debug_export_point_pairs", true);
    debug_points_reference_frame_ =
      this->declare_parameter<std::string>("debug_points_reference_frame", "map");
    debug_area_header_file_ = this->declare_parameter<std::string>("debug_area_header_file", "");
    debug_point_pairs_output_file_ =
      this->declare_parameter<std::string>("debug_point_pairs_output_file", "");

    sub_target_rel_ = this->create_subscription<auto_aim_common::msg::RelativeTarget>(
      input_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::targetRelCallback, this, std::placeholders::_1));
    sub_goal_pos_raw_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      input_goal_pos_raw_topic_,
      rclcpp::QoS(10),
      std::bind(&TargetRelToGoalPosNode::goalPosRawCallback, this, std::placeholders::_1));

    pub_goal_pos_ =
      this->create_publisher<std_msgs::msg::UInt16MultiArray>(output_goal_pos_topic_, 10);

    if (publish_target_map_) {
      pub_target_map_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>(output_target_map_topic_, 10);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Started target_rel -> goal_pos bridge. in=%s out=%s map_frame=%s base_frame=%s "
      "fallback_base_frame=%s raw_goal_in=%s raw_goal_frame=%s invert_y_axis=%s y_axis_max_cm=%d preferred_distance_cm=%d "
      "distance_deadband_cm=%d stop_when_no_target=%s allow_reverse_goal=%s",
      input_topic_.c_str(),
      output_goal_pos_topic_.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str(),
      fallback_base_frame_.c_str(),
      input_goal_pos_raw_topic_.c_str(),
      goal_pos_raw_frame_.c_str(),
      invert_y_axis_ ? "true" : "false",
      y_axis_max_cm_,
      preferred_distance_cm_,
      distance_deadband_cm_,
      stop_when_no_target_ ? "true" : "false",
      allow_reverse_goal_ ? "true" : "false");

    if (debug_export_point_pairs_) {
      debug_export_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TargetRelToGoalPosNode::onDebugExportTimer, this));
      RCLCPP_INFO(
        this->get_logger(),
        "TF debug export enabled. ref_frame=%s area_header=%s out=%s",
        debug_points_reference_frame_.c_str(),
        debug_area_header_file_.empty() ? "<empty>" : debug_area_header_file_.c_str(),
        debug_point_pairs_output_file_.empty() ? "<empty>" : debug_point_pairs_output_file_.c_str());
    }
  }

private:
  std::vector<std::string> buildSourceCandidates(const std::string & msg_frame_id) const
  {
    const bool explicit_source_frame = use_msg_frame_id_ && !msg_frame_id.empty();
    const std::string requested_source_frame =
      explicit_source_frame ? msg_frame_id : base_frame_;

    std::vector<std::string> source_candidates;
    source_candidates.reserve(2);
    source_candidates.push_back(requested_source_frame);
    if (!explicit_source_frame && !fallback_base_frame_.empty() &&
      fallback_base_frame_ != requested_source_frame)
    {
      source_candidates.push_back(fallback_base_frame_);
    }
    return source_candidates;
  }

  struct NamedPointCm
  {
    std::string name;
    int x_cm{0};
    int y_cm{0};
  };

  bool parseAreaHeaderPoints(std::vector<NamedPointCm> & points_out) const
  {
    points_out.clear();
    if (debug_area_header_file_.empty()) {
      return false;
    }

    std::ifstream ifs(debug_area_header_file_);
    if (!ifs.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: cannot open area header file: %s",
        debug_area_header_file_.c_str());
      return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    // Match lines like:
    // Location<std::uint16_t> Home{ {393, 810}, {2408, 683} };
    const std::regex pattern(
      R"(Location<\s*std::uint16_t\s*>\s*([A-Za-z0-9_]+)\s*\{\s*\{\s*([0-9]+)\s*,\s*([0-9]+)\s*\}\s*,\s*\{\s*([0-9]+)\s*,\s*([0-9]+)\s*\}\s*\}\s*;)");

    std::sregex_iterator it(content.begin(), content.end(), pattern);
    std::sregex_iterator end;
    for (; it != end; ++it) {
      const auto & m = *it;
      const std::string name = m[1].str();
      const int red_x = std::stoi(m[2].str());
      const int red_y = std::stoi(m[3].str());
      const int blue_x = std::stoi(m[4].str());
      const int blue_y = std::stoi(m[5].str());
      points_out.push_back(NamedPointCm{name + ".red", red_x, red_y});
      points_out.push_back(NamedPointCm{name + ".blue", blue_x, blue_y});
    }

    if (points_out.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: no Location<std::uint16_t> points found in %s",
        debug_area_header_file_.c_str());
      return false;
    }

    return true;
  }

  bool lookupTransformWithCandidates(
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    geometry_msgs::msg::TransformStamped & tf_out,
    std::string & resolved_source_frame,
    std::string & last_tf_error) const
  {
    for (const auto & source_frame : source_candidates) {
      try {
        tf_out = tf_buffer_.lookupTransform(
          map_frame_,
          source_frame,
          transform_time,
          rclcpp::Duration::from_seconds(0.05));
        resolved_source_frame = source_frame;
        return true;
      } catch (const tf2::TransformException & ex) {
        last_tf_error = ex.what();
      }
    }
    return false;
  }

  bool exportDebugPointPairs(
    const std::string & tf_ready_source_frame,
    const geometry_msgs::msg::TransformStamped & tf_map_ref)
  {
    std::vector<NamedPointCm> points_cm;
    if (!parseAreaHeaderPoints(points_cm)) {
      return false;
    }
    if (debug_point_pairs_output_file_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: debug_point_pairs_output_file is empty.");
      return false;
    }

    std::filesystem::path out_path(debug_point_pairs_output_file_);
    if (out_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(out_path.parent_path(), ec);
      if (ec) {
        RCLCPP_WARN(
          this->get_logger(),
          "TF debug export: failed to create parent dir for %s: %s",
          debug_point_pairs_output_file_.c_str(),
          ec.message().c_str());
        return false;
      }
    }

    std::ofstream ofs(debug_point_pairs_output_file_, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: failed to open output file: %s",
        debug_point_pairs_output_file_.c_str());
      return false;
    }

    ofs << "meta:\n";
    ofs << "  generated_ns: " << this->now().nanoseconds() << "\n";
    ofs << "  map_frame: " << map_frame_ << "\n";
    ofs << "  tf_ready_source_frame: " << tf_ready_source_frame << "\n";
    ofs << "  reference_frame: " << debug_points_reference_frame_ << "\n";
    ofs << "  area_header_file: " << debug_area_header_file_ << "\n";
    ofs << "points:\n";

    for (const auto & point : points_cm) {
      geometry_msgs::msg::PointStamped in;
      in.header.frame_id = debug_points_reference_frame_;
      in.header.stamp = tf_map_ref.header.stamp;
      in.point.x = static_cast<double>(point.x_cm) * 0.01;
      in.point.y = static_cast<double>(point.y_cm) * 0.01;
      in.point.z = 0.0;

      geometry_msgs::msg::PointStamped out;
      tf2::doTransform(in, out, tf_map_ref);
      const long tx_cm = std::lround(out.point.x * 100.0);
      const long ty_cm = std::lround(out.point.y * 100.0);
      const long dx_cm = tx_cm - static_cast<long>(point.x_cm);
      const long dy_cm = ty_cm - static_cast<long>(point.y_cm);

      ofs << "  - name: " << point.name << "\n";
      ofs << "    original_cm: {x: " << point.x_cm << ", y: " << point.y_cm << "}\n";
      ofs << "    transformed_cm: {x: " << tx_cm << ", y: " << ty_cm << "}\n";
      ofs << "    delta_cm: {dx: " << dx_cm << ", dy: " << dy_cm << "}\n";
    }

    return true;
  }

  void onDebugExportTimer()
  {
    if (!debug_export_point_pairs_ || debug_point_pairs_exported_) {
      if (debug_export_timer_) {
        debug_export_timer_->cancel();
      }
      return;
    }

    const auto source_candidates = buildSourceCandidates("");
    geometry_msgs::msg::TransformStamped tf_map_source;
    std::string last_tf_error;
    std::string resolved_source_frame;
    const rclcpp::Time latest_time(0, 0, this->get_clock()->get_clock_type());
    if (!lookupTransformWithCandidates(
        source_candidates, latest_time, tf_map_source, resolved_source_frame, last_tf_error))
    {
      if (!debug_tf_missing_logged_once_) {
        std::ostringstream oss;
        for (const auto & frame : source_candidates) {
          oss << frame << " ";
        }
        RCLCPP_WARN(
          this->get_logger(),
          "TF frame chain not ready yet (%s <- [%s]). Last error: %s",
          map_frame_.c_str(),
          oss.str().c_str(),
          last_tf_error.c_str());
        debug_tf_missing_logged_once_ = true;
      }
      return;
    }

    if (!debug_tf_ready_logged_once_) {
      RCLCPP_INFO(
        this->get_logger(),
        "TF frame chain ready: %s <- %s",
        map_frame_.c_str(),
        resolved_source_frame.c_str());
      debug_tf_ready_logged_once_ = true;
    }

    geometry_msgs::msg::TransformStamped tf_map_ref;
    std::string tf_ref_error;
    std::string tf_ref_resolved;
    if (!lookupTransformWithCandidates(
        {debug_points_reference_frame_},
        latest_time,
        tf_map_ref,
        tf_ref_resolved,
        tf_ref_error))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "TF debug export: failed to resolve reference frame (%s <- %s): %s",
        map_frame_.c_str(),
        debug_points_reference_frame_.c_str(),
        tf_ref_error.c_str());
      if (debug_export_timer_) {
        debug_export_timer_->cancel();
      }
      return;
    }

    if (exportDebugPointPairs(resolved_source_frame, tf_map_ref)) {
      debug_point_pairs_exported_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "TF debug point pairs exported: %s",
        debug_point_pairs_output_file_.c_str());
    }
    if (debug_export_timer_) {
      debug_export_timer_->cancel();
    }
  }

  bool buildRelativeGoalPoint(
    const auto_aim_common::msg::RelativeTarget & msg,
    geometry_msgs::msg::Point & point_out)
  {
    point_out.x = 0.0;
    point_out.y = 0.0;
    point_out.z = 0.0;

    if (!msg.valid) {
      return stop_when_no_target_;
    }

    const double target_x = static_cast<double>(msg.x);
    const double target_y = static_cast<double>(msg.y);
    if (!std::isfinite(target_x) || !std::isfinite(target_y)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Drop invalid target_rel: x=%.3f y=%.3f",
        target_x,
        target_y);
      return false;
    }

    const double planar_distance_m = std::hypot(target_x, target_y);
    constexpr double kMinPlanarDistanceM = 1e-6;
    if (planar_distance_m <= kMinPlanarDistanceM) {
      return true;
    }

    const double preferred_distance_m = static_cast<double>(preferred_distance_cm_) * 0.01;
    const double deadband_m = static_cast<double>(distance_deadband_cm_) * 0.01;
    const double move_distance_m = planar_distance_m - preferred_distance_m;

    if (std::abs(move_distance_m) <= deadband_m) {
      return true;
    }

    if (move_distance_m < 0.0 && !allow_reverse_goal_) {
      return true;
    }

    const double scale = move_distance_m / planar_distance_m;
    point_out.x = target_x * scale;
    point_out.y = target_y * scale;
    return true;
  }

  bool transformRelativePointToMap(
    const geometry_msgs::msg::Point & point_rel,
    const std::vector<std::string> & source_candidates,
    const rclcpp::Time & transform_time,
    geometry_msgs::msg::PointStamped & point_map,
    std::string & resolved_source_frame,
    std::string & last_tf_error)
  {
    geometry_msgs::msg::PointStamped point_in;
    point_in.header.stamp = transform_time;
    point_in.point = point_rel;

    for (const auto & source_frame : source_candidates) {
      point_in.header.frame_id = source_frame;
      try {
        const geometry_msgs::msg::TransformStamped tf_map_source =
          tf_buffer_.lookupTransform(
          map_frame_,
          source_frame,
          transform_time,
          rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(point_in, point_map, tf_map_source);
        resolved_source_frame = source_frame;
        return true;
      } catch (const tf2::TransformException & ex) {
        last_tf_error = ex.what();
      }
    }

    return false;
  }

  void publishMapPointAsGoal(
    const geometry_msgs::msg::Point & point_map,
    const std::string & resolved_source_frame)
  {
    long x_cm = std::lround(point_map.x * 100.0);
    long y_cm = std::lround(point_map.y * 100.0);

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
        point_map.x,
        point_map.y,
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

  void goalPosRawCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (!enable_goal_pos_raw_bridge_) {
      return;
    }
    if (!msg || msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Drop invalid goal_pos_raw: data size=%zu (need >=2)",
        msg ? msg->data.size() : 0);
      return;
    }

    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = goal_pos_raw_frame_;
    point_in.header.stamp = this->now();
    point_in.point.x = static_cast<double>(msg->data[0]) * 0.01;
    point_in.point.y = static_cast<double>(msg->data[1]) * 0.01;
    point_in.point.z = 0.0;

    geometry_msgs::msg::PointStamped point_map;
    if (goal_pos_raw_frame_ == map_frame_) {
      point_map = point_in;
    } else {
      try {
        const geometry_msgs::msg::TransformStamped tf_map_raw =
          tf_buffer_.lookupTransform(
          map_frame_,
          goal_pos_raw_frame_,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(point_in, point_map, tf_map_raw);
        if (!goal_raw_tf_ready_logged_once_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Raw goal TF chain ready: %s <- %s",
            map_frame_.c_str(),
            goal_pos_raw_frame_.c_str());
          goal_raw_tf_ready_logged_once_ = true;
        }
      } catch (const tf2::TransformException & ex) {
        if (!goal_raw_tf_missing_logged_once_) {
          RCLCPP_WARN(
            this->get_logger(),
            "Raw goal TF chain not ready (%s <- %s): %s",
            map_frame_.c_str(),
            goal_pos_raw_frame_.c_str(),
            ex.what());
          goal_raw_tf_missing_logged_once_ = true;
        }
        return;
      }
    }

    publishMapPointAsGoal(point_map.point, goal_pos_raw_frame_);
  }

  void targetRelCallback(const auto_aim_common::msg::RelativeTarget::SharedPtr msg)
  {
    geometry_msgs::msg::Point relative_goal_point;
    if (!buildRelativeGoalPoint(*msg, relative_goal_point)) {
      return;
    }

    const auto source_candidates = buildSourceCandidates(msg->header.frame_id);
    const rclcpp::Time transform_time(msg->header.stamp);

    geometry_msgs::msg::PointStamped point_map;
    std::string last_tf_error;
    std::string resolved_source_frame;
    if (!transformRelativePointToMap(
        relative_goal_point,
        source_candidates,
        transform_time,
        point_map,
        resolved_source_frame,
        last_tf_error))
    {
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
    point_map.header.stamp =
      (transform_time.nanoseconds() == 0) ? this->now() : transform_time;

    if (publish_target_map_ && pub_target_map_) {
      pub_target_map_->publish(point_map);
    }
    publishMapPointAsGoal(point_map.point, resolved_source_frame);
  }

  std::string input_topic_;
  std::string input_goal_pos_raw_topic_;
  std::string output_goal_pos_topic_;
  std::string output_target_map_topic_;

  std::string map_frame_;
  std::string base_frame_;
  std::string fallback_base_frame_;
  bool use_msg_frame_id_{true};

  bool publish_target_map_{true};
  bool invert_y_axis_{false};
  int y_axis_max_cm_{1500};
  int preferred_distance_cm_{100};
  int distance_deadband_cm_{50};
  bool stop_when_no_target_{true};
  bool allow_reverse_goal_{false};
  bool enable_goal_pos_raw_bridge_{true};
  std::string goal_pos_raw_frame_{"map"};
  bool debug_export_point_pairs_{true};
  std::string debug_points_reference_frame_{"map"};
  std::string debug_area_header_file_;
  std::string debug_point_pairs_output_file_;
  bool debug_tf_ready_logged_once_{false};
  bool debug_tf_missing_logged_once_{false};
  bool goal_raw_tf_ready_logged_once_{false};
  bool goal_raw_tf_missing_logged_once_{false};
  bool debug_point_pairs_exported_{false};
  rclcpp::TimerBase::SharedPtr debug_export_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<auto_aim_common::msg::RelativeTarget>::SharedPtr sub_target_rel_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_goal_pos_raw_;
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
