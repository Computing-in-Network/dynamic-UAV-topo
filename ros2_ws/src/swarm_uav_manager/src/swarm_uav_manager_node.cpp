#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>

#include "swarm_interfaces/msg/mission_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/swarm_state.hpp"
#include "swarm_interfaces/msg/uav_state.hpp"
#include "swarm_uav_manager/uav_manager.hpp"

namespace {

uint8_t ToMsgStatus(swarm_uav_manager::UavStatus status) {
  switch (status) {
    case swarm_uav_manager::UavStatus::kMission:
      return swarm_interfaces::msg::UavState::STATUS_MISSION;
    case swarm_uav_manager::UavStatus::kAlarm:
      return swarm_interfaces::msg::UavState::STATUS_ALARM;
    case swarm_uav_manager::UavStatus::kLanded:
      return swarm_interfaces::msg::UavState::STATUS_LANDED;
  }
  return swarm_interfaces::msg::UavState::STATUS_MISSION;
}

class SwarmUavManagerNode : public rclcpp::Node {
 public:
  SwarmUavManagerNode() : Node("swarm_uav_manager") {
    const int instance_count = declare_parameter<int>("instance_count", 4);
    const int total_cores = declare_parameter<int>("total_cores", 8);
    const std::string command_template = declare_parameter<std::string>(
        "command_template", "sleep 1000");
    const int publish_hz = declare_parameter<int>("publish_hz", 5);
    const int healthcheck_hz = declare_parameter<int>("healthcheck_hz", 1);
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/swarm/state");
    demo_motion_ = declare_parameter<bool>("demo_motion", true);
    mission_follow_enabled_ = declare_parameter<bool>("mission_follow_enabled", false);
    mission_topic_ = declare_parameter<std::string>("mission_topic", "/swarm/mission_targets");
    demo_origin_lat_ = declare_parameter<double>("demo_origin_lat", 39.9042);
    demo_origin_lon_ = declare_parameter<double>("demo_origin_lon", 116.4074);
    demo_radius_deg_ = declare_parameter<double>("demo_radius_deg", 0.0025);
    mission_speed_mps_ = declare_parameter<double>("mission_speed_mps", 25.0);
    mission_alt_speed_mps_ = declare_parameter<double>("mission_alt_speed_mps", 8.0);
    publish_hz_ = std::max(1, publish_hz);

    publisher_ = create_publisher<swarm_interfaces::msg::SwarmState>(output_topic, 10);
    mission_subscriber_ = create_subscription<swarm_interfaces::msg::MissionPlan>(
        mission_topic_, 10, [this](const swarm_interfaces::msg::MissionPlan::SharedPtr msg) {
          OnMissionPlan(*msg);
        });

    auto configs = swarm_uav_manager::NodeUavManager::BuildDefaultConfigs(
        static_cast<std::size_t>(instance_count), command_template);

    std::string error;
    if (!manager_.StartAll(configs, static_cast<std::size_t>(total_cores), &error)) {
      throw std::runtime_error("failed to start NodeUavManager: " + error);
    }

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / publish_hz_),
        [this]() { PublishSnapshot(); });

    healthcheck_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(1, healthcheck_hz)),
        [this]() { TickHealthCheck(); });

    RCLCPP_INFO(get_logger(),
                "swarm_uav_manager started, instances=%d output=%s mission_follow=%s mission_topic=%s",
                instance_count, output_topic.c_str(), mission_follow_enabled_ ? "on" : "off",
                mission_topic_.c_str());
  }

  ~SwarmUavManagerNode() override {
    manager_.StopAll();
  }

 private:
  static double ClampStep(double from, double to, double max_step) {
    const double d = to - from;
    if (std::abs(d) <= max_step) {
      return to;
    }
    return from + (d > 0.0 ? max_step : -max_step);
  }

  std::array<double, 3> NextMissionPose(const std::string& uav_id, std::size_t index, double now_s) {
    auto cur_it = mission_current_pose_.find(uav_id);
    if (cur_it == mission_current_pose_.end()) {
      const double phase = now_s * 0.35 + static_cast<double>(index) * 1.3;
      mission_current_pose_[uav_id] = {
          demo_origin_lat_ + std::sin(phase) * demo_radius_deg_,
          demo_origin_lon_ + std::cos(phase) * demo_radius_deg_,
          100.0 + std::sin(phase * 2.0) * 20.0,
      };
      cur_it = mission_current_pose_.find(uav_id);
    }

    auto target_it = mission_targets_.find(uav_id);
    if (target_it == mission_targets_.end()) {
      return cur_it->second;
    }

    auto cur = cur_it->second;
    const auto& target = target_it->second;
    const double dt = 1.0 / static_cast<double>(publish_hz_);
    auto vel_it = mission_velocity_mps_.find(uav_id);
    if (vel_it == mission_velocity_mps_.end()) {
      mission_velocity_mps_[uav_id] = {0.0, 0.0, 0.0};
      vel_it = mission_velocity_mps_.find(uav_id);
    }
    auto vel = vel_it->second;

    // Convert lat/lon delta to local meters at current latitude.
    constexpr double kPi = 3.14159265358979323846;
    const double lat_scale = 111000.0;
    const double lon_scale = 111000.0 * std::max(0.15, std::cos(cur[0] * kPi / 180.0));
    const double dx_m = (target[1] - cur[1]) * lon_scale;
    const double dy_m = (target[0] - cur[0]) * lat_scale;
    const double dz_m = target[2] - cur[2];
    const double dist_xy = std::hypot(dx_m, dy_m);

    // Desired speed profile: slow down near target, keep smooth cruise otherwise.
    const double arrive_slow_m = 80.0;
    const double speed_scale = std::clamp(dist_xy / arrive_slow_m, 0.20, 1.0);
    const double desired_speed = mission_speed_mps_ * speed_scale;
    double desired_vx = 0.0;
    double desired_vy = 0.0;
    if (dist_xy > 1e-3) {
      desired_vx = (dx_m / dist_xy) * desired_speed;
      desired_vy = (dy_m / dist_xy) * desired_speed;
    }
    const double desired_vz = std::clamp(dz_m / std::max(1e-3, dt), -mission_alt_speed_mps_, mission_alt_speed_mps_);

    // First-order velocity smoothing to avoid abrupt heading changes.
    const double alpha = 0.26;
    vel[0] = (1.0 - alpha) * vel[0] + alpha * desired_vx;
    vel[1] = (1.0 - alpha) * vel[1] + alpha * desired_vy;
    vel[2] = (1.0 - alpha) * vel[2] + alpha * desired_vz;

    const double vxy = std::hypot(vel[0], vel[1]);
    if (vxy > mission_speed_mps_) {
      const double scale = mission_speed_mps_ / std::max(1e-6, vxy);
      vel[0] *= scale;
      vel[1] *= scale;
    }
    vel[2] = std::clamp(vel[2], -mission_alt_speed_mps_, mission_alt_speed_mps_);

    cur[0] += (vel[1] * dt) / lat_scale;
    cur[1] += (vel[0] * dt) / lon_scale;
    cur[2] += vel[2] * dt;

    // Snap and damp near target to avoid tiny oscillations.
    if (dist_xy < 5.0) {
      cur[0] = target[0];
      cur[1] = target[1];
      vel[0] *= 0.25;
      vel[1] *= 0.25;
    }
    if (std::abs(target[2] - cur[2]) < 1.2) {
      cur[2] = target[2];
      vel[2] *= 0.25;
    }

    mission_velocity_mps_[uav_id] = vel;
    mission_current_pose_[uav_id] = cur;
    return cur;
  }

  void OnMissionPlan(const swarm_interfaces::msg::MissionPlan& msg) {
    mission_targets_.clear();
    for (const auto& t : msg.targets) {
      mission_targets_[t.uav_id] = {t.position[0], t.position[1], t.position[2]};
    }
  }

  void PublishSnapshot() {
    const auto snapshot = manager_.BuildSnapshot();
    const double now_s = now().seconds();

    swarm_interfaces::msg::SwarmState msg;
    msg.stamp = now();
    msg.uavs.reserve(snapshot.uavs.size());

    for (std::size_t i = 0; i < snapshot.uavs.size(); ++i) {
      const auto& uav = snapshot.uavs[i];
      swarm_interfaces::msg::UavState uav_msg;
      uav_msg.id = uav.id;
      double lat = uav.pose.lat;
      double lon = uav.pose.lon;
      double alt = uav.pose.alt;
      if (demo_motion_) {
        if (mission_follow_enabled_) {
          const auto pose = NextMissionPose(uav.id, i, now_s);
          lat = pose[0];
          lon = pose[1];
          alt = pose[2];
        } else {
          const double phase = now_s * 0.35 + static_cast<double>(i) * 1.3;
          lat = demo_origin_lat_ + std::sin(phase) * demo_radius_deg_;
          lon = demo_origin_lon_ + std::cos(phase) * demo_radius_deg_;
          alt = 100.0 + std::sin(phase * 2.0) * 20.0;
        }
      }
      uav_msg.position = {lat, lon, alt};
      uav_msg.velocity = {uav.velocity.vx, uav.velocity.vy, uav.velocity.vz};
      uav_msg.battery = static_cast<float>(uav.battery);
      uav_msg.status = ToMsgStatus(uav.status);
      msg.uavs.push_back(uav_msg);
    }

    publisher_->publish(msg);
  }

  void TickHealthCheck() {
    std::string error;
    const int restarted = manager_.TickHealthCheck(&error);
    if (restarted > 0) {
      RCLCPP_WARN(get_logger(), "healthcheck restarted %d instance(s)", restarted);
    }
    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "healthcheck error: %s", error.c_str());
    }
  }

  swarm_uav_manager::NodeUavManager manager_;
  rclcpp::Publisher<swarm_interfaces::msg::SwarmState>::SharedPtr publisher_;
  rclcpp::Subscription<swarm_interfaces::msg::MissionPlan>::SharedPtr mission_subscriber_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr healthcheck_timer_;
  bool demo_motion_{true};
  bool mission_follow_enabled_{false};
  std::string mission_topic_;
  double demo_origin_lat_{0.0};
  double demo_origin_lon_{0.0};
  double demo_radius_deg_{0.0};
  double mission_speed_mps_{25.0};
  double mission_alt_speed_mps_{8.0};
  int publish_hz_{5};
  std::unordered_map<std::string, std::array<double, 3>> mission_targets_;
  std::unordered_map<std::string, std::array<double, 3>> mission_current_pose_;
  std::unordered_map<std::string, std::array<double, 3>> mission_velocity_mps_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<SwarmUavManagerNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("swarm_uav_manager"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
