#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

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
    demo_motion_ = declare_parameter<bool>("demo_motion", true);
    demo_origin_lat_ = declare_parameter<double>("demo_origin_lat", 39.9042);
    demo_origin_lon_ = declare_parameter<double>("demo_origin_lon", 116.4074);
    demo_radius_deg_ = declare_parameter<double>("demo_radius_deg", 0.0025);

    publisher_ = create_publisher<swarm_interfaces::msg::SwarmState>("/swarm/state", 10);

    auto configs = swarm_uav_manager::NodeUavManager::BuildDefaultConfigs(
        static_cast<std::size_t>(instance_count), command_template);

    std::string error;
    if (!manager_.StartAll(configs, static_cast<std::size_t>(total_cores), &error)) {
      throw std::runtime_error("failed to start NodeUavManager: " + error);
    }

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(1, publish_hz)),
        [this]() { PublishSnapshot(); });

    healthcheck_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(1, healthcheck_hz)),
        [this]() { TickHealthCheck(); });

    RCLCPP_INFO(get_logger(), "swarm_uav_manager started, instances=%d", instance_count);
  }

  ~SwarmUavManagerNode() override {
    manager_.StopAll();
  }

 private:
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
        const double phase = now_s * 0.35 + static_cast<double>(i) * 1.3;
        lat = demo_origin_lat_ + std::sin(phase) * demo_radius_deg_;
        lon = demo_origin_lon_ + std::cos(phase) * demo_radius_deg_;
        alt = 100.0 + std::sin(phase * 2.0) * 20.0;
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
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr healthcheck_timer_;
  bool demo_motion_{true};
  double demo_origin_lat_{0.0};
  double demo_origin_lon_{0.0};
  double demo_radius_deg_{0.0};
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
