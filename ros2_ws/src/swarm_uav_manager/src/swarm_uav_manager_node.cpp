#include <algorithm>
#include <chrono>
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

    swarm_interfaces::msg::SwarmState msg;
    msg.stamp = now();
    msg.uavs.reserve(snapshot.uavs.size());

    for (const auto& uav : snapshot.uavs) {
      swarm_interfaces::msg::UavState uav_msg;
      uav_msg.id = uav.id;
      uav_msg.position = {uav.pose.lat, uav.pose.lon, uav.pose.alt};
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
