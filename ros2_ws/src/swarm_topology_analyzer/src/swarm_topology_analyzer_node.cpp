#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/link_state.hpp"
#include "swarm_interfaces/msg/swarm_state.hpp"
#include "swarm_interfaces/msg/uav_state.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

double ToDistanceMeters(const swarm_interfaces::msg::UavState& a,
                        const swarm_interfaces::msg::UavState& b) {
  const double lat1 = a.position[0];
  const double lon1 = a.position[1];
  const double alt1 = a.position[2];
  const double lat2 = b.position[0];
  const double lon2 = b.position[1];
  const double alt2 = b.position[2];

  const double dlat = (lat1 - lat2) * 111000.0;
  const double mid_lat_rad = (lat1 + lat2) * 0.5 * kPi / 180.0;
  const double dlon = (lon1 - lon2) * 111000.0 * std::cos(mid_lat_rad);
  const double dalt = alt1 - alt2;
  return std::sqrt(dlat * dlat + dlon * dlon + dalt * dalt);
}

bool IsOccluded(const swarm_interfaces::msg::UavState& a,
                const swarm_interfaces::msg::UavState& b,
                double altitude_gap_m,
                double distance_m) {
  const double dalt = std::abs(a.position[2] - b.position[2]);
  return (dalt >= altitude_gap_m) && (distance_m >= 0.5 * altitude_gap_m);
}

class SwarmTopologyAnalyzerNode : public rclcpp::Node {
 public:
  SwarmTopologyAnalyzerNode() : Node("swarm_topology_analyzer") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/swarm/state_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/swarm/state");
    max_link_range_m_ = declare_parameter<double>("max_link_range_m", 900.0);
    min_weight_ = declare_parameter<double>("min_weight", 0.05);
    occlusion_altitude_gap_m_ = declare_parameter<double>("occlusion_altitude_gap_m", 35.0);
    occlusion_penalty_ = declare_parameter<double>("occlusion_penalty", 0.4);

    publisher_ = create_publisher<swarm_interfaces::msg::SwarmState>(output_topic_, 10);
    subscriber_ = create_subscription<swarm_interfaces::msg::SwarmState>(
        input_topic_, 10,
        [this](const swarm_interfaces::msg::SwarmState::SharedPtr msg) { OnSwarmState(*msg); });

    RCLCPP_INFO(get_logger(),
                "topology analyzer started, input=%s output=%s range=%.1f",
                input_topic_.c_str(), output_topic_.c_str(), max_link_range_m_);
  }

 private:
  void OnSwarmState(const swarm_interfaces::msg::SwarmState& in_msg) {
    swarm_interfaces::msg::SwarmState out = in_msg;
    out.links.clear();

    const auto& uavs = in_msg.uavs;
    if (uavs.size() < 2) {
      publisher_->publish(out);
      return;
    }

    out.links.reserve(uavs.size() * (uavs.size() - 1) / 2);
    for (std::size_t i = 0; i < uavs.size(); ++i) {
      for (std::size_t j = i + 1; j < uavs.size(); ++j) {
        const double dist_m = ToDistanceMeters(uavs[i], uavs[j]);
        if (dist_m > max_link_range_m_) {
          continue;
        }

        const bool occluded = IsOccluded(uavs[i], uavs[j], occlusion_altitude_gap_m_, dist_m);
        double weight = 1.0 - (dist_m / std::max(1.0, max_link_range_m_));
        if (occluded) {
          weight *= std::clamp(1.0 - occlusion_penalty_, 0.0, 1.0);
        }
        weight = std::clamp(weight, 0.0, 1.0);
        if (weight < min_weight_) {
          continue;
        }

        swarm_interfaces::msg::LinkState edge;
        edge.source = uavs[i].id;
        edge.target = uavs[j].id;
        edge.weight = static_cast<float>(weight);
        edge.is_occluded = occluded;
        out.links.push_back(std::move(edge));
      }
    }

    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  double max_link_range_m_{900.0};
  double min_weight_{0.05};
  double occlusion_altitude_gap_m_{35.0};
  double occlusion_penalty_{0.4};

  rclcpp::Subscription<swarm_interfaces::msg::SwarmState>::SharedPtr subscriber_;
  rclcpp::Publisher<swarm_interfaces::msg::SwarmState>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwarmTopologyAnalyzerNode>());
  rclcpp::shutdown();
  return 0;
}
