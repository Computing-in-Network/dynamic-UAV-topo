#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/link_state.hpp"
#include "swarm_interfaces/msg/swarm_state.hpp"

namespace {

class SwarmSemanticNetNode : public rclcpp::Node {
 public:
  SwarmSemanticNetNode() : Node("swarm_semantic_net_node"), rng_(std::random_device{}()) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/swarm/state");
    output_topic_ = declare_parameter<std::string>("output_topic", "/swarm/state_semantic");
    packet_drop_rate_ = declare_parameter<double>("packet_drop_rate", 0.0);
    drop_mode_ = declare_parameter<std::string>("drop_mode", "none");
    throttle_mode_ = declare_parameter<std::string>("throttle_mode", "weight");
    min_link_weight_ = declare_parameter<double>("min_link_weight", 0.05);
    weight_dropout_scale_ = declare_parameter<double>("weight_dropout_scale", 0.25);
    jitter_std_ = declare_parameter<double>("jitter_std", 0.05);
    delay_base_ms_ = declare_parameter<double>("delay_base_ms", 0.0);
    delay_weight_scale_ms_ = declare_parameter<double>("delay_weight_scale_ms", 0.0);
    delay_jitter_ms_ = declare_parameter<double>("delay_jitter_ms", 0.0);
    topic_rate_max_hz_ = declare_parameter<double>("topic_rate_max_hz", 20.0);
    topic_rate_min_hz_ = declare_parameter<double>("topic_rate_min_hz", 3.0);
    packet_report_every_n_ = declare_parameter<int>("packet_report_every_n", 100);
    link_report_every_n_ = declare_parameter<int>("link_report_every_n", 200);
    throttle_report_every_n_ = declare_parameter<int>("throttle_report_every_n", 200);
    rng_seed_ = declare_parameter<int>("seed", 0);

    if (rng_seed_ != 0) {
      rng_.seed(static_cast<std::uint_fast32_t>(rng_seed_));
    }
    packet_drop_rate_ = std::clamp(packet_drop_rate_, 0.0, 1.0);
    min_link_weight_ = std::clamp(min_link_weight_, 0.0, 1.0);
    weight_dropout_scale_ = std::clamp(weight_dropout_scale_, 0.0, 1.0);
    delay_base_ms_ = std::max(0.0, delay_base_ms_);
    delay_weight_scale_ms_ = std::max(0.0, delay_weight_scale_ms_);
    delay_jitter_ms_ = std::max(0.0, delay_jitter_ms_);
    topic_rate_max_hz_ = std::max(0.0, topic_rate_max_hz_);
    topic_rate_min_hz_ = std::clamp(topic_rate_min_hz_, 0.0, topic_rate_max_hz_);
    if (drop_mode_ != "none" && drop_mode_ != "fixed" && drop_mode_ != "weight") {
      RCLCPP_WARN(get_logger(), "unknown drop_mode=%s, fallback to none", drop_mode_.c_str());
      drop_mode_ = "none";
    }
    if (throttle_mode_ != "none" && throttle_mode_ != "weight") {
      RCLCPP_WARN(get_logger(),
                  "unknown throttle_mode=%s, fallback to weight",
                  throttle_mode_.c_str());
      throttle_mode_ = "weight";
    }

    publisher_ = create_publisher<swarm_interfaces::msg::SwarmState>(output_topic_, 10);
    subscriber_ = create_subscription<swarm_interfaces::msg::SwarmState>(
        input_topic_, 10,
        [this](const swarm_interfaces::msg::SwarmState::SharedPtr msg) { OnSwarmState(*msg); });

    jitter_dist_ = std::normal_distribution<double>(0.0, jitter_std_);
    delay_jitter_dist_ = std::normal_distribution<double>(0.0, delay_jitter_ms_);
    uniform_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    RCLCPP_INFO(
        get_logger(),
        "semantic net started input=%s output=%s packet_drop=%s drop_mode=%s "
        "drop_scale=%.3f min_link=%.3f throttle=%s hz=(%.2f,%.2f) delay=(%.2f,%.2f,%.2f) seed=%d",
        input_topic_.c_str(), output_topic_.c_str(),
        packet_drop_rate_ > 0.0 ? "on" : "off", drop_mode_.c_str(),
        weight_dropout_scale_, min_link_weight_, throttle_mode_.c_str(), topic_rate_min_hz_,
        topic_rate_max_hz_, delay_base_ms_, delay_weight_scale_ms_, delay_jitter_ms_, rng_seed_);
  }

 private:
  double CalcDropRate(double weight) const {
    if (drop_mode_ == "none") {
      return 0.0;
    }
    if (drop_mode_ == "fixed") {
      return std::clamp(packet_drop_rate_, 0.0, 1.0);
    }
    if (drop_mode_ == "weight") {
      return std::clamp(packet_drop_rate_ * (1.0 - weight), 0.0, 1.0);
    }
    return std::clamp(packet_drop_rate_, 0.0, 1.0);
  }

  double CalcAvgWeight(const std::vector<swarm_interfaces::msg::LinkState>& links) const {
    if (links.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const auto& link : links) {
      sum += static_cast<double>(link.weight);
    }
    return std::clamp(sum / static_cast<double>(links.size()), 0.0, 1.0);
  }

  double CalcPublishIntervalMs(double avg_weight) const {
    if (throttle_mode_ == "none" || topic_rate_max_hz_ <= 0.0) {
      return 0.0;
    }
    if (topic_rate_max_hz_ <= topic_rate_min_hz_) {
      return 1000.0 / topic_rate_max_hz_;
    }
    const double w = std::clamp(avg_weight, 0.0, 1.0);
    const double hz = topic_rate_min_hz_ + (topic_rate_max_hz_ - topic_rate_min_hz_) * w;
    if (hz <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }
    return 1000.0 / hz;
  }

  double CalcDelayMs(double avg_weight) const {
    if (delay_base_ms_ <= 0.0 && delay_weight_scale_ms_ <= 0.0 && delay_jitter_ms_ <= 0.0) {
      return 0.0;
    }
    const double degradation = 1.0 - std::clamp(avg_weight, 0.0, 1.0);
    return std::max(0.0, delay_base_ms_ + delay_weight_scale_ms_ * degradation);
  }

  bool ShouldPublish(double avg_weight) {
    if (throttle_mode_ == "none") {
      return true;
    }
    const double interval_ms = CalcPublishIntervalMs(avg_weight);
    if (!std::isfinite(interval_ms) || interval_ms <= 0.0) {
      ++throttled_packet_count_;
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (next_publish_time_.time_since_epoch().count() == 0) {
      next_publish_time_ = now;
    }
    const auto want = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(interval_ms));
    if (now < next_publish_time_) {
      ++throttled_packet_count_;
      return false;
    }
    next_publish_time_ = now + want;
    return true;
  }

  void OnSwarmState(const swarm_interfaces::msg::SwarmState& in_msg) {
    ++packet_count_;
    const double in_avg_weight = CalcAvgWeight(in_msg.links);
    const double packet_drop = CalcDropRate(in_avg_weight);
    if (uniform_dist_(rng_) < packet_drop) {
      ++droped_packet_count_;
      if (packet_report_every_n_ > 0 && (packet_count_ % static_cast<std::size_t>(packet_report_every_n_) == 0U)) {
        PrintStats();
      }
      return;
    }

    swarm_interfaces::msg::SwarmState out = in_msg;
    out.links.clear();
    std::size_t kept = 0;
    std::size_t dropped_links = 0;
    out.links.reserve(in_msg.links.size());

    for (const auto& link : in_msg.links) {
      const double drop_prob = CalcDropRate(static_cast<double>(link.weight));
      if (uniform_dist_(rng_) < drop_prob) {
        ++dropped_links;
        continue;
      }
      auto next = link;
      const double jitter = jitter_dist_(rng_);
      next.weight = std::clamp(static_cast<double>(link.weight) * (1.0 - weight_dropout_scale_) + jitter,
                              0.0, 1.0);
      if (next.weight < min_link_weight_) {
        ++dropped_links;
        continue;
      }
      out.links.push_back(std::move(next));
      ++kept;
    }
    const double out_avg_weight = CalcAvgWeight(out.links);
    if (!ShouldPublish(out_avg_weight)) {
      if (throttle_report_every_n_ > 0 &&
          (packet_count_ % static_cast<std::size_t>(throttle_report_every_n_) == 0U)) {
        RCLCPP_WARN(
            get_logger(),
            "semantic net throttle packets=%zu throttled=%zu avg_weight=%.4f",
            packet_count_, throttled_packet_count_, out_avg_weight);
      }
      return;
    }
    const double delay_ms = CalcDelayMs(out_avg_weight);
    if (delay_ms > 0.0) {
      const double jitter_delay = delay_jitter_dist_(rng_);
      std::this_thread::sleep_for(
          std::chrono::microseconds(
              static_cast<long long>(std::max(0.0, (delay_ms + jitter_delay) * 1000.0))));
    }

    if (link_report_every_n_ > 0 &&
        (packet_count_ % static_cast<std::size_t>(link_report_every_n_) == 0U)) {
      RCLCPP_INFO(
          get_logger(),
          "semantic net metrics packets=%zu kept=%zu dropped_links=%zu last_total=%zu avg_in_weight=%.4f avg_out_weight=%.4f",
          packet_count_, kept, dropped_links, in_msg.links.size(), in_avg_weight, out_avg_weight);
    }
    publisher_->publish(out);
  }

  void PrintStats() const {
    const auto packet_drop_ratio = packet_count_ > 0
                                      ? (static_cast<double>(droped_packet_count_) /
                                         static_cast<double>(packet_count_))
                                      : 0.0;
    const auto throttle_ratio = packet_count_ > 0
                                   ? (static_cast<double>(throttled_packet_count_) /
                                      static_cast<double>(packet_count_))
                                   : 0.0;
    RCLCPP_INFO(get_logger(),
                "semantic net metrics packet_drop_ratio=%.4f throttle_ratio=%.4f",
                packet_drop_ratio, throttle_ratio);
  }

  std::string input_topic_{};
  std::string output_topic_{};
  std::string drop_mode_{"none"};
  std::string throttle_mode_{"weight"};
  int rng_seed_{0};
  int packet_report_every_n_{100};
  int link_report_every_n_{200};
  int throttle_report_every_n_{200};
  double packet_drop_rate_{0.0};
  double min_link_weight_{0.05};
  double weight_dropout_scale_{0.25};
  double jitter_std_{0.05};
  double delay_base_ms_{0.0};
  double delay_weight_scale_ms_{0.0};
  double delay_jitter_ms_{0.0};
  double topic_rate_min_hz_{3.0};
  double topic_rate_max_hz_{20.0};
  std::size_t packet_count_{0};
  std::size_t droped_packet_count_{0};
  std::size_t throttled_packet_count_{0};

  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
  std::normal_distribution<double> jitter_dist_{0.0, 0.05};
  std::normal_distribution<double> delay_jitter_dist_{0.0, 0.0};

  std::chrono::steady_clock::time_point next_publish_time_{};

  rclcpp::Subscription<swarm_interfaces::msg::SwarmState>::SharedPtr subscriber_;
  rclcpp::Publisher<swarm_interfaces::msg::SwarmState>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwarmSemanticNetNode>());
  rclcpp::shutdown();
  return 0;
}
