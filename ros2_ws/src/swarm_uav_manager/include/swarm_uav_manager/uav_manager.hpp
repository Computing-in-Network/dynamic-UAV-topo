#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace swarm_uav_manager {

enum class UavStatus {
  kMission,
  kAlarm,
  kLanded,
};

struct UavPose {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

struct UavVelocity {
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
};

struct UavState {
  std::string id;
  UavPose pose;
  UavVelocity velocity;
  double battery = 100.0;
  UavStatus status = UavStatus::kMission;
};

struct SwarmState {
  std::uint64_t timestamp_ms = 0;
  std::vector<UavState> uavs;
};

struct UavInstanceConfig {
  std::string id;
  std::string sitl_command_template;
  int instance_index = 0;
  int mavlink_udp_port = 14540;
  int mavlink_tcp_port = 4560;
  int simulator_udp_port = 18570;
  int max_restarts = 3;
};

class CpuAffinityPlanner {
 public:
  static std::vector<std::vector<int>> Plan(std::size_t instance_count,
                                            std::size_t total_cores);
};

class NodeUavManager {
 public:
  struct InstanceRuntimeInfo {
    std::string id;
    int pid = -1;
    bool running = false;
    int assigned_core = -1;
    int launch_count = 0;
    int restart_count = 0;
    std::uint64_t last_start_ms = 0;
  };

  NodeUavManager();
  ~NodeUavManager();

  NodeUavManager(const NodeUavManager&) = delete;
  NodeUavManager& operator=(const NodeUavManager&) = delete;

  bool StartAll(const std::vector<UavInstanceConfig>& configs,
                std::size_t total_cores,
                std::string* error_message = nullptr);

  void StopAll(std::chrono::milliseconds grace = std::chrono::milliseconds(500));
  SwarmState BuildSnapshot() const;
  std::size_t RunningInstanceCount() const;
  int TickHealthCheck(std::string* error_message = nullptr);
  std::vector<InstanceRuntimeInfo> BuildRuntimeInfoSnapshot() const;

  static std::vector<UavInstanceConfig> BuildDefaultConfigs(std::size_t instance_count,
                                                            const std::string& command_template,
                                                            int mavlink_udp_base_port = 14540,
                                                            int mavlink_tcp_base_port = 4560,
                                                            int simulator_udp_base_port = 18570);

 private:
  struct ProcessHandle {
    std::string id;
    int pid = -1;
    bool running = false;
    int assigned_core = -1;
    int launch_count = 0;
    int restart_count = 0;
    int max_restarts = 3;
    std::uint64_t last_start_ms = 0;
    std::string command;
    std::vector<int> cores;
  };

  std::vector<ProcessHandle> handles_;
  std::unordered_map<std::string, UavState> states_;

  static bool SpawnProcess(const std::string& command,
                           const std::vector<int>& cores,
                           int* child_pid,
                           std::string* error_message);
  static void TerminateProcess(int pid, std::chrono::milliseconds grace);
  static bool HasExited(int pid);
  static std::string RenderCommandFromConfig(const UavInstanceConfig& config);
  static std::uint64_t NowMs();
};

}  // namespace swarm_uav_manager
