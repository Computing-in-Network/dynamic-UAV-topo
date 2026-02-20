#include "dynamic_uav_topo/uav_manager.hpp"

#include <sched.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string_view>
#include <thread>

namespace dynamic_uav_topo {

namespace {

bool IsProcessRunning(int pid) {
  if (pid <= 0) {
    return false;
  }
  return kill(pid, 0) == 0;
}

void ReplaceAll(std::string* target, std::string_view from, std::string_view to) {
  if (target == nullptr || from.empty()) {
    return;
  }
  std::size_t pos = 0;
  while ((pos = target->find(from, pos)) != std::string::npos) {
    target->replace(pos, from.size(), to);
    pos += to.size();
  }
}

}  // namespace

std::vector<std::vector<int>> CpuAffinityPlanner::Plan(std::size_t instance_count,
                                                       std::size_t total_cores) {
  std::vector<std::vector<int>> plan(instance_count);
  if (instance_count == 0 || total_cores == 0) {
    return plan;
  }

  for (std::size_t i = 0; i < instance_count; ++i) {
    plan[i].push_back(static_cast<int>(i % total_cores));
  }
  return plan;
}

NodeUavManager::NodeUavManager() = default;

NodeUavManager::~NodeUavManager() {
  StopAll();
}

bool NodeUavManager::StartAll(const std::vector<UavInstanceConfig>& configs,
                              std::size_t total_cores,
                              std::string* error_message) {
  StopAll();

  if (configs.empty()) {
    if (error_message != nullptr) {
      *error_message = "no UAV instances provided";
    }
    return false;
  }

  const auto affinity_plan = CpuAffinityPlanner::Plan(configs.size(), total_cores);

  handles_.reserve(configs.size());
  for (std::size_t i = 0; i < configs.size(); ++i) {
    const auto command = RenderCommandFromConfig(configs[i]);
    int pid = -1;
    std::string spawn_error;
    if (!SpawnProcess(command, affinity_plan[i], &pid, &spawn_error)) {
      if (error_message != nullptr) {
        *error_message = "unable to spawn UAV=" + configs[i].id + ": " + spawn_error;
      }
      StopAll();
      return false;
    }

    ProcessHandle handle;
    handle.id = configs[i].id;
    handle.pid = pid;
    handle.running = true;
    handle.max_restarts = configs[i].max_restarts;
    handle.launch_count = 1;
    handle.last_start_ms = NowMs();
    handle.command = command;
    handle.cores = affinity_plan[i];
    if (!affinity_plan[i].empty()) {
      handle.assigned_core = affinity_plan[i][0];
    }
    handles_.push_back(handle);

    UavState initial_state;
    initial_state.id = configs[i].id;
    states_[configs[i].id] = initial_state;
  }
  return true;
}

void NodeUavManager::StopAll(std::chrono::milliseconds grace) {
  for (auto& handle : handles_) {
    if (handle.running) {
      TerminateProcess(handle.pid, grace);
      handle.running = false;
    }
  }

  handles_.clear();
  states_.clear();
}

SwarmState NodeUavManager::BuildSnapshot() const {
  SwarmState snapshot;
  snapshot.timestamp_ms = NowMs();
  snapshot.uavs.reserve(states_.size());

  for (const auto& [id, state] : states_) {
    (void)id;
    snapshot.uavs.push_back(state);
  }

  std::sort(snapshot.uavs.begin(), snapshot.uavs.end(),
            [](const UavState& lhs, const UavState& rhs) { return lhs.id < rhs.id; });

  return snapshot;
}

std::size_t NodeUavManager::RunningInstanceCount() const {
  std::size_t count = 0;
  for (const auto& handle : handles_) {
    if (handle.running && IsProcessRunning(handle.pid) && !HasExited(handle.pid)) {
      ++count;
    }
  }
  return count;
}

int NodeUavManager::TickHealthCheck(std::string* error_message) {
  int restart_count = 0;
  for (auto& handle : handles_) {
    if (!handle.running) {
      continue;
    }

    if (!HasExited(handle.pid)) {
      continue;
    }

    handle.running = false;
    if (handle.restart_count >= handle.max_restarts) {
      auto iter = states_.find(handle.id);
      if (iter != states_.end()) {
        iter->second.status = UavStatus::kAlarm;
      }
      continue;
    }

    int new_pid = -1;
    std::string restart_error;
    if (!SpawnProcess(handle.command, handle.cores, &new_pid, &restart_error)) {
      if (error_message != nullptr) {
        *error_message = "restart failed for UAV=" + handle.id + ": " + restart_error;
      }
      auto iter = states_.find(handle.id);
      if (iter != states_.end()) {
        iter->second.status = UavStatus::kAlarm;
      }
      continue;
    }

    handle.pid = new_pid;
    handle.running = true;
    ++handle.restart_count;
    ++handle.launch_count;
    handle.last_start_ms = NowMs();
    ++restart_count;
  }
  return restart_count;
}

std::vector<NodeUavManager::InstanceRuntimeInfo> NodeUavManager::BuildRuntimeInfoSnapshot() const {
  std::vector<InstanceRuntimeInfo> infos;
  infos.reserve(handles_.size());
  for (const auto& handle : handles_) {
    infos.push_back(InstanceRuntimeInfo{
        .id = handle.id,
        .pid = handle.pid,
        .running = handle.running && IsProcessRunning(handle.pid) && !HasExited(handle.pid),
        .assigned_core = handle.assigned_core,
        .launch_count = handle.launch_count,
        .restart_count = handle.restart_count,
        .last_start_ms = handle.last_start_ms,
    });
  }
  return infos;
}

std::string NodeUavManager::RenderCommand(const std::string& command_template,
                                          const std::string& uav_id) {
  std::string command = command_template;
  ReplaceAll(&command, "{id}", uav_id);
  return command;
}

std::vector<UavInstanceConfig> NodeUavManager::BuildDefaultConfigs(
    std::size_t instance_count,
    const std::string& command_template,
    int mavlink_udp_base_port,
    int mavlink_tcp_base_port,
    int simulator_udp_base_port) {
  std::vector<UavInstanceConfig> configs;
  configs.reserve(instance_count);
  for (std::size_t i = 0; i < instance_count; ++i) {
    configs.push_back(UavInstanceConfig{
        .id = "uav_" + std::to_string(i),
        .sitl_command_template = command_template,
        .instance_index = static_cast<int>(i),
        .mavlink_udp_port = mavlink_udp_base_port + static_cast<int>(i),
        .mavlink_tcp_port = mavlink_tcp_base_port + static_cast<int>(i),
        .simulator_udp_port = simulator_udp_base_port + static_cast<int>(i),
    });
  }
  return configs;
}

bool NodeUavManager::SpawnProcess(const std::string& command,
                                  const std::vector<int>& cores,
                                  int* child_pid,
                                  std::string* error_message) {
  const pid_t pid = fork();
  if (pid < 0) {
    if (error_message != nullptr) {
      *error_message = std::strerror(errno);
    }
    return false;
  }

  if (pid == 0) {
    if (!cores.empty()) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      for (int core : cores) {
        if (core >= 0 && core < CPU_SETSIZE) {
          CPU_SET(core, &cpuset);
        }
      }
      if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        _exit(127);
      }
    }

    execl("/bin/sh", "sh", "-lc", command.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }

  if (child_pid != nullptr) {
    *child_pid = pid;
  }
  return true;
}

void NodeUavManager::TerminateProcess(int pid, std::chrono::milliseconds grace) {
  if (pid <= 0) {
    return;
  }

  kill(pid, SIGTERM);

  const auto deadline = std::chrono::steady_clock::now() + grace;
  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    const pid_t wait_result = waitpid(pid, &status, WNOHANG);
    if (wait_result == pid) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  kill(pid, SIGKILL);
  int status = 0;
  waitpid(pid, &status, 0);
}

bool NodeUavManager::HasExited(int pid) {
  if (pid <= 0) {
    return true;
  }
  int status = 0;
  const pid_t result = waitpid(pid, &status, WNOHANG);
  if (result == 0) {
    return false;
  }
  if (result == pid) {
    return true;
  }
  if (result == -1 && errno == ECHILD) {
    return true;
  }
  return false;
}

std::string NodeUavManager::RenderCommandFromConfig(const UavInstanceConfig& config) {
  std::string command = config.sitl_command_template;
  ReplaceAll(&command, "{id}", config.id);
  ReplaceAll(&command, "{index}", std::to_string(config.instance_index));
  ReplaceAll(&command, "{mavlink_udp_port}", std::to_string(config.mavlink_udp_port));
  ReplaceAll(&command, "{mavlink_tcp_port}", std::to_string(config.mavlink_tcp_port));
  ReplaceAll(&command, "{sim_udp_port}", std::to_string(config.simulator_udp_port));
  return command;
}

std::uint64_t NodeUavManager::NowMs() {
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
  return static_cast<std::uint64_t>(ms.count());
}

}  // namespace dynamic_uav_topo
