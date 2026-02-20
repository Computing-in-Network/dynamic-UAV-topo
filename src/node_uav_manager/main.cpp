#include <cstdlib>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "dynamic_uav_topo/uav_manager.hpp"

namespace {
std::atomic<bool> g_stop_requested{false};

void HandleSignal(int /*signal*/) {
  g_stop_requested.store(true);
}
}  // namespace

int main(int argc, char** argv) {
  std::size_t instance_count = 4;
  std::size_t total_cores = 8;
  std::string command_template = "sleep 1000";
  long runtime_seconds = 0;

  if (argc > 1) {
    instance_count = static_cast<std::size_t>(std::strtoul(argv[1], nullptr, 10));
  }
  if (argc > 2) {
    total_cores = static_cast<std::size_t>(std::strtoul(argv[2], nullptr, 10));
  }
  if (argc > 3) {
    command_template = argv[3];
  }
  if (argc > 4) {
    runtime_seconds = std::strtol(argv[4], nullptr, 10);
  }

  auto configs = dynamic_uav_topo::NodeUavManager::BuildDefaultConfigs(instance_count, command_template);

  dynamic_uav_topo::NodeUavManager manager;
  std::string error;
  if (!manager.StartAll(configs, total_cores, &error)) {
    std::cerr << "failed to start NodeUavManager: " << error << '\n';
    return 1;
  }

  const auto snapshot = manager.BuildSnapshot();
  std::cout << "running instances: " << manager.RunningInstanceCount() << '\n';
  std::cout << "snapshot uav count: " << snapshot.uavs.size() << '\n';
  const auto runtime_infos = manager.BuildRuntimeInfoSnapshot();
  for (const auto& info : runtime_infos) {
    std::cout << info.id << " pid=" << info.pid << " core=" << info.assigned_core
              << " launches=" << info.launch_count << " restarts=" << info.restart_count << '\n';
  }

  if (runtime_seconds != 0) {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const auto start = std::chrono::steady_clock::now();
    while (!g_stop_requested.load()) {
      std::string health_error;
      const int restart_count = manager.TickHealthCheck(&health_error);
      if (restart_count > 0) {
        std::cout << "healthcheck restarted instances: " << restart_count << '\n';
      }
      if (!health_error.empty()) {
        std::cerr << "healthcheck error: " << health_error << '\n';
      }

      if (runtime_seconds > 0) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start);
        if (elapsed.count() >= runtime_seconds) {
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  manager.StopAll();
  return 0;
}
