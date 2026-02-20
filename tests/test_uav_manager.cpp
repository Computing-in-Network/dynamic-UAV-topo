#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "dynamic_uav_topo/uav_manager.hpp"

namespace {

int Assert(bool cond, const std::string& message) {
  if (!cond) {
    std::cerr << "ASSERTION FAILED: " << message << '\n';
    return 1;
  }
  return 0;
}

int TestAffinityPlan() {
  const auto plan = dynamic_uav_topo::CpuAffinityPlanner::Plan(4, 2);
  if (Assert(plan.size() == 4, "plan size must be 4") != 0) return 1;
  if (Assert(plan[0].size() == 1 && plan[0][0] == 0, "uav_0 -> core 0") != 0) return 1;
  if (Assert(plan[1].size() == 1 && plan[1][0] == 1, "uav_1 -> core 1") != 0) return 1;
  if (Assert(plan[2].size() == 1 && plan[2][0] == 0, "uav_2 -> core 0") != 0) return 1;
  if (Assert(plan[3].size() == 1 && plan[3][0] == 1, "uav_3 -> core 1") != 0) return 1;
  return 0;
}

int TestRenderCommand() {
  const std::string cmd = dynamic_uav_topo::NodeUavManager::RenderCommand(
      "px4-sitl --instance {id} --name {id}", "uav_03");
  return Assert(cmd == "px4-sitl --instance uav_03 --name uav_03", "placeholder replacement");
}

int TestLifecycle() {
  dynamic_uav_topo::NodeUavManager manager;
  auto configs = dynamic_uav_topo::NodeUavManager::BuildDefaultConfigs(2, "sleep 5");

  std::string error;
  if (!manager.StartAll(configs, 2, &error)) {
    std::cerr << "start failed: " << error << '\n';
    return 1;
  }

  if (Assert(manager.RunningInstanceCount() == 2, "both instances should run") != 0) return 1;

  const auto snapshot = manager.BuildSnapshot();
  if (Assert(snapshot.uavs.size() == 2, "snapshot should include two UAVs") != 0) return 1;

  manager.StopAll();
  if (Assert(manager.RunningInstanceCount() == 0, "all instances should stop") != 0) return 1;
  return 0;
}

int TestHealthCheckRestart() {
  dynamic_uav_topo::NodeUavManager manager;
  auto configs = dynamic_uav_topo::NodeUavManager::BuildDefaultConfigs(1, "sleep 0.1");
  configs[0].max_restarts = 2;

  std::string error;
  if (!manager.StartAll(configs, 1, &error)) {
    std::cerr << "start failed: " << error << '\n';
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const int restart_count = manager.TickHealthCheck(&error);
  if (Assert(restart_count >= 1, "health check should restart exited process") != 0) return 1;

  const auto runtime_infos = manager.BuildRuntimeInfoSnapshot();
  if (Assert(runtime_infos.size() == 1, "runtime info should include one instance") != 0) return 1;
  if (Assert(runtime_infos[0].restart_count >= 1, "restart count should be incremented") != 0) return 1;

  manager.StopAll();
  return 0;
}

}  // namespace

int main() {
  if (TestAffinityPlan() != 0) return 1;
  if (TestRenderCommand() != 0) return 1;
  if (TestLifecycle() != 0) return 1;
  if (TestHealthCheckRestart() != 0) return 1;

  std::cout << "All tests passed\n";
  return 0;
}
