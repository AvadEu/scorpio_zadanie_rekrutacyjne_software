#include "tester.hpp"
#include <CameraController.hpp>
#include <chrono>
#include <thread>
#include <memory>

int solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt) {
  const auto controller = std::make_unique<CameraController>(tester, preempt);
  std::this_thread::sleep_for(std::chrono::seconds(30));

  return 0;
}
