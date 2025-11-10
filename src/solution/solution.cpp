#include "tester.hpp"
#include <CameraController.hpp>


// You can remove or add any includes you need
#include <chrono>
#include <iostream>
#include <thread>
#include <memory>

int solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt) {
  const auto controller = std::make_unique<CameraController>(tester, preempt);

  return 0;
}
