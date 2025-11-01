#include "CameraController.hpp"

constexpr int ENCODER_RESOLUTION = 4096;

CameraController::CameraController(backend_interface::Tester* tester, bool preempt_mode)
  : motor1_(tester->get_motor_1()),
    motor2_(tester->get_motor_2()),
    commands_(tester->get_commands()),
    preempt_mode_(preempt_mode),
    ENCODER_SCOPE(ENCODER_RESOLUTION) {
}