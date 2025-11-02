#include "CameraController.hpp"
#include <cmath>

constexpr int ENCODER_RESOLUTION = 4096;

CameraController::CameraController(std::shared_ptr<backend_interface::Tester> tester, const bool preempt_mode)
  : motor1_(tester->get_motor_1()),
    motor2_(tester->get_motor_2()),
    commands_(tester->get_commands()),
    preempt_mode_(preempt_mode),
    ENCODER_SCOPE(ENCODER_RESOLUTION),
    onTheWayToTarget_(false) {
}

void CameraController::onCommandReceived(const Point &target) {
  onTheWayToTarget_ = true;
  convertCoordinatesForEncoder(target);
}

void CameraController::convertCoordinatesForEncoder(const Point &target) {
  // Horizontal Rotation
  const double encoderStepRad = ENCODER_SCOPE / (2.0 * M_PI);
  const double horizontalPlaneAngle = std::atan2(target.y, target.x);
  targetEncoderVal1_ = horizontalPlaneAngle * encoderStepRad;

  // Vertical Rotation
  const double targetDistXY = std::sqrt(target.x * target.x + target.y * target.y);
  const double verticalPlaneAngle = std::atan2(target.z, targetDistXY);
  targetEncoderVal2_ = verticalPlaneAngle * encoderStepRad;

  // Correct encoder values in case of negative atan score
  targetEncoderVal1_ = (targetEncoderVal1_ < 0.0) ? targetEncoderVal1_ += ENCODER_SCOPE : targetEncoderVal1_;
  targetEncoderVal2_ = (targetEncoderVal2_ < 0.0) ? targetEncoderVal2_ += ENCODER_SCOPE : targetEncoderVal2_;
}
