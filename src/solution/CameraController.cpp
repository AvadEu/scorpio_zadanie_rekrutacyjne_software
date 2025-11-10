#include "CameraController.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>

constexpr int ENCODER_RESOLUTION = 4096;
constexpr double ENCODER_ERROR_MARGIN = 25.0;

CameraController::CameraController(std::shared_ptr<backend_interface::Tester> tester, const bool preempt_mode)
  : motor1_(tester->get_motor_1()),
    motor2_(tester->get_motor_2()),
    commands_(tester->get_commands()),
    preempt_mode_(preempt_mode),
    ENCODER_SCOPE(ENCODER_RESOLUTION),
    onTheWayToTarget_(false) {

  motor1_->add_data_callback([this](const uint16_t &position) {
    this->onMotor1Data(position);
  });

  motor2_->add_data_callback([this](const uint16_t &position) {
    this->onMotor2Data(position);
  });

  commands_->add_data_callback([this](const Point& target) {
    this->onCommandReceived(target);
  });
}

void CameraController::onCommandReceived(const Point &target) {
  // Debug log
  std::cout << "!!! OTRZYMANO CEL: (" << target.x << ", " << target.y << ", " << target.z << ") !!!" << std::endl;

  // Accept new target and abandon last one
  if (preempt_mode_) {
    convertCoordinatesForEncoder(target);
    onTheWayToTarget_ = true;
    motor1InPosition_ = false;
    motor2InPosition_ = false;
  } else {
    // Move to single target
    if (!onTheWayToTarget_) {
      convertCoordinatesForEncoder(target);
      onTheWayToTarget_ = true;
      motor1InPosition_ = false;
      motor2InPosition_ = false;
    }
  }
}

void CameraController::onMotor1Data(const uint16_t &position) {
  updateMotorRegulator(position, 1);
}

void CameraController::onMotor2Data(const uint16_t &position) {
  updateMotorRegulator(position, 2);
}

void CameraController::updateMotorRegulator(const uint16_t &currPosition, int motorId) {
  // No active target
  if (!onTheWayToTarget_) {
    (motorId == 1 ? motor1_ : motor2_)->send_data(0);
    return;
  }

  const double target = (motorId == 1) ? targetEncoderVal1_ : targetEncoderVal2_;
  double Kp = (motorId == 1) ? Kp1_ : Kp2_;
  std::shared_ptr<backend_interface::Component<signed char, unsigned short> > motor = (motorId == 1 ? motor1_ : motor2_);

  // Operational err calculation
  double uchyb = target - static_cast<double>(currPosition);

  constexpr double encoderResolutionDouble = ENCODER_RESOLUTION;
  if (uchyb > encoderResolutionDouble / 2) {
    uchyb -= encoderResolutionDouble;
  } else if (uchyb < -encoderResolutionDouble / 2) {
    uchyb += encoderResolutionDouble;
  }

  if (std::abs(uchyb) < ENCODER_ERROR_MARGIN) {
    // Target achieved, stop motor
    motor->send_data(0);
    if (motorId == 1) motor1InPosition_ = true;
    else motor2InPosition_ = true;

    if (motor1InPosition_ && motor2InPosition_) {
      if (onTheWayToTarget_) {
        std::cout << "Target achieved \n";
      }
      onTheWayToTarget_ = false;
    }
    return;
  }
  if (motorId == 1) motor1InPosition_ = false;
  else motor2InPosition_ = false;

  // 7. Regulator P
  double signal = Kp * uchyb;

  // 8. Nasycenie (Clamp) - ogranicz sygnał do [-128, 127]
  signal = std::max(-128.0, std::min(127.0, signal));

  // Debug logs
  if (motorId == 1) {
    std::cout << std::fixed << std::setprecision(2)
              << "[M1] Cel: " << std::setw(8) << target
              << " | Poz: " << std::setw(8) << static_cast<double>(currPosition)
              << " | Błąd: " << std::setw(8) << uchyb
              << " | Sygnał: " << std::setw(5) << static_cast<int>(signal)
              << std::endl;
  }

  motor->send_data(static_cast<int8_t>(signal));
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
