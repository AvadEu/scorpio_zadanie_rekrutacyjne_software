#pragma once

#include "tester.hpp"
#include "../backend/mock_component.hpp"
#include "../backend/motor.hpp"

class CameraController {
public:
  CameraController(std::shared_ptr<backend_interface::Tester> tester, bool preempt_mode);
  ~CameraController() = default;

private:
  std::shared_ptr<backend_interface::Component<signed char, unsigned short> > motor1_;
  std::shared_ptr<backend_interface::Component<signed char, unsigned short> > motor2_;
  std::shared_ptr<backend_interface::Component<backend_interface::Tester::Impossible, Point> > commands_;
  bool preempt_mode_;
  const unsigned int ENCODER_SCOPE;
  bool onTheWayToTarget_;
  double targetEncoderVal1_{0.0};
  double targetEncoderVal2_{0.0};
  bool motor1InPosition_{false};
  bool motor2InPosition_{false};

  // Proportional gain for motors
  double Kp1_{1.5};
  double Kp2_{1.5};

  // Callbacks
  void onMotor1Data(const uint16_t &position);
  void onMotor2Data(const uint16_t &position);
  void onCommandReceived(const Point &target);

  // Moving utils
  void convertCoordinatesForEncoder(const Point &target);
  void updateMotorRegulator(const uint16_t &currPosition, int motorId);
};

