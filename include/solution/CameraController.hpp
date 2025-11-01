#pragma once

#include "tester.hpp"
#include "../backend/mock_component.hpp"
#include "../backend/motor.hpp"

class CameraController {
public:
    CameraController(backend_interface::Tester*, bool preempt_mode);
    ~CameraController() = default;
private:
    std::shared_ptr<backend_interface::Component<signed char, unsigned short>> motor1_;
    std::shared_ptr<backend_interface::Component<signed char, unsigned short>> motor2_;
    std::shared_ptr<backend_interface::Component<backend_interface::Tester::Impossible, Point>> commands_;
    bool preempt_mode_;
    const unsigned int ENCODER_SCOPE;
};

