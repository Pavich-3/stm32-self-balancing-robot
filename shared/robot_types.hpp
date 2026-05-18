#pragma once

#include <cstdint>

enum class RobotState : uint8_t {
    Initializing = 0,
    Balancing = 1, 
    Fallen = 2,
    Error = 3
};

enum class MotorDirection : uint8_t {
    Forward = 0,
    Backward = 1,
    Coast = 2, 
    Brake = 3
};