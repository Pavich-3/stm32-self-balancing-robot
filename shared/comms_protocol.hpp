#pragma once

#include <cstdint>
#include <cstddef>
#include "robot_types.hpp"

struct __attribute__((packed)) TelemetryPacket {
    static constexpr uint8_t START_BYTE = 0xAA; 
    static constexpr uint8_t PACKET_ID = 0x01; 

    uint8_t startByte {START_BYTE};
    uint8_t packetId  {PACKET_ID};
    float tiltAngleDeg, tiltRateDPS;
    float speedLeftRPM, speedRightRPM;
    float pwmDutyLeft, pwmDutyRight;
    RobotState state;
    uint8_t checksum;
};

struct __attribute__((packed)) ParamPacket {
    uint8_t startByte;
    uint8_t packedID;
    float kpAngle, kiAngle, kdAngle;
    float kpSpeed, kiSpeed, kdSpeed;
    uint8_t checksum;
};

template <typename T>
uint8_t computeChecksum(const T& packet) {
    const auto* ptr = reinterpret_cast<const uint8_t*>(&packet);
    uint8_t checksum = 0;
    for (std::size_t i = 0; i < sizeof(T) - 1; ++i) {
        checksum ^= ptr[i];
    }
    return checksum;
}