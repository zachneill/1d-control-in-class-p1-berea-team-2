#ifndef WALL_FOLLOWER_COMMON_DRIVE_H
#define WALL_FOLLOWER_COMMON_DRIVE_H

#include <lcm/lcm-cpp.hpp>

#define MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"
#define MBOT_MOTOR_COMMAND_CHANNEL "MBOT_MOTOR_COMMAND"

static lcm::LCM lcmInstance(MULTICAST_URL);

void drive(const float vx, const float vy, const float wz);

#endif  // WALL_FOLLOWER_COMMON_DRIVE_H
