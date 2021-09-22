#ifndef WALL_FOLLOWER_COMMON_LIDAR_H
#define WALL_FOLLOWER_COMMON_LIDAR_H

#include <vector>

#include <rplidar/rplidar.h>

#include "utils.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;


RPlidarDriver* initLidarDriver();
void stopLidarDriver(RPlidarDriver* drv);
bool checkRPLIDARHealth(RPlidarDriver* drv);
LidarScan readLidarScan(RPlidarDriver* drv);

#endif  // WALL_FOLLOWER_COMMON_LIDAR_H
