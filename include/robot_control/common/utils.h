#ifndef WALL_FOLLOWER_COMMON_UTILS_H
#define WALL_FOLLOWER_COMMON_UTILS_H

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#define PI 3.1415926535f
#define HIGH 1e6

struct LidarScan
{
    bool good;

    int utime;
    int num_ranges;

    std::vector<float> ranges;
    std::vector<float> thetas;
    std::vector<float> intensities;
    std::vector<float> times;
};

/**
 * Gets the current time in microseconds.
 */
static inline int getTimeMicro()
{
    auto now = std::chrono::system_clock::now();
    return now.time_since_epoch().count();
}

/**
 * Sleeps for the provided number of seconds.
 */
static inline void sleepFor(const double secs)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(secs * 1000)));
}

float findFwdDist(const LidarScan& scan);

#endif  // WALL_FOLLOWER_COMMON_UTILS_H
