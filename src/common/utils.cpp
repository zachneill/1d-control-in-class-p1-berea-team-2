#include <iostream>

#include <robot_control/common/utils.h>

float findFwdDist(const LidarScan& scan)
{
    // Simple average filter over the front rays in case a few are invalid.
    int num_range = 5;
    int num_scans = 0;
    float sum = 0;

    for (int i = 0; i < num_range; ++i)
    {
        if (scan.intensities[i] > 0)
        {
            sum += scan.ranges[i] * cos(scan.thetas[i]);
            num_scans++;
        }
    }
    for (int i = scan.num_ranges - num_range; i < scan.num_ranges; ++i)
    {
        if (scan.intensities[i] > 0)
        {
            sum += scan.ranges[i] * cos(scan.thetas[i]);
            num_scans++;
        }
    }

    if (num_scans < 1) return -1;

    return sum / num_scans;
}
