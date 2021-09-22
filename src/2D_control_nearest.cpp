/**
 * File: 2D_control_nearest.cpp
 *
 * Controls the robot to maintain a given distance to the nearest wall.
 *
 * This code finds the distance to the nearest wall in the Lidar scan. It
 * applies a control to the robot in the direction of the wall using the angle
 * to the scan.
 */

#include <iostream>
#include <cmath>

#include <signal.h>

#include <robot_control/common/utils.h>
#include <robot_control/common/lidar.h>
#include <robot_control/common/drive.h>


float feedbackControl(float dist_to_wall, float setpoint)
{
    float vel = 0;
    /**
     * TODO: Calculate the control command to send to the robot given the
     * current distance to the wall and the desired setpoint.
     *
     * You can use either Bang-Bang control or P-control. Reuse your code from
     * the 1D control activity.
     **/
    return vel;
}


int findMinDist(const LidarScan& scan)
{
    int min_idx = 0;
    /**
     * TODO: Return the index of the shortest ray in the Lidar scan. For
     * example, if the shortest ray is the third one, at index 2, return 2.
     *
     * HINT: The length of each ray is stored in the vector scan.ranges.
     *
     * HINT: Do not take into account any rays which have 0 intensity. Those rays
     * will have default range 0, which will always be the minimum if you forget
     * to check the intensity. The intensities are stored in scan.intensities.
     **/
    return min_idx;
}


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main(int argc, const char *argv[])
{
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize the driver and make sure initialization was successful.
    auto drv = initLidarDriver();

    if (!drv)
    {
        std::cerr << "Failed to initialize driver." << std::endl;
        return -1;
    }

    float dt = 0.01;        // seconds
    float setpoint = 0.35;  // meters

    while (true) {
        LidarScan scan = readLidarScan(drv);

        if (scan.good)
        {
            // Get the distance to the wall.
            float min_idx = findMinDist(scan);
            float dist_to_wall = scan.ranges[min_idx];
            float angle_to_wall = scan.thetas[min_idx];

            std::cout << "Min distance: " << dist_to_wall << " Angle: " << angle_to_wall;
            std::cout << " Intensity: " << scan.intensities[min_idx] << " | ";

            // Calculate the appropriate control signal.
            float vel = feedbackControl(dist_to_wall, setpoint);

            std::cout << "Setpoint: " << setpoint << " Velocity: " << vel;

            // Apply the control signal.
            float vx = 0;
            float vy = 0;
            /**
             * TODO: Use the angle to the wall (angle_to_wall) to decompose the
             * velocity command (vel) into its x and y components. Store these
             * in vx and vy respectively.
             **/
            std::cout << " (vx, vy): (" << vx << ", " << vy << ")\n";

            drive(vx, vy, 0);
        }

        sleepFor(dt);

        if (ctrl_c_pressed) break;
    }

    // Stop the robot and clean up the driver.
    drive(0, 0, 0);
    stopLidarDriver(drv);
    return 0;
}
