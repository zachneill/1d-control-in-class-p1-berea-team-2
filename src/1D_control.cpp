/**
 * File: 1D_control.cpp
 *
 * Controls the robot to maintain a given distance to the wall directly in
 * front of it.
 *
 * This code uses Bang-Bang control or P-control to maintain the setpoint
 * distance.
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
  float control_mag = dist_to_wall - setpoint;
  if (control_mag < 0) {
    vel = -10;
  }
  else if (control_mag >0) {
    vel = control_mag;
  }
  return vel;
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
            float dist_to_wall = findFwdDist(scan);
            if (dist_to_wall < 0) continue;

            // Calculate the appropriate control signal.
            float vel = feedbackControl(dist_to_wall, setpoint);

            std::cout << "Setpoint: " << setpoint << " Current distance: " << dist_to_wall;
            std::cout << " Velocity command: " << vel << "\n";

            // Apply the control signal.
            drive(vel, 0, 0);
        }

        sleepFor(dt);

        if (ctrl_c_pressed) break;
    }

    // Stop the robot and clean up the driver.
    drive(0, 0, 0);
    stopLidarDriver(drv);
    return 0;
}
