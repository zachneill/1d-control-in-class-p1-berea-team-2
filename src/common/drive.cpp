#include <omnibot_msgs/omni_motor_command_t.hpp>

#include <robot_control/common/utils.h>
#include <robot_control/common/drive.h>

void drive(const float vx, const float vy, const float wz)
{
    omnibot_msgs::omni_motor_command_t cmd;
    cmd.utime = getTimeMicro();
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.wz = wz;

    lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
}
