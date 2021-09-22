#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <robot_control/common/lidar.h>

RPlidarDriver* initLidarDriver()
{
    //Shouldn't need to adjust these with the exception of pwm
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;
    uint16_t pwm = 700;

    printf("Initializing RPLIDAR.\n"
           "Version: %s\n", RPLIDAR_SDK_VERSION);

    if (!opt_com_path) {
        opt_com_path = "/dev/ttyUSB0";
    }

    // create the driver instance
    RPlidarDriver* drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        return nullptr;
    }

    rplidar_response_device_info_t devinfo;

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot get device info.\n");
        return nullptr;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        return nullptr;
    }

    drv->startMotor();
    // start scan...
    drv->setMotorPWM(pwm);
    drv->startScan();

    return drv;
}


void stopLidarDriver(RPlidarDriver* drv)
{
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
}


bool checkRPLIDARHealth(RPlidarDriver* drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


LidarScan readLidarScan(RPlidarDriver* drv)
{
    rplidar_response_measurement_node_t nodes[360*2];
    size_t count = _countof(nodes);

    u_result op_result = drv->grabScanData(nodes, count);

    LidarScan scan;

    int64_t now = getTimeMicro();  // get current timestamp in milliseconds
    scan.utime = now;

    // If the scan is not valid, mark it and return.
    if (!IS_OK(op_result))
    {
        scan.good = false;
        return scan;
    }

    scan.good = true;

    drv->ascendScanData(nodes, count);

    scan.num_ranges = count;

    scan.ranges.resize(count);
    scan.thetas.resize(count);
    scan.intensities.resize(count);
    scan.times.resize(count);

    for (int pos = 0; pos < (int)count ; ++pos)
    {
        now = getTimeMicro();  // get current timestamp in milliseconds
        // The scan angles are opposite to the positive theta direction, so reverse them here.
        int scan_idx = (int)count - pos - 1;
        scan.ranges[pos] = nodes[scan_idx].distance_q2/4000.0f;
        scan.thetas[pos] = 2*PI - (nodes[scan_idx].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)*PI/11520.0f;
        scan.intensities[pos] = nodes[scan_idx].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        scan.times[pos] = now;
    }

    return scan;
}
