#include "include/lidar_driver.h"


__lidar_driver::__lidar_driver() {

}

int __lidar_driver::init() {

    u_result     op_result;

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB2";
#endif
    }

    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        return -2;
    }

    bool connectSuccess = false;
    size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
    for (size_t i = 0; i < baudRateArraySize; ++i)
    {
        if (!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
                break;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        return -3;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
        "Firmware Ver: %d.%02d\n"
        "Hardware Rev: %d\n"
        , devinfo.firmware_version >> 8
        , devinfo.firmware_version & 0xFF
        , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        return -4;
    }

    signal(SIGINT, ctrlc);

    drv->startMotor();
    // start scan...
    //drv->startScan(0,1);
    drv->startScanExpress(0, RPLIDAR_CONF_SCAN_COMMAND_HQ);

    return 0;

}

bool __lidar_driver::checkRPLIDARHealth(RPlidarDriver * drv)
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
        }
        else {
            return true;
        }

    }
    else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int __lidar_driver::grab_ScanData() {

    rplidar_response_measurement_node_t nodes[8192];
    size_t   count = _countof(nodes);
    u_result     op_result;

    op_result = drv->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
        drv->ascendScanData(nodes, count);

        laserData.clear();
        for (int pos = 0; pos < (int)count; ++pos) {
            __scandot dot;
            if (!nodes[pos].distance_q2) continue;

            //dot.qualtiy = (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            dot.angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            dot.dst = nodes[pos].distance_q2 / 4.0f;
            laserData.push_back(dot);
        }

        // scandot转换double数组
        memset(laserArray, 0, sizeof(double) * 720);
        for (int i = 0; i < laserData.size(); i++) {
            int integer = (int)laserData[i].angle;				// 角度整数部分
            float decimal = laserData[i].angle - integer;		// 角度小数部分

            if (decimal < 0.25f) {
                laserArray[integer * 2] = laserData[i].dst;
            }
            else if (decimal >= 0.25f && decimal < 0.75f) {
                laserArray[integer * 2 + 1] = laserData[i].dst;
            }
            else if (decimal >= 0.75f) {
                laserArray[integer * 2 + 2] = laserData[i].dst;
            }
        }

        return __SUCCEEDED;
    }

    return __FAILED;

}

//将扫描点映射到画布上
int __lidar_driver::draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show) {
    Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(29, 230, 181));		// 图片格式： BGR
    dst = zero.clone();
    zero.release();

    //在中心加上一个圆心
    //circle(dst, Point(dst.cols / 2, dst.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

    int x, y;
    double theta, rho;
    int halfWidth = dst.cols / 2;
    int halfHeight = dst.rows / 2;


    for (unsigned int i = 0; i < data.size(); i++) {	// scan_data.size()

        __scandot dot;
        dot = data[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

        theta = dot.angle * PI / 180;
        rho = dot.dst;

        x = (int)(rho  * sin(theta) * LidarImageScale) + halfWidth;
        y = (int)(-rho * cos(theta) * LidarImageScale) + halfHeight;

        circle(dst, Point(x, y), 1, Scalar(0, 0, 255), -1, 8, 0);

    }

    //char s[35];
    //sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
    //putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

    if (is_show)
        imshow(window_name, dst);

    return __SUCCEEDED;
}

int __lidar_driver::draw(Mat &dst, double data[], char window_name[], bool is_show) {
    Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(29, 230, 181));		// 图片格式： BGR
    dst = zero.clone();
    zero.release();

    //在中心加上一个圆心
    //circle(dst, Point(dst.cols / 2, dst.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

    int x, y;
    double theta, rho;
    int halfWidth = dst.cols / 2;
    int halfHeight = dst.rows / 2;


    for (unsigned int i = 0; i < 720; i++) {	// scan_data.size()
        theta = i * 0.5 * PI / 180;
        rho = data[i];

        x = (int)(rho  * sin(theta) * LidarImageScale) + halfWidth;
        y = (int)(-rho * cos(theta) * LidarImageScale) + halfHeight;

        circle(dst, Point(x, y), 1, Scalar(0, 0, 255), -1, 8, 0);

    }

    //char s[35];
    //sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
    //putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

    if (is_show)
        imshow(window_name, dst);

    return __SUCCEEDED;
}

// 系统的回调
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
