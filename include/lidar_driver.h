#ifndef __Lidar_Driver_Sim_H
#define __Lidar_Driver_Sim_H

//#define WIN32

#include <config.h>
#include <misc_tools.hpp>

#include <rplidar.h>
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
using namespace rp::standalone::rplidar;

void ctrlc(int);

// ע��:���������Ϊ��RplidarA2��Ƶ�
// �����A1���Ƕȼ����1�����0.5�㣬��Ҫ�������
class __lidar_driver {
public:
	__lidar_driver();

	vector<__scandot> laserData;
	double            laserArray[720];					// 0�㿪ʼ, 0.5����

	int init();

	int grab_ScanData();			// ����ɨ������

	int draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show);
	int draw(Mat &dst, double data[], char window_name[], bool is_show);

private:

	char	* opt_com_path = NULL;
	_u32 baudrateArray[2] = { 115200, 256000 };
	_u32 opt_com_baudrate = 0;

	RPlidarDriver * drv;

	rplidar_response_device_info_t devinfo;

	bool checkRPLIDARHealth(RPlidarDriver * drv);

};


#endif	// !__Lidar_Driver_Sim_H
