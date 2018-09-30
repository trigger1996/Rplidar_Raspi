#include "include/config.h"
#include "include/lidar_driver.h"
#include "include/grid_mapping.h"
#include "include/icp_interface.h"

using namespace std;
using namespace cv;

inline int draw(Mat &dst, vector<__scandot> data, vector<__scandot> data_last);

int main(int argc, char **argv) {

    int i;					// 计数变量
    int stat;				// 状态变量

    double x_esti = 0, y_esti = 0, yaw_esti = 0;

    __lidar_driver *lidar;
    GridMapping *map_laser;

    lidar = new __lidar_driver;
    lidar->init();

    __icp *_icp;
    _icp = new __icp;

    map_laser = new GridMapping(0, 2.2, -2.2, 200, 0.5, 13500, 0, 0);		// 0, 2.2, -2.2, 200, 0.5, 15000, 0

    stat = __SUCCEEDED;
    i = 1;

    vector<__scandot> data, data_last;
    Mat dst;

    while (true) {

        stat = lidar->grab_ScanData();
        if (stat == __SUCCEEDED) {
            //lidar->draw(dst, lidar->laserData, (char *)"raw", true);

            data_last = data;
            data = lidar->laserData;
            _icp->set_Pts_To(data_last);
            _icp->set_Pts_Ref(data);
            _icp->run();
            _icp->draw_DataResult(LidarImageScale, 0.5);

            x_esti += _icp->dx;
            y_esti += _icp->dy;
            yaw_esti += _icp->d_yaw;
            cout << "dx " << _icp->dx << "\t dy " << _icp->dy << "\t x: " << x_esti << "\t y: " << y_esti << "\t yaw: " << yaw_esti << endl;        // mm

            draw(dst, data, data_last);
            imshow("data", dst);
        }

        i++;
        waitKey(30);
    }
    waitKey(0);
    return 0;
}// int main(int argc, char **argv)

inline int draw(Mat &dst, vector<__scandot> data, vector<__scandot> data_last) {
    Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
    dst = zero.clone();
    zero.release();

    int x, y;
    double theta, rho;
    int halfWidth = dst.cols / 2;
    int halfHeight = dst.rows / 2;

    for (unsigned int i = 0; i < data_last.size(); i++) {	// scan_data.size()
        __scandot dot;
        dot = data_last[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

        theta = dot.angle * PI / 180;
        rho = dot.dst;

        x = (int)(rho  * sin(theta) * LidarImageScale) + halfWidth;
        y = (int)(-rho * cos(theta) * LidarImageScale) + halfHeight;

        circle(dst, Point(x, y), 1, Scalar(128, 255, 0), -1, 8, 0);
    }

    for (unsigned int i = 0; i < data.size(); i++) {	// scan_data.size()
        __scandot dot;
        dot = data[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

        theta = dot.angle * PI / 180;
        rho = dot.dst;

        x = (int)(rho  * sin(theta) * LidarImageScale) + halfWidth;
        y = (int)(-rho * cos(theta) * LidarImageScale) + halfHeight;

        circle(dst, Point(x, y), 1, Scalar(0, 0, 255), -1, 8, 0);
    }

    return __SUCCEEDED;
}
