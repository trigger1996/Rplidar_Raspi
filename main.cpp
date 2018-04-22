#include <main.h>

using namespace cv;

int main(int argc, char *argv[]) {
    int stat;

    __lidar_driver *lidar;
    lidar = new __lidar_driver;
    lidar->init();

    __icp *_icp;
    _icp = new __icp;

    GridMapping *map_global;
	map_global = new GridMapping(0, 2.2, -2.2, 200, 0.5, 80000, 150, 0);

    vector<__scandot> data, data_last;
    data.clear();
    data_last.clear();

    double x_esti = 0, y_esti = 0;
	double robotTheta = 0;

    while (true) {

        stat = lidar->grab_ScanData();
        if (stat == __SUCCEEDED) {
            Mat lidarimg;
            lidar->draw(lidarimg, lidar->laserArray, (char *)"233", true);

            data_last = data;
            data = lidar->laserData;
            _icp->set_Pts_To(data_last);
            _icp->set_Pts_Ref(data);
            _icp->run();
            _icp->draw_DataResult(LidarImageScale, LidarImageSize / 2);

            x_esti += _icp->dx;
            y_esti += _icp->dy;

            cout << _icp->dx << "       " << _icp->dy << endl;

            map_global->updateGridMap_Laser(x_esti, y_esti, robotTheta * (CV_PI / 180), data);
			map_global->showGridMap("Laser Grid Map");

        }

        waitKey(1);
    }

    waitKey(0);
    return 0;
}



