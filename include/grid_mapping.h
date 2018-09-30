#ifndef GRID_MAPPING_H
#define GRID_MAPPING_H

#include "config.h"

/** Opencv includes */
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <string>
#include <algorithm>

#include <math.h>


using namespace std;
using namespace cv;


class GridMapping {
public:
    GridMapping(const double &l0, const double &locc, const double &lfree, const double &alpha, const double &beta, const double &Zmax, const double &Zmin, const unsigned char &sensorType);

//  IplImage *get_Image() { return gridMapImage; }
    Mat       get_Image() { return Mat(gridMapImage); }

    void updateGridMap(const double &robotX, const double &robotY, const double &robotTheta, const double sensorData[]);
    void drawGridMap();
    void showGridMap(const string &windowName);
    void saveGridMap(const string &fileName);

    void updateGridMap_Laser(const double &robotX, const double &robotY, const double &robotTheta, const double sensorData[]);
    void updateGridMap_Laser(const double &robotX, const double &robotY, const double &robotTheta, const vector<__scandot> sensorData);

    void updateEdge(double threshold);

    /** Grid Size (mm) */
    const static int gridWidth  = 50; // 100 for data1, 100 for data2, 100 for data3, 100 for data4, 100 for data5,
    const static int gridHeight = 50; // 100 for data1, 100 for data2, 100 for data3, 100 for data4, 100 for data5,

    /** Map Size (mm) */
    const static int mapWidth  = 60000; // 35000 for data1, 30000 for data2, 45000 for data3, 15000 for data4, 30000 for data5
    const static int mapHeight = 35000; // 25000 for data1, 30000 for data2, 25000 for data3, 50000 for data4, 15000 for data5

    double l[mapWidth / gridWidth][mapHeight / gridHeight];
    vector<CvPoint3D32f> edge;

private:
    IplImage *gridMapImage;

    /** Thickness of obstacles (mm)*/
    double alpha;

    /** Width of the beam (radian)*/
    double beta;

    double l0;
    double locc;
    double lfree;
    double Zmax;
    double Zmin;

    unsigned char sensorType;

    /** Robot Location on grid map image (mm) */
    const static int robotXOffset = mapWidth  / 5; // 5 for data1, 5 for data2, 1.5 for data3, 3 for data4, 5 for data5
    const static int robotYOffset = mapHeight / 3; // 4 for data1, 4 for data2, 2 for data3, 5 for data4, 3 for data5

    double inverseSensorModel(const double &x, const double &y, const double &theta, const double &xi, const double &yi, const double sensorData[]);

    void gridToXY(const int &x, const int &y, double &xi, double &yi);
};


#endif // GRID_MAPPING_H
