#ifndef __Config_H
#define __Config_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

//#define WIN32

#define PI 3.1415926535897932384626

#define __SUCCEEDED	0
#define __FAILED		1
#define __STANDBY		2
#define __FINISHED	4

#define LidarImageSize   600
#define LidarImageWidth  LidarImageSize
#define LidarImageHeight LidarImageSize
#define LidarImageScale  0.025				// 0.025, 默认: A1早期版本，6m半径——1 / 20

#ifndef WIN32
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

typedef struct {
    double dst;
    double angle;
} __scandot;

typedef struct {
    double x;
    double y;
    double z;
} __vector3f;

typedef struct {
    double x;
    double y;
} __vector2f;

typedef struct {
    double x;
    double y;
    double yaw;
} __posvec2f;

typedef struct {
    double x;
    double y;
    double z;
    double pitch;
    double roll;
    double yaw;
} __posvec3f;



#endif	// __Config_H

