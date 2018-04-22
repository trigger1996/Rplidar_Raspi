#ifndef __Config_H
#define __Config_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#ifndef _countof
    #define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))       // linux没有这个宏，只能自己定义
#endif

#define PI 3.1415926535897932384626

#define __SUCCEEDED	    0
#define __FAILED		1
#define __STANDBY		2
#define __FINISHED	    4

#define LidarImageSize   600
#define LidarImageWidth  LidarImageSize
#define LidarImageHeight LidarImageSize
#define LidarImageScale  0.025				// 0.025, Ä¬ÈÏ: A1ÔçÆÚ°æ±¾£¬6m°ë¾¶¡ª¡ª1 / 20

typedef struct {
	double dst;
	double angle;
} __scandot;



#endif	// __Config_H
