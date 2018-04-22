#ifndef __Misc_Tools_HPP
#define __Misc_Tools_HPP

#include "config.h"

#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

static bool compare_scandot(const __scandot& a1, const __scandot& a2)
{
	return a1.angle <= a2.angle;
}

static double gaussrand(double E, double V)
{
	// 高斯噪声产生工具
	// https://www.cnblogs.com/yeahgis/archive/2012/07/13/2590485.html
	// 仿真用
	// 期望E, 方差V

	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	X = X * V + E;

	return X;
}

#endif	// !__Misc_Tools_HPP