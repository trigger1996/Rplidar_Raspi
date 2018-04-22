#ifndef __ICP_Interface_H
#define __ICP_Interface_H

#include <config.h>
#include <opencv2/opencv.hpp>

#include <icp.h>

using namespace cv;

class __icp
{
public:

	__icp();

	double dx, dy;
	double d_yaw;
	float     R[4], T[2];				// 旋转矩阵、平移矩阵
	float     err;

	int set_Pts_Ref(vector<__scandot> in);
	int set_Pts_To(vector<__scandot> in);
	int set_Pts_Ref(vector<CvPoint2D32f> in);
	int set_Pts_To(vector<CvPoint2D32f> in);
	int set_Pts_Ref(vector<CvPoint3D32f> in);
	int set_Pts_To(vector<CvPoint3D32f> in);

	vector<CvPoint2D32f> get_Data_Shifted() { return data_shifted; }

	int run(vector<__scandot> ref, vector<__scandot> to, bool is_show);
	int run();

	int run_weighted(vector<CvPoint3D32f> ref, vector<CvPoint3D32f> to, bool is_show);
	int run_weighted();

	void draw_DataResult(double scale, double offset_scale);

private:

	vector<CvPoint2D32f> data_ref, data_to, data_shifted;
	vector<double> weight_ref, weight_to;
	//CvMat r, t;					// 旋转矩阵，平移矩阵的OpenCV指针，这个改成临时变量

	float dx_temp[3], dy_temp[3];	// 滑动平均值滤波
	float dyaw_temp[3];

	int calc_dR_dYaw();
};


#endif	/* __ICP_Interface_H */
