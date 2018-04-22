#include <icp_interface.h>

// http://lib.csdn.net/article/opencv/32238
// 主要现在人状态比较爆炸，不想手写这个库，所以先凑合着用

__icp::__icp()
{
	R[0] = 1.0f, R[1] = 0.0f, R[2] = 0.0f, R[3] = 1.0f;
	T[0] = T[1] = 0.0f;

	dx_temp[0] = dx_temp[1] = dx_temp[2] = 0.0f;
	dy_temp[0] = dy_temp[1] = dy_temp[2] = 0.0f;
	dyaw_temp[0] = dyaw_temp[1] = dyaw_temp[2] = 0.0f;

	d_yaw = 0;
	dx = dy = 0;

}// __icp::__icp()

int __icp::set_Pts_Ref(vector<__scandot> in)
{
	int i;
	CvPoint2D32f temp;
	float theta, rho;

	data_ref.clear();
	for (i = 0; i < in.size(); i++)
	{
		__scandot dot;
		dot = in[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

		theta = dot.angle * PI / 180;
		rho = dot.dst;

		temp.x = rho  * sin(theta);			// * LidarImageScale + halfWidth
		temp.y = -rho * cos(theta);			// 一个主要的失真原因是作图的时候的比例尺缩放
											// 解决这个事情以后，即不缩放，就算采样一次处理一次信噪比也够
		data_ref.push_back(temp);
	}

	return in.size();
}

int __icp::set_Pts_To(vector<__scandot> in)
{
	int i;
	CvPoint2D32f temp;
	float theta, rho;

	data_to.clear();
	for (i = 0; i < in.size(); i++)
	{
		__scandot dot;
		dot = in[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

		theta = dot.angle * PI / 180;
		rho = dot.dst;

		temp.x = rho  * sin(theta);			// * LidarImageScale + halfWidth
		temp.y = -rho * cos(theta);			// 一个主要的失真原因是作图的时候的比例尺缩放
											// 解决这个事情以后，即不缩放，就算采样一次处理一次信噪比也够
		data_to.push_back(temp);
	}

	return in.size();
}


int __icp::set_Pts_Ref(vector<CvPoint2D32f> in)
{
	int i;
	CvPoint2D32f temp;
	float theta, rho;

	data_ref.clear();
	for (i = 0; i < in.size(); i++)
	{
		temp = in[i];
		data_ref.push_back(temp);
	}

	return in.size();
}

int __icp::set_Pts_To(vector<CvPoint2D32f> in)
{
	int i;
	CvPoint2D32f temp;
	float theta, rho;

	data_to.clear();
	for (i = 0; i < in.size(); i++)
	{
		temp = in[i];
		data_to.push_back(temp);
	}

	return in.size();
}

int __icp::set_Pts_Ref(vector<CvPoint3D32f> in)
{
	int i;
	CvPoint2D32f temp;
	float theta, rho;

	data_ref.clear();
	for (i = 0; i < in.size(); i++)
	{
		temp.x = in[i].x;
		temp.y = in[i].y;
		weight_ref.push_back(in[i].z);
		data_ref.push_back(temp);
	}

	return in.size();
}

int __icp::set_Pts_To(vector<CvPoint3D32f> in)
{
	int i;
	CvPoint2D32f temp;
	float theta, rho;

	data_to.clear();
	weight_to.clear();
	for (i = 0; i < in.size(); i++)
	{
		temp.x = in[i].x;
		temp.y = in[i].y;
		weight_to.push_back(in[i].z);
		data_to.push_back(temp);
	}

	return in.size();
}


int __icp::run(vector<__scandot> ref, vector<__scandot> to, bool is_show)
{
	int stat;

	set_Pts_Ref(ref);
	set_Pts_To(to);
	stat = run();

	if (is_show) {
		draw_DataResult(LidarImageScale, 1.0f);
	}

	return stat;
}// int __icp::run(vector<__scandot> in)

int __icp::run()
{

	//image = image_base.clone();

	// 没数据就放弃计算
	if (!data_to.size() || !data_ref.size())
		return -1;

	CvMat r = cvMat(2, 2, CV_32F, R);
	CvMat t = cvMat(2, 1, CV_32F, T);

	// ICP主程序
	err = icp(&data_to[0], data_to.size(),
		&data_ref[0], data_ref.size(),
		&r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 60, 0.00001));	// cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)
																//cout << "err: " << err << endl;															// 静态下err大约不会超过4000，运动时error在10000以上

																// 测算和保存平移数据，这段和data_shifted都可以不要其实
	data_shifted.clear();
	for (int i = 0; i < (int)data_to.size(); i++)
	{
		CvPoint2D32f temp;
		float x = data_to[i].x;
		float y = data_to[i].y;
		float X = (R[0] * x + R[1] * y + T[0]);
		float Y = (R[2] * x + R[3] * y + T[1]);
		temp.x = X;
		temp.y = Y;
		data_shifted.push_back(temp);
	}


	// 关键，计算结果
	calc_dR_dYaw();

	// 打印数据，显示结果
	//printf("err = %f\n", err);
	//cout << "dX: " << dx << " dY: " << dy << endl;
	//cout << "d_yaw: " << d_yaw * 180.0f / PI << endl;
	//cout << "R[0]: " << R[0] << " R[1]: " << R[1] << endl;

	return __SUCCEEDED;

}// int __icp::run()

int __icp::run_weighted(vector<CvPoint3D32f> ref, vector<CvPoint3D32f> to, bool is_show)
{
	int stat;

	set_Pts_Ref(ref);
	set_Pts_To(to);

	stat = run_weighted();

	if (is_show) {
		draw_DataResult(LidarImageScale, 1.0f);
	}

	return stat;
}// int __icp::run_weighted(vector<__scandot> in)

int __icp::run_weighted()
{

	//image = image_base.clone();

	// 没数据就放弃计算
	if (!data_to.size() || !data_ref.size())
		return -1;

	if (weight_to.size() < data_to.size()) {
		for (int i = weight_to.size(); i < data_to.size(); i++)
			weight_to.push_back(1.0);
	}
	if (weight_ref.size() < data_ref.size()) {
		for (int i = weight_ref.size(); i < data_ref.size(); i++)
			weight_ref.push_back(1.0);
	}

	CvMat r = cvMat(2, 2, CV_32F, R);
	CvMat t = cvMat(2, 1, CV_32F, T);

	// ICP主程序
	err = icp(&data_to[0], data_to.size(),
		&data_ref[0], data_ref.size(),
		&r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 60, 0.00001));	// cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)
																//cout << "err: " << err << endl;															// 静态下err大约不会超过4000，运动时error在10000以上

																// 测算和保存平移数据，这段和data_shifted都可以不要其实
	data_shifted.clear();
	for (int i = 0; i < (int)data_to.size(); i++)
	{
		CvPoint2D32f temp;
		float x = data_to[i].x;
		float y = data_to[i].y;
		float X = (R[0] * x + R[1] * y + T[0]);
		float Y = (R[2] * x + R[3] * y + T[1]);
		temp.x = X;
		temp.y = Y;
		data_shifted.push_back(temp);
	}


	// 关键，计算结果
	calc_dR_dYaw();

	// 打印数据，显示结果
	//printf("err = %f\n", err);
	//cout << "dX: " << dx << " dY: " << dy << endl;
	//cout << "d_yaw: " << d_yaw * 180.0f / PI << endl;
	//cout << "R[0]: " << R[0] << " R[1]: " << R[1] << endl;

	return __SUCCEEDED;

}// int __icp::run_weighted()

void __icp::draw_DataResult(double scale, double offset_scale) {

	//Mat image_base(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));
	Mat image(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));

	const int halfWidth  = LidarImageWidth / 2;
	const int halfHeight = LidarImageHeight / 2;

	int offset_x = halfWidth * offset_scale;
	int offset_y = halfHeight * offset_scale;


	// 作图
	// 画图，顺带储存数据
	// 红的：变换后的点
	// 绿的：变换前的点
	// 蓝的：参考点
	// 理论上，红蓝应该几乎重合，绿色和蓝色不重合
	for (int i = 0; i < (int)data_shifted.size(); i++)
	{
		float x_pt = data_shifted[i].x * scale + offset_x;
		float y_pt = data_shifted[i].y * scale + offset_y;
		circle(image, Point(x_pt, y_pt), 1, Scalar(0, 0, 255), -1, 8, 0);		// R
	}
	for (int i = 0; i < data_ref.size(); i++)
	{
		float x_pt = data_ref[i].x * scale + offset_x;
		float y_pt = data_ref[i].y * scale + offset_y;
		circle(image, Point(x_pt, y_pt), 1, Scalar(255, 0, 0), -1, 8, 0);		// B
	}
	for (int i = 0; i < data_to.size(); i++)
	{
		float x_pt = data_to[i].x * scale + offset_x;
		float y_pt = data_to[i].y * scale + offset_y;
		circle(image, Point(x_pt, y_pt), 1, Scalar(0, 255, 0), -1, 8, 0);		// G
	}
	imshow("icp", image);
}

int __icp::calc_dR_dYaw()
{
	//  计算平移和旋转矩阵

	// 平移矩阵
	// 滑动平均值滤波
	dx_temp[2] = dx_temp[1], dx_temp[1] = dx_temp[0], dx_temp[0] = T[0];
	dy_temp[2] = dy_temp[1], dy_temp[1] = dy_temp[0], dy_temp[0] = T[1];

	dx = (dx_temp[0] + dx_temp[1] + dx_temp[2]) / 3.0f;
	dy = (dy_temp[0] + dy_temp[1] + dy_temp[2]) / 3.0f;

	// 旋转矩阵
	float dyaw_cos = R[0];
	float dyaw_sin = R[1];
	// 因为dyaw很小，所以无需考虑-180~+180的情况，但是yaw就要了
	// 滑动平均值滤波
	dyaw_temp[2] = dyaw_temp[1], dyaw_temp[1] = dyaw_temp[0];
	//dyaw_temp[0] = (acos(dyaw_cos) + asin(dyaw_sin)) / 2.0f;		// acos有点小毛病，做不出负值，所以就很奇怪
	dyaw_temp[0] = asin(dyaw_sin);									// sin的正负注意一下
	d_yaw = (dyaw_temp[0] + dyaw_temp[1] + dyaw_temp[2]) / 3.0f;

	return __SUCCEEDED;

}// int __icp::calc_dR_dYaw()
