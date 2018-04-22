#include <og_mapping.h>

/*
这段代码是git上下载下来以后，用于参考的
https://github.com/markcsie/OccupancyGridMapping
代码进行了优化，提高了其激光数据的运算速度
调用范例
while(1)前
GridMapping *laserGridMapping;
laserGridMapping = new GridMapping(0, 2.2, -2.2, 200, 0.5, 15000, 0, 0);	// 0, 2.2, -2.2, 200, 0.5, 80000, 0, 0
while(1)内
laserGridMapping->updateGridMap_Laser(robotX, robotY, robotTheta * (CV_PI / 180), laserData);
laserGridMapping->showGridMap("Laser Grid Map");
waitKey(1);
最后一行需要openCV

*/

GridMapping::GridMapping(const double &l0, const double &locc, const double &lfree, const double &alpha, const double &beta, const double &Zmax, const double &Zmin, const unsigned char &sensorType) {
	this->l0 = l0;
	this->locc = locc;
	this->lfree = lfree;
	this->alpha = alpha;
	this->beta = beta * CV_PI / 180;
	this->Zmax = Zmax;
	this->Zmin = Zmin;
	this->sensorType = sensorType;

	for (int x = 0; x < mapWidth / gridWidth; x++) {
		for (int y = 0; y < mapHeight / gridHeight; y++) {
			l[x][y] = l0;
		}
	}
	gridMapImage = cvCreateImage(cvSize(mapWidth / gridWidth, mapHeight / gridHeight), IPL_DEPTH_8U, 1);
	edgeMapImage = cvCreateImage(cvSize(mapWidth / gridWidth, mapHeight / gridHeight), IPL_DEPTH_8U, 1);
}

GridMapping::~GridMapping() {
	cvReleaseImage(&gridMapImage);
	cvReleaseImage(&edgeMapImage);
}

/** Occupancy Grid Mapping Algorithm, please refer to textbook "Probabilistic Robotics" */
void GridMapping::updateGridMap(const double &robotX, const double &robotY, const double &robotTheta, const double sensorData[]) {

	for (int x = 0; x < mapWidth / gridWidth; x++) {
		for (int y = 0; y < mapHeight / gridHeight; y++) {
			double xi, yi;
			gridToXY(x, y, xi, yi);
			if (sqrt(pow(xi - robotX, 2) + pow(yi - robotY, 2)) <= Zmax) {
				l[x][y] = l[x][y] + inverseSensorModel(robotX, robotY, robotTheta, xi, yi, sensorData) - l0;
			}
		}
	}
}
double GridMapping::inverseSensorModel(const double &x, const double &y, const double &theta, const double &xi, const double &yi, const double sensorData[]) {

	double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
	double phi = atan2(yi - y, xi - x) - theta;


	double Zk;
	double thetaK;

	double sensorTheta;
	double minDelta = -1;

	switch (sensorType) {
	case 0:
		for (int i = 0; i < 361; i++) {
			sensorTheta = (-90 + i * 0.5) * (CV_PI / 180);
			if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
				Zk = sensorData[i];
				thetaK = sensorTheta;
				minDelta = fabs(phi - sensorTheta);
			}
		}
		break;
	case 1:
		/** -90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90*/

		for (int i = 0; i < 8; i++) {
			if (i == 0) {
				sensorTheta = -90 * (CV_PI / 180);
			}
			else if (i == 1) {
				sensorTheta = -37.5 * (CV_PI / 180);
			}
			else if (i == 6) {
				sensorTheta = 37.5 * (CV_PI / 180);
			}
			else if (i == 7) {
				sensorTheta = 90 * (CV_PI / 180);
			}
			else {
				sensorTheta = (-37.5 + (i - 1) * 15) * (CV_PI / 180);
			}

			if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
				Zk = sensorData[i];
				thetaK = sensorTheta;
				minDelta = fabs(phi - sensorTheta);
			}
		}

		break;
	default:
		cout << "Unknown Sensor Type " << endl;
	}

	if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
		return l0;
	}
	else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
		return locc;
	}
	else if (r <= Zk) {
		return lfree;
	}
}

void GridMapping::updateGridMap_Laser(const double &robotX, const double &robotY, const double &robotTheta, const vector<__scandot> sensorData) {

	int x_start, x_end;
	int y_start, y_end;

	x_start = robotX - Zmax;
	x_end = robotX + Zmax;
	if (x_start < 0) x_start = 0;
	if (x_end > mapWidth / gridWidth) x_end = mapWidth / gridWidth;
	y_start = robotY - Zmax;
	y_end = robotY + Zmax;
	if (y_start < 0) y_start = 0;
	if (y_end > mapHeight / gridHeight) y_end = mapHeight / gridHeight;

	for (int x = x_start; x < x_end; x++) {			// int x = 0; x < mapWidth / gridWidth; x++
		for (int y = y_start; y < y_end; y++) {		// int y = 0; y < mapHeight / gridHeight; y++
			double xi, yi;
			gridToXY(x, y, xi, yi);
			if (sqrt(pow(xi - robotX, 2) + pow(yi - robotY, 2)) <= Zmax) {
				double sensor_inv = l0;

				double r = sqrt(pow(xi - robotX, 2) + pow(yi - robotY, 2));
				double phi = atan2(yi - robotY, xi - robotX) - robotTheta;

				double Zk;
				double thetaK;

				double sensorTheta;
				double minDelta = -1;

				// 不难发现:
				// sensorTheta->i: (sensorTheta * (180.0f / CV_PI) + 90.0f) / 0.5f;
				// 利用这个特性，可以对运算速度做一个很大的优化，只计算一部分的角度
				const double angle_fan = 6.0f;		// 角度的运算区间，即phi附近多大的范围搜索minDelta的极小值
				int start_angle, end_angle;
				start_angle = (phi * (180.0f / CV_PI) + 90.0f) / 0.5f - angle_fan / 2;
				//end_angle = (phi * (180.0f / CV_PI) + 90.0f) / 0.5f + angle_fan / 2;

				// 这四个条件是为了归一化角度到0~361
				while (start_angle < 0)				// 这个是可以等于0的
					start_angle += 360;
				while (start_angle >= 360)
					start_angle -= 360;
				end_angle = start_angle + angle_fan;
				if (end_angle >= 361)
					end_angle = 361;


				for (int i = start_angle; i < end_angle; i++) {			// for (int i = 0; i < 361; i++), 如果改用scandot，这里需要在前一级排序好
					double dot_angle = sensorData[i].angle;
					sensorTheta = (-90 + dot_angle * 0.5) * (CV_PI / 180);
					if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
						Zk = sensorData[i].dst;
						thetaK = sensorTheta;
						minDelta = fabs(phi - sensorTheta);
					}
				}

				if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
					sensor_inv = l0;
				}
				else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
					sensor_inv = locc;
				}
				else if (r <= Zk) {
					sensor_inv = lfree;
				}

				l[x][y] = l[x][y] + sensor_inv - l0;
			}
		}
	}
}

void GridMapping::updateGridMap_Laser(const double &robotX, const double &robotY, const double &robotTheta, const double sensorData[]) {

	for (int x = 0; x < mapWidth / gridWidth; x++) {
		for (int y = 0; y < mapHeight / gridHeight; y++) {
			double xi, yi;
			gridToXY(x, y, xi, yi);
			double dst_robot_dot = sqrt(pow(xi - robotX, 2) + pow(yi - robotY, 2));		// 当前计算的点距离机器人的位置
			if (dst_robot_dot <= Zmax && dst_robot_dot >= Zmin) {
				double sensor_inv = l0;

				double r = sqrt(pow(xi - robotX, 2) + pow(yi - robotY, 2));
				double phi = atan2(yi - robotY, xi - robotX) - robotTheta;

				while (phi < -CV_PI)
					phi += 2 * CV_PI;
				while (phi >= CV_PI)
					phi -= 2 * CV_PI;

				double Zk;
				double thetaK;

				double sensorTheta;
				double minDelta = -1;

				// 不难发现:
				// sensorTheta->i: (sensorTheta * (180.0f / CV_PI) + 90.0f) / 0.5f;
				// 利用这个特性，可以对运算速度做一个很大的优化，只计算一部分的角度
				const double angle_fan = 6.0f;		// 角度的运算区间，即phi附近多大的范围搜索minDelta的极小值
				int start_angle, end_angle;
				start_angle = (phi * (180.0f / CV_PI) + 90.0f) / 0.5f - angle_fan / 2;
				//end_angle = (phi * (180.0f / CV_PI) + 90.0f) / 0.5f + angle_fan / 2;

				// 这四个条件是为了归一化角度到0~361
				while (start_angle < 0)				// 这个是可以等于0的
					start_angle += 360;
				while (start_angle >= 360)
					start_angle -= 360;
				end_angle = start_angle + angle_fan;
				if (end_angle >= 361)
					end_angle = 361;

				for (int i = start_angle; i < end_angle; i++) {			// for (int i = 0; i < 361; i++)
					sensorTheta = (-90 + i * 0.5) * (CV_PI / 180);
					if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
						Zk = sensorData[i];
						thetaK = sensorTheta;
						minDelta = fabs(phi - sensorTheta);
					}
				}

				if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
					sensor_inv = l0;
				}
				else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
					sensor_inv = locc;
				}
				else if (r <= Zk) {
					sensor_inv = lfree;
				}

				l[x][y] = l[x][y] + sensor_inv - l0;
			}
		}
	}
}

void GridMapping::gridToXY(const int &x, const int &y, double &xi, double &yi) {
	xi = x * gridWidth + gridWidth / 2 - robotXOffset;
	yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
}

void GridMapping::showGridMap(const string &windowName) {
	for (int x = 0; x < mapWidth / gridWidth; x++) {
		for (int y = 0; y < mapHeight / gridHeight; y++) {
			double p = 1 - 1 / (1 + exp(l[x][y]));
			double grayValue = (double)((1 - p) * 255);
			CvScalar pixel = cvRealScalar(grayValue);
			cvSet2D(gridMapImage, y, x, pixel);
		}
	}

	cvNamedWindow(windowName.c_str(), 0);
	cvShowImage(windowName.c_str(), gridMapImage);

}

void GridMapping::saveGridMap(const string &fileName) {
	cvSaveImage(fileName.c_str(), gridMapImage);
}

void GridMapping::updateEdge(double threshold) {
	edge.clear();
	for (int x = 0; x < mapWidth / gridWidth; x++) {
		for (int y = 0; y < mapHeight / gridHeight; y++) {
			if (l[x][y] >= locc * threshold) {
				CvPoint3D32f temp;
				temp.x = x;
				temp.y = y;
				temp.z = l[x][y];
				edge.push_back(temp);
			}
		}
	}

	if (!edge.size())
		return;

	// 归一化数据
	// 如果一组数据归一化，那么这组数据可以看成一个线性的方程
	// 必过(min, 0), (max, 1)这两个点
	// 例如: 一组位于[8,15]之间的数，其然在x = 8时，y = 0, x = 15时y = 1
	// 所以y - 0 = (1 - 0)/(max - min) * (x - min)
	double max, min;
	max = -1e6;
	min = 1e6;
	for (int i = 0; i < edge.size(); i++) {
		if (edge[i].z < min)
			min = edge[i].z;
		if (edge[i].z > max)
			max = edge[i].z;
	}
	for (int i = 0; i < edge.size(); i++) {
		if (max == min) {
			// 保护一下
			edge[i].z = 0.5;
		}
		else {
			edge[i].z = 1 / (max - min) * (edge[i].z - min);
		}
	}

}

void GridMapping::getEdge_RealXY(vector<CvPoint2D32f> &out) {
	out.clear();
	for (int i = 0; i < edge.size(); i++) {
		double x,  y;
		double xi, yi;
		x = edge[i].x;
		y = edge[i].y;
		gridToXY(x, y, xi, yi);
		CvPoint2D32f temp;
		temp.x = xi;
		temp.y = -yi;
		out.push_back(temp);
	}
}

void GridMapping::show_Edge(const string &windowName) {

	for (int i = 0; i < edge.size(); i++) {
			int x = edge[i].x;
			int y = edge[i].y;
			CvScalar pixel = cvRealScalar(0);
			cvSet2D(edgeMapImage, y, x, pixel);
	}
	cvNamedWindow(windowName.c_str(), 0);
	cvShowImage(windowName.c_str(), edgeMapImage);
}
