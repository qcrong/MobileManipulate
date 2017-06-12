/************************************************************************/
/* 与视觉伺服和图像处理算法相关的函数
与底层硬件无关*/
/************************************************************************/
#pragma once

#include <opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class MmVisualServoAlgorithm
{
public:
	MmVisualServoAlgorithm();
	~MmVisualServoAlgorithm();

	//通过四个圆的圆心世界坐标利用solvePnP计算相机坐标到世界坐标系的变换关系
	int coPlanarity4PointSolvePnP(const Mat &grayImage, const vector<cv::Point3f> Points3D, Mat &rotM, Mat & tvec);

public:
	//摄像机内参数
	double camIntrinsicParam[9];
	//摄像机畸变参数
	double camDistortionParam[5];

private:

};
