/************************************************************************/
/* 与视觉伺服和图像处理算法相关的函数
与底层硬件无关*/
/************************************************************************/
#pragma once

#include <opencv.hpp>
#include <iostream>
#include <visp3/gui/vpDisplayOpenCV.h>
//#include <visp3/visual_features/vpFeatureBuilder.h>  //该头文件会导致多线程无法执行
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#ifdef VISP_HAVE_MODULE_BLOB
#include <visp3/blob/vpDot2.h>
#endif

#include <visp3/io/vpImageIo.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

using namespace std;
using namespace cv;

class MmVisualServoAlgorithm
{
public:
	MmVisualServoAlgorithm();
	~MmVisualServoAlgorithm();

	//通过四个圆的圆心世界坐标利用solvePnP计算相机坐标到世界坐标系的变换关系
	int coPlanarity4PointSolvePnP(const Mat &grayImage, const vector<cv::Point3f> Points3D, Mat &rotM, Mat & tvec);
	//像素坐标到图像坐标的转换，不考虑畸变
	//实现VISP中的 vpFeatureBuilder::create (vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot2 &d)功能
	void pixelToImage(vpFeaturePoint &s, const vpCameraParameters &cam, const vpImagePoint &t);

public:
	//摄像机内参数
	double camIntrinsicParam[9];
	//摄像机畸变参数
	double camDistortionParam[5];

private:

};
