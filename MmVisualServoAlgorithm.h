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
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/klt/vpKltOpencv.h>


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
	//VISP中4参数相机内参数vpCameraParameters转OpenCV中的9参数相机内参数cam_intrinsic_matrix
	void setCamIntrinsic(double camIntrinsic[9]);
	//设置相机畸变参数cam_Distortion_matrix的值
	void setCamDistortion( double camDistortion[5]);
	//设置由OpenCV中Mat格式的3X3旋转矩阵和3X1平移矩阵设置VISP齐次变换矩阵
	void setvpHomogeneousMatrix(vpHomogeneousMatrix &outM, const Mat &rM, const Mat &tM);
	//显示图像特征点运动轨迹
	void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot);
	//转动关节，连杆间的速度传递，由Vi计算Vi+1
	//VWi为i关节6X1的线速度和角速度矢量，i1Ri为关节i+1到关节i的旋转矩阵，iPi1为关节i到i+1的平移向量
	void linkageVTransmit(const vpColVector &VWi, const Mat &i1Ri, const Mat &iPi1, Mat &outVWi1);
	//将末端手爪坐标系下的末端速度转换到基坐标系下
	void eVeTransmitTofVe(const Mat &eVe, const Mat &fRe, Mat &outfVe);

public:
	//摄像机内参数
	Mat cam_intrinsic_matrix = Mat(3, 3, CV_64FC1);
	double camIntrinsicParam[9];
	//摄像机畸变参数
	Mat cam_distortion_matrix = Mat(5, 1, CV_64FC1);
	double camDistortionParam[5];
	//机械臂末端运动速度
	vpColVector eV;
	//与vpHomography::project（const vpHomography & bHa, const vpPoint & Pa ）作用一致
	//将a图像平面上的pa(xa,ya,1)映射到b图像平面上的pb(xb,yb,1),单位是m
	vpFeaturePoint project(const vpHomography & bHa, const vpFeaturePoint & Pa);


private:
	//绘制特征点在图像上的运动轨迹
	vector<vpImagePoint> traj[4];
};
