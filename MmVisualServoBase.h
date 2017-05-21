/************************************************************************/
/*视觉伺服控制的基础类
  是与相机硬件相关的底层封装*/
/************************************************************************/
#pragma once
#include <visp3/io/vpImageIo.h>
#include <opencv.hpp>
#include <pylon/PylonIncludes.h>
#include <stdio.h>

using namespace Pylon;
using namespace std;
using namespace cv;

class MmVisualServoBase
{
public:
	MmVisualServoBase();
	~MmVisualServoBase();
	
	//Basler相机初始化，对现有的acA640-300gc和acA1300-60gc都适用
	//灰度图像
	void baslerOpen(vpImage< unsigned char > &I);
	//Basler相机初始化，对现有的acA640-300gc和acA1300-60gc都适用
	//RGB图像
	void baslerOpen(vpImage< vpRGBa > &I);
	//关闭相机
	void close(void);
	//获取Basler相机的灰度图像
	void acquireBaslerImg(vpImage< unsigned char> &I);
	//获取Basler相机的RGB图像,图像格式vpImage< vpRGBa>
	void acquireBaslerImg(vpImage< vpRGBa> &I);
	//获取Basler相机的RGB图像,图像格式Mat
	void acquireBaslerImg(vpImage<unsigned char> &I, Mat &opencvImage);
	//获取图片高度
	unsigned int getHeight(void) const;
	//获取图片宽度
	unsigned int getWidth(void) const;


	//Axis云台相机初始化，返回1表示初始化成功，否则返回0
	bool axisInit();
	//获取Axis云台相机的图像
	void acquireAxisImg(Mat *img);
	
	

private:
	/**********Basler相机参数**********/
	PylonAutoInitTerm autoInitTerm;  // PylonInitialize() 需要在打开摄像头之前调用，该类直接调用该函数
	CInstantCamera camera;
	CImageFormatConverter formatConverter;
	GenApi::CIntegerPtr width;
	GenApi::CIntegerPtr height;
	CPylonImage pylonImage;


	CvCapture *AxisCamera;


};

