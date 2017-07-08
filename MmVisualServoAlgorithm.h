/************************************************************************/
/* ���Ӿ��ŷ���ͼ�����㷨��صĺ���
��ײ�Ӳ���޹�*/
/************************************************************************/
#pragma once

#include <opencv.hpp>
#include <iostream>
#include <visp3/gui/vpDisplayOpenCV.h>
//#include <visp3/visual_features/vpFeatureBuilder.h>  //��ͷ�ļ��ᵼ�¶��߳��޷�ִ��
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

	//ͨ���ĸ�Բ��Բ��������������solvePnP����������굽��������ϵ�ı任��ϵ
	int coPlanarity4PointSolvePnP(const Mat &grayImage, const vector<cv::Point3f> Points3D, Mat &rotM, Mat & tvec);
	//�������굽ͼ�������ת���������ǻ���
	//ʵ��VISP�е� vpFeatureBuilder::create (vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot2 &d)����
	void pixelToImage(vpFeaturePoint &s, const vpCameraParameters &cam, const vpImagePoint &t);

public:
	//������ڲ���
	double camIntrinsicParam[9];
	//������������
	double camDistortionParam[5];

private:

};
