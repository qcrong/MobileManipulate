/************************************************************************/
/* ���Ӿ��ŷ���ͼ�����㷨��صĺ���
��ײ�Ӳ���޹�*/
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

	//ͨ���ĸ�Բ��Բ��������������solvePnP����������굽��������ϵ�ı任��ϵ
	int coPlanarity4PointSolvePnP(const Mat &grayImage, const vector<cv::Point3f> Points3D, Mat &rotM, Mat & tvec);

public:
	//������ڲ���
	double camIntrinsicParam[9];
	//������������
	double camDistortionParam[5];

private:

};
