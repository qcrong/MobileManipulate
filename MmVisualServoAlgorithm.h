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

	//ͨ���ĸ�Բ��Բ��������������solvePnP����������굽��������ϵ�ı任��ϵ
	int coPlanarity4PointSolvePnP(const Mat &grayImage, const vector<cv::Point3f> Points3D, Mat &rotM, Mat & tvec);
	//�������굽ͼ�������ת���������ǻ���
	//ʵ��VISP�е� vpFeatureBuilder::create (vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot2 &d)����
	void pixelToImage(vpFeaturePoint &s, const vpCameraParameters &cam, const vpImagePoint &t);
	//VISP��4��������ڲ���vpCameraParametersתOpenCV�е�9��������ڲ���cam_intrinsic_matrix
	void setCamIntrinsic(double camIntrinsic[9]);
	//��������������cam_Distortion_matrix��ֵ
	void setCamDistortion( double camDistortion[5]);
	//������OpenCV��Mat��ʽ��3X3��ת�����3X1ƽ�ƾ�������VISP��α任����
	void setvpHomogeneousMatrix(vpHomogeneousMatrix &outM, const Mat &rM, const Mat &tM);
	//��ʾͼ���������˶��켣
	void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot);
	//ת���ؽڣ����˼���ٶȴ��ݣ���Vi����Vi+1
	//VWiΪi�ؽ�6X1�����ٶȺͽ��ٶ�ʸ����i1RiΪ�ؽ�i+1���ؽ�i����ת����iPi1Ϊ�ؽ�i��i+1��ƽ������
	void linkageVTransmit(const vpColVector &VWi, const Mat &i1Ri, const Mat &iPi1, Mat &outVWi1);
	//��ĩ����צ����ϵ�µ�ĩ���ٶ�ת����������ϵ��
	void eVeTransmitTofVe(const Mat &eVe, const Mat &fRe, Mat &outfVe);

public:
	//������ڲ���
	Mat cam_intrinsic_matrix = Mat(3, 3, CV_64FC1);
	double camIntrinsicParam[9];
	//������������
	Mat cam_distortion_matrix = Mat(5, 1, CV_64FC1);
	double camDistortionParam[5];
	//��е��ĩ���˶��ٶ�
	vpColVector eV;
	//��vpHomography::project��const vpHomography & bHa, const vpPoint & Pa ������һ��
	//��aͼ��ƽ���ϵ�pa(xa,ya,1)ӳ�䵽bͼ��ƽ���ϵ�pb(xb,yb,1),��λ��m
	vpFeaturePoint project(const vpHomography & bHa, const vpFeaturePoint & Pa);


private:
	//������������ͼ���ϵ��˶��켣
	vector<vpImagePoint> traj[4];
};
