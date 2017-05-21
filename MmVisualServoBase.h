/************************************************************************/
/*�Ӿ��ŷ����ƵĻ�����
  �������Ӳ����صĵײ��װ*/
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
	
	//Basler�����ʼ���������е�acA640-300gc��acA1300-60gc������
	//�Ҷ�ͼ��
	void baslerOpen(vpImage< unsigned char > &I);
	//Basler�����ʼ���������е�acA640-300gc��acA1300-60gc������
	//RGBͼ��
	void baslerOpen(vpImage< vpRGBa > &I);
	//�ر����
	void close(void);
	//��ȡBasler����ĻҶ�ͼ��
	void acquireBaslerImg(vpImage< unsigned char> &I);
	//��ȡBasler�����RGBͼ��,ͼ���ʽvpImage< vpRGBa>
	void acquireBaslerImg(vpImage< vpRGBa> &I);
	//��ȡBasler�����RGBͼ��,ͼ���ʽMat
	void acquireBaslerImg(vpImage<unsigned char> &I, Mat &opencvImage);
	//��ȡͼƬ�߶�
	unsigned int getHeight(void) const;
	//��ȡͼƬ���
	unsigned int getWidth(void) const;


	//Axis��̨�����ʼ��������1��ʾ��ʼ���ɹ������򷵻�0
	bool axisInit();
	//��ȡAxis��̨�����ͼ��
	void acquireAxisImg(Mat *img);
	
	

private:
	/**********Basler�������**********/
	PylonAutoInitTerm autoInitTerm;  // PylonInitialize() ��Ҫ�ڴ�����ͷ֮ǰ���ã�����ֱ�ӵ��øú���
	CInstantCamera camera;
	CImageFormatConverter formatConverter;
	GenApi::CIntegerPtr width;
	GenApi::CIntegerPtr height;
	CPylonImage pylonImage;


	CvCapture *AxisCamera;


};

