/************************************************************************/
/*�ͻ��������˽���TCP/IPͨ�ŵ��࣬�������ݵ��շ�
  �ͻ�����ָԶ�̵�GUI������ʾ���������������ƶ��������ϵĳ��ص���
  ����Ϊ�������*/
/************************************************************************/
#pragma once
#include <opencv.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;


class MmSocketServer
{
public:
	MmSocketServer();
	~MmSocketServer();
	////����ͼ��,���Ͳ��յ���Ӧ��������1�����򷵻�0
	////imgΪҪ���͵�ͼ��cameraFlagΪ������ͱ�־λ��0ΪAXIS�����1ΪBasler���
	//bool SendImg(Mat *img, int cameraFlag = 0);
	////����RFID��λ��Ϣ,����X��Y��th�����Ͳ��յ���Ӧ��������1�����򷵻�0
	//bool SendRfidPose(double X, double Y, double th);
	////�����ƶ�������λ����Ϣ������X��Y��th,���Ͳ��յ���Ӧ��������1�����򷵻�0
	//bool SendMrPose(double X, double Y, double th);
	////���ͻ�е��λ����Ϣ,���Ͳ��յ���Ӧ��������1�����򷵻�0
	////������е��ĩ�˵�XYZ���꼰����ת
	//bool SendArmPose(double X, double Y, double th, double Xth, double Yth, double Zth);
	////����Զ�̿ͻ��˷������ƶ����������˵Ŀ���ָ��
	////��ָ�����һЩ���ر�־λ�ȣ���Ҫ��һ����дЭ��
	//void GetContral(int command);

	//��ͻ��˷�������
	void sendData();
	//�������Կͻ��˵�����
	void receiveData();


private:

};