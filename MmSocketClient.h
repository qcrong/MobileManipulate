/************************************************************************/
/*�ͻ��������˽���TCP/IPͨ�ŵ��࣬�������ݵ��շ�
�ͻ�����ָԶ�̵�GUI������ʾ���������������ƶ��������ϵĳ��ص���
����Ϊ�ͻ�����*/
/************************************************************************/
#pragma once
#include <opencv.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

class MmSocketClient
{
public:
	MmSocketClient();
	~MmSocketClient();
	////����ͼ��
	////imgΪ���յ���ͼ��cameraFlagΪ������ͱ�־λ��0ΪAXIS�����1ΪBasler���
	//void GetImg(Mat *img, int cameraFlag = 0);
	////����RFID��λ��Ϣ,����X��Y��th
	//void GetRfidPose(double X, double Y, double th);
	////�����ƶ�������λ����Ϣ������X��Y��th
	//void GetMrPose(double X, double Y, double th);
	////���ջ�е��λ����Ϣ
	////������е��ĩ�˵�XYZ���꼰����ת
	//void GetArmPose(double X, double Y, double th, double Xth, double Yth, double Zth);
	////���ƶ��������ϵķ���˷��Ϳ���ָ����Ͳ��յ���Ӧ��������1�����򷵻�0
	////��ָ�����һЩ���ر�־λ�ȣ���Ҫ��һ����дЭ��
	//bool SendContral(int command);
	////��GUI��������ʾͼ��
	//void ShowImg(Mat *img);

	//�����˷�������
	void sendData();
	//�������Է���˵�����
	void receiveData();


private:

};