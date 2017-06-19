/************************************************************************/
/*�ͻ��������˽���TCP/IPͨ�ŵ��࣬�������ݵ��շ�
  �ͻ�����ָԶ�̵�GUI������ʾ���������������ƶ��������ϵĳ��ص���
  ����Ϊ�������*/
/************************************************************************/
#pragma once
#include "highgui.h"	//ʹ��Opencv�е�MAT��
#include <winsock2.h>
#include <windows.h>
//#include <stdio.h>

//using namespace std;
using namespace cv;

//�����źŶ���
const int NOSIGNAL = 0;
const int NAVIGATIONSTART = 1;
const int EXITPROGRAM = -1;
const int EXITSOCKET = -2;
const int NAVIGATIONSTOP = -3;


//�ͻ��˸�����˷������ݵı�־λ��Ϊ1ʱ��������
extern bool sendDatasFlag;


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

/************************************************************************/
/* �ͻ��������˷��͵�������*/
/************************************************************************/
class MmClientToSrvDatas
{
public:
	MmClientToSrvDatas();
	~MmClientToSrvDatas();
	int contralSignal;		//�����ź�

private:

};
extern MmClientToSrvDatas clientToServDatas;

/************************************************************************/
/* �ͻ��������˷��͵�������*/
/************************************************************************/

class Datas
{
public:
	Datas();
	~Datas();
	char name[10];
	double score;
	int number;
	bool threadFlag;
	void play();
private:

};

/************************************************************************/
/* �ͻ��������˷��͵�������*/
/************************************************************************/
//DWORD WINAPI SendDatas(LPVOID lpParameter);
//
//DWORD WINAPI RecvDatas(LPVOID lpParameter);