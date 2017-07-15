/************************************************************************/
/*客户端与服务端进行TCP/IP通信的类，负责数据的收发
  客户端是指远程的GUI界面显示计算机，服务端是移动机器人上的车载电脑
  本类为服务端类*/
/************************************************************************/
#pragma once
#include "highgui.h"	//使用Opencv中的MAT类
#include <winsock2.h>
#include <windows.h>
//#include <stdio.h>

//using namespace std;
using namespace cv;

//控制信号定义
const int NOSIGNAL = 0;
const int NAVIGATIONSTART = 1;		//自动连续导航
const int RFIDNAVIGATION = 2;		//RFID导航
const int VISUALNAVIGATION = 3;		//视觉调整
const int ARMCONTROL = 4;			//机械臂控制

const int EXITPROGRAM = -1;			//退出程序
const int EXITSOCKET = -2;			//断开远程连接
const int NAVIGATIONSTOP = -3;		//导航停止
const int FINISHARMCONTROL = -4;	//机械臂控制结束


//客户端给服务端发送数据的标志位，为1时发送数据
extern bool sendDatasFlag;


class MmSocketServer
{
public:
	MmSocketServer();
	~MmSocketServer();
	////发送图像,发送并收到回应函数返回1，否则返回0
	////img为要发送的图像，cameraFlag为相机类型标志位，0为AXIS相机，1为Basler相机
	//bool SendImg(Mat *img, int cameraFlag = 0);
	////发送RFID定位信息,包含X、Y、th，发送并收到回应函数返回1，否则返回0
	//bool SendRfidPose(double X, double Y, double th);
	////发送移动机器人位姿信息，包含X、Y、th,发送并收到回应函数返回1，否则返回0
	//bool SendMrPose(double X, double Y, double th);
	////发送机械臂位姿信息,发送并收到回应函数返回1，否则返回0
	////包含机械臂末端的XYZ坐标及其旋转
	//bool SendArmPose(double X, double Y, double th, double Xth, double Yth, double Zth);
	////接收远程客户端发来的移动操作机器人的控制指令
	////该指令包括一些开关标志位等，需要进一步编写协议
	//void GetContral(int command);

	//向客户端发送数据
	void sendData();
	//接收来自客户端的数据
	void receiveData();
	

private:

};

/************************************************************************/
/* 客户端向服务端发送的数据类*/
/************************************************************************/
class MmClientToSrvDatas
{
public:
	MmClientToSrvDatas();
	~MmClientToSrvDatas();
	int contralSignal;		//控制信号

private:

};
extern MmClientToSrvDatas clientToServDatas;

/************************************************************************/
/* 服务端向客户端发送的数据类*/
/************************************************************************/

class MmsrvToClientDatas
{
public:
	MmsrvToClientDatas();
	~MmsrvToClientDatas();
	void init();
	//RFID定位结果
	double rfidX, rfidY, rfidTh;
	//视觉定位结果
	double visualX, visualY, visualTh;
	//里程计定位结果
	double odomX, odomY, odomTh;
	//机械臂末端姿态
	double armX, armY, armZ, armAlpha, armBeta, armGamma;
	
	//是否有文字信息，有为1，无为0
	bool haveWorldsFlag;	
	//存储要发送的文字信息
	char worlds[20];	

	int contralSignal;		//控制信号
	bool threadFlag;	//服务端套接字退出标志位，发送给客户端，为1时客户端接收线程退出
	void play();
private:

};
extern MmsrvToClientDatas srvToClientDatas;

