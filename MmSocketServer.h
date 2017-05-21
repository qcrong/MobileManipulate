/************************************************************************/
/*客户端与服务端进行TCP/IP通信的类，负责数据的收发
  客户端是指远程的GUI界面显示计算机，服务端是移动机器人上的车载电脑
  本类为服务端类*/
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