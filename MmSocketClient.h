/************************************************************************/
/*客户端与服务端进行TCP/IP通信的类，负责数据的收发
客户端是指远程的GUI界面显示计算机，服务端是移动机器人上的车载电脑
本类为客户端类*/
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
	////接收图像
	////img为接收到的图像，cameraFlag为相机类型标志位，0为AXIS相机，1为Basler相机
	//void GetImg(Mat *img, int cameraFlag = 0);
	////接收RFID定位信息,包含X、Y、th
	//void GetRfidPose(double X, double Y, double th);
	////接收移动机器人位姿信息，包含X、Y、th
	//void GetMrPose(double X, double Y, double th);
	////接收机械臂位姿信息
	////包含机械臂末端的XYZ坐标及其旋转
	//void GetArmPose(double X, double Y, double th, double Xth, double Yth, double Zth);
	////给移动机器人上的服务端发送控制指令，发送并收到回应函数返回1，否则返回0
	////该指令包括一些开关标志位等，需要进一步编写协议
	//bool SendContral(int command);
	////在GUI界面上显示图像
	//void ShowImg(Mat *img);

	//向服务端发送数据
	void sendData();
	//接收来自服务端的数据
	void receiveData();


private:

};