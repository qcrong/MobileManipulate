/************************************************************************/
/* 机械臂控制类，目前针Robai gramma1500，基于该机械臂的类进行开发，
   与机械臂的硬件相关*/
/************************************************************************/

#pragma once
#include <opencv.hpp>   //????为什么要包含这个才能使用vector
#include <stdio.h>

using namespace std;
//using namespace cv;

class MmArmBase
{
public:
	MmArmBase();
	~MmArmBase();
	//机械臂末端运动到指定位姿,到达指定位姿返回1，否则返回0
	//goal中六个参数，分别对应XYZ及其旋转角
	bool setGoal(vector<double>* goal);
	//机械臂末端速度控制
	//vw中六个参数，分别对应XYZ轴平移线速度和旋转角速度
	void setVelRot(vector<double>* vw);
	//机械臂各关节角度控制
	//w中依次对应0号关节开始的各关节角速度，默认关节角速度为0
	void setJointVel(vector<double>* w);
	//机械臂末端位姿读取
	//pose中六个参数依次对应机械臂末端XYZ及其旋转角
	void getPose(vector<double>* pose);


private:

};