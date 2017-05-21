/************************************************************************/
/* 移动机器人运动控制类，目前针对先锋机器人P3-DX
   与机器人的硬件相关*/
/************************************************************************/

#pragma once
#include "Aria.h"

class MmMobileRobotBase
{
public:
	MmMobileRobotBase();
	~MmMobileRobotBase();
	//运动到一个指定的位姿,成功到达返回1，否则返回0
	//goal包含x,y,th
	bool setGoal(ArPose* goal);
	//分别设置前进和旋转速度让移动机器人运动
	//v:前进速度m/s；w：旋转角速度°/s
	void setVelRot(double v, double w);
	//读取移动机器人当前位姿
	void getPose(ArPose* pose);

private:

};
