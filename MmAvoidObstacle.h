/************************************************************************/
/*移动机器人和机械臂的避障算法，与底层硬件无关*/
/************************************************************************/

#pragma once

class MmAvoidObstacle
{
public:
	MmAvoidObstacle();
	~MmAvoidObstacle();
	//移动机器人避障
	void mobileRobotAvoidObstacle();
	//机械臂避障
	void armAvoidObstacle();

private:

};