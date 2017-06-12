/************************************************************************/
/* 模仿ArPose建立的表示移动机器人位姿的类*/
/************************************************************************/

#pragma once
#include <iostream>

using namespace std;

class MmVehiclePose
{
public:
	MmVehiclePose(double x = 0, double y = 0, double th = 0);
	MmVehiclePose(const MmVehiclePose &mypose);
	void getPose(double *x, double *y, double *th = NULL) const;
	double getTh(void) const;
	double getX(void) const;
	double getY(void) const;
	void setPose(double x = 0, double y = 0, double th = 0);
	void setPose(MmVehiclePose mypose);
	void setTh(double th);
	void setX(double x);
	void setY(double y);

	~MmVehiclePose();

private:
	double myTh;
	double myX;
	double myY;
};

