/************************************************************************/
/* �ƶ��������˶������࣬Ŀǰ����ȷ������P3-DX
   ������˵�Ӳ�����*/
/************************************************************************/

#pragma once
#include "Aria.h"

class MmMobileRobotBase
{
public:
	MmMobileRobotBase();
	~MmMobileRobotBase();
	//�˶���һ��ָ����λ��,�ɹ����ﷵ��1�����򷵻�0
	//goal����x,y,th
	bool setGoal(ArPose* goal);
	//�ֱ�����ǰ������ת�ٶ����ƶ��������˶�
	//v:ǰ���ٶ�m/s��w����ת���ٶȡ�/s
	void setVelRot(double v, double w);
	//��ȡ�ƶ������˵�ǰλ��
	void getPose(ArPose* pose);

private:

};
