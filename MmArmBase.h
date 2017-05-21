/************************************************************************/
/* ��е�ۿ����࣬Ŀǰ��Robai gramma1500�����ڸû�е�۵�����п�����
   ���е�۵�Ӳ�����*/
/************************************************************************/

#pragma once
#include <opencv.hpp>   //????ΪʲôҪ�����������ʹ��vector
#include <stdio.h>

using namespace std;
//using namespace cv;

class MmArmBase
{
public:
	MmArmBase();
	~MmArmBase();
	//��е��ĩ���˶���ָ��λ��,����ָ��λ�˷���1�����򷵻�0
	//goal�������������ֱ��ӦXYZ������ת��
	bool setGoal(vector<double>* goal);
	//��е��ĩ���ٶȿ���
	//vw�������������ֱ��ӦXYZ��ƽ�����ٶȺ���ת���ٶ�
	void setVelRot(vector<double>* vw);
	//��е�۸��ؽڽǶȿ���
	//w�����ζ�Ӧ0�Źؽڿ�ʼ�ĸ��ؽڽ��ٶȣ�Ĭ�Ϲؽڽ��ٶ�Ϊ0
	void setJointVel(vector<double>* w);
	//��е��ĩ��λ�˶�ȡ
	//pose�������������ζ�Ӧ��е��ĩ��XYZ������ת��
	void getPose(vector<double>* pose);


private:

};