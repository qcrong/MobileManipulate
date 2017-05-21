//------------------------------------------------------------------------------
//�ƶ�����ƽ̨��������
//�����ڻ����˱�����
/// @file main.cpp
/// @brief Remote commands tester
//2017��
//by ����
//------------------------------------------------------------------------------


//�Խ����ͷ�ļ�
#include "MmArmBase.h"
#include "MmAvoidObstacle.h"
#include "MmMobileRobotBase.h"
#include "MmPose.h"
#include "MmRFIDAlgorithm.h"
#include "MmRFIDBase.h"
#include "MmSocketServer.h"
#include "MmVisualServoAlgorithm.h"
#include "MmVisualServoBase.h"

#include <windows.h>
#include <iostream>

using namespace std;

//ȫ�ֱ���
//VisualNavigationThread�߳����б�־λ�������߳̽���ʱ��־λ��1
volatile int VisualNavigationFlag = 0;  //volatile�����߱��������ȫ�ֱ������ױ�ģ��ñ�������Ҫ��������������Ż�
//ArmMotionThread�߳����б�־λ�������߳̽���ʱ��־λ��1
volatile int ArmMotionFlag = 0;

//ʹ�ùؼ�����ν����̼߳�ͬ��
CRITICAL_SECTION thread_cs;  


/************************************************************************/
/*�̺߳���*/
/************************************************************************/
//RFID�����̺߳���
DWORD WINAPI RFIDNavigationFun(LPVOID lpParameter)
{
	cout << "RFIDNavigationThread is running." << endl;
	Sleep(100);
	cout << "RFIDNavigationThread has finished." << endl;
	return 0;
}

//�Ӿ������̺߳���
DWORD WINAPI VisualNavigationFun(LPVOID lpParameter)
{
	cout << "VisualNavigationThread is running." << endl;
	Sleep(100);
	cout << "VisualNavigationThread has finished." << endl;
	
	//�߳̽�����־λ��1
	EnterCriticalSection(&thread_cs);
	VisualNavigationFlag = 1;
	LeaveCriticalSection(&thread_cs);
	return 0;
}

//��е���˶������̺߳���
DWORD WINAPI ArmMotionFun(LPVOID lpParameter)
{
	cout << "ArmMotionThread is running." << endl;
	Sleep(100);
	cout << "ArmMotionThread has finished." << endl;

	//�߳̽�����־λ��1
	EnterCriticalSection(&thread_cs);
	ArmMotionFlag = 1;
	LeaveCriticalSection(&thread_cs);
	return 0;
}

//�ƶ������˱����߳�
DWORD WINAPI MobileRobotAviodFun(LPVOID lpParameter)
{
	cout << "MobileRobotAviodThread is running." << endl;
	while (VisualNavigationFlag == 0)
	{
		Sleep(50);
	}
	cout << "MobileRobotAviodThread has finished." << endl;
	return 0;
}

//��е�۱����߳�
DWORD WINAPI ArmAvoidFun(LPVOID lpParameter)
{
	cout << "ArmAvoidThread is running." << endl;
	while (ArmMotionFlag == 0)
	{
		Sleep(50);
	}
	cout << "ArmAvoidThread has finished." << endl;
	return 0;
}



/************************************************************************/
/*���������*/
/************************************************************************/
void main
   (
   int argc,
   char **argv
   )
{
	//�����߳�
	HANDLE RFIDNavigationThread = CreateThread(
		NULL,				//�����߳�ʹ��Ĭ�ϰ�ȫ��
		0,					//�����̳߳�ʼջ�Ĵ�С�������̲߳���������߳�һ����ջ��С
		RFIDNavigationFun,			//���߳���ں����ĵ�ַ
		NULL,				//�����̴߳��ݵĲ������������������һ����ֵ��Ҳ������ָ��������Ϣ��ָ��
		CREATE_SUSPENDED,					//�����̴߳����ĸ��ӱ�ǣ�Ϊ0�򴴽������߳�����ִ�У���ΪCREATE_SUSPENDED�����̴߳���������ͣ״̬��ֱ�����������ResumeThread
		NULL				//һ������ֵ��ָ��һ�����������������߳�ID������ΪNULL��ʾ���̵߳�ID������Ȥ�����᷵���̵߳ı�ʶ��
		);
	HANDLE VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, CREATE_SUSPENDED, NULL);
	HANDLE ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, CREATE_SUSPENDED, NULL);
	HANDLE MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
	HANDLE ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);

	InitializeCriticalSection(&thread_cs);		//��ʼ���ؼ������

	//����RFID�����̼߳�С�������߳�
	ResumeThread(RFIDNavigationThread);
	ResumeThread(MobileRobotAviodThread);
	//�ȴ�RFID�����߳��߳��˳�
	WaitForSingleObject(RFIDNavigationThread, INFINITE);
	//RFID��������������Ӿ�λ�˵���
	ResumeThread(VisualNavigationThread);
	WaitForSingleObject(VisualNavigationThread, INFINITE);
	WaitForSingleObject(MobileRobotAviodThread, INFINITE);
	//�Ӿ�λ�˵�������������е���˶��̣߳���������е�۱����߳�
	ResumeThread(ArmMotionThread);
	ResumeThread(ArmAvoidThread);
	WaitForSingleObject(ArmMotionThread, INFINITE);	
	WaitForSingleObject(ArmAvoidThread, INFINITE);

	//�ر��߳̾��
	CloseHandle(RFIDNavigationThread);
	CloseHandle(VisualNavigationThread);
	CloseHandle(ArmMotionThread);
	CloseHandle(MobileRobotAviodThread);
	CloseHandle(ArmAvoidThread);
}
