//------------------------------------------------------------------------------
//移动操作平台服务端软件
//运行在机器人本体上
/// @file main.cpp
/// @brief Remote commands tester
//2017年
//by 丘椿荣
//------------------------------------------------------------------------------


//自建类的头文件
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

//全局变量
//VisualNavigationThread线程运行标志位，当该线程结束时标志位置1
volatile int VisualNavigationFlag = 0;  //volatile来告诉编译器这个全局变量是易变的，让编译器不要对这个变量进行优化
//ArmMotionThread线程运行标志位，当该线程结束时标志位置1
volatile int ArmMotionFlag = 0;

//使用关键代码段进行线程间同步
CRITICAL_SECTION thread_cs;  


/************************************************************************/
/*线程函数*/
/************************************************************************/
//RFID导航线程函数
DWORD WINAPI RFIDNavigationFun(LPVOID lpParameter)
{
	cout << "RFIDNavigationThread is running." << endl;
	Sleep(100);
	cout << "RFIDNavigationThread has finished." << endl;
	return 0;
}

//视觉导航线程函数
DWORD WINAPI VisualNavigationFun(LPVOID lpParameter)
{
	cout << "VisualNavigationThread is running." << endl;
	Sleep(100);
	cout << "VisualNavigationThread has finished." << endl;
	
	//线程结束标志位置1
	EnterCriticalSection(&thread_cs);
	VisualNavigationFlag = 1;
	LeaveCriticalSection(&thread_cs);
	return 0;
}

//机械臂运动控制线程函数
DWORD WINAPI ArmMotionFun(LPVOID lpParameter)
{
	cout << "ArmMotionThread is running." << endl;
	Sleep(100);
	cout << "ArmMotionThread has finished." << endl;

	//线程结束标志位置1
	EnterCriticalSection(&thread_cs);
	ArmMotionFlag = 1;
	LeaveCriticalSection(&thread_cs);
	return 0;
}

//移动机器人避障线程
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

//机械臂避障线程
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
/*主程序入口*/
/************************************************************************/
void main
   (
   int argc,
   char **argv
   )
{
	//创建线程
	HANDLE RFIDNavigationThread = CreateThread(
		NULL,				//让新线程使用默认安全性
		0,					//设置线程初始栈的大小，让新线程采用与调用线程一样的栈大小
		RFIDNavigationFun,			//新线程入口函数的地址
		NULL,				//向新线程传递的参数，这个参数可以是一个数值，也可以是指向其他信息的指针
		CREATE_SUSPENDED,					//控制线程创建的附加标记，为0则创建后立线程立即执行，若为CREATE_SUSPENDED，则线程创建后处于暂停状态，直到程序调用了ResumeThread
		NULL				//一个返回值，指向一个变量，用来接收线程ID，设置为NULL表示对线程的ID不感兴趣，不会返回线程的标识符
		);
	HANDLE VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, CREATE_SUSPENDED, NULL);
	HANDLE ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, CREATE_SUSPENDED, NULL);
	HANDLE MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
	HANDLE ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);

	InitializeCriticalSection(&thread_cs);		//初始化关键代码段

	//启动RFID导航线程及小车避障线程
	ResumeThread(RFIDNavigationThread);
	ResumeThread(MobileRobotAviodThread);
	//等待RFID导航线程线程退出
	WaitForSingleObject(RFIDNavigationThread, INFINITE);
	//RFID导航结束后进入视觉位姿调整
	ResumeThread(VisualNavigationThread);
	WaitForSingleObject(VisualNavigationThread, INFINITE);
	WaitForSingleObject(MobileRobotAviodThread, INFINITE);
	//视觉位姿调整结束后进入机械臂运动线程，并启动机械臂避障线程
	ResumeThread(ArmMotionThread);
	ResumeThread(ArmAvoidThread);
	WaitForSingleObject(ArmMotionThread, INFINITE);	
	WaitForSingleObject(ArmAvoidThread, INFINITE);

	//关闭线程句柄
	CloseHandle(RFIDNavigationThread);
	CloseHandle(VisualNavigationThread);
	CloseHandle(ArmMotionThread);
	CloseHandle(MobileRobotAviodThread);
	CloseHandle(ArmAvoidThread);
}
