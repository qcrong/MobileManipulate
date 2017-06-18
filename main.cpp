//------------------------------------------------------------------------------
//移动操作平台服务端软件
//运行在机器人本体上
/// @file main.cpp
/// @brief Remote commands tester
//2017年
//by 丘椿荣
//------------------------------------------------------------------------------


//自建类的头文件
//#include <WinSock2.h>
#include "MmArmBase.h"
#include "MmAvoidObstacle.h"
#include "MmMobileRobotBase.h"
#include "MmData.h"
#include "MmRFIDAlgorithm.h"
#include "MmRFIDBase.h"
#include "MmSocketSrv.h"
#include "MmVisualServoAlgorithm.h"
#include "MmVisualServoBase.h"


#include <windows.h>
#include <iostream>
#include <stdio.h>

//using namespace std;

//全局变量

//VisualNavigationThread线程运行标志位，当该线程结束时标志位置1
volatile int VisualNavigationFlag = 0;  //volatile来告诉编译器这个全局变量是易变的，让编译器不要对这个变量进行优化
//ArmMotionThread线程运行标志位，当该线程结束时标志位置1
volatile int ArmMotionFlag = 0;
//使用关键代码段进行线程间同步
CRITICAL_SECTION thread_cs;  


//创建线程句柄
HANDLE RFIDNavigationThread;		//RFID大范围导航线程
HANDLE VisualNavigationThread;		//视觉局部调整线程
HANDLE ArmMotionThread;				//机械臂抓取线程
HANDLE MobileRobotAviodThread;		//移动机器人避障线程
HANDLE ArmAvoidThread;				//机械臂避障线程
HANDLE sendDatasThread;				//套接字数据发送线程
//HANDLE recvDatasRhread;				//套接字数据接收线程

//TCP全局变量
bool tcpThreadFlag = 0;		//线程函数终止标志位，为1时线程退出


/************************************************************************/
/*线程函数*/
/************************************************************************/
//RFID导航线程函数
DWORD WINAPI RFIDNavigationFun(LPVOID lpParameter)
{
	ResumeThread(MobileRobotAviodThread);
	cout << "RFIDNavigationThread is running." << endl;
	Sleep(100);
	cout << "RFIDNavigationThread has finished." << endl;

	ResumeThread(VisualNavigationThread);
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

	//启动机械臂运动控制和避障线程
	ResumeThread(ArmMotionThread);
	ResumeThread(ArmAvoidThread);
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

//套接字数据发送线程
DWORD WINAPI SendDatas(LPVOID lpParameter)
{
	const SOCKET sockConn = (SOCKET)lpParameter;
	Datas data;
	strcpy_s(data.name, "Server");
	data.score = 92.5;
	data.number = 0;
	data.threadFlag = 0;
	char sendBuf[100];
	while (tcpThreadFlag != 1)
	{
		memset(sendBuf, 0, sizeof(sendBuf));		//对该段内存清零
		memcpy(sendBuf, &data, sizeof(Datas));

		send(sockConn, sendBuf, sizeof(Datas), 0);
		Sleep(1000);
		data.number++;
	}
	data.threadFlag = 1;
	memset(sendBuf, 0, sizeof(sendBuf));		//对该段内存清零
	memcpy(sendBuf, &data, sizeof(Datas));
	send(sockConn, sendBuf, sizeof(Datas), 0);

	return 0;
}

//套接字数据接收线程
//DWORD WINAPI RecvDatas(LPVOID lpParameter)
//{
//	const SOCKET sockConn = (SOCKET)lpParameter;
//	int signalFlag = 0;
//
//	while (true)
//	{
//		char recvBuf[100];
//		memset(recvBuf, 0, sizeof(recvBuf));
//
//		//接收数据
//		recv(sockConn, recvBuf, 100, 0);		//第三参数为缓冲区的长度
//
//		memcpy(&signalFlag, recvBuf, sizeof(signalFlag));
//
//		//输出接收到的数据
//		printf("%d\n", signalFlag);
//
//		if (signalFlag == 30)
//		{
//			tcpThreadFlag = 1;		//线程标志位置1，退出线程
//			break;
//		}
//	}
//
//	return 0;
//}



/************************************************************************/
/*主程序入口*/
/************************************************************************/
void main
   (
   int argc,
   char **argv
   )
{
	/************************************************************************/
	/*创建套接字*/
	/************************************************************************/
	//加载套接字库
	WORD wVersionRequest;		//用来指定准备加载的Winsock库的版本
	WSADATA wsaData;
	int err;

	wVersionRequest = MAKEWORD(1, 1);		//指定版本号(x,y),其中x是高位字节，y是低位字节
	err = WSAStartup(wVersionRequest, &wsaData);
	if (err != 0)
	{
		return;
	}
	//验证版本号
	if (LOBYTE(wsaData.wVersion) != 1 ||
		HIBYTE(wsaData.wVersion) != 1)
	{
		WSACleanup();
		return;
	}

	//创建用于监听的套接字
	SOCKET sockSrv = socket(AF_INET,		//Windows Sockets只支持一个通信域：网际域（AF_INET）
		SOCK_STREAM,		//基于TCP协议的网络程序，需要创建流式套接字
		0);			//根据格式地址和套接字类别自动选择一个合适的协议

	SOCKADDR_IN addrSrv;
	//给该结构体成员变量进行赋值，htonl将u_long类型的值从主机字节顺序转换为TCP/IP网络字节顺序
	addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);// inet_addr("192.168.0.115");		//htonl(INADDR_ANY)允许套接字向任何分配给本地机器的IP地址发送或接收数据
	addrSrv.sin_family = AF_INET;		//这里只能指定AF_INET
	//htonl将u_long类型的值从主机字节顺序转换为TCP / IP网络字节顺序
	addrSrv.sin_port = htons(6001);		//指定端口号，要求大于1024小于65535

	//绑定套接字,把套接字sockSrv绑定到本地地址和指定的端口上
	//在这个程序里要加上::指定全局作用域，否则会与Aria.h冲突，导致bind失败
	::bind(sockSrv,			//需要绑定的套接字
		(SOCKADDR*)&addrSrv,	//强制类型装换，基于TCP/IP的socket编程过程中，可以用SOCKADDR_IN结构替换SOCKADDR，方便填写信息
		sizeof(SOCKADDR)		//指定地址结构的长度
		);

	//设定已绑定的套接字为监听模式，准备接收客户请求
	listen(sockSrv, 20);		//第二个参数为等待队列的最大长度
	SOCKADDR_IN addrClient;		//定义一个地址结构体SOCKADDR_IN变量，用来接收客户端的地址信息
	printf("Waiting for client connect.\n");
	//等待并接收客户端的连接请求
	//accept返回一个相当于当前这个新连接的一个套接字描述符，保存于sockConn，然后利用这个套接字与客户端通信
	int len = sizeof(SOCKADDR);
	SOCKET sockConn = accept(sockSrv, (SOCKADDR*)&addrClient, &len);
	printf("Accept client connect.\n");

	
	
	/************************************************************************/
	/*创建线程*/
	/************************************************************************/
	RFIDNavigationThread = CreateThread(
		NULL,				//让新线程使用默认安全性
		0,					//设置线程初始栈的大小，让新线程采用与调用线程一样的栈大小
		RFIDNavigationFun,			//新线程入口函数的地址
		NULL,				//向新线程传递的参数，这个参数可以是一个数值，也可以是指向其他信息的指针
		CREATE_SUSPENDED,					//控制线程创建的附加标记，为0则创建后立线程立即执行，若为CREATE_SUSPENDED，则线程创建后处于暂停状态，直到程序调用了ResumeThread
		NULL				//一个返回值，指向一个变量，用来接收线程ID，设置为NULL表示对线程的ID不感兴趣，不会返回线程的标识符
		);
	VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, CREATE_SUSPENDED, NULL);
	ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, CREATE_SUSPENDED, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
	ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);
	sendDatasThread = CreateThread(NULL, 0, SendDatas, (LPVOID)sockConn, 0, NULL);
	
	InitializeCriticalSection(&thread_cs);		//初始化关键代码段


	/************************************************************************/
	/*接收客户端指令并*/
	/************************************************************************/
	MmClientToSrvDatas clientToServDatas;

	while (true)
	{
		char recvBuf[100];
		memset(recvBuf, 0, sizeof(recvBuf));

		//接收数据
		recv(sockConn, recvBuf, 100, 0);		//第三参数为缓冲区的长度

		memcpy(&clientToServDatas, recvBuf, sizeof(MmClientToSrvDatas));

		switch (clientToServDatas.contralSignal)
		{
		case NAVIGATIONSTART:ResumeThread(RFIDNavigationThread); break;


			
		default:
			break;
		}

		//输出接收到的数据
//		printf("%d\n", clientToServDatas.contralSignal);

		if (clientToServDatas.contralSignal == EXITPROGRAM)
		{
			tcpThreadFlag = 1;		//线程标志位置1，退出线程
			break;
		}
	}

	
//	recvDatasRhread = CreateThread(NULL, 0, RecvDatas, (LPVOID)sockConn, 0, NULL);


	//启动RFID导航线程及小车避障线程
	//ResumeThread(RFIDNavigationThread);
	//ResumeThread(MobileRobotAviodThread);
	////等待RFID导航线程线程退出
	//WaitForSingleObject(RFIDNavigationThread, INFINITE);
	////RFID导航结束后进入视觉位姿调整
	//ResumeThread(VisualNavigationThread);
	//WaitForSingleObject(VisualNavigationThread, INFINITE);
	//WaitForSingleObject(MobileRobotAviodThread, INFINITE);
	////视觉位姿调整结束后进入机械臂运动线程，并启动机械臂避障线程
	//ResumeThread(ArmMotionThread);
	//ResumeThread(ArmAvoidThread);
	//WaitForSingleObject(ArmMotionThread, INFINITE);	
	//WaitForSingleObject(ArmAvoidThread, INFINITE);
	////等待SOCKET线程退出
	//WaitForSingleObject(sendDatasThread, INFINITE);
//	WaitForSingleObject(recvDatasRhread, INFINITE);

	//关闭线程句柄
	CloseHandle(RFIDNavigationThread);
	CloseHandle(VisualNavigationThread);
	CloseHandle(ArmMotionThread);
	CloseHandle(MobileRobotAviodThread);
	CloseHandle(ArmAvoidThread);
	CloseHandle(sendDatasThread);

	cout << "exit program" << endl;
}
