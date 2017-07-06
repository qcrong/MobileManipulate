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
//volatile来告诉编译器这个全局变量是易变的，让编译器不要对这个变量进行优化
//VisualNavigationThread线程运行标志位，当该线程结束时标志位置1
volatile int VisualNavigationFlag = 0;  
//ArmMotionThread线程运行标志位，当该线程结束时标志位置1
volatile int ArmMotionFlag = 0;
//客户端按下停止按钮时，ThreadsExitFlag置1，机器人线程退出
volatile int ThreadsExitFlag = 0;
//使用关键代码段进行线程间同步
CRITICAL_SECTION thread_cs;  
//自动导航标志位，该标志位为NAVIGATIONSTART时，先后自动执行RFID导航，视觉位姿调整和机械臂抓取
int navigationAutoflag = 0;		

//创建线程句柄
HANDLE RFIDNavigationThread;		//RFID大范围导航线程
HANDLE VisualNavigationThread;		//视觉局部调整线程
HANDLE ArmMotionThread;				//机械臂抓取线程
HANDLE MobileRobotAviodThread;		//移动机器人避障线程
HANDLE ArmAvoidThread;				//机械臂避障线程
HANDLE SendDatasThread;				//套接字数据发送线程
HANDLE CamThread;				//套接字数据接收线程

//TCP全局变量
int tcpThreadFlag = 0;		//线程函数终止标志位，为1时线程退出



/************************************************************************/
/*线程函数*/
/************************************************************************/
//RFID导航线程函数
DWORD WINAPI RFIDNavigationFun(LPVOID lpParameter)
{
	ResumeThread(MobileRobotAviodThread);
	while (ThreadsExitFlag != NAVIGATIONSTOP)
	{
		cout << "1. RFIDNavigationThread is running." << endl;
		Sleep(100);
		cout << "3. RFIDNavigationThread has finished." << endl;
		break;
	}
	
	//连续自动导航，启动下一环节
	if (navigationAutoflag == NAVIGATIONSTART)
	{
		ResumeThread(VisualNavigationThread);
	}
	else
	{
		//避障线程结束标志位置1，避障线程退出
		EnterCriticalSection(&thread_cs);
		VisualNavigationFlag = 1;
		LeaveCriticalSection(&thread_cs);
	}
	
	return 0;
}

//视觉导航线程函数
DWORD WINAPI VisualNavigationFun(LPVOID lpParameter)
{
	//非自动导航情况下，避障线程需要单独开启
	if (navigationAutoflag != NAVIGATIONSTART)
	{
		ResumeThread(MobileRobotAviodThread);
	}

	while (ThreadsExitFlag != NAVIGATIONSTOP)
	{
		cout << "4. VisualNavigationThread is running." << endl;
		Sleep(100);
		cout << "5. VisualNavigationThread has finished." << endl;
		break;
	}
	
	//避障线程结束标志位置1，避障线程退出
	EnterCriticalSection(&thread_cs);
	VisualNavigationFlag = 1;
	LeaveCriticalSection(&thread_cs);

	return 0;
}

//机械臂运动控制线程函数
DWORD WINAPI ArmMotionFun(LPVOID lpParameter)
{
	ResumeThread(ArmAvoidThread);
	while (ThreadsExitFlag != NAVIGATIONSTOP)
	{
		cout << "7. ArmMotionThread is running." << endl;
//		Sleep(100);
		

		
		cout << "9. ArmMotionThread has finished." << endl;
		break;
	}
	
	//线程结束标志位置1
	EnterCriticalSection(&thread_cs);
	ArmMotionFlag = 1;
	LeaveCriticalSection(&thread_cs);
	return 0;
}

//摄像机线程
DWORD WINAPI Camera(LPVOID lpParameter)
{
	
	cout << "9. CamThread is running." << endl;
//	while (ArmMotionFlag == 0)
//	{
		try
		{
			vpCameraParameters cam(1017.13738, 1016.70876, 332.59991, 238.92479); //摄像相机内参数px py u0 v0
			vpHomogeneousMatrix eMc;//手到眼的齐次变换矩阵
			eMc[0][0] = 0.02392906951946957;
			eMc[0][1] = -0.9996695279615721;
			eMc[0][2] = -0.009393321937448875;
			eMc[0][3] = 3.921601488734311;
			eMc[1][0] = -0.9978755364802699;
			eMc[1][1] = -0.02445353703931933;
			eMc[1][2] = 0.06038574517616292;
			eMc[1][3] = 71.88804515813692;
			eMc[2][0] = -0.0605954893217822;
			eMc[2][1] = 0.007928391473358304;
			eMc[2][2] = -0.9981309169054426;
			eMc[2][3] = -7.847588857483204;

			//目标物坐标系下特征点的坐标VISP
			vector<vpPoint> point;
			point.push_back(vpPoint(-0.05, -0.05, 0));
			point.push_back(vpPoint( 0.05, -0.05, 0));
			point.push_back(vpPoint( 0.05,  0.05, 0));
			point.push_back(vpPoint(-0.05,  0.05, 0));
			//目标物坐标系下特征点的坐标OpenCV,用于solvePnP
			vector<Point3f> point3D;
			point3D.push_back(Point3f(-0.05, -0.05, 0));
			point3D.push_back(Point3f( 0.05, -0.05, 0));
			point3D.push_back(Point3f( 0.05,  0.05, 0));
			point3D.push_back(Point3f(-0.05,  0.05, 0));
			//像素坐标系下特征点的坐标OpenCV,用于solvePnP
			vector<Point2f> point2D;

			vpServo task;
			task.setServo(vpServo::EYEINHAND_CAMERA);
			task.setForceInteractionMatrixComputation(vpServo::CURRENT);	//使用当前点的深度信息
			task.setLambda(0.5);    //系数

			//获取图像
			MmVisualServoBase baslerCam;	//相机类
			vpImage<unsigned char> currentImage;		//当前灰度图像
			baslerCam.baslerOpen(currentImage);		//打开相机

			vpFeaturePoint p[4], pd[4];		//当前和理想4个特征点坐标
			vector<vpDot2> desireDot(4), currentDot(4);			//理想和当前的4个色块

			enum { DETECTION = 0, DESIRE = 1, CURRENT = 2, TRACKING = 3 };
			int mode = DETECTION;
			string msg = "Press 'd' to start get 4 desire feature points";		//图片上显示的提示信息
			int key;	//按键值

			vpDisplayOpenCV d(currentImage, 0, 0, "Current camera image");
			while (ArmMotionFlag == 0)
			{
				baslerCam.acquireBaslerImg(currentImage);

				key = 0xff & waitKey(100);
				if (key == 'd')
				{
					mode = DESIRE;
				}
				if (key == 'c')
				{
					mode = CURRENT;
				}

				if (mode == DETECTION)
				{
					vpDisplay::display(currentImage);
					vpDisplay::displayText(currentImage, 10, 10, msg, vpColor::red);
					vpDisplay::flush(currentImage);		//显示图像
					
				}
				if (mode == DESIRE)
				{
					vpDisplay::display(currentImage);
					vpDisplay::displayText(currentImage, 10, 10, "Click in the 4 dots to initialize the desire feature points", vpColor::red);
					vpDisplay::flush(currentImage);		//显示图像

					//理想位置特征记录
					vpImagePoint dot;
					for (unsigned int i = 0; i < 4; i++)
					{
						desireDot[i].setGraphics(true);
						desireDot[i].initTracking(currentImage);
						vpDisplay::flush(currentImage);
						vpFeatureBuilder::create(pd[i], cam, desireDot[i].getCog());		//获取色块重心图像坐标，这里没有深度信息
						dot = desireDot[i].getCog();
						cout << "pd" << i << ": " << pd[i].get_x() << "  " << pd[i].get_y() << endl;
						cout << "dot" << i << ": " << dot.get_u() << "  " << dot.get_v() << endl;
					}

					//solvePnP求解cdMo



					//返回刷图模式，等待按键c
					mode = DETECTION;
					msg = "Press 'c' to start get 4 current feature points";
				}
				if (mode = CURRENT)
				{
					vpDisplay::display(currentImage);
					vpDisplay::displayText(currentImage, 10, 10, "Click in the 4 dots to initialize the current feature points", vpColor::red);
					vpDisplay::flush(currentImage);		//显示图像

					//理想位置特征记录
					for (unsigned int i = 0; i < 4; i++)
					{
						currentDot[i].setGraphics(true);
						currentDot[i].initTracking(currentImage);
						vpDisplay::flush(currentImage);
						vpFeatureBuilder::create(p[i], cam, currentDot[i].getCog());		//获取色块重心图像坐标，这里没有深度信息
					}
					for (unsigned int i = 0; i < 4; i++)
					{
						task.addFeature(p[i], pd[i]);
					}
					mode = TRACKING;
				}
				if (mode == TRACKING)
				{

				}
				
			}


			

			

			


		}
		catch (vpException &e) {
			std::cout << "Catch an exception: " << e << std::endl;
		}


//	}
	cout << "10.CamThread has finished." << endl;

	return 0;
}

//移动机器人避障线程
DWORD WINAPI MobileRobotAviodFun(LPVOID lpParameter)
{
	cout << "2. MobileRobotAviodThread is running." << endl;
	while (VisualNavigationFlag == 0)
	{
		Sleep(50);
	}
	cout << "6. MobileRobotAviodThread has finished." << endl;

	if (navigationAutoflag == NAVIGATIONSTART)
	{
		//启动机械臂运动控制线程
		ResumeThread(ArmMotionThread);
	}
	
	return 0;
}

//机械臂避障线程
DWORD WINAPI ArmAvoidFun(LPVOID lpParameter)
{
	ResumeThread(CamThread);
	cout << "8. ArmAvoidThread is running." << endl;
	while (ArmMotionFlag == 0)
	{
		Sleep(50);
	}
	cout << "10. ArmAvoidThread has finished." << endl;

	navigationAutoflag = 0;			//自动导航完成，标志位置零
	return 0;
}

//套接字数据发送线程
DWORD WINAPI SendDatas(LPVOID lpParameter)
{
	srvToClientDatas.init();
	const SOCKET sockConn = (SOCKET)lpParameter;
	strcpy_s(srvToClientDatas.worlds, "Server");
	srvToClientDatas.rfidX = 92.5;
	srvToClientDatas.visualY = 10.2;
	srvToClientDatas.odomTh = 0.3;
	srvToClientDatas.armGamma = 40.5;
	char sendBuf[500];
	while (tcpThreadFlag != EXITSOCKET)
	{
		memset(sendBuf, 0, sizeof(sendBuf));		//对该段内存清零
		memcpy(sendBuf, &srvToClientDatas, sizeof(MmsrvToClientDatas));

		send(sockConn, sendBuf, sizeof(MmsrvToClientDatas), 0);
		Sleep(100);
		srvToClientDatas.play();
	}
	//srvToClientDatas.threadFlag = 1;
	//memset(sendBuf, 0, sizeof(sendBuf));		//对该段内存清零
	//memcpy(sendBuf, &srvToClientDatas, sizeof(MmsrvToClientDatas));
	//send(sockConn, sendBuf, sizeof(MmsrvToClientDatas), 0);
	//Sleep(500);
	tcpThreadFlag = 0;
	return 0;
}
//自动连续定位导航开启
void startAutoNavigation()
{
	VisualNavigationFlag = 0;	
	ArmMotionFlag = 0;			
	ThreadsExitFlag = 0;
	navigationAutoflag = NAVIGATIONSTART;

	RFIDNavigationThread = CreateThread(
		NULL,				//让新线程使用默认安全性
		0,					//设置线程初始栈的大小，让新线程采用与调用线程一样的栈大小
		RFIDNavigationFun,			//新线程入口函数的地址
		NULL,				//向新线程传递的参数，这个参数可以是一个数值，也可以是指向其他信息的指针
		0,					//控制线程创建的附加标记，为0则创建后立线程立即执行，若为CREATE_SUSPENDED，则线程创建后处于暂停状态，直到程序调用了ResumeThread
		NULL				//一个返回值，指向一个变量，用来接收线程ID，设置为NULL表示对线程的ID不感兴趣，不会返回线程的标识符
		);
	VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, CREATE_SUSPENDED, NULL);
	ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, CREATE_SUSPENDED, NULL);
	CamThread = CreateThread(NULL, 0, Camera, NULL, CREATE_SUSPENDED, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
	ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);
}

//RFID定位导航线程函数创建
void startRFIDNavigation()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = RFIDNAVIGATION;

	RFIDNavigationThread = CreateThread(NULL, 0, RFIDNavigationFun, NULL, 0, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
}

//视觉调整线程函数创建
void startVisualNavigation()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = VISUALNAVIGATION;

	VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, 0, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
}

//机械臂控制线程函数创建
void startArmControl()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = ARMCONTROL;

	ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, 0, NULL);
	ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);
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
	InitializeCriticalSection(&thread_cs);		//初始化关键代码段，用于线程间同步

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
	int len = sizeof(SOCKADDR);
	
	/************************************************************************/
	/*接收客户端指令并执行操作*/
	/************************************************************************/
	MmClientToSrvDatas clientToServDatas;

	while (true)
	{
		printf("Waiting for client connect.\n");
		//等待并接收客户端的连接请求
		//accept返回一个相当于当前这个新连接的一个套接字描述符，保存于sockConn，然后利用这个套接字与客户端通信
		SOCKET sockConn = accept(sockSrv, (SOCKADDR*)&addrClient, &len);
		SendDatasThread = CreateThread(NULL, 0, SendDatas, (LPVOID)sockConn, 0, NULL);
		printf("Accept client connect.\n");

		while (true)
		{
			char recvBuf[100];
			memset(recvBuf, 0, sizeof(recvBuf));

			//接收数据
			recv(sockConn, recvBuf, 100, 0);		//第三参数为缓冲区的长度

			memcpy(&clientToServDatas, recvBuf, sizeof(MmClientToSrvDatas));

			switch (clientToServDatas.contralSignal)
			{
			case NAVIGATIONSTART:startAutoNavigation(); break;		//自动连续导航，启动
			case NAVIGATIONSTOP:ThreadsExitFlag = NAVIGATIONSTOP; break;		//导航退出
			case RFIDNAVIGATION:startRFIDNavigation(); break;		//RFID单独导航，RFID
			case VISUALNAVIGATION:startVisualNavigation(); break;	//视觉调整，移动平台
			case ARMCONTROL:startArmControl(); break;			//机械臂控制，机械臂

			default:
				break;
			}

			//输出接收到的数据
			//		printf("%d\n", clientToServDatas.contralSignal);

			if (clientToServDatas.contralSignal == EXITSOCKET || clientToServDatas.contralSignal == EXITPROGRAM)
			{
				tcpThreadFlag = EXITSOCKET;		//线程标志位置1，退出线程
				printf("远程连接断开\n");
				break;
			}
		}
		
		while (tcpThreadFlag == EXITSOCKET)
		{
			Sleep(100);
		}
		closesocket(sockConn);		//关闭套接字
		if (clientToServDatas.contralSignal == EXITPROGRAM)
		{
			printf("退出程序\n");
			break;		//退出程序
		}
		
	}

	Sleep(500);
	//关闭线程句柄
	CloseHandle(RFIDNavigationThread);
	CloseHandle(VisualNavigationThread);
	CloseHandle(ArmMotionThread);
	CloseHandle(MobileRobotAviodThread);
	CloseHandle(ArmAvoidThread);
	CloseHandle(SendDatasThread);
}
