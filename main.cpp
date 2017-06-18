//------------------------------------------------------------------------------
//�ƶ�����ƽ̨��������
//�����ڻ����˱�����
/// @file main.cpp
/// @brief Remote commands tester
//2017��
//by ����
//------------------------------------------------------------------------------


//�Խ����ͷ�ļ�
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

//ȫ�ֱ���

//VisualNavigationThread�߳����б�־λ�������߳̽���ʱ��־λ��1
volatile int VisualNavigationFlag = 0;  //volatile�����߱��������ȫ�ֱ������ױ�ģ��ñ�������Ҫ��������������Ż�
//ArmMotionThread�߳����б�־λ�������߳̽���ʱ��־λ��1
volatile int ArmMotionFlag = 0;
//ʹ�ùؼ�����ν����̼߳�ͬ��
CRITICAL_SECTION thread_cs;  


//�����߳̾��
HANDLE RFIDNavigationThread;		//RFID��Χ�����߳�
HANDLE VisualNavigationThread;		//�Ӿ��ֲ������߳�
HANDLE ArmMotionThread;				//��е��ץȡ�߳�
HANDLE MobileRobotAviodThread;		//�ƶ������˱����߳�
HANDLE ArmAvoidThread;				//��е�۱����߳�
HANDLE sendDatasThread;				//�׽������ݷ����߳�
//HANDLE recvDatasRhread;				//�׽������ݽ����߳�

//TCPȫ�ֱ���
bool tcpThreadFlag = 0;		//�̺߳�����ֹ��־λ��Ϊ1ʱ�߳��˳�


/************************************************************************/
/*�̺߳���*/
/************************************************************************/
//RFID�����̺߳���
DWORD WINAPI RFIDNavigationFun(LPVOID lpParameter)
{
	ResumeThread(MobileRobotAviodThread);
	cout << "RFIDNavigationThread is running." << endl;
	Sleep(100);
	cout << "RFIDNavigationThread has finished." << endl;

	ResumeThread(VisualNavigationThread);
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

	//������е���˶����ƺͱ����߳�
	ResumeThread(ArmMotionThread);
	ResumeThread(ArmAvoidThread);
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

//�׽������ݷ����߳�
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
		memset(sendBuf, 0, sizeof(sendBuf));		//�Ըö��ڴ�����
		memcpy(sendBuf, &data, sizeof(Datas));

		send(sockConn, sendBuf, sizeof(Datas), 0);
		Sleep(1000);
		data.number++;
	}
	data.threadFlag = 1;
	memset(sendBuf, 0, sizeof(sendBuf));		//�Ըö��ڴ�����
	memcpy(sendBuf, &data, sizeof(Datas));
	send(sockConn, sendBuf, sizeof(Datas), 0);

	return 0;
}

//�׽������ݽ����߳�
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
//		//��������
//		recv(sockConn, recvBuf, 100, 0);		//��������Ϊ�������ĳ���
//
//		memcpy(&signalFlag, recvBuf, sizeof(signalFlag));
//
//		//������յ�������
//		printf("%d\n", signalFlag);
//
//		if (signalFlag == 30)
//		{
//			tcpThreadFlag = 1;		//�̱߳�־λ��1���˳��߳�
//			break;
//		}
//	}
//
//	return 0;
//}



/************************************************************************/
/*���������*/
/************************************************************************/
void main
   (
   int argc,
   char **argv
   )
{
	/************************************************************************/
	/*�����׽���*/
	/************************************************************************/
	//�����׽��ֿ�
	WORD wVersionRequest;		//����ָ��׼�����ص�Winsock��İ汾
	WSADATA wsaData;
	int err;

	wVersionRequest = MAKEWORD(1, 1);		//ָ���汾��(x,y),����x�Ǹ�λ�ֽڣ�y�ǵ�λ�ֽ�
	err = WSAStartup(wVersionRequest, &wsaData);
	if (err != 0)
	{
		return;
	}
	//��֤�汾��
	if (LOBYTE(wsaData.wVersion) != 1 ||
		HIBYTE(wsaData.wVersion) != 1)
	{
		WSACleanup();
		return;
	}

	//�������ڼ������׽���
	SOCKET sockSrv = socket(AF_INET,		//Windows Socketsֻ֧��һ��ͨ����������AF_INET��
		SOCK_STREAM,		//����TCPЭ������������Ҫ������ʽ�׽���
		0);			//���ݸ�ʽ��ַ���׽�������Զ�ѡ��һ�����ʵ�Э��

	SOCKADDR_IN addrSrv;
	//���ýṹ���Ա�������и�ֵ��htonl��u_long���͵�ֵ�������ֽ�˳��ת��ΪTCP/IP�����ֽ�˳��
	addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);// inet_addr("192.168.0.115");		//htonl(INADDR_ANY)�����׽������κη�������ػ�����IP��ַ���ͻ��������
	addrSrv.sin_family = AF_INET;		//����ֻ��ָ��AF_INET
	//htonl��u_long���͵�ֵ�������ֽ�˳��ת��ΪTCP / IP�����ֽ�˳��
	addrSrv.sin_port = htons(6001);		//ָ���˿ںţ�Ҫ�����1024С��65535

	//���׽���,���׽���sockSrv�󶨵����ص�ַ��ָ���Ķ˿���
	//�����������Ҫ����::ָ��ȫ�������򣬷������Aria.h��ͻ������bindʧ��
	::bind(sockSrv,			//��Ҫ�󶨵��׽���
		(SOCKADDR*)&addrSrv,	//ǿ������װ��������TCP/IP��socket��̹����У�������SOCKADDR_IN�ṹ�滻SOCKADDR��������д��Ϣ
		sizeof(SOCKADDR)		//ָ����ַ�ṹ�ĳ���
		);

	//�趨�Ѱ󶨵��׽���Ϊ����ģʽ��׼�����տͻ�����
	listen(sockSrv, 20);		//�ڶ�������Ϊ�ȴ����е���󳤶�
	SOCKADDR_IN addrClient;		//����һ����ַ�ṹ��SOCKADDR_IN�������������տͻ��˵ĵ�ַ��Ϣ
	printf("Waiting for client connect.\n");
	//�ȴ������տͻ��˵���������
	//accept����һ���൱�ڵ�ǰ��������ӵ�һ���׽�����������������sockConn��Ȼ����������׽�����ͻ���ͨ��
	int len = sizeof(SOCKADDR);
	SOCKET sockConn = accept(sockSrv, (SOCKADDR*)&addrClient, &len);
	printf("Accept client connect.\n");

	
	
	/************************************************************************/
	/*�����߳�*/
	/************************************************************************/
	RFIDNavigationThread = CreateThread(
		NULL,				//�����߳�ʹ��Ĭ�ϰ�ȫ��
		0,					//�����̳߳�ʼջ�Ĵ�С�������̲߳���������߳�һ����ջ��С
		RFIDNavigationFun,			//���߳���ں����ĵ�ַ
		NULL,				//�����̴߳��ݵĲ������������������һ����ֵ��Ҳ������ָ��������Ϣ��ָ��
		CREATE_SUSPENDED,					//�����̴߳����ĸ��ӱ�ǣ�Ϊ0�򴴽������߳�����ִ�У���ΪCREATE_SUSPENDED�����̴߳���������ͣ״̬��ֱ�����������ResumeThread
		NULL				//һ������ֵ��ָ��һ�����������������߳�ID������ΪNULL��ʾ���̵߳�ID������Ȥ�����᷵���̵߳ı�ʶ��
		);
	VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, CREATE_SUSPENDED, NULL);
	ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, CREATE_SUSPENDED, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
	ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);
	sendDatasThread = CreateThread(NULL, 0, SendDatas, (LPVOID)sockConn, 0, NULL);
	
	InitializeCriticalSection(&thread_cs);		//��ʼ���ؼ������


	/************************************************************************/
	/*���տͻ���ָ�*/
	/************************************************************************/
	MmClientToSrvDatas clientToServDatas;

	while (true)
	{
		char recvBuf[100];
		memset(recvBuf, 0, sizeof(recvBuf));

		//��������
		recv(sockConn, recvBuf, 100, 0);		//��������Ϊ�������ĳ���

		memcpy(&clientToServDatas, recvBuf, sizeof(MmClientToSrvDatas));

		switch (clientToServDatas.contralSignal)
		{
		case NAVIGATIONSTART:ResumeThread(RFIDNavigationThread); break;


			
		default:
			break;
		}

		//������յ�������
//		printf("%d\n", clientToServDatas.contralSignal);

		if (clientToServDatas.contralSignal == EXITPROGRAM)
		{
			tcpThreadFlag = 1;		//�̱߳�־λ��1���˳��߳�
			break;
		}
	}

	
//	recvDatasRhread = CreateThread(NULL, 0, RecvDatas, (LPVOID)sockConn, 0, NULL);


	//����RFID�����̼߳�С�������߳�
	//ResumeThread(RFIDNavigationThread);
	//ResumeThread(MobileRobotAviodThread);
	////�ȴ�RFID�����߳��߳��˳�
	//WaitForSingleObject(RFIDNavigationThread, INFINITE);
	////RFID��������������Ӿ�λ�˵���
	//ResumeThread(VisualNavigationThread);
	//WaitForSingleObject(VisualNavigationThread, INFINITE);
	//WaitForSingleObject(MobileRobotAviodThread, INFINITE);
	////�Ӿ�λ�˵�������������е���˶��̣߳���������е�۱����߳�
	//ResumeThread(ArmMotionThread);
	//ResumeThread(ArmAvoidThread);
	//WaitForSingleObject(ArmMotionThread, INFINITE);	
	//WaitForSingleObject(ArmAvoidThread, INFINITE);
	////�ȴ�SOCKET�߳��˳�
	//WaitForSingleObject(sendDatasThread, INFINITE);
//	WaitForSingleObject(recvDatasRhread, INFINITE);

	//�ر��߳̾��
	CloseHandle(RFIDNavigationThread);
	CloseHandle(VisualNavigationThread);
	CloseHandle(ArmMotionThread);
	CloseHandle(MobileRobotAviodThread);
	CloseHandle(ArmAvoidThread);
	CloseHandle(sendDatasThread);

	cout << "exit program" << endl;
}
