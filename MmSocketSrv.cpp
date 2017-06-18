/************************************************************************/
/*客户端与服务端进行TCP/IP通信的类，负责数据的收发
客户端是指远程的GUI界面显示计算机，服务端是移动机器人上的车载电脑
本类为服务端类*/
/************************************************************************/

#include "MmSocketSrv.h"

MmSocketServer::MmSocketServer()
{
}

MmSocketServer::~MmSocketServer()
{
}


/************************************************************************/
/* 客户端向服务端发送的数据类*/
/************************************************************************/

MmClientToSrvDatas::MmClientToSrvDatas()
{
	contralSignal = 0;
}

MmClientToSrvDatas::~MmClientToSrvDatas()
{
}

/************************************************************************/
/* 客户端向服务端发送的数据类*/
/************************************************************************/
Datas::Datas()
{
}

Datas::~Datas()
{
}

void Datas::play()
{
	number++;
}

/************************************************************************/
/* 客户端向服务端发送的数据类*/
/************************************************************************/

//MmClientToSrvDatas clientToServDatas;
//bool sendDatasFlag = 0;
//
//DWORD WINAPI SendDatas(LPVOID lpParameter)
//{
//	const SOCKET sockClient = (SOCKET)lpParameter;
//	char sendBuf[100];
//	while (true)
//	{
//		if (sendDatasFlag==1)
//		{
//			sendDatasFlag = 0;
//			memset(sendBuf, 0, sizeof(sendBuf));		//对该段内存清零
//			memcpy(sendBuf, &clientToServDatas, sizeof(MmClientToSrvDatas));
//
//			send(sockClient, sendBuf, sizeof(MmClientToSrvDatas), 0);
//			Sleep(100);
//		}
//		if (clientToServDatas.contralSignal = EXITPROGRAM)
//		{
//			break;
//		}
//
//		Sleep(100);
//	}
//
//	return 0;
//}
//
//DWORD WINAPI RecvDatas(LPVOID lpParameter)
//{
//	const SOCKET sockClient = (SOCKET)lpParameter;
//	char recvBuf[100];
//	Datas data;
//
//	while (true)
//	{
//		memset(recvBuf, 0, sizeof(recvBuf));
//
//		//接收数据
//		recv(sockClient, recvBuf, 100, 0);		//第三参数为缓冲区的长度
//
//		memcpy(&data, recvBuf, sizeof(Datas));
//
//		//输出接收到的数据
//		//		printf("%d %s %lf\n", data.number, data.name, data.score);
//		//		cout << data.number << " " << data.name << " " << data.score << endl;
//
//		if (data.threadFlag == 1)
//		{
//			break;
//		}
//	}
//
//	return 0;
//}