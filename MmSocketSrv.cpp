/************************************************************************/
/*�ͻ��������˽���TCP/IPͨ�ŵ��࣬�������ݵ��շ�
�ͻ�����ָԶ�̵�GUI������ʾ���������������ƶ��������ϵĳ��ص���
����Ϊ�������*/
/************************************************************************/

#include "MmSocketSrv.h"

MmSocketServer::MmSocketServer()
{
}

MmSocketServer::~MmSocketServer()
{
}


/************************************************************************/
/* �ͻ��������˷��͵�������*/
/************************************************************************/

MmClientToSrvDatas::MmClientToSrvDatas()
{
	contralSignal = 0;
}

MmClientToSrvDatas::~MmClientToSrvDatas()
{
}

/************************************************************************/
/* �ͻ��������˷��͵�������*/
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
/* �ͻ��������˷��͵�������*/
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
//			memset(sendBuf, 0, sizeof(sendBuf));		//�Ըö��ڴ�����
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
//		//��������
//		recv(sockClient, recvBuf, 100, 0);		//��������Ϊ�������ĳ���
//
//		memcpy(&data, recvBuf, sizeof(Datas));
//
//		//������յ�������
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