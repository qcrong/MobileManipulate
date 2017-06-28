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
	contralSignal = NOSIGNAL;
}

MmClientToSrvDatas::~MmClientToSrvDatas()
{
}

/************************************************************************/
/* �������ͻ��˷��͵�������*/
/************************************************************************/
MmsrvToClientDatas::MmsrvToClientDatas()
{
	////RFID��λ���
	//rfidX = 0.0;
	//rfidY = 0.0;
	//rfidTh = 0.0;
	////�Ӿ���λ���
	//visualX = 0.0;
	//visualY = 0.0;
	//visualTh = 0.0;
	////��̼ƶ�λ���
	//odomX = 0, 0; 
	//odomY = 0.0;
	//odomTh = 0.0;
	////��е��ĩ����̬
	//armX = 0.0; 
	//armY = 0.0; 
	//armZ = 0.0; 
	//armAlpha = 0.0;
	//armBeta = 0.0;
	//armGamma = 0.0;
	////�Ƿ���������Ϣ����Ϊ1����Ϊ0
	//bool haveWorldsFlag = 0;

	//contralSignal = NOSIGNAL;		//�����ź�
	//threadFlag = 0;	//������׽����˳���־λ�����͸��ͻ��ˣ�Ϊ1ʱ�ͻ��˽����߳��˳�
}

MmsrvToClientDatas::~MmsrvToClientDatas()
{
}

MmsrvToClientDatas srvToClientDatas;

void MmsrvToClientDatas::init()
{
	//RFID��λ���
	rfidX = 0.0;
	rfidY = 0.0;
	rfidTh = 0.0;
	//�Ӿ���λ���
	visualX = 0.0;
	visualY = 0.0;
	visualTh = 0.0;
	//��̼ƶ�λ���
	odomX = 0, 0;
	odomY = 0.0;
	odomTh = 0.0;
	//��е��ĩ����̬
	armX = 0.0;
	armY = 0.0;
	armZ = 0.0;
	armAlpha = 0.0;
	armBeta = 0.0;
	armGamma = 0.0;
	//�Ƿ���������Ϣ����Ϊ1����Ϊ0
	bool haveWorldsFlag = 0;

	contralSignal = NOSIGNAL;		//�����ź�
	threadFlag = 0;	//������׽����˳���־λ�����͸��ͻ��ˣ�Ϊ1ʱ�ͻ��˽����߳��˳�
}


void MmsrvToClientDatas::play()
{
	rfidTh += 0.1;

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