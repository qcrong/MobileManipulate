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
//volatile�����߱��������ȫ�ֱ������ױ�ģ��ñ�������Ҫ��������������Ż�
//VisualNavigationThread�߳����б�־λ�������߳̽���ʱ��־λ��1
volatile int VisualNavigationFlag = 0;  
//ArmMotionThread�߳����б�־λ�������߳̽���ʱ��־λ��1
volatile int ArmMotionFlag = 0;
//�ͻ��˰���ֹͣ��ťʱ��ThreadsExitFlag��1���������߳��˳�
volatile int ThreadsExitFlag = 0;
//��е�ۺ�����̵߳�ͬ�����Ʊ�־λ������˶���λʱ��1��������������2
volatile int armCamFlag = 0;

//ʹ�ùؼ�����ν����̼߳�ͬ��
CRITICAL_SECTION thread_cs;  
//�Զ�������־λ���ñ�־λΪNAVIGATIONSTARTʱ���Ⱥ��Զ�ִ��RFID�������Ӿ�λ�˵����ͻ�е��ץȡ
int navigationAutoflag = 0;		

//�����߳̾��
HANDLE RFIDNavigationThread;		//RFID��Χ�����߳�
HANDLE VisualNavigationThread;		//�Ӿ��ֲ������߳�
HANDLE ArmMotionThread;				//��е��ץȡ�߳�
HANDLE MobileRobotAviodThread;		//�ƶ������˱����߳�
HANDLE ArmAvoidThread;				//��е�۱����߳�
HANDLE SendDatasThread;				//�׽������ݷ����߳�
HANDLE CamThread;				//�׽������ݽ����߳�

//TCPȫ�ֱ���
int tcpThreadFlag = 0;		//�̺߳�����ֹ��־λ��Ϊ1ʱ�߳��˳�



/************************************************************************/
/*�̺߳���*/
/************************************************************************/
//RFID�����̺߳���
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
	
	//�����Զ�������������һ����
	if (navigationAutoflag == NAVIGATIONSTART)
	{
		ResumeThread(VisualNavigationThread);
	}
	else
	{
		//�����߳̽�����־λ��1�������߳��˳�
		EnterCriticalSection(&thread_cs);
		VisualNavigationFlag = 1;
		LeaveCriticalSection(&thread_cs);
	}
	
	return 0;
}

//�Ӿ������̺߳���
DWORD WINAPI VisualNavigationFun(LPVOID lpParameter)
{
	//���Զ���������£������߳���Ҫ��������
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
	
	//�����߳̽�����־λ��1�������߳��˳�
	EnterCriticalSection(&thread_cs);
	VisualNavigationFlag = 1;
	LeaveCriticalSection(&thread_cs);

	return 0;
}

//��е���˶������̺߳���
DWORD WINAPI ArmMotionFun(LPVOID lpParameter)
{
	ResumeThread(ArmAvoidThread);
	while (ThreadsExitFlag != NAVIGATIONSTOP)
	{
		cout << "7. ArmMotionThread is running." << endl;
		//��е�۳�ʼ��
		EcCytonCommands cytonCommands;								//ʵ����
		///�򿪻�����Զ�̿���
		EcString ipAddress = "127.0.0.1";
		EcString cytonVersion = "1500";
		EcString cytonDir = Ec::Application::getDataDirectory("cyton");
		if (cytonDir.empty())
		{
			cytonDir = ".";
		}
		cytonCommands.openNetwork(ipAddress);

		//�궨��ʼ����������е�۵���ʼλ��
		EcRealVector jointposition(7);		//���ڻ�е��λ�˿���
		jointposition.resize(7);		//��е�ۻص���ֱλ��
		RC_CHECK(cytonCommands.MoveJointsExample(jointposition, 0.000001));

		cout << "����λ��" << endl;
		jointposition[1] = EcPi / 180 * 90;
		jointposition[3] = -EcPi / 180 * 90;
		jointposition[5] = EcPi / 180 * 4;
		RC_CHECK(cytonCommands.MoveJointsExample(jointposition, .000001));
		//�ӹؽڽǿ����л���λ�˿��ƣ��Ե�ǰλ�˽��г�ʼ��
		EcCoordinateSystemTransformation desiredPose;
		cytonCommands.changeToFrameEE(desiredPose);
		EcSLEEPMS(10000);
		EcVector relaTransform(0.0, 0.0, -0.03);		//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		EcOrientation relaOriention;            //����ڵ�ǰ��צĩ������ϵZ-Y-X����ת��
		relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		EcCoordinateSystemTransformation relativetrans(relaTransform, relaOriention);		//����ƽ����ת��
		RC_CHECK(cytonCommands.frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(2000);
		armCamFlag = 1;    //����˶�������λ��
		
		//�ȴ������ȡ����λ����Ϣ
		while (armCamFlag!=2)
		{
			Sleep(100);
		}

		cout << "�˶�����ǰλ��" << endl;
		relaTransform.set(0.01, 0.0, 0.0);		//XYZƽ������
		relaOriention.setFrom321Euler(-EcPi / 12, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		RC_CHECK(cytonCommands.frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(2000);
		armCamFlag = 3;    //����˶�����ǰλ��

		//�ȴ������ȡ��ǰλ����Ϣ
		while (armCamFlag != 4)
		{
			Sleep(100);
		}



		cytonCommands.closeNetwork();
		cout << "9. ArmMotionThread has finished." << endl;
		break;
	}
	
	//�߳̽�����־λ��1
	EnterCriticalSection(&thread_cs);
	ArmMotionFlag = 1;
	LeaveCriticalSection(&thread_cs);
	return 0;
}

//������߳�
DWORD WINAPI Camera(LPVOID lpParameter)
{
	
	cout << "9. CamThread is running." << endl;
	MmVisualServoAlgorithm visualServoAlgorithm;
//	while (ArmMotionFlag == 0)
//	{
		try
		{
			vpCameraParameters cam(1017.13738, 1016.70876, 332.59991, 238.92479); //��������ڲ���px py u0 v0
			double camIntrinsic[9] = { 1017.13738, 0, 332.59991,
				0, 1016.70876, 238.92479,
				0, 0, 1 };
			visualServoAlgorithm.setCamIntrinsic(camIntrinsic);				//������ڲ���ת����OpenCV��ʽ��
			cout << visualServoAlgorithm.cam_intrinsic_matrix << endl << endl;
			double camDistortion[5] = { -0.4034783300939766, 0.6235344618772364, -0.0003784404964069526, -0.0006034915824095659, -1.59162547482239 };	//������������
			visualServoAlgorithm.setCamDistortion(camDistortion);
			cout << visualServoAlgorithm.cam_distortion_matrix << endl;
			vpHomogeneousMatrix eMc;	//�ֵ��۵���α任����
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

			//Ŀ��������ϵ�������������VISP,��λm
			vector<vpPoint> point;
			point.push_back(vpPoint(-0.05, -0.05, 0));
			point.push_back(vpPoint( 0.05, -0.05, 0));
			point.push_back(vpPoint( 0.05,  0.05, 0));
			point.push_back(vpPoint(-0.05,  0.05, 0));
			//Ŀ��������ϵ�������������OpenCV,����solvePnP����λmm
			vector<Point3f> point3D;
			point3D.push_back(Point3f(-50, -50, 0));
			point3D.push_back(Point3f( 50, -50, 0));
			point3D.push_back(Point3f( 50,  50, 0));
			point3D.push_back(Point3f(-50,  50, 0));
			//����͵�ǰ�������ϵ��Ŀ������ϵ����α任����
			vpHomogeneousMatrix cdMo, cMo; 

			vpServo task;		//�Ӿ��ŷ�����
			task.setServo(vpServo::EYEINHAND_CAMERA);
			task.setForceInteractionMatrixComputation(vpServo::CURRENT);	//ʹ�õ�ǰ��������Ϣ
			task.setLambda(0.5);    //ϵ��

			//��ȡͼ��
			MmVisualServoBase baslerCam;	//�����
			vpImage<unsigned char> currentImage;		//��ǰ�Ҷ�ͼ��
			baslerCam.baslerOpen(currentImage);		//�����
			baslerCam.acquireBaslerImg(currentImage);

			vpFeaturePoint p[4], pd[4];		//��ǰ������4������������
			vector<vpDot2> desireDot(4), currentDot(4);			//����͵�ǰ��4��ɫ��

			//��ͬģʽ
			enum { DETECTION = 0, DESIRE = 1, CURRENT = 2, TRACKING = 3 };
			int mode = DETECTION;
			string msg = "��е��������λ���˶�";		//ͼƬ����ʾ����ʾ��Ϣ
			int key;	//����ֵ

			vpDisplayOpenCV d(currentImage, 0, 0, "Current camera image");
			while (1)//ArmMotionFlag == 0
			{
				baslerCam.acquireBaslerImg(currentImage);

				key = 0xff & waitKey(30);
				if (key == 'd'&& armCamFlag == 1)
				{
					mode = DESIRE;
				}
				if (key == 'c'&& armCamFlag == 3)
				{
					mode = CURRENT;
				}
				if ((key & 255) == 27)   //  esc�˳���
				{
					break;
				}

				if (mode == DETECTION)
				{
					if (armCamFlag==1)
					{
						msg = "��е�۵�������λ�ˣ���d��������ѡȡ";
					}
					if (armCamFlag == 3)
					{
						msg = "��е�۵��ﵱǰλ�ˣ���c��������ѡȡ";
					}
					vpDisplay::display(currentImage);
					vpDisplay::displayText(currentImage, 10, 10, msg, vpColor::red);
					vpDisplay::flush(currentImage);		//��ʾͼ��
				}
				if (mode == DESIRE)
				{
					vpDisplay::display(currentImage);
					vpDisplay::displayText(currentImage, 10, 10, "ѡȡ4��ɫ�飬��ʼ������������", vpColor::red);
					vpDisplay::flush(currentImage);		//��ʾͼ��

					//����λ��������¼
					//��������ϵ�������������OpenCV,����solvePnP
					vector<Point2f> point2D;
					for (unsigned int i = 0; i < 4; i++)
					{
						desireDot[i].setGraphics(true);
						desireDot[i].initTracking(currentImage);
						vpDisplay::flush(currentImage);
						vpImagePoint dot;
						dot = desireDot[i].getCog();		//ɫ�����ĵ���������
						visualServoAlgorithm.pixelToImage(pd[i], cam, dot);		//��ȡɫ������ͼ�����굥λ���ף�����û�������Ϣ
						//dot = desireDot[i].getCog();
						//cout << "pd" << i << ": " << pd[i].get_x() << "  " << pd[i].get_y() << endl;
						//cout << "dot" << i << ": " << dot.get_u() << "  " << dot.get_v() << endl;
						point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
						//cout << "point2D " << point2D[i] << endl;
					}
					//solvePnP���cdMo
					Mat rvec = Mat::zeros(3, 1, CV_64FC1); //��ת����
					Mat rM; //��ת����
					Mat tvec;//ƽ������
					solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
					Rodrigues(rvec, rM);
					cout << "��ת����:" << endl;
					cout << rM << endl;
					cout << "ƽ������:" << endl;
					cout << tvec << endl;
					visualServoAlgorithm.setvpHomogeneousMatrix(cdMo, rM, tvec);
					cout << "����λ����α任����:" << endl;
					cout << cdMo << endl;
					//���������Ϣ
					for (int i = 0; i < 4; i++)
					{
						vpColVector cP;		//�������ϵ�����������ά����
						point[i].changeFrame(cdMo, cP);
						pd[i].set_Z(cP[2]);
					}

					//����ˢͼģʽ���ȴ�����c
					mode = DETECTION;
					armCamFlag = 2;   //����������ȡ��ɣ���е���˶�����ǰλ��
					msg = "��е����ǰλ���˶�";
				}
				if (mode == CURRENT)
				{
					vpDisplay::display(currentImage);
					vpDisplay::displayText(currentImage, 10, 10, "ѡȡ4��ɫ�飬��ʼ����ǰ������", vpColor::red);
					vpDisplay::flush(currentImage);		//��ʾͼ��

					//����λ��������¼
					vector<Point2f> point2D;
					for (unsigned int i = 0; i < 4; i++)
					{
						currentDot[i].setGraphics(true);
						currentDot[i].initTracking(currentImage);
						vpDisplay::flush(currentImage);
						vpImagePoint dot;
						dot = currentDot[i].getCog();
						visualServoAlgorithm.pixelToImage(p[i], cam, dot);	//��ȡɫ������ͼ�����굥λ���ף�����û�������Ϣ
						point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
					}
					Mat rvec = Mat::zeros(3, 1, CV_64FC1); //��ת����
					Mat rM; //��ת����
					Mat tvec;//ƽ������
					solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
					Rodrigues(rvec, rM);
					cout << "��ת����:" << endl;
					cout << rM << endl;
					cout << "ƽ������:" << endl;
					cout << tvec << endl;
					visualServoAlgorithm.setvpHomogeneousMatrix(cMo, rM, tvec);
					cout << "��ǰλ����α任����:" << endl;
					cout << cMo << endl;
					for (int i = 0; i < 4; i++)
					{
						vpColVector cP;		//�������ϵ�����������ά����
						point[i].changeFrame(cMo, cP);
						p[i].set_Z(cP[2]);
					}

					//д����������Ϣ������ͼ������������Ϣ����λΪm
					for (unsigned int i = 0; i < 4; i++)
					{
						task.addFeature(p[i], pd[i]);
						cout << "p" << i << ": " << p[i].get_x() << ", " << p[i].get_y() << ", " << p[i].get_Z() << endl;
						cout << "pd" << i << ": " << pd[i].get_x() << ", " << pd[i].get_y() << ", " << pd[i].get_Z() << endl;
					}
					mode = DETECTION;
					armCamFlag = 4;
					msg = "��ʼ�ŷ�����";
				}
				if (mode == TRACKING)
				{

				}
				
			}
			baslerCam.close();

			

			

			


		}
		catch (vpException &e) {
			std::cout << "Catch an exception: " << e << std::endl;
		}


//	}

	cout << "10.CamThread has finished." << endl;

	return 0;
}

//�ƶ������˱����߳�
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
		//������е���˶������߳�
		ResumeThread(ArmMotionThread);
	}
	
	return 0;
}

//��е�۱����߳�
DWORD WINAPI ArmAvoidFun(LPVOID lpParameter)
{
	ResumeThread(CamThread);
	cout << "8. ArmAvoidThread is running." << endl;
	while (ArmMotionFlag == 0)
	{
		Sleep(50);
	}
	cout << "10. ArmAvoidThread has finished." << endl;

	navigationAutoflag = 0;			//�Զ�������ɣ���־λ����
	return 0;
}

//�׽������ݷ����߳�
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
		memset(sendBuf, 0, sizeof(sendBuf));		//�Ըö��ڴ�����
		memcpy(sendBuf, &srvToClientDatas, sizeof(MmsrvToClientDatas));

		send(sockConn, sendBuf, sizeof(MmsrvToClientDatas), 0);
		Sleep(100);
		srvToClientDatas.play();
	}
	//srvToClientDatas.threadFlag = 1;
	//memset(sendBuf, 0, sizeof(sendBuf));		//�Ըö��ڴ�����
	//memcpy(sendBuf, &srvToClientDatas, sizeof(MmsrvToClientDatas));
	//send(sockConn, sendBuf, sizeof(MmsrvToClientDatas), 0);
	//Sleep(500);
	tcpThreadFlag = 0;
	return 0;
}
//�Զ�������λ��������
void startAutoNavigation()
{
	VisualNavigationFlag = 0;	
	ArmMotionFlag = 0;			
	ThreadsExitFlag = 0;
	navigationAutoflag = NAVIGATIONSTART;

	RFIDNavigationThread = CreateThread(
		NULL,				//�����߳�ʹ��Ĭ�ϰ�ȫ��
		0,					//�����̳߳�ʼջ�Ĵ�С�������̲߳���������߳�һ����ջ��С
		RFIDNavigationFun,			//���߳���ں����ĵ�ַ
		NULL,				//�����̴߳��ݵĲ������������������һ����ֵ��Ҳ������ָ��������Ϣ��ָ��
		0,					//�����̴߳����ĸ��ӱ�ǣ�Ϊ0�򴴽������߳�����ִ�У���ΪCREATE_SUSPENDED�����̴߳���������ͣ״̬��ֱ�����������ResumeThread
		NULL				//һ������ֵ��ָ��һ�����������������߳�ID������ΪNULL��ʾ���̵߳�ID������Ȥ�����᷵���̵߳ı�ʶ��
		);
	VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, CREATE_SUSPENDED, NULL);
	ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, CREATE_SUSPENDED, NULL);
	CamThread = CreateThread(NULL, 0, Camera, NULL, CREATE_SUSPENDED, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
	ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);
}

//RFID��λ�����̺߳�������
void startRFIDNavigation()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = RFIDNAVIGATION;

	RFIDNavigationThread = CreateThread(NULL, 0, RFIDNavigationFun, NULL, 0, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
}

//�Ӿ������̺߳�������
void startVisualNavigation()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = VISUALNAVIGATION;

	VisualNavigationThread = CreateThread(NULL, 0, VisualNavigationFun, NULL, 0, NULL);
	MobileRobotAviodThread = CreateThread(NULL, 0, MobileRobotAviodFun, NULL, CREATE_SUSPENDED, NULL);
}

//��е�ۿ����̺߳�������
void startArmControl()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = ARMCONTROL;

	ArmMotionThread = CreateThread(NULL, 0, ArmMotionFun, NULL, 0, NULL);
	CamThread = CreateThread(NULL, 0, Camera, NULL, CREATE_SUSPENDED, NULL);
	ArmAvoidThread = CreateThread(NULL, 0, ArmAvoidFun, NULL, CREATE_SUSPENDED, NULL);
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
	InitializeCriticalSection(&thread_cs);		//��ʼ���ؼ�����Σ������̼߳�ͬ��

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
	int len = sizeof(SOCKADDR);
	
	/************************************************************************/
	/*���տͻ���ָ�ִ�в���*/
	/************************************************************************/
	MmClientToSrvDatas clientToServDatas;

	while (true)
	{
		printf("Waiting for client connect.\n");
		//�ȴ������տͻ��˵���������
		//accept����һ���൱�ڵ�ǰ��������ӵ�һ���׽�����������������sockConn��Ȼ����������׽�����ͻ���ͨ��
		SOCKET sockConn = accept(sockSrv, (SOCKADDR*)&addrClient, &len);
		SendDatasThread = CreateThread(NULL, 0, SendDatas, (LPVOID)sockConn, 0, NULL);
		printf("Accept client connect.\n");

		while (true)
		{
			char recvBuf[100];
			memset(recvBuf, 0, sizeof(recvBuf));

			//��������
			recv(sockConn, recvBuf, 100, 0);		//��������Ϊ�������ĳ���

			memcpy(&clientToServDatas, recvBuf, sizeof(MmClientToSrvDatas));

			switch (clientToServDatas.contralSignal)
			{
			case NAVIGATIONSTART:startAutoNavigation(); break;		//�Զ���������������
			case NAVIGATIONSTOP:ThreadsExitFlag = NAVIGATIONSTOP; break;		//�����˳�
			case RFIDNAVIGATION:startRFIDNavigation(); break;		//RFID����������RFID
			case VISUALNAVIGATION:startVisualNavigation(); break;	//�Ӿ��������ƶ�ƽ̨
			case ARMCONTROL:startArmControl(); break;			//��е�ۿ��ƣ���е��

			default:
				break;
			}

			//������յ�������
			//		printf("%d\n", clientToServDatas.contralSignal);

			if (clientToServDatas.contralSignal == EXITSOCKET || clientToServDatas.contralSignal == EXITPROGRAM)
			{
				tcpThreadFlag = EXITSOCKET;		//�̱߳�־λ��1���˳��߳�
				printf("Զ�����ӶϿ�\n");
				break;
			}
		}
		
		while (tcpThreadFlag == EXITSOCKET)
		{
			Sleep(100);
		}
		closesocket(sockConn);		//�ر��׽���
		if (clientToServDatas.contralSignal == EXITPROGRAM)
		{
			printf("�˳�����\n");
			break;		//�˳�����
		}
		
	}

	Sleep(500);
	//�ر��߳̾��
	CloseHandle(RFIDNavigationThread);
	CloseHandle(VisualNavigationThread);
	CloseHandle(ArmMotionThread);
	CloseHandle(MobileRobotAviodThread);
	CloseHandle(ArmAvoidThread);
	CloseHandle(SendDatasThread);
}
