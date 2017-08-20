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

using namespace std;

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
//�Ӿ���������Ļ�����ϵ�»�е��ĩ���˶��ٶ�
VectorXd fVe(6);
//��е�ۿ�����ָ�룬��Ϊȫ�ֱ��������Ӿ��̻߳�ȡ��е����Ϣ
EcCytonCommands *cytonCommands;

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
		//��Ե������ԣ������߳̽�����־λ��1�������߳��˳�
		//EnterCriticalSection(&thread_cs);
		VisualNavigationFlag = 1;
		srvToClientDatas.contralSignal = FINISH_RFID_CONTROL;  //��ͻ��˷����źţ��ָ��������ư�ť
		//LeaveCriticalSection(&thread_cs);
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


	if (navigationAutoflag == NAVIGATIONSTART)
	{
		//�����߳̽�����־λ��1�������߳��˳�
		//EnterCriticalSection(&thread_cs);
		VisualNavigationFlag = 1;
		//LeaveCriticalSection(&thread_cs);
	}
	else
	{
		//�����߳̽�����־λ��1�������߳��˳�
		VisualNavigationFlag = 1;
		srvToClientDatas.contralSignal = FINISH_VISUALNAVIGATION_CONTROL;  //��ͻ��˷����źţ��ָ��������ư�ť
	}

	


	return 0;
}

//��е���˶������̺߳���
DWORD WINAPI ArmMotionFun(LPVOID lpParameter)
{
	ResumeThread(ArmAvoidThread);
	cout << "7. ArmMotionThread is running." << endl;

	//��е�۳�ʼ��
	cytonCommands = new EcCytonCommands;
	///�򿪻�����Զ�̿���
	EcString ipAddress = "127.0.0.1";
	EcString cytonVersion = "1500";
	EcString cytonDir = Ec::Application::getDataDirectory("cyton");
	if (cytonDir.empty())
	{
		cytonDir = ".";
	}
	cytonCommands->openNetwork(ipAddress);

	//�궨��ʼ����������е�۵���ʼλ��
	EcRealVector jointposition(7);		//���ڻ�е��λ�˿���
	jointposition.resize(7);		//��е�ۻص���ֱλ��
	//RC_CHECK(cytonCommands->MoveJointsExample(jointposition, 0.000001));



	cout << "����λ��" << endl;
	jointposition[1] = EcPi / 180 * 90;
	jointposition[3] = -EcPi / 180 * 80;
	jointposition[5] = -EcPi / 180 * 70;
	RC_CHECK(cytonCommands->MoveJointsExample(jointposition, .000001));
	cytonCommands->moveGripperExample(.0079);
	//�ӹؽڽǿ����л���λ�˿��ƣ��Ե�ǰλ�˽��г�ʼ��
	EcCoordinateSystemTransformation desiredPose;
	cytonCommands->changeToFrameEE(desiredPose);
	EcSLEEPMS(100);
	EcVector relaTransform(-0.01, 0.0, 0.0);		//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
	EcOrientation relaOriention;            //����ڵ�ǰ��צĩ������ϵZ-Y-X����ת��
	relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
	EcCoordinateSystemTransformation relativetrans(relaTransform, relaOriention);		//����ƽ����ת��
	desiredPose = desiredPose*relativetrans;
	RC_CHECK(cytonCommands->frameMovementExample(desiredPose));
	EcSLEEPMS(500);
	armCamFlag = 1;    //����˶�����ǰλ��

	//�ȴ������ȡ����λ����Ϣ�͵�ǰλ����Ϣ
	while (armCamFlag != 3 && armCamFlag != -1)
	{
		Sleep(100);
		if (ThreadsExitFlag == NAVIGATIONSTOP)
		{
			cytonCommands->closeNetwork();
			cout << "9. ArmMotionThread has finished." << endl;
			//EnterCriticalSection(&thread_cs);
			ArmMotionFlag = 1;
			//LeaveCriticalSection(&thread_cs);
			return 1;
		}
	}

	//cout << "�˶�����ǰλ��" << endl;
	//relaTransform.set(0.01, 0.05, -0.05);		//XYZƽ������
	//relaOriention.setFrom321Euler(-EcPi / 12, 0, EcPi / 36);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
	//relativetrans.outboardTransformBy(relaTransform, relaOriention);
	//RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
	//EcSLEEPMS(500);
	//armCamFlag = 3;    //����˶�����ǰλ��

	////�ȴ������ȡ��ǰλ����Ϣ
	//while (armCamFlag != 4 && armCamFlag != -1)
	//{
	//	Sleep(100);
	//	if (ThreadsExitFlag == NAVIGATIONSTOP)
	//	{
	//		cytonCommands->closeNetwork();
	//		cout << "9. ArmMotionThread has finished." << endl;
	//		//EnterCriticalSection(&thread_cs);
	//		ArmMotionFlag = 1;
	//		//LeaveCriticalSection(&thread_cs);
	//		return 1;
	//	}
	//}

	//������̬
	EnterCriticalSection(&thread_cs);
	fVe << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	LeaveCriticalSection(&thread_cs);		
	cout << "��е��λ�˵�����ʼ" << endl;
	while (ThreadsExitFlag != NAVIGATIONSTOP && armCamFlag != 4 && armCamFlag != -1)
	{
		//Sleep(100);
		cytonCommands->SetEEVelocity(fVe);
		//cout << fVe << endl << endl;
	}
	fVe << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	cytonCommands->SetEEVelocity(fVe);

	//����ץȡ
	if (armCamFlag==4)
	{
		cytonCommands->changeToFrameEE(desiredPose);
		relaTransform.set(0.0, 0.0, 0.0);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(EcPi / 2, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//EcSLEEPMS(2000);

		relaTransform.set(0.0, 0.02, 0.0);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(0, 0, EcPi / 60);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//////EcSLEEPMS(2000);

		relaTransform.set(0.0, 0.0, -0.130);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(500);

		cytonCommands->moveGripperExample(-0.005);
		EcSLEEPMS(500);

		////����
		relaTransform.set(0.0, 0.0, 0.15);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(500);

		relaTransform.set(0.0, 0.0, -0.13);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(-EcPi / 4, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(500);
	}
	
	cytonCommands->moveGripperExample(0.0079);
	EcSLEEPMS(500);

	relaTransform.set(0.0, 0.0, 0.1);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
	relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
	relativetrans.outboardTransformBy(relaTransform, relaOriention);
	RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
	RC_CHECK(cytonCommands->MoveJointsExample(jointposition, .000001));

	//desiredPose.setTranslation(EcVector(0.0192, 0.1532, 0.529));
	//
	////relaTransform.set(0.082, 0.148, 0.514);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
	//relaOriention.setFrom321Euler(-1.62967, -0.01254, -3.10882);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
	//desiredPose.setOrientation(relaOriention);
	////relativetrans.outboardTransformBy(relaTransform, relaOriention);
	////desiredPose = desiredPose * relativetrans;
	//RC_CHECK(cytonCommands->frameMovementExample(desiredPose));

	cytonCommands->closeNetwork();
	delete cytonCommands;
	cout << "9. ArmMotionThread has finished." << endl;
	//�߳̽�����־λ��1
	//EnterCriticalSection(&thread_cs);
	ArmMotionFlag = 1;
	//LeaveCriticalSection(&thread_cs);
	return 0;
}

//������߳�
DWORD WINAPI Camera(LPVOID lpParameter)
{

	cout << "9. CamThread is running." << endl;
	MmVisualServoAlgorithm visualServoAlgorithm;
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
        //�ֵ��۹�ϵ
		Mat eRc = Mat(3, 3, CV_32FC1); //�ֵ��۵���ת����
		eRc.at<float>(0, 0) = 0.02393;
		eRc.at<float>(0, 1) = -0.99788;
		eRc.at<float>(0, 2) = -0.0606;
		eRc.at<float>(1, 0) = -0.99967;
		eRc.at<float>(1, 1) = -0.02445;
		eRc.at<float>(1, 2) = 0.00793;
		eRc.at<float>(2, 0) = -0.00939;
		eRc.at<float>(2, 1) = 0.06039;
		eRc.at<float>(2, 2) = -0.99813;
		Mat cPe = Mat(3, 1, CV_32FC1); //�۵��ֵ�ƽ����������λΪm
		cPe.at<float>(0, 0) = 0.00392;
		cPe.at<float>(1, 0) = 0.07189;
		cPe.at<float>(2, 0) = -0.00785;

		//Ŀ��������ϵ�������������VISP,��λm
		vector<vpPoint> point;
		point.push_back(vpPoint(-0.025, -0.025, 0));
		point.push_back(vpPoint(0.025, -0.025, 0));
		point.push_back(vpPoint(0.025, 0.025, 0));
		point.push_back(vpPoint(-0.025, 0.025, 0));
		//Ŀ��������ϵ�������������OpenCV,����solvePnP����λmm
		vector<Point3f> point3D;
		point3D.push_back(Point3f(-25, -25, 0));
		point3D.push_back(Point3f(25, -25, 0));
		point3D.push_back(Point3f(25, 25, 0));
		point3D.push_back(Point3f(-25, 25, 0));
		//����͵�ǰ�������ϵ��Ŀ������ϵ����α任����
		vpHomogeneousMatrix cdMo, cMo, cMf;

		vpServo task;		//�Ӿ��ŷ�����
		task.setServo(vpServo::EYEINHAND_CAMERA);
		task.setForceInteractionMatrixComputation(vpServo::CURRENT);	//ʹ�õ�ǰ��������Ϣ
		task.setLambda(0.4);    //ϵ��

		//��ȡͼ��
		MmVisualServoBase baslerCam;	//�����
		vpImage<unsigned char> desireImage;		//��ǰ�Ҷ�ͼ��
		vpImage<unsigned char> currentImage;		//��ǰ�Ҷ�ͼ��
		Mat cvCurrentImage;
		vpImage<unsigned char> combinationImage;		//����ͼ���뵱ǰͼ������
		baslerCam.baslerOpen(currentImage);		//�����
		baslerCam.acquireBaslerImg(currentImage);

		vpFeaturePoint p[4], pd[4];		//��ǰ������4������������
		vector<vpDot2> desireDot(4), currentDot(4);			//����͵�ǰ��4��ɫ��

		//��ͬģʽ
		enum { DETECTION = 0, DESIRE = 1, CURRENT = 2, TRACKING = 3 };
		int mode = DETECTION;
		string msg = "Arm is moving to initial pose";		//ͼƬ����ʾ����ʾ��Ϣ
		int key;	//����ֵ

		DWORD tStrat, tEnd1, tEnd2;  //����ʱ��

		//������
		const std::string detectorName = "ORB";
		const std::string extractorName = "ORB";
		//Hamming distance must be used with ORB
		const std::string matcherName = "BruteForce-Hamming";
		vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
		vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);

		//��������ͼƬ�������������
		vpImageIo::read(desireImage, "desireImage.jpg");
		std::cout << "Reference keypoints=" << keypoint.buildReference(desireImage) << std::endl;


		combinationImage.resize(currentImage.getHeight(), 2 * currentImage.getWidth());
		combinationImage.insert(desireImage, vpImagePoint(0, 0));	//��߲�������ͼ��
		combinationImage.insert(currentImage, vpImagePoint(0, currentImage.getWidth()));	//�ұ߲��뵱ǰͼ��
		vpDisplayOpenCV ibvs(combinationImage, 0, 0, "IBVS");
		vpDisplay::display(combinationImage);
		vpDisplay::flush(combinationImage);
		
		//RANSAC��ʣ������������
		unsigned int nbMatchRANSAC = 0;
		//ѡ����ƥ��������
		//vpImagePoint iPref, iPcur;
		vector<cv::Point2f> iPrefSelect, iPcurSelect;

		while (true)
		{

			baslerCam.acquireBaslerImg(currentImage);

			//����ͼ��
			combinationImage.insert(currentImage, vpImagePoint(0, currentImage.getWidth()));
			vpDisplay::display(combinationImage);
			vpDisplay::displayLine(combinationImage, vpImagePoint(0, currentImage.getWidth()), vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//ͼ��ָ���
			vpDisplay::flush(combinationImage);

			//������ƥ��
			if (armCamFlag == 1)	//��е���˶�����ʼλ��
			{
				unsigned int nbMatch = keypoint.matchPoint(currentImage);
				std::cout << "Matches=" << nbMatch << std::endl;
				//RANSACȥ����ƥ��
				std::vector<vpImagePoint> iPref(nbMatch), iPcur(nbMatch); // Coordinates in pixels (for display only)
				std::vector<vpImagePoint> iPrefInliers, iPcurInliers;   //RANSACAɸѡ����ڵ�
				//! [Allocation]
				std::vector<double> mPref_x(nbMatch), mPref_y(nbMatch);
				std::vector<double> mPcur_x(nbMatch), mPcur_y(nbMatch);
				std::vector<bool> inliers(nbMatch);
				//! [Allocation]

				for (unsigned int i = 0; i < nbMatch; i++) 
				{
					keypoint.getMatchedPoints(i, iPref[i], iPcur[i]);
					//! [Pixel conversion]
					vpPixelMeterConversion::convertPoint(cam, iPref[i], mPref_x[i], mPref_y[i]);
					vpPixelMeterConversion::convertPoint(cam, iPcur[i], mPcur_x[i], mPcur_y[i]);
					//! [Pixel conversion]
				}

				double residual;
				vpHomography curHref;
				vpHomography::ransac(mPref_x, mPref_y, mPcur_x, mPcur_y, curHref, inliers, residual,
					(unsigned int)(mPref_x.size()*0.05), 8.0 / cam.get_px(), true);

				

				for (unsigned int i = 0; i < nbMatch; i++)
				{
					if (inliers[i] == true)
					{
						vpDisplay::displayLine(combinationImage, iPref[i], iPcur[i] + vpImagePoint(0, desireImage.getWidth()), vpColor::green);
						iPrefInliers.push_back(iPref[i]);
						iPcurInliers.push_back(iPcur[i]);
					}
						
				}

				nbMatchRANSAC = iPrefInliers.size();
				std::cout << "number of nbMatchRANSAC: " << nbMatchRANSAC << endl;
				if (nbMatchRANSAC >= 6)
				{
					//ѡȡ����������
					int interval = nbMatchRANSAC / 6;
					//int n = 0;
					for (unsigned int i = 0; i < nbMatchRANSAC; i += interval)  //i += interval
					{
						//keypoint.getMatchedPoints(i, iPref, iPcur);
						//vpDisplay::displayCross(desireImage, iPrefSel, 10, vpColor::white);
						//n++;
						//vpDisplay::displayLine(combinationImage, iPrefInliers[i], iPcurInliers[i] + vpImagePoint(0, desireImage.getWidth()), vpColor::green);
						iPrefSelect.push_back(Point2f(iPrefInliers[i].get_u(),iPrefInliers[i].get_v()));
						iPcurSelect.push_back(Point2f(iPcurInliers[i].get_u(), iPcurInliers[i].get_v()));
					}
					
					//����ͼ��
					//combinationImage.insert(desireImage, vpImagePoint(0, 0));	//��߲�������ͼ��
					//vpDisplay::display(combinationImage);
					//vpDisplay::displayLine(combinationImage, vpImagePoint(0, currentImage.getWidth()), vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//ͼ��ָ���
					vpImageConvert::convert(currentImage, cvCurrentImage);
					vpDisplay::flush(combinationImage);

					break;
				}
				else
				{
					std::cout << "û���㹻�����������ڸ���" << endl;
				}
			}

			key = 0xff & waitKey(30);
			if ((key & 255) == 27)   //  esc�˳���
			{
				break;
			}
		}

		//! [Create tracker]
		vpKltOpencv tracker;
		tracker.setMaxFeatures(20);
		tracker.setWindowSize(10);
		//! [Quality]
		tracker.setQuality(0.01);
		//! [Quality]
		tracker.setMinDistance(15);
		tracker.setHarrisFreeParameter(0.04);
		tracker.setBlockSize(9);
		tracker.setUseHarris(1);
		tracker.setPyramidLevels(3);
		//! [Create tracker]

		tracker.initTracking(cvCurrentImage, iPcurSelect);

		while (true)
		{
			baslerCam.acquireBaslerImg(currentImage);

			vpImageConvert::convert(currentImage, cvCurrentImage);
			tracker.track(cvCurrentImage);



			tracker.display(currentImage, vpColor::red);
			//����ͼ��
			combinationImage.insert(currentImage, vpImagePoint(0, currentImage.getWidth()));
			vpDisplay::display(combinationImage);
			vpDisplay::displayLine(combinationImage, vpImagePoint(0, currentImage.getWidth()), vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//ͼ��ָ���
			vpDisplay::flush(combinationImage);

			key = 0xff & waitKey(30);
			if ((key & 255) == 27)   //  esc�˳���
			{
				break;
			}

		}
		



		//	if (mode == DETECTION)
		//	{
		//		if (armCamFlag == 1)
		//		{
		//			msg = "Arm had arrived at initial pose,press 'd' to select desired feature points";
		//		}
		//		if (armCamFlag == 2)
		//		{
		//			msg = "got desired feature points,press 'c' to select feature point";
		//		}

		//		vpDisplay::display(currentImage);
		//		vpDisplay::displayText(currentImage, 10, 10, msg, vpColor::red);
		//		vpDisplay::flush(currentImage);		//��ʾͼ��
		//	}
		//	if (mode == DESIRE)
		//	{
		//		vpImageIo::read(desireImage, "desireImage.jpg");
		//		vpDisplayOpenCV desire(desireImage, 700, 0, "Desire camera image");
		//		
		//		//��������㣬��������������
		//		std::cout << "Reference keypoints=" << keypoint.buildReference(desireImage) << std::endl;


		//		vpDisplay::display(desireImage);
		//		vpDisplay::displayText(desireImage, 10, 10, "Select 4 dot to initalize the desired feature points", vpColor::red);
		//		vpDisplay::flush(desireImage);		//��ʾͼ��

		//		//����λ��������¼
		//		//��������ϵ�������������OpenCV,����solvePnP
		//		vector<Point2f> point2D;
		//		for (unsigned int i = 0; i < 4; i++)
		//		{
		//			desireDot[i].setGraphics(true);
		//			desireDot[i].initTracking(desireImage);
		//			vpDisplay::flush(desireImage);
		//			vpImagePoint dot;
		//			dot = desireDot[i].getCog();		//ɫ�����ĵ���������
		//			visualServoAlgorithm.pixelToImage(pd[i], cam, dot);		//��ȡɫ������ͼ�����굥λ���ף�����û�������Ϣ
		//			//dot = desireDot[i].getCog();
		//			//cout << "pd" << i << ": " << pd[i].get_x() << "  " << pd[i].get_y() << endl;
		//			//cout << "dot" << i << ": " << dot.get_u() << "  " << dot.get_v() << endl;
		//			point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
		//			//cout << "point2D " << point2D[i] << endl;
		//		}
		//		//solvePnP���cdMo
		//		Mat rvec = Mat::zeros(3, 1, CV_64FC1); //��ת����
		//		Mat rM; //��ת����
		//		Mat tvec;//ƽ������
		//		solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
		//		Rodrigues(rvec, rM);
		//		cout << "��ת����:" << endl;
		//		cout << rM << endl;
		//		cout << "ƽ������:" << endl;
		//		cout << tvec << endl;
		//		visualServoAlgorithm.setvpHomogeneousMatrix(cdMo, rM, tvec);
		//		cout << "����λ����α任����:" << endl;
		//		cout << cdMo << endl;
		//		//���������Ϣ
		//		for (int i = 0; i < 4; i++)
		//		{
		//			vpColVector cP;		//�������ϵ�����������ά����
		//			point[i].changeFrame(cdMo, cP);
		//			pd[i].set_Z(cP[2]);
		//			//�����Ϣ�̶����䣬��ǰ�����Ϣ��Ϊ���������Ϣ
		//			p[i].set_Z(cP[2]);
		//		}

		//		//����ˢͼģʽ���ȴ�����c
		//		mode = DETECTION;
		//		armCamFlag = 2;   //����������ȡ��ɣ���е���˶�����ǰλ��
		//		msg = "Arm is moving to current place";
		//	}
		//	if (mode == CURRENT)
		//	{
		//		vpDisplay::display(currentImage);
		//		vpDisplay::displayText(currentImage, 10, 10, "Select 4 dot to initalize the current feature points", vpColor::red);
		//		vpDisplay::flush(currentImage);		//��ʾͼ��

		//		//����λ��������¼
		//		vector<Point2f> point2D;
		//		for (unsigned int i = 0; i < 4; i++)
		//		{
		//			currentDot[i].setGraphics(true);
		//			currentDot[i].initTracking(currentImage);
		//			vpDisplay::flush(currentImage);
		//			vpImagePoint dot;
		//			dot = currentDot[i].getCog();
		//			visualServoAlgorithm.pixelToImage(p[i], cam, dot);	//��ȡɫ������ͼ�����굥λ���ף�����û�������Ϣ
		//			point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
		//		}

		//		/************************************************************************/
		//		/* �̶������Ϣע�͵����㵱ǰ���*/
		//		/************************************************************************/
		//		//Mat rvec = Mat::zeros(3, 1, CV_64FC1); //��ת����
		//		//Mat rM; //��ת����
		//		//Mat tvec;//ƽ������
		//		//solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
		//		//Rodrigues(rvec, rM);
		//		//cout << "��ת����:" << endl;
		//		//cout << rM << endl;
		//		//cout << "ƽ������:" << endl;
		//		//cout << tvec << endl;
		//		//visualServoAlgorithm.setvpHomogeneousMatrix(cMo, rM, tvec);
		//		//cout << "��ǰλ����α任����:" << endl;
		//		//cout << cMo << endl;
		//		////���������Ϣ
		//		//for (int i = 0; i < 4; i++)
		//		//{
		//		//	vpColVector cP;		//�������ϵ�����������ά����
		//		//	point[i].changeFrame(cMo, cP);
		//		//	p[i].set_Z(cP[2]);
		//		//}


		//		//д����������Ϣ������ͼ������������Ϣ����λΪm
		//		for (unsigned int i = 0; i < 4; i++)
		//		{
		//			task.addFeature(p[i], pd[i]);
		//			cout << "p" << i << ": " << p[i].get_x() << ", " << p[i].get_y() << ", " << p[i].get_Z() << endl;
		//			cout << "pd" << i << ": " << pd[i].get_x() << ", " << pd[i].get_y() << ", " << pd[i].get_Z() << endl;
		//		}
		//		mode = TRACKING;
		//		armCamFlag = 3;
		//		msg = "Start tracking,press esc to quit";
		//	}
		//	if (mode == TRACKING)
		//	{
		//		while (true)
		//		{
		//			tStrat = 0;
		//			tEnd1 = 0;
		//			tEnd2 = 0;
		//			tStrat = GetTickCount();

		//			baslerCam.acquireBaslerImg(currentImage);

		//			//��ȡ��е��ĩ���ڻ�����ϵ�µ�λ��
		//			EcReArray fTe;
		//			cytonCommands->GetPose(fTe);
		//			//cout << "fTe: " << endl << fTe << endl;

		//			//��ǰͼ������������ƥ��
		//			unsigned int nbMatch = keypoint.matchPoint(currentImage);
		//			std::cout << "Matches=" << nbMatch << std::endl;

		//			//����������
		//			vpImagePoint iPref, iPcur;
		//			for (unsigned int i = 0; i < nbMatch; i++)
		//			{
		//				keypoint.getMatchedPoints(i, iPref, iPcur);
		//				//! [Get matches]
		//				//! [Display matches]
		//				vpDisplay::displayCross(currentImage, iPcur, 5, vpColor::green);
		//				//! [Display matches]
		//			}

		//			vpDisplay::display(currentImage);
		//			//����λ��������¼
		//			vector<Point2f> point2D;
		//			for (int i = 0; i < 4; i++)
		//			{
		//				currentDot[i].track(currentImage);
		//				vpImagePoint dot;
		//				dot = currentDot[i].getCog();
		//				visualServoAlgorithm.pixelToImage(p[i], cam, dot);	//��ȡɫ������ͼ�����굥λ���ף�����û�������Ϣ
		//				point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
		//			}

		//			/************************************************************************/
		//			/* �̶������Ϣע�͵����㵱ǰ���*/
		//			/************************************************************************/
		//			//Mat rvec = Mat::zeros(3, 1, CV_64FC1); //��ת����
		//			//Mat rM; //��ת����
		//			//Mat tvec;//ƽ������
		//			//solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
		//			//Rodrigues(rvec, rM);
		//			///*cout << "��ת����:" << endl;
		//			//cout << rM << endl;
		//			//cout << "ƽ������:" << endl;
		//			//cout << tvec << endl;*/
		//			//visualServoAlgorithm.setvpHomogeneousMatrix(cMo, rM, tvec);
		//			///*cout << "��ǰλ����α任����:" << endl;
		//			//cout << cMo << endl;*/
		//			////���������Ϣ
		//			//for (int i = 0; i < 4; i++)
		//			//{
		//			//	vpColVector cP;		//�������ϵ�����������ά����
		//			//	point[i].changeFrame(cMo, cP);
		//			//	p[i].set_Z(cP[2]);
		//			//}
		//			
		//			//task.set_fVe(fMe);
		//			//cMf = cMe*(fMe.inverse());
		//			//task.set_cVf(cMf);

		//			//�����е��ĩ���˶��ٶ�
		//			vpColVector vpVc;
		//			vpVc = task.computeControlLaw();		//��е��ĩ���˶��ٶ�
		//			//cout << "vpVc" << endl << vpVc << endl;
		//			//�������ϵ��������ٶ�
		//			Mat eVe = Mat(6, 1, CV_32FC1);
		//			visualServoAlgorithm.linkageVTransmit(vpVc, eRc, cPe, eVe);
		//			//cout << "eVe:" << endl << eVe << endl;
		//			//��ĩ����צ����ϵ�µ�ĩ���ٶ�ת����������ϵ��
		//			//������ϵ��ĩ����צ����ת����
		//			Mat fRe = Mat(3, 3, CV_32FC1);
		//			for (int i = 0; i < 3; i++)
		//			{
		//				for (int j = 0; j < 3; j++)
		//				{
		//					fRe.at<float>(i, j) = fTe[i][j];
		//				}
		//			}
		//			//cout << "fRe:" << endl << fRe << endl;
		//			Mat matfVe = Mat(6, 1, CV_32FC1);
		//			visualServoAlgorithm.eVeTransmitTofVe(eVe, fRe, matfVe);

		//			for (unsigned int i = 0; i < 6; i++)
		//			{
		//				EnterCriticalSection(&thread_cs);
		//				fVe(i) = matfVe.at<float>(i, 0);
		//				LeaveCriticalSection(&thread_cs);					
		//			}

		//			tEnd1 = GetTickCount();
		//			printf("Use Time:%f\n", (tEnd1 - tStrat)*1.0 / 1000);

		//			//cout << "fVe:" << endl << fVe << endl << endl;
		//			//cout << "/////////////////////" << endl << endl;

		//			//����ͼ��켣
		//			visualServoAlgorithm.display_trajectory(currentImage, currentDot);
		//			vpServoDisplay::display(task, cam, currentImage, vpColor::green, vpColor::red);	//����������ĵ�ǰλ�ú�����λ��
		//			vpDisplay::flush(currentImage);

		//			tEnd2 = GetTickCount();
		//			printf("Use Time:%f\n\n\n\n", (tEnd2 - tEnd1)*1.0 / 1000);

		//			key = 0xff & waitKey(5);
		//			if ((key & 255) == 27 || (task.getError()).sumSquare() < 0.00005)   //  esc�˳���
		//			{
		//				if ((key & 255) == 27)
		//				{
		//					armCamFlag = -1;    //��е���˳��˶�
		//				}
		//				else
		//				{
		//					armCamFlag = 4;		//��е����̬����
		//				}
		//				mode = DETECTION;
		//				vpDisplay::getClick(currentImage, true);
		//				cout << "�����" << (task.getError()).sumSquare() << endl;
		//				//msg = "Exit tracking, press esc to quit";
		//				break;
		//			}
		//		}
		//		break;  //�˳�TRACKING
		//	}
		//}



		armCamFlag = -1;		//��е���˳��˶�
		baslerCam.close();
		task.kill();

	}
	catch (vpException &e) {
		std::cout << "Catch an exception: " << e << std::endl;
	}


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
	if (navigationAutoflag == NAVIGATIONSTART)
	{	
		srvToClientDatas.contralSignal = FINISH_AUTONAVIGATION_CONTROL;  //�����˷����Զ��������ƽ�����ָ��
	}
	else
	{
		srvToClientDatas.contralSignal = FINISHARMCONTROL;  //�����˷��ͻ�е���˶���ɵ�ָ��	
	}

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
