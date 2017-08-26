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
HANDLE CamThread;				    //�׽������ݽ����߳�
HANDLE ShowArmCameraThread;         //�ֱ�����ͷͼ����ʾ

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
	cytonCommands->moveGripperExample(.0078);
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

	//������̬
	EnterCriticalSection(&thread_cs);
	fVe << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	LeaveCriticalSection(&thread_cs);		
	cout << "��е��λ�˵�����ʼ" << endl;
	while (ThreadsExitFlag != NAVIGATIONSTOP && armCamFlag != 4 && armCamFlag != -1)
	{	
		cytonCommands->SetEEVelocity(fVe);
		//Sleep(50);
		cout << fVe << endl << endl;
	}
	fVe << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	cytonCommands->SetEEVelocity(fVe);

	//����ץȡ
	if (armCamFlag==4)
	{
		cytonCommands->changeToFrameEE(desiredPose);
		relaTransform.set(0.04, 0.005, -0.145);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//EcSLEEPMS(2000);

		//relaTransform.set(0.0, 0.02, 0.0);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		//relaOriention.setFrom321Euler(0, 0, EcPi / 60);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		//relativetrans.outboardTransformBy(relaTransform, relaOriention);
		////desiredPose = desiredPose * relativetrans;
		//RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		////////EcSLEEPMS(2000);

		//relaTransform.set(0.0, 0.0, -0.130);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		//relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		//relativetrans.outboardTransformBy(relaTransform, relaOriention);
		////desiredPose = desiredPose * relativetrans;
		//RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//EcSLEEPMS(500);

		cytonCommands->moveGripperExample(-0.005);
		EcSLEEPMS(500);

		//////����
		relaTransform.set(0.0, 0.0, 0.15);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//EcSLEEPMS(500);

		relaTransform.set(0.0, 0.0, -0.14);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
		relaOriention.setFrom321Euler(-EcPi / 8, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//EcSLEEPMS(500);
	}
	
	cytonCommands->moveGripperExample(0.0078);
	EcSLEEPMS(500);

	relaTransform.set(0.0, 0.02, 0.05);				//����ڵ�ǰ��צĩ������ϵXYZ��ƽ����
	relaOriention.setFrom321Euler(0, 0, 0);    //��Z-Y-Xŷ���ǶԵ�ǰĩ����צ��̬������ת
	relativetrans.outboardTransformBy(relaTransform, relaOriention);
	RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
	RC_CHECK(cytonCommands->MoveJointsExample(jointposition, .000001));

	jointposition.resize(7);		//�ؽ�ֵ����
	cout << "�ص�����λ��" << endl;
	jointposition[1] = EcPi / 180 * 90;
	jointposition[3] = -EcPi / 180 * 80;
	jointposition[5] = -EcPi / 180 * 70;
	RC_CHECK(cytonCommands->MoveJointsExample(jointposition, .000001));


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
		//��������ڲ���
		double camIntrinsic[9] = { 1009.417988415312, 0, 329.3878251889672,
		0, 1008.286720339768, 234.3207569556778,
		0, 0, 1 };
		visualServoAlgorithm.setCamIntrinsic(camIntrinsic);				//������ڲ���ת����OpenCV��ʽ��
		vpCameraParameters cam(camIntrinsic[0], camIntrinsic[4], camIntrinsic[2], camIntrinsic[5]); //��������ڲ���px py u0 v0
		//������������
		double camDistortion[5] = { -0.3752770440859098, 0.04860296472141773, -0.0001071908873890624, -0.0007867887466124389, 1.639413740673039 };
		visualServoAlgorithm.setCamDistortion(camDistortion);
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

		vpServo task;		//�Ӿ��ŷ�����
		task.setServo(vpServo::EYEINHAND_CAMERA);
		task.setForceInteractionMatrixComputation(vpServo::CURRENT);	//ʹ�õ�ǰ��������Ϣ
		task.setLambda(0.4);    //����ϵ��
		//vpAdaptiveGain lambda(1, 0.4, 30);
		//task.setLambda(lambda);

		//��ȡͼ��
		MmVisualServoBase baslerCam;	//�����	
		vpImage<unsigned char> desireImage;		//��ǰ�Ҷ�ͼ��
		vpImage<unsigned char> currentImage;		//��ǰ�Ҷ�ͼ��
		Mat cvCurrentImage;		//��ǰͼ���OpenCV��ʽ���������������
		vpImage<unsigned char> combinationImage;		//����ͼ���뵱ǰͼ������
		baslerCam.baslerOpen(currentImage);		//�����
		baslerCam.acquireBaslerImg(currentImage);

		//���ٵ����������
		const int nbPointSelect = 4;
		vector<vpImagePoint> ipd(nbPointSelect);
		ipd[0].set_uv(170, 90);
		ipd[1].set_uv(470, 90); 
		ipd[2].set_uv(170, 390); 
		ipd[3].set_uv(470, 390);
		vpFeaturePoint p[nbPointSelect], pd[nbPointSelect];		//��ǰ���������������꣬���������Ϣ����λ��m
		//���������Ϣ
		for (int i = 0; i < nbPointSelect; i++)
		{
			double ip_x = 0, ip_y = 0;
			vpPixelMeterConversion::convertPoint(cam, ipd[i], ip_x, ip_y);
			pd[i].set_x(ip_x);
			pd[i].set_y(ip_y);
			p[i].set_Z(0.21);
			pd[i].set_Z(0.21);
			//���������
			task.addFeature(p[i], pd[i]);
		}

		int key;	//����ֵ

		//DWORD tStrat, tEnd1, tEnd2;  //����ʱ��

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
		vpImagePoint offset(0, currentImage.getWidth());
		combinationImage.insert(currentImage, offset);	//�ұ߲��뵱ǰͼ��
		vpDisplay::displayLine(combinationImage, offset, vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//ͼ��ָ���
		vpDisplayOpenCV ibvs(combinationImage, 0, 0, "IBVS");
		vpDisplay::display(combinationImage);
		vpDisplay::flush(combinationImage);

		while (true)
		{

			//RANSAC��ʣ������������
			unsigned int nbMatchRANSAC = 0;
			//ѡ����ƥ��������
			//vector<cv::Point2f>  iPcurSelect(nbPointSelect), iPrefSelect(nbPointSelect);		//��λ������

			baslerCam.acquireBaslerImg(currentImage);
			combinationImage.insert(currentImage, offset);
			vpDisplay::display(combinationImage);
			vpDisplay::displayLine(combinationImage, offset, vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//ͼ��ָ���

			//������ƥ��
			if (armCamFlag == 1 || armCamFlag == 3)	//��е���˶�����ʼλ��,���������½������¸���
			{
				armCamFlag = 3;
				//��ȡ��е��ĩ���ڻ�����ϵ�µ�λ��
				EcReArray fTe;
				cytonCommands->GetPose(fTe);

				unsigned int nbMatch = keypoint.matchPoint(currentImage);
				//std::cout << "Matches=" << nbMatch << std::endl;

				if (nbMatch<5)
				{
					cout << "Number of matches points are less than 5" << endl;
					break;
				}


				//RANSACȥ����ƥ��
				std::vector<vpImagePoint> iPref(nbMatch), iPcur(nbMatch); // Coordinates in pixels (for display only)
				//std::vector<vpImagePoint> iPrefInliers(nbMatch), iPcurInliers(nbMatch);   //RANSACAɸѡ����ڵ�
				//! [Allocation]
				std::vector<double> mPref_x(nbMatch), mPref_y(nbMatch);
				std::vector<double> mPcur_x(nbMatch), mPcur_y(nbMatch);
				//std::vector<double> mPrefInliers_x(nbMatch), mPrefInliers_y(nbMatch);
				std::vector<bool> inliers(nbMatch);
				//! [Allocation]

				for (unsigned int i = 0; i < nbMatch; i++)
				{
					keypoint.getMatchedPoints(i, iPref[i], iPcur[i]);
					//! [Pixel conversion]��λ������ת������
					vpPixelMeterConversion::convertPoint(cam, iPref[i], mPref_x[i], mPref_y[i]);
					vpPixelMeterConversion::convertPoint(cam, iPcur[i], mPcur_x[i], mPcur_y[i]);
					//! [Pixel conversion]
				}

				double residual;
				vpHomography curHref;
				bool calculate_fVe = 0;
				//calculate_fVe = vpHomography::ransac(mPref_x, mPref_y, mPcur_x, mPcur_y, curHref, inliers, residual,
					//(unsigned int)(mPref_x.size()*0.25), 1.0 / cam.get_px(), true);

				// ����robust�㷨���Ƶ�Ӧ�Ծ��󣬽���洢��curHref
				vpHomography::robust(mPref_x, mPref_y, mPcur_x, mPcur_y, curHref, inliers, residual,
					0.2, 8, true);

				vector<vpImagePoint> pa2(4);
				//if (calculate_fVe == true)
				{
					int ii = 0;
					for (int i = 0; i < 4; i++)
					{
						vpFeaturePoint pa;
						pa = visualServoAlgorithm.project(curHref, pd[ii]);
						p[ii].set_x(pa.get_x());
						p[ii].set_y(pa.get_y());
						//cout << "p" << ii << ": " << p[ii].get_x() << ",  " << p[ii].get_y() << endl;
						
						pa2[ii] = vpHomography::project(cam, curHref, ipd[ii]);
						//cout << "pa2" << ii << ": " << pa2[ii] << endl;

						ii++;
					}

					//�����е��ĩ���˶��ٶ�
					vpColVector vpVc;
					vpVc = task.computeControlLaw();		//�������ϵ�˶��ٶ�
					//cout << "vpVc" << endl << vpVc << endl;
					//��צĩ������ϵ����צĩ�˵��ٶ�
					Mat eVe = Mat(6, 1, CV_32FC1);
					visualServoAlgorithm.linkageVTransmit(vpVc, eRc, cPe, eVe);
					//cout << "eVe:" << endl << eVe << endl;
					//��ĩ����צ����ϵ�µ�ĩ���ٶ�ת����������ϵ��
					//������ϵ��ĩ����צ����ת����
					Mat fRe = Mat(3, 3, CV_32FC1);
					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
						{
							fRe.at<float>(i, j) = fTe[i][j];
						}
					}
					//cout << "fRe:" << endl << fRe << endl;
					Mat matfVe = Mat(6, 1, CV_32FC1);
					visualServoAlgorithm.eVeTransmitTofVe(eVe, fRe, matfVe);
					//������ϵ����צĩ���ٶ�
					for (unsigned int i = 0; i < 6; i++)
					{
						EnterCriticalSection(&thread_cs);
						fVe(i) = matfVe.at<float>(i, 0);
						LeaveCriticalSection(&thread_cs);
					}

					for (unsigned int i = 0; i < nbMatch; i++)
					{
						if (inliers[i]==true)
						{
							vpDisplay::displayLine(combinationImage, iPref[i], vpImagePoint(iPcur[i].get_v(), iPcur[i].get_u() + desireImage.getWidth()), vpColor::green);
						}
					}
					//����4�����붨λ�㣬��4����ǰ��λ��
					for (int i = 0; i < 4; i++)
					{
						vpDisplay::displayCross(combinationImage, vpImagePoint(ipd[i].get_v(), ipd[i].get_u() + desireImage.getWidth()), 15, vpColor::red);
						vpDisplay::displayCross(combinationImage, vpImagePoint(pa2[i].get_v(), pa2[i].get_u() + desireImage.getWidth()), 15, vpColor::blue);
					}

					if ((task.getError()).sumSquare() < 0.0005)
					{
						armCamFlag = 4;		//��е����̬����

						cout << "�����" << (task.getError()).sumSquare() << endl;
						vpDisplay::getClick(combinationImage, true);

						break;
					}
				}

			}

			//����ͼ��
			vpDisplay::flush(combinationImage);

			key = 0xff & waitKey(30);
			if ((key & 255) == 27 )   //  esc�˳���  || (task.getError()).sumSquare() < 0.0001
			{
				//if ((key & 255) == 27)
				//{
					armCamFlag = -1;    //��е���˳��˶�
				//}
				//else
				//{
				//	armCamFlag = 4;		//��е����̬����
				//}

				//cout << "�����" << (task.getError()).sumSquare() << endl;
				//vpDisplay::getClick(combinationImage, true);

				break;
			}
		}
		
		armCamFlag = -1;		//��е���˳��˶�
		baslerCam.close();
		task.kill();
	}

	catch (vpException &e) 
	{
		std::cout << "Catch an exception: " << e << std::endl;
		armCamFlag = -1;		//��е���˳��˶�
	}


	cout << "10.CamThread has finished." << endl;
	return 0;
}

//��е������ͷͼ����ʾ
DWORD WINAPI ShowArmCameraFun(LPVOID lpParameter)
{
	cout << "ShowArmCameraThread is running." << endl;

	MmVisualServoBase baslerCam;	//�����
	vpImage<unsigned char> currentImage;		//��ǰ�Ҷ�ͼ��
	baslerCam.baslerOpen(currentImage);		//�����
	baslerCam.acquireBaslerImg(currentImage);

	vpDisplayOpenCV armCamera(currentImage, 0, 0, "image of arm cammera");
	int key;

	while (true)
	{
		baslerCam.acquireBaslerImg(currentImage);

		vpDisplay::display(currentImage);
		vpDisplay::flush(currentImage);
		
		key = 0xff & waitKey(50);
		if ((key & 255) == 27 || ThreadsExitFlag==ARM_CAMERA_SHOW_CLOSE)
		{
			break;
		}
		if ((key & 255)== 's')
		{
			vpImageIo::write(currentImage, "desireImageNew.jpg");
			cout << "save image" << endl;
		}

	}

	baslerCam.close();
	cout << "ShowArmCameraThread has been closed." << endl;
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

//�ֱ�����ͷͼ���ȡ�̺߳�������
void startArmCameraShow()
{
	VisualNavigationFlag = 0;
	ArmMotionFlag = 0;
	ThreadsExitFlag = 0;
	navigationAutoflag = ARMCONTROL;

	ShowArmCameraThread = CreateThread(NULL, 0, ShowArmCameraFun, NULL, 0, NULL);
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
			case ARM_CAMERA_SHOW:startArmCameraShow(); break;      //��е������ͷͼ����ʾ
			case ARM_CAMERA_SHOW_CLOSE:ThreadsExitFlag = ARM_CAMERA_SHOW_CLOSE; break; //�ֱ�����ͷ��ʾ�ر�

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
