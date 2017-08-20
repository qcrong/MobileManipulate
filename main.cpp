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

using namespace std;

//全局变量
//volatile来告诉编译器这个全局变量是易变的，让编译器不要对这个变量进行优化
//VisualNavigationThread线程运行标志位，当该线程结束时标志位置1
volatile int VisualNavigationFlag = 0;
//ArmMotionThread线程运行标志位，当该线程结束时标志位置1
volatile int ArmMotionFlag = 0;
//客户端按下停止按钮时，ThreadsExitFlag置1，机器人线程退出
volatile int ThreadsExitFlag = 0;
//机械臂和相机线程的同步控制标志位，相机运动到位时置1，相机处理完成置2
volatile int armCamFlag = 0;
//视觉计算出来的基坐标系下机械臂末端运动速度
VectorXd fVe(6);
//机械臂控制类指针，作为全局变量方便视觉线程获取机械臂信息
EcCytonCommands *cytonCommands;

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
		//针对单独调试，避障线程结束标志位置1，避障线程退出
		//EnterCriticalSection(&thread_cs);
		VisualNavigationFlag = 1;
		srvToClientDatas.contralSignal = FINISH_RFID_CONTROL;  //向客户端返回信号，恢复单独控制按钮
		//LeaveCriticalSection(&thread_cs);
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


	if (navigationAutoflag == NAVIGATIONSTART)
	{
		//避障线程结束标志位置1，避障线程退出
		//EnterCriticalSection(&thread_cs);
		VisualNavigationFlag = 1;
		//LeaveCriticalSection(&thread_cs);
	}
	else
	{
		//避障线程结束标志位置1，避障线程退出
		VisualNavigationFlag = 1;
		srvToClientDatas.contralSignal = FINISH_VISUALNAVIGATION_CONTROL;  //向客户端返回信号，恢复单独控制按钮
	}

	


	return 0;
}

//机械臂运动控制线程函数
DWORD WINAPI ArmMotionFun(LPVOID lpParameter)
{
	ResumeThread(ArmAvoidThread);
	cout << "7. ArmMotionThread is running." << endl;

	//机械臂初始化
	cytonCommands = new EcCytonCommands;
	///打开机器人远程控制
	EcString ipAddress = "127.0.0.1";
	EcString cytonVersion = "1500";
	EcString cytonDir = Ec::Application::getDataDirectory("cyton");
	if (cytonDir.empty())
	{
		cytonDir = ".";
	}
	cytonCommands->openNetwork(ipAddress);

	//标定初始化，调整机械臂到初始位置
	EcRealVector jointposition(7);		//用于机械臂位姿控制
	jointposition.resize(7);		//机械臂回到垂直位置
	//RC_CHECK(cytonCommands->MoveJointsExample(jointposition, 0.000001));



	cout << "理想位姿" << endl;
	jointposition[1] = EcPi / 180 * 90;
	jointposition[3] = -EcPi / 180 * 80;
	jointposition[5] = -EcPi / 180 * 70;
	RC_CHECK(cytonCommands->MoveJointsExample(jointposition, .000001));
	cytonCommands->moveGripperExample(.0079);
	//从关节角控制切换到位姿控制，对当前位姿进行初始化
	EcCoordinateSystemTransformation desiredPose;
	cytonCommands->changeToFrameEE(desiredPose);
	EcSLEEPMS(100);
	EcVector relaTransform(-0.01, 0.0, 0.0);		//相对于当前手爪末端坐标系XYZ的平移量
	EcOrientation relaOriention;            //相对于当前手爪末端坐标系Z-Y-X的旋转量
	relaOriention.setFrom321Euler(0, 0, 0);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
	EcCoordinateSystemTransformation relativetrans(relaTransform, relaOriention);		//设置平移旋转量
	desiredPose = desiredPose*relativetrans;
	RC_CHECK(cytonCommands->frameMovementExample(desiredPose));
	EcSLEEPMS(500);
	armCamFlag = 1;    //相机运动到当前位姿

	//等待相机获取理想位姿信息和当前位姿信息
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

	//cout << "运动到当前位姿" << endl;
	//relaTransform.set(0.01, 0.05, -0.05);		//XYZ平移向量
	//relaOriention.setFrom321Euler(-EcPi / 12, 0, EcPi / 36);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
	//relativetrans.outboardTransformBy(relaTransform, relaOriention);
	//RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
	//EcSLEEPMS(500);
	//armCamFlag = 3;    //相机运动到当前位姿

	////等待相机获取当前位姿信息
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

	//调整姿态
	EnterCriticalSection(&thread_cs);
	fVe << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	LeaveCriticalSection(&thread_cs);		
	cout << "机械臂位姿调整开始" << endl;
	while (ThreadsExitFlag != NAVIGATIONSTOP && armCamFlag != 4 && armCamFlag != -1)
	{
		//Sleep(100);
		cytonCommands->SetEEVelocity(fVe);
		//cout << fVe << endl << endl;
	}
	fVe << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	cytonCommands->SetEEVelocity(fVe);

	//进行抓取
	if (armCamFlag==4)
	{
		cytonCommands->changeToFrameEE(desiredPose);
		relaTransform.set(0.0, 0.0, 0.0);				//相对于当前手爪末端坐标系XYZ的平移量
		relaOriention.setFrom321Euler(EcPi / 2, 0, 0);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//EcSLEEPMS(2000);

		relaTransform.set(0.0, 0.02, 0.0);				//相对于当前手爪末端坐标系XYZ的平移量
		relaOriention.setFrom321Euler(0, 0, EcPi / 60);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		//////EcSLEEPMS(2000);

		relaTransform.set(0.0, 0.0, -0.130);				//相对于当前手爪末端坐标系XYZ的平移量
		relaOriention.setFrom321Euler(0, 0, 0);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(500);

		cytonCommands->moveGripperExample(-0.005);
		EcSLEEPMS(500);

		////放置
		relaTransform.set(0.0, 0.0, 0.15);				//相对于当前手爪末端坐标系XYZ的平移量
		relaOriention.setFrom321Euler(0, 0, 0);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(500);

		relaTransform.set(0.0, 0.0, -0.13);				//相对于当前手爪末端坐标系XYZ的平移量
		relaOriention.setFrom321Euler(-EcPi / 4, 0, 0);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
		relativetrans.outboardTransformBy(relaTransform, relaOriention);
		//desiredPose = desiredPose * relativetrans;
		RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
		EcSLEEPMS(500);
	}
	
	cytonCommands->moveGripperExample(0.0079);
	EcSLEEPMS(500);

	relaTransform.set(0.0, 0.0, 0.1);				//相对于当前手爪末端坐标系XYZ的平移量
	relaOriention.setFrom321Euler(0, 0, 0);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
	relativetrans.outboardTransformBy(relaTransform, relaOriention);
	RC_CHECK(cytonCommands->frameMovementExample(desiredPose * relativetrans));
	RC_CHECK(cytonCommands->MoveJointsExample(jointposition, .000001));

	//desiredPose.setTranslation(EcVector(0.0192, 0.1532, 0.529));
	//
	////relaTransform.set(0.082, 0.148, 0.514);				//相对于当前手爪末端坐标系XYZ的平移量
	//relaOriention.setFrom321Euler(-1.62967, -0.01254, -3.10882);    //绕Z-Y-X欧拉角对当前末端手爪姿态进行旋转
	//desiredPose.setOrientation(relaOriention);
	////relativetrans.outboardTransformBy(relaTransform, relaOriention);
	////desiredPose = desiredPose * relativetrans;
	//RC_CHECK(cytonCommands->frameMovementExample(desiredPose));

	cytonCommands->closeNetwork();
	delete cytonCommands;
	cout << "9. ArmMotionThread has finished." << endl;
	//线程结束标志位置1
	//EnterCriticalSection(&thread_cs);
	ArmMotionFlag = 1;
	//LeaveCriticalSection(&thread_cs);
	return 0;
}

//摄像机线程
DWORD WINAPI Camera(LPVOID lpParameter)
{

	cout << "9. CamThread is running." << endl;
	MmVisualServoAlgorithm visualServoAlgorithm;
	try
	{
		vpCameraParameters cam(1017.13738, 1016.70876, 332.59991, 238.92479); //摄像相机内参数px py u0 v0
		double camIntrinsic[9] = { 1017.13738, 0, 332.59991,
			0, 1016.70876, 238.92479,
			0, 0, 1 };
		visualServoAlgorithm.setCamIntrinsic(camIntrinsic);				//摄像机内参数转换到OpenCV格式下
		cout << visualServoAlgorithm.cam_intrinsic_matrix << endl << endl;
		double camDistortion[5] = { -0.4034783300939766, 0.6235344618772364, -0.0003784404964069526, -0.0006034915824095659, -1.59162547482239 };	//摄像机畸变参数
		visualServoAlgorithm.setCamDistortion(camDistortion);
		cout << visualServoAlgorithm.cam_distortion_matrix << endl;
        //手到眼关系
		Mat eRc = Mat(3, 3, CV_32FC1); //手到眼的旋转矩阵
		eRc.at<float>(0, 0) = 0.02393;
		eRc.at<float>(0, 1) = -0.99788;
		eRc.at<float>(0, 2) = -0.0606;
		eRc.at<float>(1, 0) = -0.99967;
		eRc.at<float>(1, 1) = -0.02445;
		eRc.at<float>(1, 2) = 0.00793;
		eRc.at<float>(2, 0) = -0.00939;
		eRc.at<float>(2, 1) = 0.06039;
		eRc.at<float>(2, 2) = -0.99813;
		Mat cPe = Mat(3, 1, CV_32FC1); //眼到手的平移向量，单位为m
		cPe.at<float>(0, 0) = 0.00392;
		cPe.at<float>(1, 0) = 0.07189;
		cPe.at<float>(2, 0) = -0.00785;

		//目标物坐标系下特征点的坐标VISP,单位m
		vector<vpPoint> point;
		point.push_back(vpPoint(-0.025, -0.025, 0));
		point.push_back(vpPoint(0.025, -0.025, 0));
		point.push_back(vpPoint(0.025, 0.025, 0));
		point.push_back(vpPoint(-0.025, 0.025, 0));
		//目标物坐标系下特征点的坐标OpenCV,用于solvePnP，单位mm
		vector<Point3f> point3D;
		point3D.push_back(Point3f(-25, -25, 0));
		point3D.push_back(Point3f(25, -25, 0));
		point3D.push_back(Point3f(25, 25, 0));
		point3D.push_back(Point3f(-25, 25, 0));
		//理想和当前相机坐标系到目标坐标系的其次变换矩阵
		vpHomogeneousMatrix cdMo, cMo, cMf;

		vpServo task;		//视觉伺服处理
		task.setServo(vpServo::EYEINHAND_CAMERA);
		task.setForceInteractionMatrixComputation(vpServo::CURRENT);	//使用当前点的深度信息
		task.setLambda(0.4);    //系数

		//获取图像
		MmVisualServoBase baslerCam;	//相机类
		vpImage<unsigned char> desireImage;		//当前灰度图像
		vpImage<unsigned char> currentImage;		//当前灰度图像
		Mat cvCurrentImage;
		vpImage<unsigned char> combinationImage;		//理想图像与当前图像的组合
		baslerCam.baslerOpen(currentImage);		//打开相机
		baslerCam.acquireBaslerImg(currentImage);

		vpFeaturePoint p[4], pd[4];		//当前和理想4个特征点坐标
		vector<vpDot2> desireDot(4), currentDot(4);			//理想和当前的4个色块

		//不同模式
		enum { DETECTION = 0, DESIRE = 1, CURRENT = 2, TRACKING = 3 };
		int mode = DETECTION;
		string msg = "Arm is moving to initial pose";		//图片上显示的提示信息
		int key;	//按键值

		DWORD tStrat, tEnd1, tEnd2;  //计算时间

		//特征点
		const std::string detectorName = "ORB";
		const std::string extractorName = "ORB";
		//Hamming distance must be used with ORB
		const std::string matcherName = "BruteForce-Hamming";
		vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
		vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);

		//读入理想图片，并检测特征点
		vpImageIo::read(desireImage, "desireImage.jpg");
		std::cout << "Reference keypoints=" << keypoint.buildReference(desireImage) << std::endl;


		combinationImage.resize(currentImage.getHeight(), 2 * currentImage.getWidth());
		combinationImage.insert(desireImage, vpImagePoint(0, 0));	//左边插入理想图像
		combinationImage.insert(currentImage, vpImagePoint(0, currentImage.getWidth()));	//右边插入当前图像
		vpDisplayOpenCV ibvs(combinationImage, 0, 0, "IBVS");
		vpDisplay::display(combinationImage);
		vpDisplay::flush(combinationImage);
		
		//RANSAC后剩余的特征点个数
		unsigned int nbMatchRANSAC = 0;
		//选择后的匹配特征点
		//vpImagePoint iPref, iPcur;
		vector<cv::Point2f> iPrefSelect, iPcurSelect;

		while (true)
		{

			baslerCam.acquireBaslerImg(currentImage);

			//更新图像
			combinationImage.insert(currentImage, vpImagePoint(0, currentImage.getWidth()));
			vpDisplay::display(combinationImage);
			vpDisplay::displayLine(combinationImage, vpImagePoint(0, currentImage.getWidth()), vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//图像分割线
			vpDisplay::flush(combinationImage);

			//特征点匹配
			if (armCamFlag == 1)	//机械臂运动到初始位姿
			{
				unsigned int nbMatch = keypoint.matchPoint(currentImage);
				std::cout << "Matches=" << nbMatch << std::endl;
				//RANSAC去除误匹配
				std::vector<vpImagePoint> iPref(nbMatch), iPcur(nbMatch); // Coordinates in pixels (for display only)
				std::vector<vpImagePoint> iPrefInliers, iPcurInliers;   //RANSACA筛选后的内点
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
					//选取部分特征点
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
					
					//更新图像
					//combinationImage.insert(desireImage, vpImagePoint(0, 0));	//左边插入理想图像
					//vpDisplay::display(combinationImage);
					//vpDisplay::displayLine(combinationImage, vpImagePoint(0, currentImage.getWidth()), vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//图像分割线
					vpImageConvert::convert(currentImage, cvCurrentImage);
					vpDisplay::flush(combinationImage);

					break;
				}
				else
				{
					std::cout << "没有足够的特征点用于跟踪" << endl;
				}
			}

			key = 0xff & waitKey(30);
			if ((key & 255) == 27)   //  esc退出键
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
			//更新图像
			combinationImage.insert(currentImage, vpImagePoint(0, currentImage.getWidth()));
			vpDisplay::display(combinationImage);
			vpDisplay::displayLine(combinationImage, vpImagePoint(0, currentImage.getWidth()), vpImagePoint(currentImage.getHeight(), currentImage.getWidth()), vpColor::white, 2);//图像分割线
			vpDisplay::flush(combinationImage);

			key = 0xff & waitKey(30);
			if ((key & 255) == 27)   //  esc退出键
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
		//		vpDisplay::flush(currentImage);		//显示图像
		//	}
		//	if (mode == DESIRE)
		//	{
		//		vpImageIo::read(desireImage, "desireImage.jpg");
		//		vpDisplayOpenCV desire(desireImage, 700, 0, "Desire camera image");
		//		
		//		//检测特征点，并输出特征点个数
		//		std::cout << "Reference keypoints=" << keypoint.buildReference(desireImage) << std::endl;


		//		vpDisplay::display(desireImage);
		//		vpDisplay::displayText(desireImage, 10, 10, "Select 4 dot to initalize the desired feature points", vpColor::red);
		//		vpDisplay::flush(desireImage);		//显示图像

		//		//理想位置特征记录
		//		//像素坐标系下特征点的坐标OpenCV,用于solvePnP
		//		vector<Point2f> point2D;
		//		for (unsigned int i = 0; i < 4; i++)
		//		{
		//			desireDot[i].setGraphics(true);
		//			desireDot[i].initTracking(desireImage);
		//			vpDisplay::flush(desireImage);
		//			vpImagePoint dot;
		//			dot = desireDot[i].getCog();		//色块重心的像素坐标
		//			visualServoAlgorithm.pixelToImage(pd[i], cam, dot);		//获取色块重心图像坐标单位是米，这里没有深度信息
		//			//dot = desireDot[i].getCog();
		//			//cout << "pd" << i << ": " << pd[i].get_x() << "  " << pd[i].get_y() << endl;
		//			//cout << "dot" << i << ": " << dot.get_u() << "  " << dot.get_v() << endl;
		//			point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
		//			//cout << "point2D " << point2D[i] << endl;
		//		}
		//		//solvePnP求解cdMo
		//		Mat rvec = Mat::zeros(3, 1, CV_64FC1); //旋转向量
		//		Mat rM; //旋转矩阵
		//		Mat tvec;//平移向量
		//		solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
		//		Rodrigues(rvec, rM);
		//		cout << "旋转矩阵:" << endl;
		//		cout << rM << endl;
		//		cout << "平移向量:" << endl;
		//		cout << tvec << endl;
		//		visualServoAlgorithm.setvpHomogeneousMatrix(cdMo, rM, tvec);
		//		cout << "理想位置其次变换矩阵:" << endl;
		//		cout << cdMo << endl;
		//		//计算深度信息
		//		for (int i = 0; i < 4; i++)
		//		{
		//			vpColVector cP;		//相机坐标系下特征点的三维坐标
		//			point[i].changeFrame(cdMo, cP);
		//			pd[i].set_Z(cP[2]);
		//			//深度信息固定不变，当前深度信息设为理想深度信息
		//			p[i].set_Z(cP[2]);
		//		}

		//		//返回刷图模式，等待按键c
		//		mode = DETECTION;
		//		armCamFlag = 2;   //理想特征提取完成，机械臂运动到当前位姿
		//		msg = "Arm is moving to current place";
		//	}
		//	if (mode == CURRENT)
		//	{
		//		vpDisplay::display(currentImage);
		//		vpDisplay::displayText(currentImage, 10, 10, "Select 4 dot to initalize the current feature points", vpColor::red);
		//		vpDisplay::flush(currentImage);		//显示图像

		//		//理想位置特征记录
		//		vector<Point2f> point2D;
		//		for (unsigned int i = 0; i < 4; i++)
		//		{
		//			currentDot[i].setGraphics(true);
		//			currentDot[i].initTracking(currentImage);
		//			vpDisplay::flush(currentImage);
		//			vpImagePoint dot;
		//			dot = currentDot[i].getCog();
		//			visualServoAlgorithm.pixelToImage(p[i], cam, dot);	//获取色块重心图像坐标单位是米，这里没有深度信息
		//			point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
		//		}

		//		/************************************************************************/
		//		/* 固定深度信息注释掉计算当前深度*/
		//		/************************************************************************/
		//		//Mat rvec = Mat::zeros(3, 1, CV_64FC1); //旋转向量
		//		//Mat rM; //旋转矩阵
		//		//Mat tvec;//平移向量
		//		//solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
		//		//Rodrigues(rvec, rM);
		//		//cout << "旋转矩阵:" << endl;
		//		//cout << rM << endl;
		//		//cout << "平移向量:" << endl;
		//		//cout << tvec << endl;
		//		//visualServoAlgorithm.setvpHomogeneousMatrix(cMo, rM, tvec);
		//		//cout << "当前位置其次变换矩阵:" << endl;
		//		//cout << cMo << endl;
		//		////计算深度信息
		//		//for (int i = 0; i < 4; i++)
		//		//{
		//		//	vpColVector cP;		//相机坐标系下特征点的三维坐标
		//		//	point[i].changeFrame(cMo, cP);
		//		//	p[i].set_Z(cP[2]);
		//		//}


		//		//写入特征点信息，包括图像坐标和深度信息，单位为m
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

		//			//获取机械臂末端在基坐标系下的位姿
		//			EcReArray fTe;
		//			cytonCommands->GetPose(fTe);
		//			//cout << "fTe: " << endl << fTe << endl;

		//			//当前图像特征点检测与匹配
		//			unsigned int nbMatch = keypoint.matchPoint(currentImage);
		//			std::cout << "Matches=" << nbMatch << std::endl;

		//			//绘制特征点
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
		//			//理想位置特征记录
		//			vector<Point2f> point2D;
		//			for (int i = 0; i < 4; i++)
		//			{
		//				currentDot[i].track(currentImage);
		//				vpImagePoint dot;
		//				dot = currentDot[i].getCog();
		//				visualServoAlgorithm.pixelToImage(p[i], cam, dot);	//获取色块重心图像坐标单位是米，这里没有深度信息
		//				point2D.push_back(cv::Point2f(dot.get_u(), dot.get_v()));
		//			}

		//			/************************************************************************/
		//			/* 固定深度信息注释掉计算当前深度*/
		//			/************************************************************************/
		//			//Mat rvec = Mat::zeros(3, 1, CV_64FC1); //旋转向量
		//			//Mat rM; //旋转矩阵
		//			//Mat tvec;//平移向量
		//			//solvePnP(point3D, point2D, visualServoAlgorithm.cam_intrinsic_matrix, visualServoAlgorithm.cam_distortion_matrix, rvec, tvec, false, CV_ITERATIVE);
		//			//Rodrigues(rvec, rM);
		//			///*cout << "旋转矩阵:" << endl;
		//			//cout << rM << endl;
		//			//cout << "平移向量:" << endl;
		//			//cout << tvec << endl;*/
		//			//visualServoAlgorithm.setvpHomogeneousMatrix(cMo, rM, tvec);
		//			///*cout << "当前位置其次变换矩阵:" << endl;
		//			//cout << cMo << endl;*/
		//			////计算深度信息
		//			//for (int i = 0; i < 4; i++)
		//			//{
		//			//	vpColVector cP;		//相机坐标系下特征点的三维坐标
		//			//	point[i].changeFrame(cMo, cP);
		//			//	p[i].set_Z(cP[2]);
		//			//}
		//			
		//			//task.set_fVe(fMe);
		//			//cMf = cMe*(fMe.inverse());
		//			//task.set_cVf(cMf);

		//			//计算机械臂末端运动速度
		//			vpColVector vpVc;
		//			vpVc = task.computeControlLaw();		//机械臂末端运动速度
		//			//cout << "vpVc" << endl << vpVc << endl;
		//			//相机坐标系下相机的速度
		//			Mat eVe = Mat(6, 1, CV_32FC1);
		//			visualServoAlgorithm.linkageVTransmit(vpVc, eRc, cPe, eVe);
		//			//cout << "eVe:" << endl << eVe << endl;
		//			//将末端手爪坐标系下的末端速度转换到基坐标系下
		//			//基坐标系到末端手爪的旋转矩阵
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

		//			//绘制图像轨迹
		//			visualServoAlgorithm.display_trajectory(currentImage, currentDot);
		//			vpServoDisplay::display(task, cam, currentImage, vpColor::green, vpColor::red);	//绘制特征点的当前位置和理想位置
		//			vpDisplay::flush(currentImage);

		//			tEnd2 = GetTickCount();
		//			printf("Use Time:%f\n\n\n\n", (tEnd2 - tEnd1)*1.0 / 1000);

		//			key = 0xff & waitKey(5);
		//			if ((key & 255) == 27 || (task.getError()).sumSquare() < 0.00005)   //  esc退出键
		//			{
		//				if ((key & 255) == 27)
		//				{
		//					armCamFlag = -1;    //机械臂退出运动
		//				}
		//				else
		//				{
		//					armCamFlag = 4;		//机械臂姿态调整
		//				}
		//				mode = DETECTION;
		//				vpDisplay::getClick(currentImage, true);
		//				cout << "均方差：" << (task.getError()).sumSquare() << endl;
		//				//msg = "Exit tracking, press esc to quit";
		//				break;
		//			}
		//		}
		//		break;  //退出TRACKING
		//	}
		//}



		armCamFlag = -1;		//机械臂退出运动
		baslerCam.close();
		task.kill();

	}
	catch (vpException &e) {
		std::cout << "Catch an exception: " << e << std::endl;
	}


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
	if (navigationAutoflag == NAVIGATIONSTART)
	{	
		srvToClientDatas.contralSignal = FINISH_AUTONAVIGATION_CONTROL;  //向服务端发送自动导航控制结束的指令
	}
	else
	{
		srvToClientDatas.contralSignal = FINISHARMCONTROL;  //向服务端发送机械臂运动完成的指令	
	}

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
	CamThread = CreateThread(NULL, 0, Camera, NULL, CREATE_SUSPENDED, NULL);
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
