
#include "MmVisualServoAlgorithm.h"


MmVisualServoAlgorithm::MmVisualServoAlgorithm()
{
}

MmVisualServoAlgorithm::~MmVisualServoAlgorithm()
{
}

//通过四个圆的圆心世界坐标利用solvePnP计算相机坐标到世界坐标系的变换关系
int MmVisualServoAlgorithm::coPlanarity4PointSolvePnP(const Mat &grayImage, const vector<cv::Point3f> Points3D, Mat &rotM, Mat & tvec)
{
	Mat imageShold;
	threshold(grayImage, imageShold, 100, 255, THRESH_BINARY);//图像二值化

	cvNamedWindow("二值化图像");
	cv::imshow("二值化图像", imageShold);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(imageShold, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
	
	//轮廓刷选
	unsigned int cmin = 100;   // 最小轮廓像素值  
	unsigned int cmax = 1000;   // 最大轮廓像素值  
	vector<vector<Point>>::const_iterator itc = contours.begin();
	while (itc != contours.end())
	{
		if ((itc->size()) < cmin || (itc->size()) > cmax)
		{
			itc = contours.erase(itc);
		}
		else ++itc;
	}

	//检验是否检测到4个质心点
	if (contours.size() != 4)
	{
		cout << "二值化图像后质心检测个数为：" << contours.size() << endl;
		return false;
	}

	//计算轮廓矩 
	vector<Moments> mu(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}
	//计算轮廓的质心     
	vector<Point2f> mc(contours.size());
//	int u[4];
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	float a[4], b;
	int i1, j1 = 0, k1;
	for (i1 = 0; i1<4; i1++){
		a[i1] = mc[i1].y;
	}
	for (i1 = 0; i1<4; i1++){
		for (k1 = j1; k1<4; k1++)
		{
			if (a[k1] >= a[j1]){
				b = a[j1];
				a[j1] = a[k1];
				a[k1] = b;
			}
		}
		j1++;
	}
	/*for(i1=0;i1<4;i1++){
	cout<<a[i1]<<endl;
	}
	cout<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]<<endl;
	*/
	float c[4], d;
	int i, j = 0, k;
	for (i = 0; i<4; i++){
		c[i] = mc[i].x;
	}
	for (i = 0; i<4; i++){
		for (k = j; k<4; k++)
		{
			if (c[k] >= c[j]){
				d = c[j];
				c[j] = c[k];
				c[k] = d;
			}
		}
		j++;
	}
	/*for(i=0;i<4;i++){
	cout<<c[i]<<endl;
	}
	cout<<c[0]<<" "<<c[1]<<" "<<c[2]<<" "<<c[3]<<endl;
	*/

	//特征点图像坐标
	vector<cv::Point2f> Points2D;
	Points2D.push_back(cv::Point2f(c[3], a[3]));	//P1
	Points2D.push_back(cv::Point2f(c[1], a[2]));   //P2
	Points2D.push_back(cv::Point2f(c[2], a[1]));	//P3
	Points2D.push_back(cv::Point2f(c[0], a[0]));   //P4


	//初始化输出矩阵
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
//	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

	//初始化相机参数Opencv
	/*double camD[9] = {
		1018.903427812478, 0, 330.08351953809,
		0, 1018.503663503252, 237.0835471388219,
		0, 0, 1 };*/
	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camIntrinsicParam);

	//畸变参数
//	double distCoeffD[5] = { -0.3967094938995925, 0.5065335117843332, 0.0003377083646548675, -0.0005085439166897146, -1.550758618176076 };
	cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, camDistortionParam);


	//三种方法求解
	solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);	//实测迭代法似乎只能用4个共面特征点求解，5个点或非共面4点解不出正确的解
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);			//Gao的方法可以使用任意四个特征点，特征点数量不能少于4也不能多于4
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);			//该方法可以用于N点位姿估计

	//旋转向量变旋转矩阵
	//提取旋转矩阵
//	double rm[9];
//	cv::Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, rotM);
	/*double r11 = rotM.ptr<double>(0)[0];
	double r12 = rotM.ptr<double>(0)[1];
	double r13 = rotM.ptr<double>(0)[2];
	double r21 = rotM.ptr<double>(1)[0];
	double r22 = rotM.ptr<double>(1)[1];
	double r23 = rotM.ptr<double>(1)[2];
	double r31 = rotM.ptr<double>(2)[0];
	double r32 = rotM.ptr<double>(2)[1];
	double r33 = rotM.ptr<double>(2)[2];*/

//	ofstream fout("旋转.txt");
//	fout << rotM << endl;
	cout << "旋转矩阵:" << endl;
	cout << rotM << endl;
	cout << "平移向量:" << endl;
	cout << tvec << endl;
	return true;
}