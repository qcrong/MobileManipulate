#include "MmVisualServoBase.h"

//using namespace pylon;
//using namespace std;
//using namespace cv;

MmVisualServoBase::MmVisualServoBase() :camera(CTlFactory::GetInstance().CreateFirstDevice())
{	
}

MmVisualServoBase::~MmVisualServoBase()
{
//	cvReleaseCapture(&AxisCamera);
}


//Basler相机初始化，对现有的acA640-300gc和acA1300-60gc都适用
//灰度图像
void MmVisualServoBase::baslerOpen(vpImage< unsigned char > &I)
{
	GenApi::INodeMap& nodemap = camera.GetNodeMap();
	cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
	camera.Open();

	width = nodemap.GetNode("Width");
	height = nodemap.GetNode("Height");

	formatConverter.OutputPixelFormat = PixelType_Mono8;
}

//Basler相机初始化，对现有的acA640-300gc和acA1300-60gc都适用
//RGB图像
void MmVisualServoBase::baslerOpen(vpImage< vpRGBa > &I)
{
	GenApi::INodeMap& nodemap = camera.GetNodeMap();
	cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
	camera.Open();

	width = nodemap.GetNode("Width");
	height = nodemap.GetNode("Height");

	formatConverter.OutputPixelFormat = PixelType_BGRA8packed;
}

//Balser相机关闭
void MmVisualServoBase::close(void)
{
	camera.Close();
}

//获取Basler相机的灰度图像
void MmVisualServoBase::acquireBaslerImg(vpImage< unsigned char> &I)
{
	cv::Size frameSize = Size((int)width->GetValue(), (int)height->GetValue());
	CGrabResultPtr ptrGrabResult;
	Mat opencvImage;

	camera.StartGrabbing(1, GrabStrategy_LatestImageOnly);
	while (camera.IsGrabbing())
	{
		camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
		if (ptrGrabResult->GrabSucceeded())
		{
			formatConverter.Convert(pylonImage, ptrGrabResult);
			opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint_t*)pylonImage.GetBuffer());
		}
		vpImageConvert::convert(opencvImage, I);
	}
}

//获取Basler相机的RGB图像
void MmVisualServoBase::acquireBaslerImg(vpImage< vpRGBa> &I)
{
	cv::Size frameSize = Size((int)width->GetValue(), (int)height->GetValue());
	CGrabResultPtr ptrGrabResult;
	Mat opencvImage;

	camera.StartGrabbing(1, GrabStrategy_LatestImageOnly);
	try
	{
		while (camera.IsGrabbing())
		{
			camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
			if (ptrGrabResult->GrabSucceeded())
			{
				formatConverter.Convert(pylonImage, ptrGrabResult);
				opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint_t*)pylonImage.GetBuffer());
			}
			vpImageConvert::convert(opencvImage, I);
		}
	}
	catch (Exception e)
	{
		cout << "Grab failed!" << endl;
	}
}

//获取Basler相机的RGB图像,图像格式Mat
void MmVisualServoBase::acquireBaslerImg(vpImage<unsigned char> &I, Mat &opencvImage)
{
	cv::Size frameSize = Size((int)width->GetValue(), (int)height->GetValue());
	CGrabResultPtr ptrGrabResult;
//	Mat opencvImage;

	camera.StartGrabbing(1, GrabStrategy_LatestImageOnly);
	while (camera.IsGrabbing())
	{
		camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
		if (ptrGrabResult->GrabSucceeded())
		{
			formatConverter.Convert(pylonImage, ptrGrabResult);
			opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint_t*)pylonImage.GetBuffer());
		}
		vpImageConvert::convert(opencvImage, I);
	}
}

//获取图片高度
unsigned int MmVisualServoBase::getHeight(void) const
{
	return (unsigned int)height->GetValue();
}

//获取图片宽度
unsigned int MmVisualServoBase::getWidth(void) const
{
	return (unsigned int)width->GetValue();
}


//Axis云台相机初始化，返回1表示初始化成功，否则返回0
bool MmVisualServoBase::axisInit()
{
	//获取相机图片地址
	AxisCamera = cvCaptureFromFile("http://192.168.0.90/axis-cgi/mjpg/video.cgi?resolution=640x480&req_fps=30&.mjpg");
	if (AxisCamera == NULL)
	{
		cout << "Axis Camera inited defeated." << endl;
		return false;
	}
	return true;
}

//获取Axis云台相机的图像,返回Mat类型图片
void MmVisualServoBase::acquireAxisImg(Mat *img)
{
	
	if (AxisCamera == NULL)
	{
		cout << "camera is null" << endl;
	}
	
	IplImage* originalimgI = cvQueryFrame(AxisCamera);
	Mat originalImgM = cv::cvarrToMat(originalimgI, true);

}