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


//Basler�����ʼ���������е�acA640-300gc��acA1300-60gc������
//�Ҷ�ͼ��
void MmVisualServoBase::baslerOpen(vpImage< unsigned char > &I)
{
	GenApi::INodeMap& nodemap = camera.GetNodeMap();
	cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
	camera.Open();

	width = nodemap.GetNode("Width");
	height = nodemap.GetNode("Height");

	formatConverter.OutputPixelFormat = PixelType_Mono8;
}

//Basler�����ʼ���������е�acA640-300gc��acA1300-60gc������
//RGBͼ��
void MmVisualServoBase::baslerOpen(vpImage< vpRGBa > &I)
{
	GenApi::INodeMap& nodemap = camera.GetNodeMap();
	cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
	camera.Open();

	width = nodemap.GetNode("Width");
	height = nodemap.GetNode("Height");

	formatConverter.OutputPixelFormat = PixelType_BGRA8packed;
}

//Balser����ر�
void MmVisualServoBase::close(void)
{
	camera.Close();
}

//��ȡBasler����ĻҶ�ͼ��
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

//��ȡBasler�����RGBͼ��
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

//��ȡBasler�����RGBͼ��,ͼ���ʽMat
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

//��ȡͼƬ�߶�
unsigned int MmVisualServoBase::getHeight(void) const
{
	return (unsigned int)height->GetValue();
}

//��ȡͼƬ���
unsigned int MmVisualServoBase::getWidth(void) const
{
	return (unsigned int)width->GetValue();
}


//Axis��̨�����ʼ��������1��ʾ��ʼ���ɹ������򷵻�0
bool MmVisualServoBase::axisInit()
{
	//��ȡ���ͼƬ��ַ
	AxisCamera = cvCaptureFromFile("http://192.168.0.90/axis-cgi/mjpg/video.cgi?resolution=640x480&req_fps=30&.mjpg");
	if (AxisCamera == NULL)
	{
		cout << "Axis Camera inited defeated." << endl;
		return false;
	}
	return true;
}

//��ȡAxis��̨�����ͼ��,����Mat����ͼƬ
void MmVisualServoBase::acquireAxisImg(Mat *img)
{
	
	if (AxisCamera == NULL)
	{
		cout << "camera is null" << endl;
	}
	
	IplImage* originalimgI = cvQueryFrame(AxisCamera);
	Mat originalImgM = cv::cvarrToMat(originalimgI, true);

}