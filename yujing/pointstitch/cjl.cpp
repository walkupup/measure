
#include "nwDevice4JlU.h"
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include<ctime> 
#pragma comment( lib, "shlwapi.lib" )
#include "IKapBoard.h"
#include <opencv2/opencv.hpp>
//#include <opencv2/aruco.hpp>
#include<opencv2\\imgproc\\types_c.h>
#include <math.h>
using namespace std;
using namespace cv;

bool PrepareBuffer(HANDLE hDev);
void ErrorMessage(char*, char*);

void GrabCoordContinuous(int *pPointPerScan,
	double *coordX,	double *coordY,	double *coordZ)
{
	int num, count = 0;
	unsigned int jobNumber = 0;

	nwSetJobNumber(1);
	// Start the continuous mode
	nwStartContinuousGrab();
	int scans = nwRetrieveCoord(pPointPerScan, coordX, coordY, coordZ, &jobNumber);

	// you must set m_bContinuous to true somewhere else.
	//while (m_bContinuous)
	for (int i = 0; i < 100; i++)
	{
		// retrieve data from the device
		int scans = nwRetrieveCoord(pPointPerScan, coordX, coordY, coordZ, &jobNumber);
		printf("%d\n", scans);
		// Get coordinates from scanning system with which the sensor is working.
		// Add the coodinates together.
		int  count = 0;
		for (int jj = 0; jj < scans; jj++)
		{
			int numPoints = pPointPerScan[jj];
			for (int ii = 0; ii < numPoints; ii++)
			{
				double xx = coordX[count];
				double yy = coordY[count];
				double zz = coordZ[count];
				count++;
				// User can do something here, such as storing data in his own data
				// structure for display and data processing;
			}
			printf("%f, %f, %f, %f\n", coordY[0], coordZ[0], coordY[numPoints - 1], coordZ[numPoints - 1]);
		}

	}
	// Cancel the continuous mode to stop data acquisition
	nwEndContinuousGrab();

	// There may be more data left in the device after you cancel the continuous mode. 
	// So get it from the device. 
	int remainLines = 0;
	while (nwRetrieveCoord(pPointPerScan, coordX, coordY, coordZ, &jobNumber) > 0)
	{
		// do the same data processing as above

		remainLines++;
	}

	int totalScans = count + remainLines;

	// check if there is any error during grabbing
	if (unsigned int err = nwGetDeviceError())
	{
		nwClearDeviceError();
	}

}


int processRawPoints(std::string file, float dx, float dy, float dz, std::string outFile)
{
	std::ifstream ab(file);
	std::ofstream out(outFile, std::ofstream::app);

	float x01, y01, z01;
	while (ab >> x01 >> y01 >> z01)
	{
		x01 += dx;
		y01 += dy;
		z01 += dz;
		out << x01 << " " << y01 << " " << z01 << std::endl;
	}
	return 1;
}


int processRawPoints(std::string file, float dx, float dy, float dz, pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud)
{
	std::ifstream ab(file);
	float x01, y01, z01;
	while (ab >> x01 >> y01 >> z01)
	{
		x01 += dx;
		y01 += dy;
		z01 += dz;
		cloud->push_back(pcl::PointXYZ(x01, y01, z01));
	}
	return 1;
}

int processRawPoints(std::string file, Mat r ,Mat t,  pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud)
{
	std::ifstream ab(file);
	float x01, y01, z01;
	while (ab >> x01 >> y01 >> z01)
	{
		Mat p = Mat::ones(3, 1, CV_32F);
	    p = (Mat_<float>(3, 1) << x01, y01, z01);
		p = r*p + 10*t;
		cloud->push_back(pcl::PointXYZ(p.at<float>(0), p.at<float>(1), p.at<float>(2)));
	}
	return 1;
}

//Mat R = Mat::ones(3, 3, CV_32F);
Mat R=(Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
Mat T = Mat::zeros(3, 1, CV_32F);
std::mutex mtx;
std::mutex mtx1;
//unsigned char* g_imageData = new unsigned char[2048 * 2048];
Mat resultImg = Mat::zeros(2048, 2048, CV_8UC1);
DWORD hSize = 2048 * 2048;

static Point3i digitsWorldPos[2][2];


//void CALLBACK OnTimeout(void *Context)
//{
//	printf("Grab image timeout\n");
//}

int nFrameIndex = 0;
int nImageTotal = 0;
// Callback function, called when one or more frame is ready to read in the buffer pool.
void CALLBACK OnFrameReady(void *Context)
{

	HANDLE hDev = (HANDLE)Context;
	unsigned char* pUserBuffer = NULL;
	int nFrameSize = 0;
	int nFrameCount = 0;
	IKAPBUFFERSTATUS status;
	IKapGetInfo(hDev, IKP_FRAME_COUNT, &nFrameCount);
	IKapGetBufferStatus(hDev, nFrameIndex, &status);

	//	resultImg= Mat::zeros(2048, 2048, CV_8UC1);
	while (status.uFull == 1 && nFrameIndex < nFrameCount)
	{
		IKapGetInfo(hDev, IKP_FRAME_SIZE, &nFrameSize);
		IKapGetBufferAddress(hDev, nFrameIndex, (void**)&pUserBuffer);
		mtx1.lock();
		memcpy(/*g_imageData*/resultImg.data, pUserBuffer/*imageBuffer*/, hSize);
		mtx1.unlock();
		//		imwrite("D:\\1.bmp", resultImg);

		//		imshow("RE", resultImg);

		/************************************************************************/
		/*Process The Image Data here.
		/*e.g. ProcessImage(unsigned char* pUserBuffer, int nFrameSize)
		/*
		/************************************************************************/
		//When disable IKapBoard Auto Empty machine, user should empty frame buffer manually. 
		IKapReleaseBuffer(hDev, nFrameIndex);
		nFrameIndex++;
		IKapGetBufferStatus(hDev, nFrameIndex, &status);

		//nImageTotal++;
		//printf("Total image %d\n", nImageTotal);
	}
	nFrameIndex = nFrameIndex % nFrameCount;



	//	imshow("RE", resultImg);
}

// Callback function, which is called when the continuous acquisition stops
void CALLBACK OnGrabStop(void *Context)
{
	printf("Stop grabbing image(continuously)\n");

	HANDLE hDev = (HANDLE)Context;
	IKAPERRORINFO pIKErrInfo;

	memset(&pIKErrInfo, 0, sizeof(IKAPERRORINFO));
	IKapGetLastError(&pIKErrInfo, true);
	if (pIKErrInfo.uErrorCode != IKStatus_Success)
	{
		/**************************************************************/
		/*The termination of the image acquisition process,
		/*Check “IKapGetLastError” to get detailed error information.
		/*e.g. DoErrorHandle( uErrorCode )*/
		/**************************************************************/
	}
}

void init_pos() {
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			digitsWorldPos[i][j].x = i * 6;
			digitsWorldPos[i][j].y = j * 6;
			digitsWorldPos[i][j].z = 0;
		}
	}
}

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

void foo()
{
	init_pos();
	Size boardSize(3, 3);
	//namedWindow("out", 512*512);
	Mat camMatrix;
	Mat distCoeffs;
	if (1) {
		//bool readOk = readCameraParameters("IndustryCam.txt", camMatrix, distCoeffs);
		bool readOk = readCameraParameters("camera.yml", camMatrix, distCoeffs);
		if (!readOk) {
			cerr << "Invalid camera file" << endl;
			//return 0;
		}
	}

	Mat image;
	image = imread("start.bmp", 0);
	flip(image, image, 1);

	cv::Mat img;
	int rx = 300, ry = 300;
	cv::resize(image, img, cv::Size(rx, ry));
	bool found;
	vector<Point2f> corners;
	found = findChessboardCorners(img, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		+ CALIB_CB_FAST_CHECK);
	for (int j = 0; j < corners.size(); j++)
	{
		corners[j].x = corners[j].x * image.cols / rx;
		corners[j].y = corners[j].y * image.rows / ry;
	}
	if (found)
		cornerSubPix(image, corners, Size(11, 11), Size(-1, -1),
			TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));

	if (!corners.size() == 0) {
		Mat rVec; 
		Mat tVec;
		vector<Point3f> obj;
		obj.push_back(digitsWorldPos[0][0]);    //right-down conrner

		obj.push_back(digitsWorldPos[0][1]);  //left-down conrner

		obj.push_back(digitsWorldPos[1][1]);    //left-up conrner

		obj.push_back(digitsWorldPos[1][0]);      //right-up conrner

		//cout << "right-down conrner:" << obj[0] << endl;
		vector<Point2f> imgPos;
		
		imgPos.push_back(corners[2]);
		imgPos.push_back(corners[8]);
		imgPos.push_back(corners[6]);
		imgPos.push_back(corners[0]);
		double a, b;
		for (int i = 0; i < 3; i++) {
			if ((imgPos[0].x + imgPos[0].y) < (imgPos[i + 1].x + imgPos[i + 1].y))
			{
				a = imgPos[0].x;
				b = imgPos[0].y;
				imgPos[0].x = imgPos[i + 1].x;
				imgPos[0].y = imgPos[i + 1].y;
				imgPos[i + 1].x = a;
				imgPos[i + 1].y = b;
			}
		}
		for (int i = 1; i < 4; i++) {
			if ((imgPos[2].x + imgPos[2].y) > (imgPos[i].x + imgPos[i].y))
			{
				a = imgPos[2].x;
				b = imgPos[2].y;
				imgPos[2].x = imgPos[i].x;
				imgPos[2].y = imgPos[i].y;
				imgPos[i].x = a;
				imgPos[i].y = b;
			}
		}
		if (imgPos[3].y > imgPos[1].y) {
			a = imgPos[1].x;
			b = imgPos[1].y;
			imgPos[1].x = imgPos[3].x;
			imgPos[1].y = imgPos[3].y;
			imgPos[3].x = a;
			imgPos[3].y = b;
		}

		solvePnP(obj, imgPos, camMatrix, distCoeffs, rVec, tVec, false, SOLVEPNP_EPNP);
		
		Mat rot;
		Rodrigues(rVec, rot);

		rot = rot.t();
		tVec = (-1)*rot*tVec;
		FileStorage fs("wai.txt", FileStorage::WRITE);
		fs << "rot" << rot;
		fs << "tVec" << tVec;
		fs.release();
	}


	//plog::init(plog::debug, "2019.12.18-2.txt");
	int frameNum = 1;
	int i = 0;
	string num;

	//ofstream debug("debug1.txt");
	//VideoCapture cap(0);

	//if (!cap.isOpened())
	//{
	//	cout << "Error opening video stream" << endl;
	//	//return -1;
	//}

	//cap.set(CAP_PROP_FRAME_WIDTH, 640); //帧宽

	//cap.set(CAP_PROP_FRAME_HEIGHT, 480);//帧高
										//cap.set(CAP_PROP_FPS,50);
										//int a = cap.get(CAP_PROP_FPS);
	while (1) {
		Mat Image;
		//cap >> Image;
		MSG msg;
		BOOL bRet;
		clock_t start, start1, end;
		start1 = start = clock();
		int nTime = 0;
		int nCount = 0;
		
			/*end = clock();
			nTime = (end - start);
			if (end - start1 <0)
				break;*/

			MSG message;
			if (::PeekMessage(&message, NULL, 0, 0, PM_REMOVE))
			{
				::TranslateMessage(&message);
				::DispatchMessage(&message);
			}
			/*	if (nTime > 10)
			{
			if (mtx.try_lock())
			{
			imshow("Img", resultImg);
			mtx.unlock();
			}
			start = clock();
			nCount++;
			}*/
			//if (nTime > 25)
			//{

				
					mtx1.lock();
					//memcpy(Image.data, resultImg.data/*g_imageData*/, hSize);
					resultImg.copyTo(Image);
					mtx1.unlock();

					//imshow("Img", resultImg);
					//nCount++;
				

			//	start = clock();
				//	nCount++;
			//}


		
		//printf("拷贝个数：%d\n", nCount);

		/*if (Image.empty())
			break;*/
		/*Mat Image;
		i++;
		ostringstream convert;
		convert << i;
		num = convert.str();
		Image = imread(".\\" + num + ".bmp", 0);*/
		//PLOGD << "frame" << frameNum++ << endl;
		//Image = imread("start.bmp",0);
		//cvtColor(image, greyimage, CV_BGR2GRAY);
		flip(Image, Image, 1);


		//cvtColor(Image, Image, COLOR_BGR2GRAY);
		cv::Mat Img;
		int rx = 400, ry = 400;
		cv::resize(Image, Img, cv::Size(rx, ry));
		bool found;
		vector<Point2f> Corners;
		found = findChessboardCorners(Img, boardSize, Corners, CALIB_CB_ADAPTIVE_THRESH /*+ CALIB_CB_NORMALIZE_IMAGE*/
			+ CALIB_CB_FAST_CHECK);

		for (int j = 0; j < Corners.size(); j++)
		{
			Corners[j].x = Corners[j].x * Image.cols / rx;
			Corners[j].y = Corners[j].y * Image.rows / ry;
		}
		if (found)
			cornerSubPix(Image, Corners, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));



		Mat ImageCopy;
		Image.copyTo(ImageCopy);
		if (!Corners.size() == 0) {
			drawChessboardCorners(ImageCopy, boardSize, Mat(Corners), found);

			Mat RVec; 
			Mat TVec;
			vector<Point3f> Obj;
			Obj.push_back(digitsWorldPos[0][0]);    //right-down conrner

			Obj.push_back(digitsWorldPos[0][1]);  //left-down conrner

			Obj.push_back(digitsWorldPos[1][1]);    //left-up conrner

			Obj.push_back(digitsWorldPos[1][0]);      //right-up conrner

			//cout << "right-down conrner:" << Obj[0] << endl;

			vector<Point2f> ImgPos;
			ImgPos.push_back(Corners[2]);
			ImgPos.push_back(Corners[8]);
			ImgPos.push_back(Corners[6]);
			ImgPos.push_back(Corners[0]);

			double a, b;
			for (int i = 0; i < 3; i++) {
				if ((ImgPos[0].x + ImgPos[0].y) < (ImgPos[i + 1].x + ImgPos[i + 1].y))
				{
					a = ImgPos[0].x;
					b = ImgPos[0].y;
					ImgPos[0].x = ImgPos[i + 1].x;
					ImgPos[0].y = ImgPos[i + 1].y;
					ImgPos[i + 1].x = a;
					ImgPos[i + 1].y = b;
				}
			}
			for (int i = 1; i < 4; i++) {
				if ((ImgPos[2].x + ImgPos[2].y) >(ImgPos[i].x + ImgPos[i].y))
				{
					a = ImgPos[2].x;
					b = ImgPos[2].y;
					ImgPos[2].x = ImgPos[i].x;
					ImgPos[2].y = ImgPos[i].y;
					ImgPos[i].x = a;
					ImgPos[i].y = b;
				}
			}
			if (ImgPos[3].y > ImgPos[1].y) {
				a = ImgPos[1].x;
				b = ImgPos[1].y;
				ImgPos[1].x = ImgPos[3].x;
				ImgPos[1].y = ImgPos[3].y;
				ImgPos[3].x = a;
				ImgPos[3].y = b;
			}

			//debug << "ImgPos=" << ImgPos << endl;
			
			solvePnP(Obj, ImgPos, camMatrix, distCoeffs, RVec, TVec, false, SOLVEPNP_EPNP);
			//PLOGD << "ImgPos" << ImgPos << endl;
			Mat Rot;
			Rodrigues(RVec, Rot);

			Mat rot; 
			Mat tVec;
			FileStorage fs("wai.txt", FileStorage::READ);
			fs["rot"] >> rot;
			fs["tVec"] >> tVec;
			fs.release();

			mtx.lock();
			//Mat R, T;
			R = rot*Rot;
			T = rot*TVec + tVec;
			R.convertTo(R, CV_32F);
			T.convertTo(T, CV_32F);
			double x = T.at<double>(0);
			double y = T.at<double>(1);
			double z = T.at<double>(2);
			double  s = sqrt(x*x + y*y + z*z);
			mtx.unlock();

			//PLOGD << "T=" << T << endl;
			//PLOGD << "s=" << s << endl;
		}
		//imshow("out", ImageCopy);
		char key = (char)waitKey(1);
		if (key == 27) break;
	}
}


int main()
{
	HANDLE hDev = INVALID_HANDLE_VALUE;
	unsigned int nPCIeDevCount = 0;

	IKapGetBoardCount(IKBoardPCIE, &nPCIeDevCount);
	if (nPCIeDevCount == 0)
	{
		ErrorMessage("IKapGetBoardCount", "0");
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -10;
		//goto FreeHandle;
	}
	// Open board
	hDev = IKapOpen(IKBoardPCIE, 0);
	if (hDev == INVALID_HANDLE_VALUE)
	{
		ErrorMessage("IKapOpen", NULL);
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -11;
		//goto FreeHandle;
	}
	// Load configuration from file 
	if (IKapLoadConfigurationFromFile(hDev, "cl2_tap8_w2048_h2048.vlcf") == false)
	{
		ErrorMessage("IKapLoadConfigurationFromFile", "cl2_tap8_w2048_h2048.vlcf");
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -12;
		//goto FreeHandle;
	}
	// Register callback function
	if (IKapRegisterCallback(hDev, IKEvent_FrameReady, OnFrameReady, hDev) == false)
	{
		ErrorMessage("Register Callback", "IKEvent_FrameReady");
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -13;
		//goto FreeHandle;
	}
	// Register callback function
	if (IKapRegisterCallback(hDev, IKEvent_GrabStop, OnGrabStop, hDev) == false)
	{
		ErrorMessage("Register Callback", "IKEvent_GrabStop");
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -14;
		//goto FreeHandle;
	}
	// Register callback function
	//if (IKapRegisterCallback(hDev, IKEvent_TimeOut, OnTimeout, hDev) == false)
	//{
	//	ErrorMessage("Register Callback", "IKEvent_TimeOut");
	//	IKapClose(hDev);
	//	hDev = INVALID_HANDLE_VALUE;
	//	return -15;
	//	//goto FreeHandle;
	//}


	// Prepare Buffer
	if (PrepareBuffer(hDev) == false)
	{
		ErrorMessage("Prepare Image Buffer", "null");
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -16;
		//goto FreeHandle;
	}

	// Start capturing image continuously
	if (IKapStartGrab(hDev, 0) == false)
	{
		ErrorMessage("Start grabbing image(continuously)", NULL);
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		return -17;
		//goto FreeHandle;
	}
	std::thread first(foo);
	//搜索USB总线上的扫描头
	nwEnumDevices();

	// 查询是否找到了扫描头
	if (!nwIsDevicePresent())
	{
		printf("没有找到扫描头\n");
		return -1;
	}

	// 查询扫描头最大输出点数 
	int maxNum = nwDeviceMaxPoints();
	if (maxNum < 1)
	 return -2;

	printf("最大输出点数: %d\n", maxNum);

	// 准备好装数据的内存 
	double *coordX = (double*)calloc(maxNum, sizeof(double));
	double *coordY = (double*)calloc(maxNum, sizeof(double));
	double *coordZ = (double*)calloc(maxNum, sizeof(double));
	int *pPointPerScan = (int*)calloc(maxNum, sizeof(*pPointPerScan));
	//GrabCoordContinuous(pPointPerScan, coordX, coordY, coordZ);
	//return 1;
	
	char filename[500];
	pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	float dx = 0;
	//ofstream debug("debug.txt");

	for ( int i = 0; i < 200; i++)
	{
		// 抓取数据。对于单激光线扫描头，coordX数组里的数据都是零，可以忽略 
		int num = nwDeviceCoord(coordX, coordY, coordZ);

		//cout << "num=" << num << endl;
		// 查看是否有错 
		if (unsigned int err = nwGetDeviceError())
		{
			char tempstr[16];
			sprintf(tempstr, "0x%x", err);

			// 清除错误 
			nwClearDeviceError();

			return -3;
		}

		if (num > 0)
		{
			sprintf(filename, "%d.txt", i);
			ofstream out(filename);
			for (int j = 0; j < num; j++)
			{
				out << coordX[j] << " " << coordY[j] << " " << coordZ[j] << std::endl;
			}
			out.close();
			//coordY 和coordZ中应该抓取到了坐标，可以对数据进行处理 
			//printf("%f, %f, %f, %f\n", coordY[num / 2], coordZ[num / 2]);
			printf("%f, %f, %f, %f\n", coordY[0], coordZ[0], coordY[num - 1], coordZ[num - 1]);
			
			Mat ROT = Mat::ones(3, 3, CV_32F);
			Mat TT = Mat::ones(3, 1, CV_32F);
			Mat r = Mat::ones(3, 3, CV_32F);
			Mat t = Mat::ones(3, 1, CV_32F);
			Mat	rVec= Mat::ones(3, 3, CV_32F);
			Mat	tVec= Mat::ones(3, 1, CV_32F);
			//ROT = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
			//TT = (Mat_<float>(3, 1) << 0, 0, 0);
			r = (Mat_<float>(3, 3) << 0, -1, 0, 0, 0, -1, 1, 0, 0);
			//r = (Mat_<float>(3, 3) << 0, 1, 0, 0, 0, 1, 1, 0, 0);
			t = (Mat_<float>(3, 1) << -1.5, -2, 18);
	
			mtx.lock();
			rVec = R;
			tVec = T;
			mtx.unlock();
			//debug << "R=" << rVec << endl;
			//debug << "T=" << tVec << endl;
			ROT = r*rVec*r.t();
            TT = -r*rVec*r.t()*t+r*tVec + t;

			/*ROT= (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
			TT.at<float>(1) = 0;
			TT.at<float>(2) = 0;
			cout << "TT=" << TT << endl;*/
			stringstream ss;
			ss << i << ".txt";
			processRawPoints(filename, ROT, TT, cloud1);
			

			/*stringstream ss;
			ss << i << ".txt";
			processRawPoints(filename, dx, 0, 0, cloud1);
			dx = dx + 1;*/
			viewer.showCloud(cloud1);
			Sleep(40);
		}
	}
	//debug.close();

	if (IKapStopGrab(hDev) == false)
	{
		ErrorMessage("Stop grabbing image(continuous)", NULL);
		return -18;
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		//goto FreeHandle;
	}

	// Unregister callback function
	if (IKapUnRegisterCallback(hDev, IKEvent_GrabStop) == false)
	{
		ErrorMessage("UnRegister Callback", "IKEvent_GrabEnd");
		return -19;
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		//goto FreeHandle;
	}
	// Unregister callback function
	if (IKapUnRegisterCallback(hDev, IKEvent_FrameReady) == false)
	{
		ErrorMessage("UnRegister Callback", "IKEvent_FrameReady");
		return -20;
		IKapClose(hDev);
		hDev = INVALID_HANDLE_VALUE;
		//goto FreeHandle;
	}
	// Unregister callback function
	//if (IKapUnRegisterCallback(hDev, IKEvent_TimeOut) == false)
	//{
	//	ErrorMessage("UnRegister Callback", "IKEvent_TimeOut");
	//	return -21;
	//	IKapClose(hDev);
	//	hDev = INVALID_HANDLE_VALUE;
	//	//goto FreeHandle;
	//}
	

/*
	//选取第50-99帧将读取的数据进行坐标转换后合并为一个text文件
	for (int i = 50; i < 100; i++)
	{
		stringstream ss;
		ss << i << ".txt";
		processRawPoints(ss.str(), dx, 0, 0, cloud1);
		dx = dx + 1;
		viewer.showCloud(cloud1);
		Sleep(1000);
	}
*/
/*
	///********将合并成的x.txt文件转换保存为PCD格式
	fstream modelRead;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PCDWriter writer;

	modelRead.open("x.txt", std::ios_base::in);
	pcl::PointXYZ pclPnt;
	while (!modelRead.eof())
	{
		modelRead >> pclPnt.x >> pclPnt.y >> pclPnt.z;
		cloud.push_back(pclPnt);
	}
	modelRead.close();
	writer.write("x.pcd", cloud);
	cout << "转换完成" << endl;

	//******************点云可视化***********************************************
	pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("x.pcd", *cloud1) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	*/
/*
	std::ifstream in("x.txt");
	float x01, y01, z01;
	while (in >> x01 >> y01 >> z01)
	{
		cloud1->push_back(pcl::PointXYZ(x01, y01, z01));
	}
	*/
	while (!viewer.wasStopped())
	{
	}
	getchar();
	return 0;

}

void ErrorMessage(char* msg1, char* msg2) {
	IKAPERRORINFO pIKErrInfo;

	memset(&pIKErrInfo, 0, sizeof(IKAPERRORINFO));
	IKapGetLastError(&pIKErrInfo, true);

	printf("%s(%s)\nType= %d\nIndex= %08x\nError Code= %08x\n", msg1, msg2, pIKErrInfo.uBoardType, pIKErrInfo.uBoardIndex, pIKErrInfo.uErrorCode);
}

bool PrepareBuffer(HANDLE hDev)
{
	int nFrameCount = 0;
	int nFrameTransferMode = IKP_FRAME_TRANSFER_SYNCHRONOUS_NEXT_EMPTY_WITH_PROTECT;
	int nFrameAutoEmpty = 0;
	printf("Please input frame count:\n");
	scanf_s("%d", &nFrameCount);

	if (IKapSetInfo(hDev, IKP_FRAME_COUNT, nFrameCount) == false)
		return false;
	if (IKapSetInfo(hDev, IKP_FRAME_TRANSFER_MODE, nFrameTransferMode) == false)
		return false;
	if (IKapSetInfo(hDev, IKP_FRAME_AUTO_CLEAR, nFrameAutoEmpty) == false)
		return false;
	return true;
}