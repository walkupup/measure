#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//#include <uart.h>
#include <math.h>
#include <plog/Log.h>
using namespace std;
using namespace cv;

// id对应的棋盘数字






static Point3i digitsWorldPos[2][2];    


void init_pos(){
    for(int i = 0;i < 2; i++)
    {
        for(int j = 0;j < 2; j++)
        {
            digitsWorldPos[i][j].x = i*7;
            digitsWorldPos[i][j].y = j*7;
            digitsWorldPos[i][j].z = 0;
        }
    }
}

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


int main() {
	string line;
	init_pos();
	namedWindow("out", 0);
	resizeWindow("out", 1024, 1024);
	int dictionaryId = 18;
	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	// CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}
	detectorParams->cornerRefinementMethod = 3;

	//detectorParams->aprilTagQuadDecimate = 0.5;
	bool estimatePose = false;//false;
	bool showRejected = false;
	Mat camMatrix, distCoeffs;
	if (1) {
		//bool readOk = readCameraParameters("IndustryCam.txt", camMatrix, distCoeffs);
		bool readOk = readCameraParameters("camera.yml", camMatrix, distCoeffs);
		if (!readOk) {
			cerr << "Invalid camera file" << endl;
			return 0;
		}
	}


	VideoCapture input;
	input.open("C:\\Users\\Admin\\Desktop\\12.13\\10.21x\\biaoding.avi");

	/* int camId=2;
	 VideoCapture input;
	 input.open("test2.avi");*/
	 //input.set(CAP_PROP_FRAME_WIDTH, 1080);//宽度 
	 //input.set(CAP_PROP_FRAME_HEIGHT, 960);//高度
	 //input.set(CAP_PROP_FPS, 90);//帧数
	 //input.open(camId);
	int waitTime = 10;

	double totalTime = 0;
	int totalIterations = 0;
	float markerLength = 0.01;


	plog::init(plog::debug, "2019.12.12-0.txt");

	vector<Point2i> newimage;
	Mat image, imageCopy, ResultImg;
	image = imread("1.bmp",0);
	ResultImg = image;
	flip(ResultImg, ResultImg, 1);
	int cols = ResultImg.cols;
	int rows = ResultImg.rows;
	double tick = (double)getTickCount();

	vector<int> ids;
	vector<vector<Point2f>> corners, rejected;
	vector<Vec3d> rvecs, tvecs;

	//detectorParams->cornerRefinementMethod = 1;
	//aruco::detectMarkers(ResultImg, dictionary, corners, ids, detectorParams, rejected);
	//if (ids.size() > 0) {

	//	int32_t xmax = 0, xmin = INT32_MAX, ymax = 0, ymin = INT32_MAX;
	//	for (int i = 0; i < corners[0].size(); i++)
	//	{
	//		int x = corners[0][i].x;
	//		int y = corners[0][i].y;
	//		if (xmax < x) {
	//			xmax = x;
	//		}
	//		if (xmin > x) {
	//			xmin = x;
	//		}
	//		if (ymax < y) {
	//			ymax = y;
	//		}
	//		if (ymin > y) {
	//			ymin = y;
	//		}
	//	}

	//	newimage.push_back(Point2i(xmin - 20, ymin - 20));
	//	newimage.push_back(Point2i(xmax + 20, ymin - 20));
	//	newimage.push_back(Point2i(xmin - 20, ymax + 20));
	//	newimage.push_back(Point2i(xmax + 20, ymax + 20));
	//	newimage.push_back(Point2i(xmax - xmin + 1 + 20 * 2, ymax - ymin + 1 + 20 * 2));
	//}
	//detectorParams->cornerRefinementMethod = 3;
	//if (!newimage.empty()) {
	//	int col = newimage[4].x;
	//	int row = newimage[4].y;
	//	Mat Image1(row, col, CV_8UC1, cv::Scalar(0, 0, 0));
	//	for (int i = 0; i < row; i++) {
	//		for (int j = 0; j < col; j++) {
	//			Image1.data[i*col + j] = ResultImg.data[cols*(i + newimage[0].y) + j + newimage[0].x];
	//		}
	//	}
	//	aruco::detectMarkers(Image1, dictionary, corners, ids, detectorParams, rejected);
	//	if (ids.size() > 0) {
	//		for (int i = 0; i < corners[0].size(); i++) {
	//			corners[0][i].x += newimage[0].x;
	//			corners[0][i].y += newimage[0].y;
	//		}
	//	}
	//	else {
	//		aruco::detectMarkers(ResultImg, dictionary, corners, ids, detectorParams, rejected);
	//	}
	//}
	//else {

	//	// detect markers and estimate pose
	//	aruco::detectMarkers(ResultImg, dictionary, corners, ids, detectorParams, rejected);
	//}
	//newimage.clear();

	 //detect markers and estimate pose
	aruco::detectMarkers(ResultImg, dictionary, corners, ids, detectorParams, rejected);
	if (estimatePose && ids.size() > 0)
		aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
			tvecs);

 	double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
	totalTime += currentTime;
	totalIterations++;
	//if(totalIterations % 30 == 0) 
	{
		cout << "Detection Time = " << currentTime * 1000 << " ms "
			<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
	}

	// draw results
	ResultImg.copyTo(imageCopy);

	if (ids.size() > 0) 
	{
		aruco::drawDetectedMarkers(imageCopy, corners, ids);

		if (estimatePose) {
			for (unsigned int i = 0; i < ids.size(); i++)
				aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
					markerLength * 0.5f);
		}

		Mat rVec, tVec;
		vector<Point3f> obj;

		obj.push_back(digitsWorldPos[0][0]);    //right-down conrner

		obj.push_back(digitsWorldPos[0][1]);  //left-down conrner

		obj.push_back(digitsWorldPos[1][1]);    //left-up conrner

		obj.push_back(digitsWorldPos[1][0]);      //right-up conrner

		cout << "right-down conrner:" << obj[0] << endl;

		vector<Point2f> imgPos = corners[0];
		solvePnP(obj, imgPos, camMatrix, distCoeffs, rVec, tVec, false, SOLVEPNP_P3P);
		Mat rot;
		Rodrigues(rVec, rot);

		rot = rot.t();
		tVec = (-1)*rot*tVec;
		FileStorage fs("1.txt", FileStorage::WRITE);
		fs << "rot" << rot;
		fs << "tVec" << tVec;
		fs.release();
	}

	int frameNum = 1;
	int i=0;
	string num;

	while (1) 
	{
		Mat Image, ImageCopy;
		i++;
		ostringstream convert;
		convert << i;
		num = convert.str();
		Image = imread(".\\" + num + ".bmp",0);
	
		/*getline(in, line);
	
		
	
		Image = imread("E:\\Yu\\Ycontinuous\\" + line + "");*/
	
		//Image = imread("E:\\2019\\Yu\\12-6\\2048-2048\\x-jiaozhen\\6.bmp",0);
		flip(Image,Image, 1);
	
		PLOGD << "frame" << frameNum++ << endl;

		vector<int> Ids;
		vector<vector<Point2f>> Corners, Rejected;
		vector<Vec3d> Rvecs, Tvecs;

		//if (!newimage.empty()) {
		//	int col = newimage[4].x;
		//	int row = newimage[4].y;
		//	Mat Image1(row, col, CV_8UC1, cv::Scalar(0, 0, 0));
		//	for (int i = 0; i < row; i++) {
		//		for (int j = 0; j < col; j++) {
		//			Image1.data[i*col + j] = Image.data[cols*(i+newimage[0].y) + j+newimage[0].x];
		//	}
		//	}
		//	aruco::detectMarkers(Image1, dictionary, Corners, Ids, detectorParams, Rejected);
		//	if (Ids.size() > 0) {
		//		for (int i = 0; i < Corners[0].size(); i++) {
		//			Corners[0][i].x += newimage[0].x;
		//			Corners[0][i].y += newimage[0].y;
		//		}
		//	}
		//	else {
		//		aruco::detectMarkers(Image, dictionary, Corners, Ids, detectorParams, Rejected);
		//	}
		//}
		//else {

		//	// detect markers and estimate pose
		//	aruco::detectMarkers(Image, dictionary, Corners, Ids, detectorParams, Rejected);
		//}
		//newimage.clear();
		//if (Ids.size() > 0) {
		//	
		//	int32_t xmax = 0, xmin = INT32_MAX, ymax = 0, ymin = INT32_MAX;
		//	for (int i = 0; i < Corners[0].size(); i++)
		//	{
		//		int x = Corners[0][i].x;
		//		int y = Corners[0][i].y;
		//		if (xmax < x) {
		//			xmax = x;
		//		}
		//		if (xmin > x) {
		//			xmin = x;
		//		}
		//		if (ymax < y) {
		//			ymax = y;
		//		}
		//		if (ymin > y) {
		//			ymin = y;
		//		}
		//	}
		//	 
		//	newimage.push_back(Point2i(xmin - 30, ymin - 30));
		//	newimage.push_back(Point2i(xmax + 30, ymin - 30));
		//	newimage.push_back(Point2i(xmin - 30, ymax + 30));
		//	newimage.push_back(Point2i(xmax + 30, ymax + 30));
		//	newimage.push_back(Point2i(xmax - xmin + 1 + 30 * 2, ymax - ymin + 1 + 30 * 2));
		//}

		detectorParams->cornerRefinementMethod = 1;
		aruco::detectMarkers(Image, dictionary, Corners, Ids, detectorParams, Rejected);
		if (Ids.size() > 0) {
		
			int32_t xmax = 0, xmin = INT32_MAX, ymax = 0, ymin = INT32_MAX;
			for (int i = 0; i < Corners[0].size(); i++)
			{
				int x = Corners[0][i].x;
				int y = Corners[0][i].y;
				if (xmax < x) {
					xmax = x;
				}
				if (xmin > x) {
					xmin = x;
				}
				if (ymax < y) {
					ymax = y;
				}
				if (ymin > y) {
					ymin = y;
				}
			}
		 
			newimage.push_back(Point2i(xmin - 20, ymin - 20));
			newimage.push_back(Point2i(xmax + 20, ymin - 20));
			newimage.push_back(Point2i(xmin - 20, ymax + 20));
			newimage.push_back(Point2i(xmax + 20, ymax + 20));
			newimage.push_back(Point2i(xmax - xmin + 1 + 20 * 2, ymax - ymin + 1 + 20 * 2));
		}
		detectorParams->cornerRefinementMethod = 3;
		if (!newimage.empty()) {
			int col = newimage[4].x;
			int row = newimage[4].y;
			Mat Image1(row, col, CV_8UC1, cv::Scalar(0, 0, 0));
			for (int i = 0; i < row; i++) {
				for (int j = 0; j < col; j++) {
					Image1.data[i*col + j] = Image.data[cols*(i+newimage[0].y) + j+newimage[0].x];
			}
			}
			aruco::detectMarkers(Image1, dictionary, Corners, Ids, detectorParams, Rejected);
			//imwrite("Image1.bmp", Image1);
			if (Ids.size() > 0) {
				for (int i = 0; i < Corners[0].size(); i++) {
					Corners[0][i].x += newimage[0].x;
					Corners[0][i].y += newimage[0].y;
				}
			}
			else {
				aruco::detectMarkers(Image, dictionary, Corners, Ids, detectorParams, Rejected);
			}
		}
		else {

			// detect markers and estimate pose
			aruco::detectMarkers(Image, dictionary, Corners, Ids, detectorParams, Rejected);
		}
		newimage.clear();

		// aruco::detectMarkers(Image, dictionary, Corners, Ids, detectorParams, Rejected);
		if (estimatePose && Ids.size() > 0)
			aruco::estimatePoseSingleMarkers(Corners, markerLength, camMatrix, distCoeffs, Rvecs,
				Tvecs);


		// draw results
		Image.copyTo(ImageCopy);
		if (Ids.size() > 0) {

			aruco::drawDetectedMarkers(ImageCopy, Corners, Ids);

			Mat RVec, TVec;
			vector<Point3f> Obj;

			Obj.push_back(digitsWorldPos[0][0]);    //right-down conrner

			Obj.push_back(digitsWorldPos[0][1]);  //left-down conrner

			Obj.push_back(digitsWorldPos[1][1]);    //left-up conrner

			Obj.push_back(digitsWorldPos[1][0]);      //right-up conrner
			cout << "right-down conrner:" << Obj[0] << endl;
			vector<Point2f> ImgPos = Corners[0];

			solvePnP(Obj, ImgPos, camMatrix, distCoeffs, RVec, TVec, false, SOLVEPNP_P3P);
			PLOGD << "ImgPos" << ImgPos<< endl;
			Mat Rot;
			Rodrigues(RVec, Rot);

			/*Rot = Rot.t();
			TVec = (-1)*Rot*TVec;*/

			cout << "Rot" << Rot << endl;
			cout << "TVec" << TVec << endl;

			Mat rot, tVec;
			FileStorage fs("1.txt", FileStorage::READ);
			fs["rot"] >> rot;
			fs["tVec"] >> tVec;
			fs.release();
			cout << "rot" << rot << endl;
			cout << "tVec" << tVec << endl;

			Mat R, T;
			R = rot*Rot;
			T = rot*TVec + tVec;
			double x=T.at<double>(0);
			double y = T.at<double>(1);
			double z = T.at<double>(2);
			double  s = sqrt(x*x + y*y + z*z) ;

	//		PLOGD << "R" << R << endl;
			PLOGD << "T" << T << endl;
			PLOGD << "s=" << s << endl;
		
		//	out << s * 10 << endl;
			cout << "R" << R << endl;
			cout << "T" << T << endl;

			FileStorage Fs("result.txt", FileStorage::WRITE);
			Fs << "R" << R;
			Fs << "T" << T;
			Fs.release();
		}

	
		imshow("out", ImageCopy); 
		char key = (char)waitKey();
		if (key == 27) break;

    }
	
	 //Serial_close();//关闭串口
     return 0;
}
