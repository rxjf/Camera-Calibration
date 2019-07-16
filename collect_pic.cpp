#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "collect_pic.h"

using namespace cv;
using namespace std;

int MonocularCollectChessPic(void)
{
	VideoCapture capture(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大

	if (!capture.isOpened()) {
		cout << "Could not open camera" << endl;
		return -1;
	}

	Mat frame;
	int pic_counter = 0;
	std::stringstream StrStm;

	while (capture.read(frame)) ///读取视频帧放入frame
	{
		imshow("camera", frame);
		char c = waitKey(50); //等待时间50ms
		if (c == 13) {
			StrStm.clear();
			StrStm.str("");
			pic_counter++;
			StrStm << "monocular_chessboard_pic/picture" << pic_counter << ".bmp";
			cout << StrStm.str() << endl;
			imwrite(StrStm.str(), frame); //enter
		}
		if (c == 27) break; //esc
	}
	capture.release();

	return 0;
}

int MonocularCollectCirclePic(void)
{
	VideoCapture capture(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大

	if (!capture.isOpened()) {
		cout << "Could not open camera" << endl;
		return -1;
	}

	Mat frame;
	int pic_counter = 0;
	std::stringstream StrStm;

	while (capture.read(frame)) ///读取视频帧放入frame
	{
		imshow("camera", frame);
		char c = waitKey(50); //等待时间50ms
		if (c == 13) {
			StrStm.clear();
			StrStm.str("");
			pic_counter++;
			StrStm << "monocular_circle_pic/pic" << pic_counter << ".bmp";
			cout << StrStm.str() << endl;
			imwrite(StrStm.str(), frame); //enter
		}
		if (c == 27) break; //esc
	}
	capture.release();

	return 0;
}

int BinocularCollectChessPic(void)
{
	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
	VideoCapture capture_1(1);

	if (!capture_0.isOpened()) {
		cout << "Could not open camera_0" << endl;
		return -1;
	}
	if (!capture_1.isOpened()) {
		cout << "Could not open camera_1" << endl;
		return -1;
	}

	Mat frame_0, frame_1;
	int pic_counter = 0;
	std::stringstream StrStm;

	while (true)
	{
		capture_0.read(frame_0);///读取视频帧放入frame
		capture_1.read(frame_1);///读取视频帧放入frame
		imshow("camera_0", frame_0);
		imshow("camera_1", frame_1);

		char c = waitKey(50); //等待时间50ms
		if (c == 13) {//enter
			pic_counter++;

			StrStm.clear();
			StrStm.str("");
			StrStm << "binocular_chessboard_pic/my_left" << pic_counter << ".bmp";
			cout << StrStm.str() << endl;
			imwrite(StrStm.str(), frame_0);

			StrStm.clear();
			StrStm.str("");
			StrStm << "binocular_chessboard_pic/my_right" << pic_counter << ".bmp";
			cout << StrStm.str() << endl;
			imwrite(StrStm.str(), frame_1);
		}
		if (c == 27) break; //esc
	}
	capture_0.release();
	capture_1.release();

	return 0;
}

int BinocularCollectCirclePic(void)
{	
	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
	VideoCapture capture_1(1);

	if (!capture_0.isOpened()) {
		cout << "Could not open camera_0" << endl;
		return -1;
	}
	if (!capture_1.isOpened()) {
		cout << "Could not open camera_1" << endl;
		return -1;
	}

	Mat frame_0,frame_1;
	int pic_counter = 0;
	std::stringstream StrStm;

	while(true)
	{
		capture_0.read(frame_0);///读取视频帧放入frame
		capture_1.read(frame_1);///读取视频帧放入frame
		imshow("camera_0", frame_0);
		imshow("camera_1", frame_1);

		char c = waitKey(50); //等待时间50ms
		if (c == 13){//enter
			pic_counter++;
			
			StrStm.clear();
			StrStm.str("");			
			StrStm <<"binocular_circle_pic/my_left"<< pic_counter<<".bmp";
			cout << StrStm.str() << endl;
			imwrite(StrStm.str(), frame_0); 

			StrStm.clear();
			StrStm.str("");
			StrStm << "binocular_circle_pic/my_right" << pic_counter << ".bmp";
			cout << StrStm.str() << endl;
			imwrite(StrStm.str(), frame_1);
		}
		if (c == 27) break; //esc
	}
	capture_0.release();
	capture_1.release();

	return 0;
}

void GeneratingChessboardPic(void)
{
	Mat frame = imread("3A4.bmp"); // cols*rows = 630*891  

	int nc = frame.channels();

	int nWidthOfROI = 90;

	for (int j = 0; j < frame.rows; j++)
	{
		uchar* data = frame.ptr<uchar>(j);
		for (int i = 0; i < frame.cols*nc; i += nc)
		{
			if ((i / nc / nWidthOfROI + j / nWidthOfROI) % 2)
			{
				// bgr  
				data[i / nc*nc + 0] = 255;
				data[i / nc*nc + 1] = 255;
				data[i / nc*nc + 2] = 255;
			}
		}
	}

	imshow("test", frame);
	waitKey(0);
	imwrite("A4.bmp", frame);
}
