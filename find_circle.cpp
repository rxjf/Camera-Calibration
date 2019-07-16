//=========================霍夫变换找圆，不太稳定=================================

//int TestA(void) {
//	
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//	Mat frame_0;
//	Mat imageSource;
//	Mat grayImage;
//	vector<Vec3f> circles;
//	Point center;
//	int radius;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		//imshow("camera_0", frame_0);
//
//		cvtColor(frame_0, imageSource, COLOR_BGR2GRAY);
//		GaussianBlur(imageSource, grayImage, Size(9, 9), 2,2);
//		imshow("grayImage", grayImage);
//		HoughCircles(grayImage, circles, HOUGH_GRADIENT, 2, 640, 100, 30, 20, 50);
//		
//		for (size_t i = 0; i < circles.size(); i++) {
//			center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
//			radius = cvRound(circles[i][2]);
//			circle(frame_0, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//			circle(frame_0, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//		}
//		imshow("circles", frame_0);
//
//		char c = waitKey(100); //等待时间50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//	return 0;
//}

//=========================轮廓提取（某种方法），再找出圆，没什么用==========================

//int TestA(void) {
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//	Mat frame_0;
//	vector<Vec3f> circles;
//	Point center;
//	int radius;
//
//	Mat img2, img3, img4;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		//imshow("camera_0", frame_0);
//
//		cvtColor(frame_0, img2, COLOR_BGR2GRAY);
//		GaussianBlur(img2, img3, Size(9, 9), 2, 2);
//		//threshold(img2, img3, 90, 255, THRESH_BINARY);//图像二值化，注意阈值变化
//		namedWindow("detecte circles1", CV_NORMAL);
//		imshow("detecte circles1", img3);
//		//Canny(img3, img3, 50, 100);//边缘检测
//		namedWindow("detect circles2", CV_NORMAL);
//		imshow("detect circles2", img3);
//		vector<vector<Point>>contours;
//		vector<Vec4i>hierarchy;
//		findContours(img3, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//检测轮廓
//		int index = 0;
//		for (; index >= 0; index = hierarchy[index][0])
//		{
//			Scalar color(rand() & 255, rand() & 255, rand() & 255);//随机颜色
//			drawContours(frame_0, contours, index, color, CV_FILLED, 8, hierarchy);//将所有轮廓画出在图片上
//		}
//
//		namedWindow("detected circles3", CV_NORMAL);
//		imshow("detected circles3", frame_0);
//		//标准圆在图片上一般是椭圆，所以采用OpenCV中拟合椭圆的方法求解中心
//		Mat pointsf;
//		Mat(contours[0]).convertTo(pointsf, CV_32F);
//		RotatedRect box = fitEllipse(pointsf);
//		circle(frame_0, box.center, 20, Scalar(0, 255, 0), 2, 8, 0);
//		namedWindow("detected circles4", CV_NORMAL);
//		imshow("detected circles4", frame_0);
//		//cout << box.center;
//		//waitKey();
//		char c = waitKey(2000); //等待时间50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//
//	return 0;
//}

//==========================点击获取HSV值========================

//Mat hsvImage;
//
//void onMouse(int event, int x, int y, int, void*)
//{
//	switch (event)
//	{
//	case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
//		cout << hsvImage.at<Vec3b>(x,y) << endl; //一定要是全局变量，变量会报错
//		break;
//	}
//}
//
//int TestA(void) {
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//	capture_0.read(frame_0);///读取视频帧放入frame
//	
//	imshow("camera_0", frame_0);
//	setMouseCallback("camera_0", onMouse, 0);
//
//	cvtColor(frame_0, hsvImage, COLOR_BGR2HSV);
//	
//	waitKey();
//	capture_0.release();
//	return 0;
//}

//==========================点击获取HSV值========================

//Mat hsvImage;
//
//void onMouse(int event, int x, int y, int, void*)
//{
//	switch (event)
//	{
//	case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
//		cout << hsvImage.at<Vec3b>(x,y) << endl; //一定要是全局变量，变量会报错
//		break;
//	}
//}
//
//int TestA(void) {
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//	capture_0.read(frame_0);///读取视频帧放入frame
//	
//	imshow("camera_0", frame_0);
//	setMouseCallback("camera_0", onMouse, 0);
//
//	cvtColor(frame_0, hsvImage, COLOR_BGR2HSV);
//	
//	waitKey();
//	capture_0.release();
//	return 0;
//}

//=============使用HSV对特定颜色进行二值化，再霍夫找圆，ok, perfect=============

//int TestA(void) {
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//
//	vector<Vec3f> circles;
//	Point center;
//	int radius;
//
//	int minh = 35, maxh = 77, mins = 43, maxs = 255, minv = 46, maxv = 255;
//	Mat frame_0, hsvImage, imgThresholded, bf, canny_output;
//	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
//	int g_nThresh = 100;
//	vector<Mat> hsvSplit;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		cvtColor(frame_0, hsvImage, COLOR_BGR2HSV);
//
//		split(hsvImage, hsvSplit);//(多通道数组-》容器)
//		equalizeHist(hsvSplit[2], hsvSplit[2]);
//		merge(hsvSplit, hsvImage);
//
//		inRange(hsvImage, Scalar(minh, mins, minv), Scalar(maxh, maxs, maxv), imgThresholded);//二值化
//
//																							  //imshow("hsvImage2", hsvImage);
//
//																							  //开操作 (去除一些噪点)
//		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//		//imshow("imgThresholded", imgThresholded);//滤过颜色
//		//闭操作 (连接一些连通域)
//		//morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
//
//		//addWeighted(imgThresholded, 1, canny_output, 1, 0., dstImage);
//		//morphologyEx(dstImage, dstImage, MORPH_CLOSE, element);
//
//		//找圆************************************************************************************************************
//
//		blur(imgThresholded, bf, Size(3, 3));
//		//用Canny算子检测边缘
//		Canny(bf, canny_output, g_nThresh, g_nThresh * 2, 3);
//		//开操作 (去除一些噪点)
//		//morphologyEx(canny_output, canny_output, MORPH_OPEN, element);
//
//		//闭操作 (连接一些连通域)
//		//morphologyEx(canny_output, canny_output, MORPH_CLOSE, element);
//		//imshow("canny_output", canny_output);
//
//		vector<Vec3f> circles;//声明一个向量，保存检测出的圆的圆心坐标和半径
//							  //HoughCircles(canny_output, circles, CV_HOUGH_GRADIENT, 2, 640, 100, 100, 5, 100);
//		HoughCircles(canny_output, circles, HOUGH_GRADIENT, 2, 640, 100, 30, 10, 200);
//
//		for (size_t i = 0; i < circles.size(); i++) {
//			center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
//			radius = cvRound(circles[i][2]);
//			circle(frame_0, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//			circle(frame_0, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//		}
//		imshow("circles", frame_0);
//
//		char c = waitKey(50); //等待时间50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//	return 0;
//}

//========================轮廓提取（某种方法）, 没什么用=========================

//int TestA(void) {
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//
//	Mat frame_0, bgr0[3];
//	double levels = 90;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		
//		split(frame_0, bgr0);
//		Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
//		dilate(bgr0[1], bgr0[1], element); //膨胀(dilate)
//		erode(bgr0[1], bgr0[1], element);//腐蚀(erode)
//
//		Mat blured;
//		blur(bgr0[1], blured, Size(7, 7));//滤波，还不如用高斯滤波得了
//		imshow("blured",blured);
//		Mat binary;
//		threshold(blured, binary, levels, 0xff, THRESH_BINARY);//二值化
//		imshow("binary", binary);
//
//		if (levels > 10) {
//			vector<vector<Point>> contours;
//			vector<Vec4i> hierarchy;
//			findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
//			Mat imageContours = Mat::zeros(binary.size(), CV_8UC1);
//			Mat Contours = Mat::zeros(binary.size(), CV_8UC1);  //绘制
//
//			//Mat drawing = frame_0.clone();
//			for (int i = 0; i < contours.size(); i++)
//			{
//				//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
//				for (int j = 0; j < contours[i].size(); j++)
//				{
//					//绘制出contours向量内所有的像素点
//					Point P = Point(contours[i][j].x, contours[i][j].y);
//					Contours.at<uchar>(P) = 255;
//				}
//				
//				//输出hierarchy向量内容
//				char ch[256];
//				string str = ch;
//				cout << "向量hierarchy的第" << str << " 个元素内容为：" << endl << hierarchy[i] << endl << endl;
//				
//				//绘制轮廓
//				drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
//			}
//			imshow("Contours Image", imageContours); //轮廓
//			imshow("Point of Contours", Contours);   //向量contours内保存的所有轮廓点集
//				
//			waitKey();
//		}
//
//		char c = waitKey(1000); //等待时间50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//	return 0;
//}