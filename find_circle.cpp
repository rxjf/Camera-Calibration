//=========================����任��Բ����̫�ȶ�=================================

//int TestA(void) {
//	
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
//	Mat frame_0;
//	Mat imageSource;
//	Mat grayImage;
//	vector<Vec3f> circles;
//	Point center;
//	int radius;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
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
//		char c = waitKey(100); //�ȴ�ʱ��50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//	return 0;
//}

//=========================������ȡ��ĳ�ַ����������ҳ�Բ��ûʲô��==========================

//int TestA(void) {
//
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
//	Mat frame_0;
//	vector<Vec3f> circles;
//	Point center;
//	int radius;
//
//	Mat img2, img3, img4;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//		//imshow("camera_0", frame_0);
//
//		cvtColor(frame_0, img2, COLOR_BGR2GRAY);
//		GaussianBlur(img2, img3, Size(9, 9), 2, 2);
//		//threshold(img2, img3, 90, 255, THRESH_BINARY);//ͼ���ֵ����ע����ֵ�仯
//		namedWindow("detecte circles1", CV_NORMAL);
//		imshow("detecte circles1", img3);
//		//Canny(img3, img3, 50, 100);//��Ե���
//		namedWindow("detect circles2", CV_NORMAL);
//		imshow("detect circles2", img3);
//		vector<vector<Point>>contours;
//		vector<Vec4i>hierarchy;
//		findContours(img3, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//�������
//		int index = 0;
//		for (; index >= 0; index = hierarchy[index][0])
//		{
//			Scalar color(rand() & 255, rand() & 255, rand() & 255);//�����ɫ
//			drawContours(frame_0, contours, index, color, CV_FILLED, 8, hierarchy);//����������������ͼƬ��
//		}
//
//		namedWindow("detected circles3", CV_NORMAL);
//		imshow("detected circles3", frame_0);
//		//��׼Բ��ͼƬ��һ������Բ�����Բ���OpenCV�������Բ�ķ����������
//		Mat pointsf;
//		Mat(contours[0]).convertTo(pointsf, CV_32F);
//		RotatedRect box = fitEllipse(pointsf);
//		circle(frame_0, box.center, 20, Scalar(0, 255, 0), 2, 8, 0);
//		namedWindow("detected circles4", CV_NORMAL);
//		imshow("detected circles4", frame_0);
//		//cout << box.center;
//		//waitKey();
//		char c = waitKey(2000); //�ȴ�ʱ��50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//
//	return 0;
//}

//==========================�����ȡHSVֵ========================

//Mat hsvImage;
//
//void onMouse(int event, int x, int y, int, void*)
//{
//	switch (event)
//	{
//	case cv::EVENT_LBUTTONDOWN:   //�����ť���µ��¼�
//		cout << hsvImage.at<Vec3b>(x,y) << endl; //һ��Ҫ��ȫ�ֱ����������ᱨ��
//		break;
//	}
//}
//
//int TestA(void) {
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
//	capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
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

//==========================�����ȡHSVֵ========================

//Mat hsvImage;
//
//void onMouse(int event, int x, int y, int, void*)
//{
//	switch (event)
//	{
//	case cv::EVENT_LBUTTONDOWN:   //�����ť���µ��¼�
//		cout << hsvImage.at<Vec3b>(x,y) << endl; //һ��Ҫ��ȫ�ֱ����������ᱨ��
//		break;
//	}
//}
//
//int TestA(void) {
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
//	capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
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

//=============ʹ��HSV���ض���ɫ���ж�ֵ�����ٻ�����Բ��ok, perfect=============

//int TestA(void) {
//
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
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
//		capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//		cvtColor(frame_0, hsvImage, COLOR_BGR2HSV);
//
//		split(hsvImage, hsvSplit);//(��ͨ������-������)
//		equalizeHist(hsvSplit[2], hsvSplit[2]);
//		merge(hsvSplit, hsvImage);
//
//		inRange(hsvImage, Scalar(minh, mins, minv), Scalar(maxh, maxs, maxv), imgThresholded);//��ֵ��
//
//																							  //imshow("hsvImage2", hsvImage);
//
//																							  //������ (ȥ��һЩ���)
//		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//		//imshow("imgThresholded", imgThresholded);//�˹���ɫ
//		//�ղ��� (����һЩ��ͨ��)
//		//morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
//
//		//addWeighted(imgThresholded, 1, canny_output, 1, 0., dstImage);
//		//morphologyEx(dstImage, dstImage, MORPH_CLOSE, element);
//
//		//��Բ************************************************************************************************************
//
//		blur(imgThresholded, bf, Size(3, 3));
//		//��Canny���Ӽ���Ե
//		Canny(bf, canny_output, g_nThresh, g_nThresh * 2, 3);
//		//������ (ȥ��һЩ���)
//		//morphologyEx(canny_output, canny_output, MORPH_OPEN, element);
//
//		//�ղ��� (����һЩ��ͨ��)
//		//morphologyEx(canny_output, canny_output, MORPH_CLOSE, element);
//		//imshow("canny_output", canny_output);
//
//		vector<Vec3f> circles;//����һ�����������������Բ��Բ������Ͱ뾶
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
//		char c = waitKey(50); //�ȴ�ʱ��50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//	return 0;
//}

//========================������ȡ��ĳ�ַ�����, ûʲô��=========================

//int TestA(void) {
//
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
//
//	Mat frame_0, bgr0[3];
//	double levels = 90;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//		
//		split(frame_0, bgr0);
//		Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
//		dilate(bgr0[1], bgr0[1], element); //����(dilate)
//		erode(bgr0[1], bgr0[1], element);//��ʴ(erode)
//
//		Mat blured;
//		blur(bgr0[1], blured, Size(7, 7));//�˲����������ø�˹�˲�����
//		imshow("blured",blured);
//		Mat binary;
//		threshold(blured, binary, levels, 0xff, THRESH_BINARY);//��ֵ��
//		imshow("binary", binary);
//
//		if (levels > 10) {
//			vector<vector<Point>> contours;
//			vector<Vec4i> hierarchy;
//			findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
//			Mat imageContours = Mat::zeros(binary.size(), CV_8UC1);
//			Mat Contours = Mat::zeros(binary.size(), CV_8UC1);  //����
//
//			//Mat drawing = frame_0.clone();
//			for (int i = 0; i < contours.size(); i++)
//			{
//				//contours[i]������ǵ�i��������contours[i].size()������ǵ�i�����������е����ص���
//				for (int j = 0; j < contours[i].size(); j++)
//				{
//					//���Ƴ�contours���������е����ص�
//					Point P = Point(contours[i][j].x, contours[i][j].y);
//					Contours.at<uchar>(P) = 255;
//				}
//				
//				//���hierarchy��������
//				char ch[256];
//				string str = ch;
//				cout << "����hierarchy�ĵ�" << str << " ��Ԫ������Ϊ��" << endl << hierarchy[i] << endl << endl;
//				
//				//��������
//				drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
//			}
//			imshow("Contours Image", imageContours); //����
//			imshow("Point of Contours", Contours);   //����contours�ڱ�������������㼯
//				
//			waitKey();
//		}
//
//		char c = waitKey(1000); //�ȴ�ʱ��50ms
//		if (c == 27) break;
//	}
//	capture_0.release();
//	return 0;
//}