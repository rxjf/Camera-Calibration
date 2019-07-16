#include "binocular_chessboard.h"

static Mat xyz;

static void onMouse(int event, int x, int y, int, void*)
{
	cv::Point origin;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
		origin = cv::Point(x, y);
		//origin = cv::Point(0, 0);
		xyz.at<cv::Vec3f>(origin)[2] += 2;
		std::cout << origin << "in world coordinate is: " << xyz.at<cv::Vec3f>(origin) << std::endl;
		break;
	}
}


static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

//===============新相机在线采集图片-->标定-->矫正映射-->在线截取一张图片矫正测试与鼠标点击得到空间坐标================

int BinocularChess(void)
{
	Size boardSize;
	string imagelistfn;

	bool displayCorners = true;
	bool useCalibrated = true;
	bool showRectified = true;
	bool useYmlData = true;

	boardSize.width = 8;//9
	boardSize.height = 5;//6
	const float squareSize = 25;

	//=================================================
	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大

	if (!capture_0.isOpened()) {
		cout << "Could not open camera_0" << endl;
		return -1;
	}

	capture_0.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	capture_0.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	Mat frame, frame_0, frame_1;
	Rect rect_0(0, 0, 640, 480);
	Rect rect_1(640, 0, 640, 480);
	//=================================================

	Mat rmap[2][2];
	Mat R1, R2, P1, P2, Q; //P 3*4
	Mat cameraMatrix[2], distCoeffs[2];
	Size imageSize;
	
	if (!useYmlData) {
		int pic_counter = 0;
		bool found1 = false;
		bool found2 = false;
		vector<Point2f> corners1;
		vector<Point2f> corners2;
		vector<vector<Point2f> > imagePoints[2];
		Mat img1,img2;

		cout << "1.开始截取标定图片:" << endl;
		cout << "    -->按enter截图, 按esc停止截图" << endl;

		while (true)
		{
			//分成两个
			capture_0.read(frame);	
			frame_0 = frame(rect_0);
			frame_1 = frame(rect_1);
			imshow("camera_0", frame_0);
			imshow("camera_1", frame_1);

			char c = waitKey(50); //等待时间50ms
			if (c == 13) {//enter		
				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//彩色、灰度均可
				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

				if (found1&&found2) {
					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
							30, 0.01));//必须传入灰度图
					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
							30, 0.01));
					drawChessboardCorners(frame_0, boardSize, corners1, found1);//彩色图，便于观看
					drawChessboardCorners(frame_1, boardSize, corners2, found1);
					imshow("camera_0_corner", frame_0);
					imshow("camera_1_corner", frame_1);
					cout << "    截图成功！" << endl;
					cout << "    角点提取成功！" << endl;
					cout << "    -->按enter确认保留, 按其他不保留" << endl;
					c = waitKey();
					if (c == 13) {
						pic_counter++;
						cout << "<-----------第" << pic_counter << "张图片角点提取已被保留----------->" << endl;
						cout << "    -->按enter继续截图, 按esc停止截图" << endl;
						imagePoints[0].push_back(corners1);
						imagePoints[1].push_back(corners2);
					}
				}
				else {
					cout << "    未能找到角点，请重新截取" << endl;
				}
			}
			if (c == 27) {
				cout << "视频截取结束" << endl << endl;
				break; //esc
			}
		}

		cout << "2.开始标定"<<endl;

		vector<vector<Point3f> > objectPoints;
		objectPoints.resize(pic_counter);//世界坐标

		for (int i = 0; i < pic_counter; i++)
		{
			for (int j = 0; j < boardSize.height; j++)
				for (int k = 0; k < boardSize.width; k++)
					objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
		}
	
		imageSize = frame_0.size();

		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// 得到初始化的摄像机矩阵K，3*3，经验证和单目标定结果一样
		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// 但两个相机内参不一样
		Mat R, T, E, F;

		//该函数必须要提供初始摄像机矩阵
		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			CALIB_FIX_ASPECT_RATIO + //固定fx/fy的比值
			CALIB_ZERO_TANGENT_DIST + //切向畸变参数（p1,p2）为零
			CALIB_USE_INTRINSIC_GUESS + //使用fx,fy,cx,cy估计值；因为经过了标定，所以要使用该模式
			CALIB_SAME_FOCAL_LENGTH + //强制保持两个摄像机的焦距相同
			CALIB_RATIONAL_MODEL + //启用畸变系数k4,k5和k6，加上k1,k2,k3,p1,p2总共8个系数
			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//对应的径向畸变在优化中保持不变，0，不优化
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
		cout << "    标定误差为：" << rms << endl;
		///<经P150程序验证，E = t^R， F = K2^-T * E * K1^-1 

		///< 前面跟单目都差不多

		// CALIBRATION QUALITY CHECK
		// because the output fundamental matrix implicitly
		// includes all the output information,
		// we can check the quality of calibration using the
		// epipolar geometry constraint对极几何约束: m2^t*F*m1=0
		// 得到了匹配的特征点后，想画出外极线，来验证一下匹配的结果是否正确，只是验证，对后面矫正没有关系

		double err = 0;
		int npoints = 0;
		vector<Vec3f> lines[2];
		for (int i = 0; i < pic_counter; i++)
		{
			int npt = (int)imagePoints[0][i].size(); // imagePoints存的是角点的像素坐标
			Mat imgpt[2];
			for (int k = 0; k < 2; k++)
			{
				imgpt[k] = Mat(imagePoints[k][i]);
				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//未变形的点，矫正后的点
																											///<当畸变系数和内外参数矩阵标定完成后，就应该进行畸变的矫正，以达到消除畸变的目的
				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//上面考虑了畸变，所以这里就是这样，哈哈
				//这里的for循环计算F*p1或FT*p2，然后后面for循环再乘p2或p1不就OK吗
			}
			for (int j = 0; j < npt; j++)
			{
				double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + 
					imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
					fabs(imagePoints[1][i][j].x*lines[0][j][0] +
						imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
				err += errij;
			}
			npoints += npt;
		}
		cout << "    平均对极线误差：" << err / npoints << endl;

		// save intrinsic parameters内参 ******外参(extrinsics)
		FileStorage fs("binocular_chessboard_pic/intrinsics.yml", FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
			fs.release();
		}
		else
			cout << "    Error: can not save the intrinsic parameters\n";

		Rect validRoi[2];

		cout << endl;

		cout << "3.求解立体矫正映射"<< endl;
		//要通过两幅图像估计物点的深度信息，就必须在两幅图像中准确的匹配到同一物点，这样才能根据该物点在两幅图像中的位置关系，计算物体深度。
		//为了降低匹配的计算量，两个摄像头的成像平面应处于同一平面(旋转即可)。但是，单单依靠严格的摆放摄像头来达到这个目的显然有些困难。
		//立体校正就是利用几何图形变换关系，使得原先不满足上述位置关系的两幅图像满足该条件。
		//https://www.cnblogs.com/german-iris/p/5199203.html
		stereoRectify(cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
		///<stereoRectify立体矫正。其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正（图像自己）所需要的旋转矩阵R1,R2
		///<和摄像机在新坐标系下的投影矩阵P1, P2，3*4, 就是以前的变换矩阵T了，其作用是将3D点的坐标转换到2维像素坐标:P*[X Y Z 1]' =[x y w] 
		///<Q为4*4的深度差异映射矩阵，重投影矩阵，即矩阵Q可以把2维像素坐标投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差
		//经验证 R1 = R2*R； P1和P2只存在t的区别，想想可知，因为已经共面了； Q就是见含义; 就这样吧，后面自己实际测量！！！
		///<flags: 0(水平或垂直地移动图像，以使得其有用的范围最大) 或 CV_CALIB_ZERO_DISPARITY(会让两幅校正后的图像的主点有相同的像素坐标)
		///<alpha: 拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
		///<imageSize其实是newImageSize，校正后的图像大小，一般跟原图像相同
		///<validPixROI1，validPixROI2-见函数帮助

		fs.open("binocular_chessboard_pic/extrinsics.yml", FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "imageSize"<< imageSize <<"R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
			fs.release();
		}
		else
			cout << "    Error: can not save the extrinsic parameters\n";

		// OpenCV can handle left-right
		// or up-down camera arrangements
		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//算了，以后再说吧

																					  // COMPUTE AND DISPLAY RECTIFICATION，下面都是显示纠正后的图
		if (!showRectified)
			return -1;

		// IF BY CALIBRATED (BOUGUET'S METHOD)
		if (useCalibrated)
		{
			// we already computed everything
		}
		// OR ELSE HARTLEY'S METHOD
		else //第二种方式，通过单应矩阵求解R1、R2、P1、P2
			 // use intrinsic parameters of each camera, but
			 // compute the rectification纠正 transformation变换 directly
			 // from the fundamental matrix
		{
			vector<Point2f> allimgpt[2];
			for (int k = 0; k < 2; k++)
			{
				for (int i = 0; i < pic_counter; i++)
					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
			}
			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
			Mat H1, H2;//单应矩阵，H与R和K之间关系，slam P146
			stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

			R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
			R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
			P1 = cameraMatrix[0];
			P2 = cameraMatrix[1];
		}
		cout << endl;
	}
	else {
		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //读的形式打开yml。当然也可以打开xml，主要看后缀
		FileStorage fs2("binocular_chessboard_pic/intrinsics.yml", FileStorage::READ); 

		if (fs1.isOpened()&& fs2.isOpened())
		{
			fs1["imageSize"] >> imageSize;
			fs1["R1"] >> R1;
			fs1["R2"] >> R2;
			fs1["P1"] >> P1;
			fs1["P2"] >> P2;
			fs1["Q"] >> Q;
			
			fs2["M1"] >> cameraMatrix[0];
			fs2["D1"] >> distCoeffs[0];
			fs2["M2"] >> cameraMatrix[1];
			fs2["D2"] >> distCoeffs[1];
		}

		fs1.release();
		fs2.release();
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	//该函数功能是计算畸变矫正和立体校正的映射变换，其中P1应该传入是纠正过后的newCameraMatrix
	//在openCV里面，校正后的计算机矩阵newCameraMatrix是跟投影矩阵P一起返回的。
	//所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);


	////使用SGBM算法验证结果,是一种立体匹配算法，准确度和速度适中，工程中比较常用
	////显示视差图，必须用灰度图像	
	
	cout << "4.矫正测试与鼠标点击得到空间坐标" << endl;
	Mat imageL, imageR;

	cout << "    -->按enter截图" << endl;

	while (true)
	{
		capture_0.read(frame);
		frame_0 = frame(rect_0);
		frame_1 = frame(rect_1);
		imshow("camera_0", frame_0);
		imshow("camera_1", frame_1);

		char c = waitKey(50); //等待时间50ms
		if (c == 13) {//enter		
			cvtColor(frame_0, imageL, COLOR_BGR2GRAY);
			cvtColor(frame_1, imageR, COLOR_BGR2GRAY);
			cout << "    视频截取结束!" << endl;
			break;
		}
	}
	capture_0.release();

	cout << "    开始矫正!" << endl;
	cv::Mat rectifyImageL, rectifyImageR;
	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
	//imshow("rectifyImageL", rectifyImageL);
	//imshow("rectifyImageR", rectifyImageR);
	///Q与remap的区别：像素图--remap-->纠正后的像素图---->视差--Q-->3D坐标

	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	int sgbmWinSize = 3;
	int cn = imageL.channels();    //
	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //结果为80；视差范围，即最大视差值和最小视差值之差, 必须是16的倍数，所以后面&-16

	sgbm->setPreFilterCap(63);//映射滤波器大小
	sgbm->setBlockSize(sgbmWinSize);//3*3
	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1计算公式
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);//最小视差
	sgbm->setNumDisparities(numberOfDisparities);//视差搜索范围
	sgbm->setUniquenessRatio(10);//唯一性比率
	sgbm->setSpeckleWindowSize(50);//视差连通区域像素点个数阈值
	sgbm->setSpeckleRange(32);//视差连通条件/阈值
	sgbm->setDisp12MaxDiff(1);//左右一致性检测最大容许误差阈值
	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

	cv::Mat disp, disp8;

	sgbm->compute(rectifyImageL, rectifyImageR, disp);//默认计算出的是左视差图，即以左图为底板，以视差为灰度值
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));//将16位符号整形的视差矩阵转换为8位无符号整形矩阵，???
	cout << "    显示视差图" << endl;
	cout << "    -->通过鼠标点击图像上的点可以得到3D坐标!!!" << endl;
	cv::imshow("disparity8", disp8); //显示视差图

	cv::reprojectImageTo3D(disp, xyz, Q, true);//视差就是用Q求
	xyz = xyz * 16; // xyz=[X/W Y/W Z/W]，乘以16得到真实坐标
	cv::setMouseCallback("disparity8", onMouse, 0);//对disparity8视差图进行鼠标回调

	for (int i = 20; i < rectifyImageL.rows; i += 20)
	{
		cv::line(rectifyImageL, cv::Point(0, i), cv::Point(rectifyImageL.cols, i), cv::Scalar(255, 255, 255));
		cv::line(rectifyImageR, cv::Point(0, i), cv::Point(rectifyImageL.cols, i), cv::Scalar(255, 255, 255));
	}
	cv::Mat imageMatches;
	cv::drawMatches(
		rectifyImageL, std::vector<cv::KeyPoint>(),  // 1st image
		rectifyImageR, std::vector<cv::KeyPoint>(),              // 2nd image
		std::vector<cv::DMatch>(),
		imageMatches,                       // the image produced
		cv::Scalar(255, 255, 255),
		cv::Scalar(255, 255, 255),
		std::vector<char>(),
		DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cout << "    显示矫正图" << endl;
	cv::imshow("imageMatches", imageMatches);
	cv::setMouseCallback("imageMatches", onMouse, 0);
	
	waitKey();

	return 0;
}


//================新相机在线采集图片-->标定-->矫正映射-->实时显示发光球3D坐标(霍夫圆查找)=======================

//int BinocularChess(void)
//{
//	Size boardSize;
//	string imagelistfn;
//
//	bool displayCorners = true;
//	bool useCalibrated = true;
//	bool showRectified = true;
//	bool useYmlData = true;
//
//	boardSize.width = 8;//9
//	boardSize.height = 5;//6
//	const float squareSize = 25;
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//
//	if (!capture_0.isOpened()) {
//		cout << "Could not open camera_0" << endl;
//		return -1;
//	}
//
//	capture_0.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//	capture_0.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//
//	Mat frame, frame_0, frame_1;
//	Mat rmap[2][2];
//	Mat R1, R2, P1, P2, Q; //P 3*4
//	Mat cameraMatrix[2], distCoeffs[2];
//	Size imageSize;
//	Rect rect_0(0, 0, 640, 480);
//	Rect rect_1(640, 0, 640, 480);
//
//	if (!useYmlData) {
//		int pic_counter = 0;
//		bool found1 = false;
//		bool found2 = false;
//		vector<Point2f> corners1;// = imagePoints[k][j];
//		vector<Point2f> corners2;
//		vector<vector<Point2f> > imagePoints[2];
//		Mat img1, img2;	
//
//		cout << "1.开始截取标定图片:" << endl;
//		cout << "    -->按enter截图, 按esc停止截图" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame);///读取视频帧放入frame
//
//			//分成两个		
//			frame_0 = frame(rect_0);
//			frame_1 = frame(rect_1);
//			imshow("camera_0", frame_0);
//			imshow("camera_1", frame_1);
//
//			char c = waitKey(50); //等待时间50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//彩色、灰度均可
//				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//
//				if (found1&&found2) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//必须传入灰度图
//					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//彩色图，便于观看
//					drawChessboardCorners(frame_1, boardSize, corners2, found1);
//					imshow("camera_0_corner", frame_0);
//					imshow("camera_1_corner", frame_1);
//					cout << "    截图成功！" << endl;
//					cout << "    角点提取成功！" << endl;
//					cout << "    -->按enter确认保留, 按其他不保留" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------第" << pic_counter << "张图片角点提取已被保留----------->" << endl;
//						cout << "    -->按enter继续截图, 按esc停止截图" << endl;
//						imagePoints[0].push_back(corners1);
//						imagePoints[1].push_back(corners2);
//					}
//				}
//				else {
//					cout << "    未能找到角点，请重新截取" << endl;
//				}
//			}
//			if (c == 27) {
//				cout << "视频截取结束" << endl << endl;
//				break; //esc
//			}
//		}
//
//		cout << "2.开始标定" << endl;
//
//		vector<vector<Point3f> > objectPoints;
//		objectPoints.resize(pic_counter);//世界坐标
//
//		for (int i = 0; i < pic_counter; i++)
//		{
//			for (int j = 0; j < boardSize.height; j++)
//				for (int k = 0; k < boardSize.width; k++)
//					objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
//		}
//
//		imageSize = frame_0.size();
//
//		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// 得到初始化的摄像机矩阵K，3*3，经验证该值和单目标定结果不一样
//		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// 两个大小不一样
//		Mat R, T, E, F;
//
//		//该函数必须要提供初始摄像机矩阵
//		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//			cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, E, F,
//			CALIB_FIX_ASPECT_RATIO + //固定fx/fy的比值
//			CALIB_ZERO_TANGENT_DIST + //切向畸变参数（p1,p2）为零
//			CALIB_USE_INTRINSIC_GUESS + //使用fx,fy,cx,cy估计值；因为经过了标定，所以要使用该模式？
//			CALIB_SAME_FOCAL_LENGTH + //强制保持两个摄像机的焦距相同
//			CALIB_RATIONAL_MODEL + //启用畸变系数k4,k5和k6，加上k1,k2,k3,p1,p2总共8个系数
//			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//对应的径向畸变在优化中保持不变，0，不优化
//			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//		cout << "    标定误差为：" << rms << endl;
//		///<经P150程序验证，E = t^R，但是 F ≠ K2^-T * E * K1^-1 为什么，算了
//
//		///< 前面跟单目都差不多
//
//		// CALIBRATION QUALITY CHECK
//		// because the output fundamental matrix implicitly
//		// includes all the output information,
//		// we can check the quality of calibration using the
//		// epipolar geometry constraint对极几何约束: m2^t*F*m1=0
//		// 得到了匹配的特征点后，想画出外极线，来验证一下匹配的结果是否正确，只是验证，对后面矫正没有关系
//
//		double err = 0;
//		int npoints = 0;
//		vector<Vec3f> lines[2];
//		for (int i = 0; i < pic_counter; i++)
//		{
//			int npt = (int)imagePoints[0][i].size(); // imagePoints存的是角点的像素坐标
//			Mat imgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				imgpt[k] = Mat(imagePoints[k][i]);
//				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//未变形的点，矫正后的点
//																											///<当畸变系数和内外参数矩阵标定完成后，就应该进行畸变的矫正，以达到消除畸变的目的
//				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//上面考虑了畸变，所以这里就是这样，哈哈
//				//这里的for循环计算F*p1或FT*p2，然后后面for循环再乘p2或p1不就OK吗
//			}
//			for (int j = 0; j < npt; j++)
//			{
//				double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + //|a1*x0+b1*y0+c1|+|a0*x1+b0*y1+c0|,对，按道理来说应该等于0，所以是误差
//					imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
//					fabs(imagePoints[1][i][j].x*lines[0][j][0] +
//						imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
//				err += errij;
//			}
//			npoints += npt;
//		}
//		cout << "    平均对极线误差：" << err / npoints << endl;
//
//		// save intrinsic parameters内参 ******外参(extrinsics)
//		FileStorage fs("binocular_chessboard_pic/intrinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the intrinsic parameters\n";
//
//		Rect validRoi[2];
//
//		cout << endl;
//
//		cout << "3.求解立体矫正映射" << endl;
//		//要通过两幅图像估计物点的深度信息，就必须在两幅图像中准确的匹配到同一物点，这样才能根据该物点在两幅图像中的位置关系，计算物体深度。
//		//为了降低匹配的计算量，两个摄像头的成像平面应处于同一平面(旋转即可)。但是，单单依靠严格的摆放摄像头来达到这个目的显然有些困难。
//		//立体校正就是利用几何图形变换关系，使得原先不满足上述位置关系的两幅图像满足该条件。
//		//https://www.cnblogs.com/german-iris/p/5199203.html
//		stereoRectify(cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, R1, R2, P1, P2, Q,
//			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//		///<stereoRectify立体矫正。其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正（图像自己）所需要的旋转矩阵R1,R2
//		///<和摄像机在新坐标系下的投影矩阵P1, P2，3*4, 就是以前的变换矩阵T了，其作用是将3D点的坐标转换到2维像素坐标:P*[X Y Z 1]' =[x y w] 
//		///<Q为4*4的深度差异映射矩阵，重投影矩阵，即矩阵Q可以把2维像素坐标投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差
//		//经验证 R1 = R2*R； P1和P2只存在t的区别，想想可知，因为已经共面了； Q就是见含义; 就这样吧，后面自己实际测量！！！
//		///<flags: 0(水平或垂直地移动图像，以使得其有用的范围最大) 或 CV_CALIB_ZERO_DISPARITY(会让两幅校正后的图像的主点有相同的像素坐标)
//		///<alpha: 拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
//		///<imageSize其实是newImageSize，校正后的图像大小，一般跟原图像相同
//		///<validPixROI1，validPixROI2-见函数帮助
//
//		fs.open("binocular_chessboard_pic/extrinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "imageSize" << imageSize << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the extrinsic parameters\n";
//
//		// OpenCV can handle left-right
//		// or up-down camera arrangements
//		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//算了，以后再说吧
//
//																					  // COMPUTE AND DISPLAY RECTIFICATION，下面都是显示纠正后的图
//		if (!showRectified)
//			return -1;
//
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useCalibrated)
//		{
//			// we already computed everything
//		}
//		// OR ELSE HARTLEY'S METHOD
//		else //第二种方式，通过单应矩阵求解R1、R2、P1、P2
//			 // use intrinsic parameters of each camera, but
//			 // compute the rectification纠正 transformation变换 directly
//			 // from the fundamental matrix
//		{
//			vector<Point2f> allimgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				for (int i = 0; i < pic_counter; i++)
//					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//			}
//			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//			Mat H1, H2;//单应矩阵，H与R和K之间关系，slam P146
//			stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
//
//			R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
//			R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
//			P1 = cameraMatrix[0];
//			P2 = cameraMatrix[1];
//		}
//		cout << endl;
//	}
//	else {
//		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //读的形式打开yml。当然也可以打开xml，主要看后缀
//		FileStorage fs2("binocular_chessboard_pic/intrinsics.yml", FileStorage::READ);
//
//		if (fs1.isOpened() && fs2.isOpened())
//		{
//			fs1["imageSize"] >> imageSize;
//			fs1["R1"] >> R1;
//			fs1["R2"] >> R2;
//			fs1["P1"] >> P1;
//			fs1["P2"] >> P2;
//			fs1["Q"] >> Q;
//
//			fs2["M1"] >> cameraMatrix[0];
//			fs2["D1"] >> distCoeffs[0];
//			fs2["M2"] >> cameraMatrix[1];
//			fs2["D2"] >> distCoeffs[1];
//		}
//
//		fs1.release();
//		fs2.release();
//	}
//
//	//Precompute maps for cv::remap()
//	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
//	//该函数功能是计算畸变矫正和立体校正的映射变换，其中P1应该传入是纠正过后的newCameraMatrix
//	//在openCV里面，校正后的计算机矩阵newCameraMatrix是跟投影矩阵P一起返回的。
//	//所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
//	//投影矩阵跟摄像机矩阵到底什么关系？？
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//
//	////使用SGBM算法验证结果,是一种立体匹配算法，准确度和速度适中，工程中比较常用
//	////显示视差图，必须用灰度图像
//
//	cout << "4.目标追踪" << endl;
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = 1;//imageL.channels()
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //结果为80；视差范围，即最大视差值和最小视差值之差, 必须是16的倍数，所以后面&-16
//
//	sgbm->setPreFilterCap(63);//映射滤波器大小
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1计算公式
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//最小视差
//	sgbm->setNumDisparities(numberOfDisparities);//视差搜索范围
//	sgbm->setUniquenessRatio(10);//唯一性比率
//	sgbm->setSpeckleWindowSize(50);//视差连通区域像素点个数阈值
//	sgbm->setSpeckleRange(32);//视差连通条件/阈值
//	sgbm->setDisp12MaxDiff(1);//左右一致性检测最大容许误差阈值
//	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//
//	cv::Mat disp, disp8;
//	Mat imageL, imageR;
//	cv::Mat rectifyImageL, rectifyImageR;
//	Point center;
//	int radius;
//	Mat grayImage;
//	vector<Vec3f> circles;
//	int minh = 35, maxh = 77, mins = 43, maxs = 255, minv = 46, maxv = 255;
//	Mat hsvImage, imgThresholded, bf, canny_output;
//	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
//	int g_nThresh = 100;
//
//	while (true)
//	{
//		capture_0.read(frame);///读取视频帧放入frame
//
//		//分成两个		
//		frame_0 = frame(rect_0);
//		frame_1 = frame(rect_1);
//		//imshow("camera_0", frame_0);
//		//imshow("camera_1", frame_1);
//
//		cvtColor(frame_0, imageL, COLOR_BGR2GRAY);
//		cvtColor(frame_1, imageR, COLOR_BGR2GRAY);
//
//		cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
//		cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
//
//		sgbm->compute(rectifyImageL, rectifyImageR, disp);
//		cv::reprojectImageTo3D(disp, xyz, Q, true);
//		xyz = xyz * 16;
//
//		//cvtColor(frame_0, hsvImage, COLOR_BGR2HSV);
//		//inRange(hsvImage, Scalar(minh, mins, minv), Scalar(maxh, maxs, maxv), imgThresholded);//二值化
//		//morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//		//blur(imgThresholded, bf, Size(3, 3));
//		//Canny(bf, canny_output, g_nThresh, g_nThresh * 2, 3);
//		//GaussianBlur(canny_output, grayImage, Size(9, 9), 2, 2);
//		
//		GaussianBlur(imageL, grayImage, Size(9, 9), 2,2);
//		HoughCircles(grayImage, circles, HOUGH_GRADIENT, 2, 640, 100, 30, 20, 50);//还必须加上颜色比较好
//
//																				  //for (size_t i = 0; i < circles.size(); i++) {
//																				  //	center = Point (cvRound(circles[i][0]), cvRound(circles[i][1]));
//																				  //	radius = cvRound(circles[i][2]);
//																				  //	circle(frame_0, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//																				  //	circle(frame_0, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//																				  //}
//
//		//if (center!= Point(cvRound(circles[0][0]), cvRound(circles[0][1]))) {
//		center = Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
//		radius = cvRound(circles[0][2]);
//
//		xyz.at<cv::Vec3f>(center)[2] += 2;
//		std::cout << "    球心的3D坐标为: " << xyz.at<cv::Vec3f>(center) << std::endl;
//
//		circle(frame_0, center, 2, Scalar(0, 255, 0), -1, 8, 0);
//		//draw the circle outline 
//		circle(frame_0, center, radius, Scalar(0, 0, 255), 2, 8, 0);
//
//		imshow("circles", frame_0);
//		//}	
//
//		char c = waitKey(50); //等待时间50ms
//		if (c == 27) {
//			break;
//		}
//	}
//	capture_0.release();
//
//	return 0;
//}

//===========从文件中读取官方图片-->标定-->矫正映射-->与鼠标点击得到空间坐标============

//int BinocularChess(void)
//{
//    Size boardSize;
//    string imagelistfn;
//    bool showRectified=true;
//	bool displayCorners = true;
//	bool useCalibrated = true;
//
//    imagelistfn = "binocular_chessboard_pic/stereo_calib.xml";
//    boardSize.width = 9;//8
//    boardSize.height = 6;//5
//
//    vector<string> imagelist;
//    bool ok = readStringList(imagelistfn, imagelist);
//    if(!ok || imagelist.empty())
//    {
//        cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
//    }
//
//	if (imagelist.size() % 2 != 0)
//	{
//		cout << "Error: the image list contains odd (non-even) number of elements\n";
//		return -1;
//	}
//
//	const int maxScale = 2;
//	const float squareSize = 25;  // Set this to your actual square size
//								  // ARRAY AND VECTOR STORAGE:
//
//	vector<vector<Point2f> > imagePoints[2];//定义"[2]==>"2个, "Point2f==>"2维数组, 动态，个数未知
//	vector<vector<Point3f> > objectPoints;//定义1个3维数组
//	Size imageSize;
//
//	int i, j, k, nimages = (int)imagelist.size() / 2; //只对最后一个赋值
//
//	imagePoints[0].resize(nimages);//扩展维度为nimages，新扩展的初始化为0
//	imagePoints[1].resize(nimages);
//	vector<string> goodImageList;
//
//	for (i = j = 0; i < nimages; i++)
//	{
//		for (k = 0; k < 2; k++)
//		{
//			const string& filename = imagelist[i * 2 + k];
//			Mat img = imread(filename, 0);
//			if (img.empty())
//				break;
//			if (imageSize == Size())
//				imageSize = img.size();
//			else if (img.size() != imageSize)
//			{
//				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
//				break;
//			}
//			bool found = false;
//			vector<Point2f>& corners = imagePoints[k][j];//相当于指针，所以下面没有pushback
//			for (int scale = 1; scale <= maxScale; scale++)
//			{
//				Mat timg;
//				if (scale == 1)
//					timg = img;
//				else
//					resize(img, timg, Size(), scale, scale);//img经过缩放，输出给timg
//															//对象.resize() <==不同于==> resize()
//															//Alt + G 查看
//															//缩放的目的：未能找到角点时，放大原图重新寻找一次
//				found = findChessboardCorners(timg, boardSize, corners,
//					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//				if (found)
//				{
//					if (scale > 1)
//					{
//						Mat cornersMat(corners); //定义一个内容为corners的cornersMat变量
//						cornersMat *= 1. / scale;//缩放回去
//					}
//					break;
//				}
//			}
//			if (displayCorners)
//			{
//				cout << filename << endl;
//				Mat cimg, cimg1;
//				cvtColor(img, cimg, COLOR_GRAY2BGR);
//				drawChessboardCorners(cimg, boardSize, corners, found);
//				double sf = 640. / MAX(img.rows, img.cols);
//				resize(cimg, cimg1, Size(), sf, sf);
//				imshow("corners", cimg1);
//				char c = (char)waitKey(500);
//				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
//					exit(-1);
//			}
//			else
//				putchar('.');
//			if (!found)
//				break;
//			//亚像素精确化，如果用find4QuadCornerSubpix需要灰度图
//			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
//				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//					30, 0.01));
//		}
//		if (k == 2)
//		{
//			goodImageList.push_back(imagelist[i * 2]);
//			goodImageList.push_back(imagelist[i * 2 + 1]);
//			j++;
//		}
//	}
//	cout << j << " pairs have been successfully detected.\n";
//	nimages = j;
//	if (nimages < 2)
//	{
//		cout << "Error: too little pairs to run the calibration\n";
//		return -1;
//	}
//
//	objectPoints.resize(nimages);//因为左右是一样的，所以共用一个objectPoint
//
//	for (i = 0; i < nimages; i++)
//	{
//		for (j = 0; j < boardSize.height; j++)
//			for (k = 0; k < boardSize.width; k++)
//				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
//	}
//
//	cout << "Running stereo calibration ...\n";
//
//	Mat cameraMatrix[2], distCoeffs[2];
//	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// 得到初始化的摄像机矩阵K，3*3，经验证和单目标定结果不一样
//	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// 两个大小不一样
//	Mat R, T, E, F;
//
//	//该函数必须要提供初始摄像机矩阵
//	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//		cameraMatrix[0], distCoeffs[0],
//		cameraMatrix[1], distCoeffs[1],
//		imageSize, R, T, E, F,
//		CALIB_FIX_ASPECT_RATIO + //固定fx/fy的比值
//		CALIB_ZERO_TANGENT_DIST + //切向畸变参数（p1,p2）为零
//		CALIB_USE_INTRINSIC_GUESS + //使用fx,fy,cx,cy估计值；因为经过了标定，所以要使用该模式？
//		CALIB_SAME_FOCAL_LENGTH + //强制保持两个摄像机的焦距相同
//		CALIB_RATIONAL_MODEL + //启用畸变系数k4,k5和k6，加上k1,k2,k3,p1,p2总共8个系数
//		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//对应的径向畸变在优化中保持不变，0，不优化
//		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//	cout << "done with RMS error=" << rms << endl;
//	///<经P150程序验证，E = t^R，但是 F ≠ K2^-T * E * K1^-1 为什么，算了
//
//	///< 前面跟单目都差不多
//
//	// CALIBRATION QUALITY CHECK
//	// because the output fundamental matrix implicitly
//	// includes all the output information,
//	// we can check the quality of calibration using the
//	// epipolar geometry constraint对极几何约束: m2^t*F*m1=0
//	// 得到了匹配的特征点后，想画出外极线，来验证一下匹配的结果是否正确，只是验证，对后面矫正没有关系
//
//	double err = 0;
//	int npoints = 0;
//	vector<Vec3f> lines[2];
//	for (i = 0; i < nimages; i++)
//	{
//		int npt = (int)imagePoints[0][i].size(); // imagePoints存的是角点的像素坐标
//		Mat imgpt[2];
//		for (k = 0; k < 2; k++)
//		{
//			imgpt[k] = Mat(imagePoints[k][i]);
//			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//未变形的点，矫正后的点
//			///<当畸变系数和内外参数矩阵标定完成后，就应该进行畸变的矫正，以达到消除畸变的目的
//			///<不使用的，可以传入空矩阵
//			///<imagePoints也会被修改！！！
//			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//上面考虑了畸变，所以这里就是这样，哈哈
//			//这里的for循环计算F*p1或FT*p2，然后后面for循环再乘p2或p1不就OK吗
//		}
//		for (j = 0; j < npt; j++)
//		{
//			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + 
//				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
//				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
//					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
//			err += errij;
//		}
//		npoints += npt;
//	}
//	cout << "average epipolar err = " << err / npoints << endl;
//
//	// save intrinsic parameters内参
//  //******外参(extrinsics)
//	FileStorage fs("../data/intrinsics.yml", FileStorage::WRITE);
//	if (fs.isOpened())
//	{
//		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
//			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
//		fs.release();
//	}
//	else
//		cout << "Error: can not save the intrinsic parameters\n";
//
//	Mat R1, R2, P1, P2, Q; //P 3*4
//	Rect validRoi[2];
//
//	//要通过两幅图像估计物点的深度信息，就必须在两幅图像中准确的匹配到同一物点，这样才能根据该物点在两幅图像中的位置关系，计算物体深度。
//	//为了降低匹配的计算量，两个摄像头的成像平面应处于同一平面(旋转即可)。但是，单单依靠严格的摆放摄像头来达到这个目的显然有些困难。
//	//立体校正就是利用几何图形变换关系，使得原先不满足上述位置关系的两幅图像满足该条件。
//	//https://www.cnblogs.com/german-iris/p/5199203.html
//	stereoRectify(cameraMatrix[0], distCoeffs[0],
//		cameraMatrix[1], distCoeffs[1],
//		imageSize, R, T, R1, R2, P1, P2, Q,
//		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//	///<stereoRectify立体矫正。其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正（图像自己）所需要的旋转矩阵R1,R2
//	///<和摄像机在新坐标系下的投影矩阵P1, P2，3*4, 就是以前的变换矩阵T了，其作用是将3D点的坐标转换到2维像素坐标:P*[X Y Z 1]' =[x y w] 
//	///<Q为4*4的深度差异映射矩阵，重投影矩阵，即矩阵Q可以把2维像素坐标投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差
//	//经验证 R1 = R2*R； P1和P2只存在t的区别，想想可知，因为已经共面了； Q就是见含义; 就这样吧，后面自己实际测量！！！
//	///<flags: 0(水平或垂直地移动图像，以使得其有用的范围最大) 或 CV_CALIB_ZERO_DISPARITY(会让两幅校正后的图像的主点有相同的像素坐标)
//	///<alpha: 拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
//	///<imageSize其实是newImageSize，校正后的图像大小，一般跟原图像相同
//	///<validPixROI1，validPixROI2-见函数帮助
//
//	fs.open("extrinsics.yml", FileStorage::WRITE);
//	if (fs.isOpened())
//	{
//		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//		fs.release();
//	}
//	else
//		cout << "Error: can not save the extrinsic parameters\n";
//
//	// OpenCV can handle left-right
//	// or up-down camera arrangements
//	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//算了，以后再说吧
//
//																				  // COMPUTE AND DISPLAY RECTIFICATION，下面都是显示纠正后的图
//	if (!showRectified)
//		return -1;
//
//	Mat rmap[2][2];
//	// IF BY CALIBRATED (BOUGUET'S METHOD)
//	if (useCalibrated)
//	{
//		// we already computed everything
//	}
//	// OR ELSE HARTLEY'S METHOD
//	else //第二种方式，通过单应矩阵求解R1、R2、P1、P2
//		 // use intrinsic parameters of each camera, but
//		 // compute the rectification纠正 transformation变换 directly
//		 // from the fundamental matrix
//	{
//		vector<Point2f> allimgpt[2];
//		for (k = 0; k < 2; k++)
//		{
//			for (i = 0; i < nimages; i++)
//				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//		}
//		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//		Mat H1, H2;//单应矩阵，H与R和K之间关系，slam P146
//		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
//
//		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
//		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
//		P1 = cameraMatrix[0];
//		P2 = cameraMatrix[1];
//	}
//
//	//Precompute maps for cv::remap()
//	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
//	///<该函数功能是计算畸变矫正和立体校正的映射变换，其中P1应该传入是纠正过后的newCameraMatrix
//	///<在openCV里面，校正后的计算机矩阵newCameraMatrix是跟投影矩阵P一起返回的。
//	///<所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//	//使用SGBM算法验证结果,是一种立体匹配算法，准确度和速度适中，工程中比较常用
//	//显示视差图，必须用灰度图像
//	Mat imageL = cv::imread("binocular_chessboard_pic/left01.bmp", 0);
//	Mat imageR = cv::imread("binocular_chessboard_pic/right01.bmp", 0);
//	cv::Mat rectifyImageL, rectifyImageR;
//	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
//	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = imageL.channels();    //
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //结果为80；视差范围，即最大视差值和最小视差值之差, 必须是16的倍数，所以后面&-16
//
//	sgbm->setPreFilterCap(63);//映射滤波器大小
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1计算公式
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//最小视差
//	sgbm->setNumDisparities(numberOfDisparities);//视差搜索范围
//	sgbm->setUniquenessRatio(10);//唯一性比率
//	sgbm->setSpeckleWindowSize(50);//视差连通区域像素点个数阈值
//	sgbm->setSpeckleRange(32);//视差连通条件/阈值
//	sgbm->setDisp12MaxDiff(1);//左右一致性检测最大容许误差阈值
//	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//
//	cv::Mat disp, disp8;
//
//	sgbm->compute(rectifyImageL, rectifyImageR, disp);//默认计算出的是左视差图，即以左图为底板，以视差为灰度值
//	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));//将16位符号整形的视差矩阵转换为8位无符号整形矩阵，???
//	cout << "显示视差图" << endl;
//	cv::imshow("disparity8", disp8); //显示视差图
//
//	cv::reprojectImageTo3D(disp, xyz, Q, true);
//	//xyz = xyz * 16; // xyz=[X/W Y/W Z/W]，乘以16得到真实坐标
//	cv::setMouseCallback("disparity8", onMouse, 0);//对disparity8视差图进行鼠标回调
//
//	for (int i = 20; i < rectifyImageL.rows; i += 20)
//	{
//		cv::line(rectifyImageL, cv::Point(0, i), cv::Point(rectifyImageL.cols, i), cv::Scalar(255, 255, 255));
//		cv::line(rectifyImageR, cv::Point(0, i), cv::Point(rectifyImageL.cols, i), cv::Scalar(255, 255, 255));
//	}
//	cv::Mat imageMatches;
//	cv::drawMatches(rectifyImageL, std::vector<cv::KeyPoint>(),  // 1st image
//		rectifyImageR, std::vector<cv::KeyPoint>(),              // 2nd image
//		std::vector<cv::DMatch>(),
//		imageMatches,                       // the image produced
//		cv::Scalar(255, 255, 255),
//		cv::Scalar(255, 255, 255),
//		std::vector<char>(),
//		2);
//
//	cv::imshow("imageMatches", imageMatches);
//	//cv::setMouseCallback("imageMatches", onMouse, 0);
//
//	waitKey();
//
//	return 0;
//}

//===============在线采集图片-->标定-->矫正映射-->实时显示发光球3D坐标(HSV颜色过滤+霍夫圆查找)=================

//int BinocularChess(void)
//{
//	Size boardSize;
//	string imagelistfn;
//
//	bool displayCorners = true;
//	bool useCalibrated = true;
//	bool showRectified = true;
//	bool useYmlData = false;
//
//	boardSize.width = 8;//9
//	boardSize.height = 5;//6
//	const float squareSize = 25;
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//	VideoCapture capture_1(1);
//
//	if (!capture_0.isOpened()) {
//		cout << "Could not open camera_0" << endl;
//		return -1;
//	}
//	if (!capture_1.isOpened()) {
//		cout << "Could not open camera_1" << endl;
//		return -1;
//	}
//
//	Mat frame_0, frame_1;
//	Mat rmap[2][2];
//	Mat R1, R2, P1, P2, Q; //P 3*4
//	Mat cameraMatrix[2], distCoeffs[2];
//	Size imageSize;
//	
//	if (!useYmlData) {
//		int pic_counter = 0;
//		bool found1 = false;
//		bool found2 = false;
//		vector<Point2f> corners1;// = imagePoints[k][j];
//		vector<Point2f> corners2;
//		vector<vector<Point2f> > imagePoints[2];
//		Mat img1,img2;		
//
//		cout << "1.开始截取标定图片:" << endl;
//		cout << "    -->按enter截图, 按esc停止截图" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame_0);///读取视频帧放入frame
//			capture_1.read(frame_1);///读取视频帧放入frame
//			imshow("camera_0", frame_0);
//			imshow("camera_1", frame_1);		
//
//			char c = waitKey(50); //等待时间50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//彩色、灰度均可
//				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//
//				if (found1&&found2) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//必须传入灰度图
//					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//彩色图，便于观看
//					drawChessboardCorners(frame_1, boardSize, corners2, found1);
//					imshow("camera_0_corner", frame_0);
//					imshow("camera_1_corner", frame_1);
//					cout << "    截图成功！" << endl;
//					cout << "    角点提取成功！" << endl;
//					cout << "    -->按enter确认保留, 按其他不保留" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------第" << pic_counter << "张图片角点提取已被保留----------->" << endl;
//						cout << "    -->按enter继续截图, 按esc停止截图" << endl;
//						imagePoints[0].push_back(corners1);
//						imagePoints[1].push_back(corners2);
//					}
//				}
//				else {
//					cout << "    未能找到角点，请重新截取" << endl;
//				}
//			}
//			if (c == 27) {
//				cout << "视频截取结束" << endl << endl;
//				break; //esc
//			}
//		}
//
//		cout << "2.开始标定"<<endl;
//
//		vector<vector<Point3f> > objectPoints;
//		objectPoints.resize(pic_counter);//世界坐标
//
//		for (int i = 0; i < pic_counter; i++)
//		{
//			for (int j = 0; j < boardSize.height; j++)
//				for (int k = 0; k < boardSize.width; k++)
//					objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
//		}
//	
//		imageSize = frame_0.size();
//
//		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// 得到初始化的摄像机矩阵K，3*3，经验证该值和单目标定结果不一样
//		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// 两个大小不一样
//		Mat R, T, E, F;
//
//		//该函数必须要提供初始摄像机矩阵
//		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//			cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, E, F,
//			CALIB_FIX_ASPECT_RATIO + //固定fx/fy的比值
//			CALIB_ZERO_TANGENT_DIST + //切向畸变参数（p1,p2）为零
//			CALIB_USE_INTRINSIC_GUESS + //使用fx,fy,cx,cy估计值；因为经过了标定，所以要使用该模式？
//			CALIB_SAME_FOCAL_LENGTH + //强制保持两个摄像机的焦距相同
//			CALIB_RATIONAL_MODEL + //启用畸变系数k4,k5和k6，加上k1,k2,k3,p1,p2总共8个系数
//			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//对应的径向畸变在优化中保持不变，0，不优化
//			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//		cout << "    标定误差为：" << rms << endl;
//		///<经P150程序验证，E = t^R，但是 F ≠ K2^-T * E * K1^-1 为什么，算了
//
//		///< 前面跟单目都差不多
//
//		// CALIBRATION QUALITY CHECK
//		// because the output fundamental matrix implicitly
//		// includes all the output information,
//		// we can check the quality of calibration using the
//		// epipolar geometry constraint对极几何约束: m2^t*F*m1=0
//		// 得到了匹配的特征点后，想画出外极线，来验证一下匹配的结果是否正确，只是验证，对后面矫正没有关系
//
//		double err = 0;
//		int npoints = 0;
//		vector<Vec3f> lines[2];
//		for (int i = 0; i < pic_counter; i++)
//		{
//			int npt = (int)imagePoints[0][i].size(); // imagePoints存的是角点的像素坐标
//			Mat imgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				imgpt[k] = Mat(imagePoints[k][i]);
//				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//未变形的点，矫正后的点
//																											///<当畸变系数和内外参数矩阵标定完成后，就应该进行畸变的矫正，以达到消除畸变的目的
//				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//上面考虑了畸变，所以这里就是这样，哈哈
//				//这里的for循环计算F*p1或FT*p2，然后后面for循环再乘p2或p1不就OK吗
//			}
//			for (int j = 0; j < npt; j++)
//			{
//				double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + 
//					imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
//					fabs(imagePoints[1][i][j].x*lines[0][j][0] +
//						imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
//				err += errij;
//			}
//			npoints += npt;
//		}
//		cout << "    平均对极线误差：" << err / npoints << endl;
//
//		// save intrinsic parameters内参 ******外参(extrinsics)
//		FileStorage fs("binocular_chessboard_pic/intrinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the intrinsic parameters\n";
//
//		Rect validRoi[2];
//
//		cout << endl;
//
//		cout << "3.求解立体矫正映射"<< endl;
//		//要通过两幅图像估计物点的深度信息，就必须在两幅图像中准确的匹配到同一物点，这样才能根据该物点在两幅图像中的位置关系，计算物体深度。
//		//为了降低匹配的计算量，两个摄像头的成像平面应处于同一平面(旋转即可)。但是，单单依靠严格的摆放摄像头来达到这个目的显然有些困难。
//		//立体校正就是利用几何图形变换关系，使得原先不满足上述位置关系的两幅图像满足该条件。
//		//https://www.cnblogs.com/german-iris/p/5199203.html
//		stereoRectify(cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, R1, R2, P1, P2, Q,
//			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//		///<stereoRectify立体矫正。其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正（图像自己）所需要的旋转矩阵R1,R2
//		///<和摄像机在新坐标系下的投影矩阵P1, P2，3*4, 就是以前的变换矩阵T了，其作用是将3D点的坐标转换到2维像素坐标:P*[X Y Z 1]' =[x y w] 
//		///<Q为4*4的深度差异映射矩阵，重投影矩阵，即矩阵Q可以把2维像素坐标投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差
//		//经验证 R1 = R2*R； P1和P2只存在t的区别，想想可知，因为已经共面了； Q就是见含义; 就这样吧，后面自己实际测量！！！
//		///<flags: 0(水平或垂直地移动图像，以使得其有用的范围最大) 或 CV_CALIB_ZERO_DISPARITY(会让两幅校正后的图像的主点有相同的像素坐标)
//		///<alpha: 拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
//		///<imageSize其实是newImageSize，校正后的图像大小，一般跟原图像相同
//		///<validPixROI1，validPixROI2-见函数帮助
//
//		fs.open("binocular_chessboard_pic/extrinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "imageSize"<< imageSize <<"R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the extrinsic parameters\n";
//
//		// OpenCV can handle left-right
//		// or up-down camera arrangements
//		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//算了，以后再说吧
//
//																					  // COMPUTE AND DISPLAY RECTIFICATION，下面都是显示纠正后的图
//		if (!showRectified)
//			return -1;
//
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useCalibrated)
//		{
//			// we already computed everything
//		}
//		// OR ELSE HARTLEY'S METHOD
//		else //第二种方式，通过单应矩阵求解R1、R2、P1、P2
//			 // use intrinsic parameters of each camera, but
//			 // compute the rectification纠正 transformation变换 directly
//			 // from the fundamental matrix
//		{
//			vector<Point2f> allimgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				for (int i = 0; i < pic_counter; i++)
//					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//			}
//			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//			Mat H1, H2;//单应矩阵，H与R和K之间关系，slam P146
//			stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
//
//			R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
//			R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
//			P1 = cameraMatrix[0];
//			P2 = cameraMatrix[1];
//		}
//		cout << endl;
//	}
//	else {
//		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //读的形式打开yml。当然也可以打开xml，主要看后缀
//		FileStorage fs2("binocular_chessboard_pic/intrinsics.yml", FileStorage::READ); 
//
//		if (fs1.isOpened()&& fs2.isOpened())
//		{
//			fs1["imageSize"] >> imageSize;
//			fs1["R1"] >> R1;
//			fs1["R2"] >> R2;
//			fs1["P1"] >> P1;
//			fs1["P2"] >> P2;
//			fs1["Q"] >> Q;
//			
//			fs2["M1"] >> cameraMatrix[0];
//			fs2["D1"] >> distCoeffs[0];
//			fs2["M2"] >> cameraMatrix[1];
//			fs2["D2"] >> distCoeffs[1];
//		}
//
//		fs1.release();
//		fs2.release();
//	}
//
//	//Precompute maps for cv::remap()
//	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
//	//该函数功能是计算畸变矫正和立体校正的映射变换，其中P1应该传入是纠正过后的newCameraMatrix
//	//在openCV里面，校正后的计算机矩阵newCameraMatrix是跟投影矩阵P一起返回的。
//	//所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
//	//投影矩阵跟摄像机矩阵到底什么关系？？
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//
//	////使用SGBM算法验证结果,是一种立体匹配算法，准确度和速度适中，工程中比较常用
//	////显示视差图，必须用灰度图像
//
//	cout << "4.目标追踪" << endl;
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = 1;//imageL.channels()
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //结果为80；视差范围，即最大视差值和最小视差值之差, 必须是16的倍数，所以后面&-16
//
//	sgbm->setPreFilterCap(63);//映射滤波器大小
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1计算公式
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//最小视差
//	sgbm->setNumDisparities(numberOfDisparities);//视差搜索范围
//	sgbm->setUniquenessRatio(10);//唯一性比率
//	sgbm->setSpeckleWindowSize(50);//视差连通区域像素点个数阈值
//	sgbm->setSpeckleRange(32);//视差连通条件/阈值
//	sgbm->setDisp12MaxDiff(1);//左右一致性检测最大容许误差阈值
//	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//
//	cv::Mat disp, disp8;	
//	Mat imageL, imageR;
//	cv::Mat rectifyImageL, rectifyImageR;
//	Point center;
//	int radius;
//	Mat grayImage;
//	vector<Vec3f> circles;
//	int minh = 35, maxh = 77, mins = 43, maxs = 255, minv = 46, maxv = 255;
//	Mat hsvImage, imgThresholded,bf, canny_output;
//	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
//	int g_nThresh = 100;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		capture_1.read(frame_1);///读取视频帧放入frame
//		//imshow("camera_0", frame_0);
//		//imshow("camera_1", frame_1);
//			
//		cvtColor(frame_0, imageL, COLOR_BGR2GRAY);
//		cvtColor(frame_1, imageR, COLOR_BGR2GRAY);
//		
//		cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
//		cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
//
//		sgbm->compute(rectifyImageL, rectifyImageR, disp);
//		cv::reprojectImageTo3D(disp, xyz, Q, true);
//		xyz = xyz * 16;
//		
//		cvtColor(frame_0, hsvImage, COLOR_BGR2HSV);
//		inRange(hsvImage, Scalar(minh, mins, minv), Scalar(maxh, maxs, maxv), imgThresholded);//二值化
//		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//		blur(imgThresholded, bf, Size(3, 3));
//		Canny(bf, canny_output, g_nThresh, g_nThresh * 2, 3);
//		GaussianBlur(canny_output, grayImage, Size(9, 9), 2, 2);		
//		HoughCircles(grayImage, circles, HOUGH_GRADIENT, 2, 640, 100, 30, 20, 50);//还必须加上颜色比较好
//		
//		//for (size_t i = 0; i < circles.size(); i++) {
//		//	center = Point (cvRound(circles[i][0]), cvRound(circles[i][1]));
//		//	radius = cvRound(circles[i][2]);
//		//	circle(frame_0, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//		//	circle(frame_0, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//		//}
//		
//		//if (center!= Point(cvRound(circles[0][0]), cvRound(circles[0][1]))) {
//		center = Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
//		radius = cvRound(circles[0][2]);
//
//		xyz.at<cv::Vec3f>(center)[2] += 2;
//		std::cout << "    球心的3D坐标为: " << xyz.at<cv::Vec3f>(center) << std::endl;
//
//		circle(frame_0, center, 2, Scalar(0, 255, 0), -1, 8, 0);
//		//draw the circle outline 
//		circle(frame_0, center, radius, Scalar(0, 0, 255), 2, 8, 0);
//
//		imshow("circles", frame_0);
//		//}	
//
//		char c = waitKey(50); //等待时间50ms
//		if (c == 27) {
//			break;
//		}
//	}
//	capture_0.release();
//	capture_1.release();
//
//	return 0;
//}


//===============在线采集图片-->标定-->矫正映射-->在线截取一张图片矫正测试与鼠标点击得到空间坐标================

//int BinocularChess(void)
//{
//	Size boardSize;
//	string imagelistfn;
//
//	bool displayCorners = true;
//	bool useCalibrated = true;
//	bool showRectified = true;
//	bool useYmlData = true;
//
//	boardSize.width = 8;//9
//	boardSize.height = 5;//6
//	const float squareSize = 25;
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//	VideoCapture capture_1(1);
//
//	if (!capture_0.isOpened()) {
//		cout << "Could not open camera_0" << endl;
//		return -1;
//	}
//	if (!capture_1.isOpened()) {
//		cout << "Could not open camera_1" << endl;
//		return -1;
//	}
//
//	Mat frame_0, frame_1;
//	Mat rmap[2][2];
//	Mat R1, R2, P1, P2, Q; //P 3*4
//	Mat cameraMatrix[2], distCoeffs[2];
//	Size imageSize;
//	
//	if (!useYmlData) {
//		int pic_counter = 0;
//		bool found1 = false;
//		bool found2 = false;
//		vector<Point2f> corners1;// = imagePoints[k][j];
//		vector<Point2f> corners2;
//		vector<vector<Point2f> > imagePoints[2];
//		Mat img1,img2;
//
//		cout << "1.开始截取标定图片:" << endl;
//		cout << "    -->按enter截图, 按esc停止截图" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame_0);///读取视频帧放入frame
//			capture_1.read(frame_1);///读取视频帧放入frame
//			imshow("camera_0", frame_0);
//			imshow("camera_1", frame_1);
//
//			char c = waitKey(50); //等待时间50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//彩色、灰度均可
//				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//
//				if (found1&&found2) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//必须传入灰度图
//					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//彩色图，便于观看
//					drawChessboardCorners(frame_1, boardSize, corners2, found1);
//					imshow("camera_0_corner", frame_0);
//					imshow("camera_1_corner", frame_1);
//					cout << "    截图成功！" << endl;
//					cout << "    角点提取成功！" << endl;
//					cout << "    -->按enter确认保留, 按其他不保留" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------第" << pic_counter << "张图片角点提取已被保留----------->" << endl;
//						cout << "    -->按enter继续截图, 按esc停止截图" << endl;
//						imagePoints[0].push_back(corners1);
//						imagePoints[1].push_back(corners2);
//					}
//				}
//				else {
//					cout << "    未能找到角点，请重新截取" << endl;
//				}
//			}
//			if (c == 27) {
//				cout << "视频截取结束" << endl << endl;
//				break; //esc
//			}
//		}
//
//		cout << "2.开始标定"<<endl;
//
//		vector<vector<Point3f> > objectPoints;
//		objectPoints.resize(pic_counter);//世界坐标
//
//		for (int i = 0; i < pic_counter; i++)
//		{
//			for (int j = 0; j < boardSize.height; j++)
//				for (int k = 0; k < boardSize.width; k++)
//					objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
//		}
//	
//		imageSize = frame_0.size();
//
//		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// 得到初始化的摄像机矩阵K，3*3，经验证和单目标定结果不一样
//		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// 两个大小不一样
//		Mat R, T, E, F;
//
//		//该函数必须要提供初始摄像机矩阵
//		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//			cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, E, F,
//			CALIB_FIX_ASPECT_RATIO + //固定fx/fy的比值
//			CALIB_ZERO_TANGENT_DIST + //切向畸变参数（p1,p2）为零
//			CALIB_USE_INTRINSIC_GUESS + //使用fx,fy,cx,cy估计值；因为经过了标定，所以要使用该模式？
//			CALIB_SAME_FOCAL_LENGTH + //强制保持两个摄像机的焦距相同
//			CALIB_RATIONAL_MODEL + //启用畸变系数k4,k5和k6，加上k1,k2,k3,p1,p2总共8个系数
//			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//对应的径向畸变在优化中保持不变，0，不优化
//			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//		cout << "    标定误差为：" << rms << endl;
//		///<经P150程序验证，E = t^R，但是 F ≠ K2^-T * E * K1^-1 为什么，算了
//
//		///< 前面跟单目都差不多
//
//		// CALIBRATION QUALITY CHECK
//		// because the output fundamental matrix implicitly
//		// includes all the output information,
//		// we can check the quality of calibration using the
//		// epipolar geometry constraint对极几何约束: m2^t*F*m1=0
//		// 得到了匹配的特征点后，想画出外极线，来验证一下匹配的结果是否正确，只是验证，对后面矫正没有关系
//
//		double err = 0;
//		int npoints = 0;
//		vector<Vec3f> lines[2];
//		for (int i = 0; i < pic_counter; i++)
//		{
//			int npt = (int)imagePoints[0][i].size(); // imagePoints存的是角点的像素坐标
//			Mat imgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				imgpt[k] = Mat(imagePoints[k][i]);
//				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//未变形的点，矫正后的点
//																											///<当畸变系数和内外参数矩阵标定完成后，就应该进行畸变的矫正，以达到消除畸变的目的
//				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//上面考虑了畸变，所以这里就是这样，哈哈
//				//这里的for循环计算F*p1或FT*p2，然后后面for循环再乘p2或p1不就OK吗
//			}
//			for (int j = 0; j < npt; j++)
//			{
//				double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + 
//					imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
//					fabs(imagePoints[1][i][j].x*lines[0][j][0] +
//						imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
//				err += errij;
//			}
//			npoints += npt;
//		}
//		cout << "    平均对极线误差：" << err / npoints << endl;
//
//		// save intrinsic parameters内参 ******外参(extrinsics)
//		FileStorage fs("binocular_chessboard_pic/intrinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the intrinsic parameters\n";
//
//		Rect validRoi[2];
//
//		cout << endl;
//
//		cout << "3.求解立体矫正映射"<< endl;
//		//要通过两幅图像估计物点的深度信息，就必须在两幅图像中准确的匹配到同一物点，这样才能根据该物点在两幅图像中的位置关系，计算物体深度。
//		//为了降低匹配的计算量，两个摄像头的成像平面应处于同一平面(旋转即可)。但是，单单依靠严格的摆放摄像头来达到这个目的显然有些困难。
//		//立体校正就是利用几何图形变换关系，使得原先不满足上述位置关系的两幅图像满足该条件。
//		//https://www.cnblogs.com/german-iris/p/5199203.html
//		stereoRectify(cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, R1, R2, P1, P2, Q,
//			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//		///<stereoRectify立体矫正。其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正（图像自己）所需要的旋转矩阵R1,R2
//		///<和摄像机在新坐标系下的投影矩阵P1, P2，3*4, 就是以前的变换矩阵T了，其作用是将3D点的坐标转换到2维像素坐标:P*[X Y Z 1]' =[x y w] 
//		///<Q为4*4的深度差异映射矩阵，重投影矩阵，即矩阵Q可以把2维像素坐标投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差
//		//经验证 R1 = R2*R； P1和P2只存在t的区别，想想可知，因为已经共面了； Q就是见含义; 就这样吧，后面自己实际测量！！！
//		///<flags: 0(水平或垂直地移动图像，以使得其有用的范围最大) 或 CV_CALIB_ZERO_DISPARITY(会让两幅校正后的图像的主点有相同的像素坐标)
//		///<alpha: 拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
//		///<imageSize其实是newImageSize，校正后的图像大小，一般跟原图像相同
//		///<validPixROI1，validPixROI2-见函数帮助
//
//		fs.open("binocular_chessboard_pic/extrinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "imageSize"<< imageSize <<"R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the extrinsic parameters\n";
//
//		// OpenCV can handle left-right
//		// or up-down camera arrangements
//		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//算了，以后再说吧
//
//																					  // COMPUTE AND DISPLAY RECTIFICATION，下面都是显示纠正后的图
//		if (!showRectified)
//			return -1;
//
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useCalibrated)
//		{
//			// we already computed everything
//		}
//		// OR ELSE HARTLEY'S METHOD
//		else //第二种方式，通过单应矩阵求解R1、R2、P1、P2
//			 // use intrinsic parameters of each camera, but
//			 // compute the rectification纠正 transformation变换 directly
//			 // from the fundamental matrix
//		{
//			vector<Point2f> allimgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				for (int i = 0; i < pic_counter; i++)
//					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//			}
//			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//			Mat H1, H2;//单应矩阵，H与R和K之间关系，slam P146
//			stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
//
//			R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
//			R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
//			P1 = cameraMatrix[0];
//			P2 = cameraMatrix[1];
//		}
//		cout << endl;
//	}
//	else {
//		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //读的形式打开yml。当然也可以打开xml，主要看后缀
//		FileStorage fs2("binocular_chessboard_pic/intrinsics.yml", FileStorage::READ); 
//
//		if (fs1.isOpened()&& fs2.isOpened())
//		{
//			fs1["imageSize"] >> imageSize;
//			fs1["R1"] >> R1;
//			fs1["R2"] >> R2;
//			fs1["P1"] >> P1;
//			fs1["P2"] >> P2;
//			fs1["Q"] >> Q;
//			
//			fs2["M1"] >> cameraMatrix[0];
//			fs2["D1"] >> distCoeffs[0];
//			fs2["M2"] >> cameraMatrix[1];
//			fs2["D2"] >> distCoeffs[1];
//		}
//
//		fs1.release();
//		fs2.release();
//	}
//
//	//Precompute maps for cv::remap()
//	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
//	//该函数功能是计算畸变矫正和立体校正的映射变换，其中P1应该传入是纠正过后的newCameraMatrix
//	//在openCV里面，校正后的计算机矩阵newCameraMatrix是跟投影矩阵P一起返回的。
//	//所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
//	//投影矩阵跟摄像机矩阵到底什么关系？？
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//
//	////使用SGBM算法验证结果,是一种立体匹配算法，准确度和速度适中，工程中比较常用
//	////显示视差图，必须用灰度图像	
//	
//	cout << "4.矫正测试与鼠标点击得到空间坐标" << endl;
//	Mat imageL, imageR;
//
//	cout << "    -->按enter截图" << endl;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		capture_1.read(frame_1);///读取视频帧放入frame
//		imshow("camera_0", frame_0);
//		imshow("camera_1", frame_1);
//
//		char c = waitKey(50); //等待时间50ms
//		if (c == 13) {//enter		
//			cvtColor(frame_0, imageL, COLOR_BGR2GRAY);
//			cvtColor(frame_1, imageR, COLOR_BGR2GRAY);
//			cout << "    视频截取结束!" << endl;
//			break;
//		}
//	}
//	capture_0.release();
//	capture_1.release();
//
//	cout << "    开始矫正!" << endl;
//	cv::Mat rectifyImageL, rectifyImageR;
//	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
//	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
//	//imshow("rectifyImageL", rectifyImageL);
//	//imshow("rectifyImageR", rectifyImageR);
//	///Q与remap的区别：像素图--remap-->纠正后的像素图---->视差--Q-->3D坐标
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = imageL.channels();    //
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //结果为80；视差范围，即最大视差值和最小视差值之差, 必须是16的倍数，所以后面&-16
//
//	sgbm->setPreFilterCap(63);//映射滤波器大小
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1计算公式
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//最小视差
//	sgbm->setNumDisparities(numberOfDisparities);//视差搜索范围
//	sgbm->setUniquenessRatio(10);//唯一性比率
//	sgbm->setSpeckleWindowSize(50);//视差连通区域像素点个数阈值
//	sgbm->setSpeckleRange(32);//视差连通条件/阈值
//	sgbm->setDisp12MaxDiff(1);//左右一致性检测最大容许误差阈值
//	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//
//	cv::Mat disp, disp8;
//
//	sgbm->compute(rectifyImageL, rectifyImageR, disp);//默认计算出的是左视差图，即以左图为底板，以视差为灰度值
//	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));//将16位符号整形的视差矩阵转换为8位无符号整形矩阵，???
//	cout << "    显示视差图" << endl;
//	cout << "    -->通过鼠标点击图像上的点可以得到3D坐标!!!" << endl;
//	cv::imshow("disparity8", disp8); //显示视差图
//
//	cv::reprojectImageTo3D(disp, xyz, Q, true);//视差就是用Q求
//	xyz = xyz * 16; // xyz=[X/W Y/W Z/W]，乘以16得到真实坐标
//	cv::setMouseCallback("disparity8", onMouse, 0);//对disparity8视差图进行鼠标回调
//
//	for (int i = 20; i < rectifyImageL.rows; i += 20)
//	{
//		cv::line(rectifyImageL, cv::Point(0, i), cv::Point(rectifyImageL.cols, i), cv::Scalar(255, 255, 255));
//		cv::line(rectifyImageR, cv::Point(0, i), cv::Point(rectifyImageL.cols, i), cv::Scalar(255, 255, 255));
//	}
//	cv::Mat imageMatches;
//	cv::drawMatches(rectifyImageL, std::vector<cv::KeyPoint>(),  // 1st image
//		rectifyImageR, std::vector<cv::KeyPoint>(),              // 2nd image
//		std::vector<cv::DMatch>(),
//		imageMatches,                       // the image produced
//		cv::Scalar(255, 255, 255),
//		cv::Scalar(255, 255, 255),
//		std::vector<char>(),
//		2);
//
//	cout << "    显示矫正图" << endl;
//	cv::imshow("imageMatches", imageMatches);
//	cv::setMouseCallback("imageMatches", onMouse, 0);
//	
//	waitKey();
//
//	return 0;
//}
