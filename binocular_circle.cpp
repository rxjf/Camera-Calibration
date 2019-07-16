#include "binocular_circle.h"

static Mat xyz;

static void onMouse(int event, int x, int y, int, void*)
{
	cv::Point origin;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
		//origin = cv::Point(x, y);
		origin = cv::Point(0, 0);
		xyz.at<cv::Vec3f>(origin)[2] += 2;
		std::cout << origin << "in world coordinate is: " << xyz.at<cv::Vec3f>(origin) << std::endl;
		break;
	}
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}

static void CircleStereoCalib(const vector<string>& imagelist, Size boardSize, bool displayCorners = false, bool useCalibrated = true, bool showRectified = true)
{
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	const int maxScale = 2;
	const float squareSize = 20;  // Set this to your actual square size
								   // ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];//定义"[2]==>"2个, "Point2f==>"2维数组, 动态，个数未知
	vector<vector<Point3f> > objectPoints;//定义1个3维数组
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2; //只对最后一个赋值

	imagePoints[0].resize(nimages);//扩展维度为nimages，新扩展的初始化为0
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];//相当于指针，所以下面没有pushback
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);//img经过缩放，输出给timg
				//对象.resize() <==不同于==> resize()
				//Alt + G 查看
				//缩放的目的：未能找到角点时，放大原图重新寻找一次
				found = findCirclesGrid(timg, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID); //非对称
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners); //定义一个内容为corners的cornersMat变量
						cornersMat *= 1. / scale;//缩放回去
					}
					break;
				}
			}
			if (displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			//cout << "imagePoints[" << k << "][" << j << "][0]" << imagePoints[k][j][0] << endl;
		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		return;
	}

	objectPoints.resize(nimages);//因为左右是一样的，所以共用一个objectPoint

	for (i = 0; i < nimages; i++)
	{
		calcBoardCornerPositions(boardSize, squareSize, objectPoints[i]);
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// 得到初始化的摄像机矩阵K，3*3，经验证和单目标定结果不一样
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// 两个大小不一样
	Mat R, T, E, F;

	//该函数必须要提供初始摄像机矩阵
	//各自标定得出内参矩阵和畸变矩阵后再进行双目标定的结果要好许多。当然也可以直接双目标定。
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO + //固定fx/fy的比值
		CALIB_ZERO_TANGENT_DIST + //切向畸变参数（p1,p2）为零
		CALIB_USE_INTRINSIC_GUESS + //使用fx,fy,cx,cy估计值；因为经过了标定，所以要使用该模式？
		CALIB_SAME_FOCAL_LENGTH + //强制保持两个摄像机的焦距相同
		CALIB_RATIONAL_MODEL + //启用畸变系数k4,k5和k6，加上k1,k2,k3,p1,p2总共8个系数
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//对应的径向畸变在优化中保持不变，0，不优化
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5)); 
	cout << "done with RMS error=" << rms << endl;
	///<经P150程序验证，E = t^R，但是 F ≠ K2^-T * E * K1^-1 为什么，算了

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
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size(); // imagePoints存的是角点的像素坐标
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//未变形的点，矫正后的点
			///<当畸变系数和内外参数矩阵标定完成后，就应该进行畸变的矫正，以达到消除畸变的目的
			///<不使用的，可以传入空矩阵
			///<imagePoints也会被修改！！！
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + 
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

	// save intrinsic parameters内参 
	//******外参(extrinsics)
	FileStorage fs("binocular_circle_pic/intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q; //P 3*4
	Rect validRoi[2];

	//要通过两幅图像，估计物点的深度信息，就必须在两幅图像中准确的匹配到同一物点，这样才能根据该物点在两幅图像中的位置关系，计算物体深度。
	//为了降低匹配的计算量，两个摄像头的成像平面应处于同一平面(旋转即可)。但是，单单依靠严格的摆放摄像头来达到这个目的显然有些困难。
	//立体校正就是利用几何图形变换关系，使得原先不满足上述位置关系的两幅图像满足该条件。
	//https://www.cnblogs.com/german-iris/p/5199203.html
	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	///<stereoRectify立体矫正。其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正（图像自己）所需要的旋转矩阵R1,R2
	///<和摄像机在新坐标系下的投影矩阵P1, P2，3*4, 包含了T和K的过程，其作用是将3D点的坐标转换到2维像素坐标:P*[X Y Z 1]' =[x y w] 
	///<Q为4*4的深度差异映射矩阵，重投影矩阵，包含了反向T和K的过程，即矩阵Q可以把2维像素坐标投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差,见https://blog.csdn.net/fb_help/article/details/83339092
	//经验证 R1 = R2*R； P1和P2只存在t的区别，想想可知，因为已经共面了； Q就是见含义; 就这样吧，后面自己实际测量！！！
	///<flags: 0(水平或垂直地移动图像，以使得其有用的范围最大) 或 CV_CALIB_ZERO_DISPARITY(会让两幅校正后的图像的主点有相同的像素坐标)
	///<alpha: 拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
	///<imageSize其实是newImageSize，校正后的图像大小，一般跟原图像相同
	///<validPixROI1，validPixROI2-见函数帮助

	//cout << "\nP2*R\n" << P2*R;

	fs.open("binocular_circle_pic/extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "imageSize" << imageSize << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//算了，以后再说吧

																				  // COMPUTE AND DISPLAY RECTIFICATION，下面都是显示纠正后的图
	if (!showRectified)
		return;

	Mat rmap[2][2];
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
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
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

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	///<该函数功能是计算畸变矫正和立体校正的映射变换，其中P1应该传入是纠正过后的newCameraMatrix
	///<在openCV里面，校正后的计算机矩阵newCameraMatrix是包含在投影矩阵P里的。
	///<所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	//cout << "reamp: " << endl << rmap[0][0] << endl;
	
	//Mat canvas;
	//double sf;
	//int w, h;
	//if (!isVerticalStereo)
	//{
	//	sf = 600. / MAX(imageSize.width, imageSize.height);// 600/640 = 15/16 = 0.9375
	//	w = cvRound(imageSize.width*sf);// 640*0.9375 = 600
	//	h = cvRound(imageSize.height*sf);// 480*0.9375 = 450
	//	canvas.create(h, w * 2, CV_8UC3);
	//}
	//else
	//{
	//	sf = 300. / MAX(imageSize.width, imageSize.height);
	//	w = cvRound(imageSize.width*sf);
	//	h = cvRound(imageSize.height*sf);
	//	canvas.create(h * 2, w, CV_8UC3);
	//}

	//
	//cout << "开始显示纠正后的圆形图" << endl;
	//for (i = 0; i < nimages; i++)
	//{
	//	for (k = 0; k < 2; k++)
	//	{
	//		Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;//读取灰度图img
	//		remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);//img几何变换为rimg
	//		cvtColor(rimg, cimg, COLOR_GRAY2BGR);//rimg转彩色图cimg
	//		Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));//获得canvas的左或右部分
	//		resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);//将cimg图像resize，并画到canvasPart中
	//		if (useCalibrated)
	//		{
	//			Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
	//				cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));//有效的ROI也*sf
	//			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);//将ROI画到图像中
	//		}
	//	}

	//	if (!isVerticalStereo)
	//		for (j = 0; j < canvas.rows; j += 16)
	//			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);//每往下16个像素画一个line
	//	else
	//		for (j = 0; j < canvas.cols; j += 16)
	//			line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
	//	imshow("rectified", canvas);
	//	char c = (char)waitKey();
	//	if (c == 27 || c == 'q' || c == 'Q')
	//		break;
	//}	
	
	//使用SGBM算法求解3D坐标，是一种立体匹配算法，准确度和速度适中，工程中比较常用，主要通过视差求解3D坐标
	//显示视差图，必须用灰度图像
	Mat imageL = cv::imread("binocular_circle_pic/left1.bmp", 0);
	Mat imageR = cv::imread("binocular_circle_pic/right1.bmp", 0);
	cv::Mat rectifyImageL, rectifyImageR;
	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
	
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
	cout << "显示视差图" << endl;
	cv::imshow("disparity8", disp8); //显示视差图
	
	cv::reprojectImageTo3D(disp, xyz, Q, true);
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

	cv::imshow("imageMatches", imageMatches);
	//cv::setMouseCallback("imageMatches", onMouse, 0);

	waitKey();
}

void BinocularCircle(void)
{
	Size boardSize;
	string imagelistfn;
	bool showRectified = true;
	bool showCorner = false;

	imagelistfn = "binocular_circle_pic/stereo_calib.xml";
	boardSize.width = 4;
	boardSize.height = 11;

	vector<string> imagelist;
	bool ok = readStringList(imagelistfn, imagelist);
	if (!ok || imagelist.empty())
	{
		cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
	}

	CircleStereoCalib(imagelist, boardSize, showCorner, true, showRectified);
}