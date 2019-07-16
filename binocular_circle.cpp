#include "binocular_circle.h"

static Mat xyz;

static void onMouse(int event, int x, int y, int, void*)
{
	cv::Point origin;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //�����ť���µ��¼�
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

	vector<vector<Point2f> > imagePoints[2];//����"[2]==>"2��, "Point2f==>"2ά����, ��̬������δ֪
	vector<vector<Point3f> > objectPoints;//����1��3ά����
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2; //ֻ�����һ����ֵ

	imagePoints[0].resize(nimages);//��չά��Ϊnimages������չ�ĳ�ʼ��Ϊ0
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
			vector<Point2f>& corners = imagePoints[k][j];//�൱��ָ�룬��������û��pushback
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);//img�������ţ������timg
				//����.resize() <==��ͬ��==> resize()
				//Alt + G �鿴
				//���ŵ�Ŀ�ģ�δ���ҵ��ǵ�ʱ���Ŵ�ԭͼ����Ѱ��һ��
				found = findCirclesGrid(timg, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID); //�ǶԳ�
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners); //����һ������Ϊcorners��cornersMat����
						cornersMat *= 1. / scale;//���Ż�ȥ
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

	objectPoints.resize(nimages);//��Ϊ������һ���ģ����Թ���һ��objectPoint

	for (i = 0; i < nimages; i++)
	{
		calcBoardCornerPositions(boardSize, squareSize, objectPoints[i]);
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// �õ���ʼ�������������K��3*3������֤�͵�Ŀ�궨�����һ��
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// ������С��һ��
	Mat R, T, E, F;

	//�ú�������Ҫ�ṩ��ʼ���������
	//���Ա궨�ó��ڲξ���ͻ��������ٽ���˫Ŀ�궨�Ľ��Ҫ����ࡣ��ȻҲ����ֱ��˫Ŀ�궨��
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO + //�̶�fx/fy�ı�ֵ
		CALIB_ZERO_TANGENT_DIST + //������������p1,p2��Ϊ��
		CALIB_USE_INTRINSIC_GUESS + //ʹ��fx,fy,cx,cy����ֵ����Ϊ�����˱궨������Ҫʹ�ø�ģʽ��
		CALIB_SAME_FOCAL_LENGTH + //ǿ�Ʊ�������������Ľ�����ͬ
		CALIB_RATIONAL_MODEL + //���û���ϵ��k4,k5��k6������k1,k2,k3,p1,p2�ܹ�8��ϵ��
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//��Ӧ�ľ���������Ż��б��ֲ��䣬0�����Ż�
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5)); 
	cout << "done with RMS error=" << rms << endl;
	///<��P150������֤��E = t^R������ F �� K2^-T * E * K1^-1 Ϊʲô������

	///< ǰ�����Ŀ�����

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint�Լ�����Լ��: m2^t*F*m1=0
	// �õ���ƥ�����������뻭���⼫�ߣ�����֤һ��ƥ��Ľ���Ƿ���ȷ��ֻ����֤���Ժ������û�й�ϵ

	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size(); // imagePoints����ǽǵ����������
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//δ���εĵ㣬������ĵ�
			///<������ϵ���������������궨��ɺ󣬾�Ӧ�ý��л���Ľ������Դﵽ���������Ŀ��
			///<��ʹ�õģ����Դ���վ���
			///<imagePointsҲ�ᱻ�޸ģ�����
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

	// save intrinsic parameters�ڲ� 
	//******���(extrinsics)
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

	//Ҫͨ������ͼ�񣬹������������Ϣ���ͱ���������ͼ����׼ȷ��ƥ�䵽ͬһ��㣬�������ܸ��ݸ����������ͼ���е�λ�ù�ϵ������������ȡ�
	//Ϊ�˽���ƥ��ļ���������������ͷ�ĳ���ƽ��Ӧ����ͬһƽ��(��ת����)�����ǣ����������ϸ�İڷ�����ͷ���ﵽ���Ŀ����Ȼ��Щ���ѡ�
	//����У���������ü���ͼ�α任��ϵ��ʹ��ԭ�Ȳ���������λ�ù�ϵ������ͼ�������������
	//https://www.cnblogs.com/german-iris/p/5199203.html
	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	///<stereoRectify��������������н��������ֱ�ӽ�ͼƬ����������������ǵó��������������ͼ���Լ�������Ҫ����ת����R1,R2
	///<���������������ϵ�µ�ͶӰ����P1, P2��3*4, ������T��K�Ĺ��̣��������ǽ�3D�������ת����2ά��������:P*[X Y Z 1]' =[x y w] 
	///<QΪ4*4����Ȳ���ӳ�������ͶӰ���󣬰����˷���T��K�Ĺ��̣�������Q���԰�2ά��������ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ����Ӳ�,��https://blog.csdn.net/fb_help/article/details/83339092
	//����֤ R1 = R2*R�� P1��P2ֻ����t�����������֪����Ϊ�Ѿ������ˣ� Q���Ǽ�����; �������ɣ������Լ�ʵ�ʲ���������
	///<flags: 0(ˮƽ��ֱ���ƶ�ͼ����ʹ�������õķ�Χ���) �� CV_CALIB_ZERO_DISPARITY(��������У�����ͼ�����������ͬ����������)
	///<alpha: ����������������Ϊ������ԣ������������졣�������Ϊ0����ôУ����ͼ��ֻ����Ч�Ĳ��ֻᱻ��ʾ��û�к�ɫ�Ĳ��֣����������Ϊ1����ô�ͻ���ʾ����ͼ������Ϊ0~1֮���ĳ��ֵ����Ч��Ҳ��������֮�䡣
	///<imageSize��ʵ��newImageSize��У�����ͼ���С��һ���ԭͼ����ͬ
	///<validPixROI1��validPixROI2-����������

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
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//���ˣ��Ժ���˵��

																				  // COMPUTE AND DISPLAY RECTIFICATION�����涼����ʾ�������ͼ
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else //�ڶ��ַ�ʽ��ͨ����Ӧ�������R1��R2��P1��P2
		 // use intrinsic parameters of each camera, but
		 // compute the rectification���� transformation�任 directly
		 // from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;//��Ӧ����H��R��K֮���ϵ��slam P146
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	///<�ú��������Ǽ���������������У����ӳ��任������P1Ӧ�ô����Ǿ��������newCameraMatrix
	///<��openCV���棬У����ļ��������newCameraMatrix�ǰ�����ͶӰ����P��ġ�
	///<�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
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
	//cout << "��ʼ��ʾ�������Բ��ͼ" << endl;
	//for (i = 0; i < nimages; i++)
	//{
	//	for (k = 0; k < 2; k++)
	//	{
	//		Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;//��ȡ�Ҷ�ͼimg
	//		remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);//img���α任Ϊrimg
	//		cvtColor(rimg, cimg, COLOR_GRAY2BGR);//rimgת��ɫͼcimg
	//		Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));//���canvas������Ҳ���
	//		resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);//��cimgͼ��resize��������canvasPart��
	//		if (useCalibrated)
	//		{
	//			Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
	//				cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));//��Ч��ROIҲ*sf
	//			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);//��ROI����ͼ����
	//		}
	//	}

	//	if (!isVerticalStereo)
	//		for (j = 0; j < canvas.rows; j += 16)
	//			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);//ÿ����16�����ػ�һ��line
	//	else
	//		for (j = 0; j < canvas.cols; j += 16)
	//			line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
	//	imshow("rectified", canvas);
	//	char c = (char)waitKey();
	//	if (c == 27 || c == 'q' || c == 'Q')
	//		break;
	//}	
	
	//ʹ��SGBM�㷨���3D���꣬��һ������ƥ���㷨��׼ȷ�Ⱥ��ٶ����У������бȽϳ��ã���Ҫͨ���Ӳ����3D����
	//��ʾ�Ӳ�ͼ�������ûҶ�ͼ��
	Mat imageL = cv::imread("binocular_circle_pic/left1.bmp", 0);
	Mat imageR = cv::imread("binocular_circle_pic/right1.bmp", 0);
	cv::Mat rectifyImageL, rectifyImageR;
	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
	
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	int sgbmWinSize = 3;
	int cn = imageL.channels();    //
	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //���Ϊ80���ӲΧ��������Ӳ�ֵ����С�Ӳ�ֵ֮��, ������16�ı��������Ժ���&-16

	sgbm->setPreFilterCap(63);//ӳ���˲�����С
	sgbm->setBlockSize(sgbmWinSize);//3*3
	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1���㹫ʽ
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);//��С�Ӳ�
	sgbm->setNumDisparities(numberOfDisparities);//�Ӳ�������Χ
	sgbm->setUniquenessRatio(10);//Ψһ�Ա���
	sgbm->setSpeckleWindowSize(50);//�Ӳ���ͨ�������ص������ֵ
	sgbm->setSpeckleRange(32);//�Ӳ���ͨ����/��ֵ
	sgbm->setDisp12MaxDiff(1);//����һ���Լ��������������ֵ
	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

	cv::Mat disp, disp8;

	sgbm->compute(rectifyImageL, rectifyImageR, disp);//Ĭ�ϼ�����������Ӳ�ͼ��������ͼΪ�װ壬���Ӳ�Ϊ�Ҷ�ֵ
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));//��16λ�������ε��Ӳ����ת��Ϊ8λ�޷������ξ���???
	cout << "��ʾ�Ӳ�ͼ" << endl;
	cv::imshow("disparity8", disp8); //��ʾ�Ӳ�ͼ
	
	cv::reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; // xyz=[X/W Y/W Z/W]������16�õ���ʵ����
	cv::setMouseCallback("disparity8", onMouse, 0);//��disparity8�Ӳ�ͼ�������ص�
	
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