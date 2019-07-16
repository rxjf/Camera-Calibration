#include "binocular_chessboard.h"

static Mat xyz;

static void onMouse(int event, int x, int y, int, void*)
{
	cv::Point origin;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //�����ť���µ��¼�
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

//===============��������߲ɼ�ͼƬ-->�궨-->����ӳ��-->���߽�ȡһ��ͼƬ����������������õ��ռ�����================

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
	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��

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

		cout << "1.��ʼ��ȡ�궨ͼƬ:" << endl;
		cout << "    -->��enter��ͼ, ��escֹͣ��ͼ" << endl;

		while (true)
		{
			//�ֳ�����
			capture_0.read(frame);	
			frame_0 = frame(rect_0);
			frame_1 = frame(rect_1);
			imshow("camera_0", frame_0);
			imshow("camera_1", frame_1);

			char c = waitKey(50); //�ȴ�ʱ��50ms
			if (c == 13) {//enter		
				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//��ɫ���ҶȾ���
				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

				if (found1&&found2) {
					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
							30, 0.01));//���봫��Ҷ�ͼ
					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
							30, 0.01));
					drawChessboardCorners(frame_0, boardSize, corners1, found1);//��ɫͼ�����ڹۿ�
					drawChessboardCorners(frame_1, boardSize, corners2, found1);
					imshow("camera_0_corner", frame_0);
					imshow("camera_1_corner", frame_1);
					cout << "    ��ͼ�ɹ���" << endl;
					cout << "    �ǵ���ȡ�ɹ���" << endl;
					cout << "    -->��enterȷ�ϱ���, ������������" << endl;
					c = waitKey();
					if (c == 13) {
						pic_counter++;
						cout << "<-----------��" << pic_counter << "��ͼƬ�ǵ���ȡ�ѱ�����----------->" << endl;
						cout << "    -->��enter������ͼ, ��escֹͣ��ͼ" << endl;
						imagePoints[0].push_back(corners1);
						imagePoints[1].push_back(corners2);
					}
				}
				else {
					cout << "    δ���ҵ��ǵ㣬�����½�ȡ" << endl;
				}
			}
			if (c == 27) {
				cout << "��Ƶ��ȡ����" << endl << endl;
				break; //esc
			}
		}

		cout << "2.��ʼ�궨"<<endl;

		vector<vector<Point3f> > objectPoints;
		objectPoints.resize(pic_counter);//��������

		for (int i = 0; i < pic_counter; i++)
		{
			for (int j = 0; j < boardSize.height; j++)
				for (int k = 0; k < boardSize.width; k++)
					objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
		}
	
		imageSize = frame_0.size();

		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// �õ���ʼ�������������K��3*3������֤�͵�Ŀ�궨���һ��
		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// ����������ڲβ�һ��
		Mat R, T, E, F;

		//�ú�������Ҫ�ṩ��ʼ���������
		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			CALIB_FIX_ASPECT_RATIO + //�̶�fx/fy�ı�ֵ
			CALIB_ZERO_TANGENT_DIST + //������������p1,p2��Ϊ��
			CALIB_USE_INTRINSIC_GUESS + //ʹ��fx,fy,cx,cy����ֵ����Ϊ�����˱궨������Ҫʹ�ø�ģʽ
			CALIB_SAME_FOCAL_LENGTH + //ǿ�Ʊ�������������Ľ�����ͬ
			CALIB_RATIONAL_MODEL + //���û���ϵ��k4,k5��k6������k1,k2,k3,p1,p2�ܹ�8��ϵ��
			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//��Ӧ�ľ���������Ż��б��ֲ��䣬0�����Ż�
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
		cout << "    �궨���Ϊ��" << rms << endl;
		///<��P150������֤��E = t^R�� F = K2^-T * E * K1^-1 

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
		for (int i = 0; i < pic_counter; i++)
		{
			int npt = (int)imagePoints[0][i].size(); // imagePoints����ǽǵ����������
			Mat imgpt[2];
			for (int k = 0; k < 2; k++)
			{
				imgpt[k] = Mat(imagePoints[k][i]);
				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//δ���εĵ㣬������ĵ�
																											///<������ϵ���������������궨��ɺ󣬾�Ӧ�ý��л���Ľ������Դﵽ���������Ŀ��
				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//���濼���˻��䣬���������������������
				//�����forѭ������F*p1��FT*p2��Ȼ�����forѭ���ٳ�p2��p1����OK��
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
		cout << "    ƽ���Լ�����" << err / npoints << endl;

		// save intrinsic parameters�ڲ� ******���(extrinsics)
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

		cout << "3.����������ӳ��"<< endl;
		//Ҫͨ������ͼ��������������Ϣ���ͱ���������ͼ����׼ȷ��ƥ�䵽ͬһ��㣬�������ܸ��ݸ����������ͼ���е�λ�ù�ϵ������������ȡ�
		//Ϊ�˽���ƥ��ļ���������������ͷ�ĳ���ƽ��Ӧ����ͬһƽ��(��ת����)�����ǣ����������ϸ�İڷ�����ͷ���ﵽ���Ŀ����Ȼ��Щ���ѡ�
		//����У���������ü���ͼ�α任��ϵ��ʹ��ԭ�Ȳ���������λ�ù�ϵ������ͼ�������������
		//https://www.cnblogs.com/german-iris/p/5199203.html
		stereoRectify(cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
		///<stereoRectify��������������н��������ֱ�ӽ�ͼƬ����������������ǵó��������������ͼ���Լ�������Ҫ����ת����R1,R2
		///<���������������ϵ�µ�ͶӰ����P1, P2��3*4, ������ǰ�ı任����T�ˣ��������ǽ�3D�������ת����2ά��������:P*[X Y Z 1]' =[x y w] 
		///<QΪ4*4����Ȳ���ӳ�������ͶӰ���󣬼�����Q���԰�2ά��������ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ����Ӳ�
		//����֤ R1 = R2*R�� P1��P2ֻ����t�����������֪����Ϊ�Ѿ������ˣ� Q���Ǽ�����; �������ɣ������Լ�ʵ�ʲ���������
		///<flags: 0(ˮƽ��ֱ���ƶ�ͼ����ʹ�������õķ�Χ���) �� CV_CALIB_ZERO_DISPARITY(��������У�����ͼ�����������ͬ����������)
		///<alpha: ����������������Ϊ������ԣ������������졣�������Ϊ0����ôУ����ͼ��ֻ����Ч�Ĳ��ֻᱻ��ʾ��û�к�ɫ�Ĳ��֣����������Ϊ1����ô�ͻ���ʾ����ͼ������Ϊ0~1֮���ĳ��ֵ����Ч��Ҳ��������֮�䡣
		///<imageSize��ʵ��newImageSize��У�����ͼ���С��һ���ԭͼ����ͬ
		///<validPixROI1��validPixROI2-����������

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
		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//���ˣ��Ժ���˵��

																					  // COMPUTE AND DISPLAY RECTIFICATION�����涼����ʾ�������ͼ
		if (!showRectified)
			return -1;

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
			for (int k = 0; k < 2; k++)
			{
				for (int i = 0; i < pic_counter; i++)
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
		cout << endl;
	}
	else {
		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //������ʽ��yml����ȻҲ���Դ�xml����Ҫ����׺
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
	//�ú��������Ǽ���������������У����ӳ��任������P1Ӧ�ô����Ǿ��������newCameraMatrix
	//��openCV���棬У����ļ��������newCameraMatrix�Ǹ�ͶӰ����Pһ�𷵻صġ�
	//�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);


	////ʹ��SGBM�㷨��֤���,��һ������ƥ���㷨��׼ȷ�Ⱥ��ٶ����У������бȽϳ���
	////��ʾ�Ӳ�ͼ�������ûҶ�ͼ��	
	
	cout << "4.����������������õ��ռ�����" << endl;
	Mat imageL, imageR;

	cout << "    -->��enter��ͼ" << endl;

	while (true)
	{
		capture_0.read(frame);
		frame_0 = frame(rect_0);
		frame_1 = frame(rect_1);
		imshow("camera_0", frame_0);
		imshow("camera_1", frame_1);

		char c = waitKey(50); //�ȴ�ʱ��50ms
		if (c == 13) {//enter		
			cvtColor(frame_0, imageL, COLOR_BGR2GRAY);
			cvtColor(frame_1, imageR, COLOR_BGR2GRAY);
			cout << "    ��Ƶ��ȡ����!" << endl;
			break;
		}
	}
	capture_0.release();

	cout << "    ��ʼ����!" << endl;
	cv::Mat rectifyImageL, rectifyImageR;
	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
	//imshow("rectifyImageL", rectifyImageL);
	//imshow("rectifyImageR", rectifyImageR);
	///Q��remap����������ͼ--remap-->�����������ͼ---->�Ӳ�--Q-->3D����

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
	cout << "    ��ʾ�Ӳ�ͼ" << endl;
	cout << "    -->ͨ�������ͼ���ϵĵ���Եõ�3D����!!!" << endl;
	cv::imshow("disparity8", disp8); //��ʾ�Ӳ�ͼ

	cv::reprojectImageTo3D(disp, xyz, Q, true);//�Ӳ������Q��
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

	cout << "    ��ʾ����ͼ" << endl;
	cv::imshow("imageMatches", imageMatches);
	cv::setMouseCallback("imageMatches", onMouse, 0);
	
	waitKey();

	return 0;
}


//================��������߲ɼ�ͼƬ-->�궨-->����ӳ��-->ʵʱ��ʾ������3D����(����Բ����)=======================

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
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
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
//		cout << "1.��ʼ��ȡ�궨ͼƬ:" << endl;
//		cout << "    -->��enter��ͼ, ��escֹͣ��ͼ" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame);///��ȡ��Ƶ֡����frame
//
//			//�ֳ�����		
//			frame_0 = frame(rect_0);
//			frame_1 = frame(rect_1);
//			imshow("camera_0", frame_0);
//			imshow("camera_1", frame_1);
//
//			char c = waitKey(50); //�ȴ�ʱ��50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//��ɫ���ҶȾ���
//				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//
//				if (found1&&found2) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//���봫��Ҷ�ͼ
//					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//��ɫͼ�����ڹۿ�
//					drawChessboardCorners(frame_1, boardSize, corners2, found1);
//					imshow("camera_0_corner", frame_0);
//					imshow("camera_1_corner", frame_1);
//					cout << "    ��ͼ�ɹ���" << endl;
//					cout << "    �ǵ���ȡ�ɹ���" << endl;
//					cout << "    -->��enterȷ�ϱ���, ������������" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------��" << pic_counter << "��ͼƬ�ǵ���ȡ�ѱ�����----------->" << endl;
//						cout << "    -->��enter������ͼ, ��escֹͣ��ͼ" << endl;
//						imagePoints[0].push_back(corners1);
//						imagePoints[1].push_back(corners2);
//					}
//				}
//				else {
//					cout << "    δ���ҵ��ǵ㣬�����½�ȡ" << endl;
//				}
//			}
//			if (c == 27) {
//				cout << "��Ƶ��ȡ����" << endl << endl;
//				break; //esc
//			}
//		}
//
//		cout << "2.��ʼ�궨" << endl;
//
//		vector<vector<Point3f> > objectPoints;
//		objectPoints.resize(pic_counter);//��������
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
//		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// �õ���ʼ�������������K��3*3������֤��ֵ�͵�Ŀ�궨�����һ��
//		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// ������С��һ��
//		Mat R, T, E, F;
//
//		//�ú�������Ҫ�ṩ��ʼ���������
//		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//			cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, E, F,
//			CALIB_FIX_ASPECT_RATIO + //�̶�fx/fy�ı�ֵ
//			CALIB_ZERO_TANGENT_DIST + //������������p1,p2��Ϊ��
//			CALIB_USE_INTRINSIC_GUESS + //ʹ��fx,fy,cx,cy����ֵ����Ϊ�����˱궨������Ҫʹ�ø�ģʽ��
//			CALIB_SAME_FOCAL_LENGTH + //ǿ�Ʊ�������������Ľ�����ͬ
//			CALIB_RATIONAL_MODEL + //���û���ϵ��k4,k5��k6������k1,k2,k3,p1,p2�ܹ�8��ϵ��
//			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//��Ӧ�ľ���������Ż��б��ֲ��䣬0�����Ż�
//			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//		cout << "    �궨���Ϊ��" << rms << endl;
//		///<��P150������֤��E = t^R������ F �� K2^-T * E * K1^-1 Ϊʲô������
//
//		///< ǰ�����Ŀ�����
//
//		// CALIBRATION QUALITY CHECK
//		// because the output fundamental matrix implicitly
//		// includes all the output information,
//		// we can check the quality of calibration using the
//		// epipolar geometry constraint�Լ�����Լ��: m2^t*F*m1=0
//		// �õ���ƥ�����������뻭���⼫�ߣ�����֤һ��ƥ��Ľ���Ƿ���ȷ��ֻ����֤���Ժ������û�й�ϵ
//
//		double err = 0;
//		int npoints = 0;
//		vector<Vec3f> lines[2];
//		for (int i = 0; i < pic_counter; i++)
//		{
//			int npt = (int)imagePoints[0][i].size(); // imagePoints����ǽǵ����������
//			Mat imgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				imgpt[k] = Mat(imagePoints[k][i]);
//				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//δ���εĵ㣬������ĵ�
//																											///<������ϵ���������������궨��ɺ󣬾�Ӧ�ý��л���Ľ������Դﵽ���������Ŀ��
//				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//���濼���˻��䣬���������������������
//				//�����forѭ������F*p1��FT*p2��Ȼ�����forѭ���ٳ�p2��p1����OK��
//			}
//			for (int j = 0; j < npt; j++)
//			{
//				double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] + //|a1*x0+b1*y0+c1|+|a0*x1+b0*y1+c0|,�ԣ���������˵Ӧ�õ���0�����������
//					imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
//					fabs(imagePoints[1][i][j].x*lines[0][j][0] +
//						imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
//				err += errij;
//			}
//			npoints += npt;
//		}
//		cout << "    ƽ���Լ�����" << err / npoints << endl;
//
//		// save intrinsic parameters�ڲ� ******���(extrinsics)
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
//		cout << "3.����������ӳ��" << endl;
//		//Ҫͨ������ͼ��������������Ϣ���ͱ���������ͼ����׼ȷ��ƥ�䵽ͬһ��㣬�������ܸ��ݸ����������ͼ���е�λ�ù�ϵ������������ȡ�
//		//Ϊ�˽���ƥ��ļ���������������ͷ�ĳ���ƽ��Ӧ����ͬһƽ��(��ת����)�����ǣ����������ϸ�İڷ�����ͷ���ﵽ���Ŀ����Ȼ��Щ���ѡ�
//		//����У���������ü���ͼ�α任��ϵ��ʹ��ԭ�Ȳ���������λ�ù�ϵ������ͼ�������������
//		//https://www.cnblogs.com/german-iris/p/5199203.html
//		stereoRectify(cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, R1, R2, P1, P2, Q,
//			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//		///<stereoRectify��������������н��������ֱ�ӽ�ͼƬ����������������ǵó��������������ͼ���Լ�������Ҫ����ת����R1,R2
//		///<���������������ϵ�µ�ͶӰ����P1, P2��3*4, ������ǰ�ı任����T�ˣ��������ǽ�3D�������ת����2ά��������:P*[X Y Z 1]' =[x y w] 
//		///<QΪ4*4����Ȳ���ӳ�������ͶӰ���󣬼�����Q���԰�2ά��������ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ����Ӳ�
//		//����֤ R1 = R2*R�� P1��P2ֻ����t�����������֪����Ϊ�Ѿ������ˣ� Q���Ǽ�����; �������ɣ������Լ�ʵ�ʲ���������
//		///<flags: 0(ˮƽ��ֱ���ƶ�ͼ����ʹ�������õķ�Χ���) �� CV_CALIB_ZERO_DISPARITY(��������У�����ͼ�����������ͬ����������)
//		///<alpha: ����������������Ϊ������ԣ������������졣�������Ϊ0����ôУ����ͼ��ֻ����Ч�Ĳ��ֻᱻ��ʾ��û�к�ɫ�Ĳ��֣����������Ϊ1����ô�ͻ���ʾ����ͼ������Ϊ0~1֮���ĳ��ֵ����Ч��Ҳ��������֮�䡣
//		///<imageSize��ʵ��newImageSize��У�����ͼ���С��һ���ԭͼ����ͬ
//		///<validPixROI1��validPixROI2-����������
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
//		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//���ˣ��Ժ���˵��
//
//																					  // COMPUTE AND DISPLAY RECTIFICATION�����涼����ʾ�������ͼ
//		if (!showRectified)
//			return -1;
//
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useCalibrated)
//		{
//			// we already computed everything
//		}
//		// OR ELSE HARTLEY'S METHOD
//		else //�ڶ��ַ�ʽ��ͨ����Ӧ�������R1��R2��P1��P2
//			 // use intrinsic parameters of each camera, but
//			 // compute the rectification���� transformation�任 directly
//			 // from the fundamental matrix
//		{
//			vector<Point2f> allimgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				for (int i = 0; i < pic_counter; i++)
//					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//			}
//			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//			Mat H1, H2;//��Ӧ����H��R��K֮���ϵ��slam P146
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
//		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //������ʽ��yml����ȻҲ���Դ�xml����Ҫ����׺
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
//	//�ú��������Ǽ���������������У����ӳ��任������P1Ӧ�ô����Ǿ��������newCameraMatrix
//	//��openCV���棬У����ļ��������newCameraMatrix�Ǹ�ͶӰ����Pһ�𷵻صġ�
//	//�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
//	//ͶӰ�������������󵽵�ʲô��ϵ����
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//
//	////ʹ��SGBM�㷨��֤���,��һ������ƥ���㷨��׼ȷ�Ⱥ��ٶ����У������бȽϳ���
//	////��ʾ�Ӳ�ͼ�������ûҶ�ͼ��
//
//	cout << "4.Ŀ��׷��" << endl;
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = 1;//imageL.channels()
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //���Ϊ80���ӲΧ��������Ӳ�ֵ����С�Ӳ�ֵ֮��, ������16�ı��������Ժ���&-16
//
//	sgbm->setPreFilterCap(63);//ӳ���˲�����С
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1���㹫ʽ
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//��С�Ӳ�
//	sgbm->setNumDisparities(numberOfDisparities);//�Ӳ�������Χ
//	sgbm->setUniquenessRatio(10);//Ψһ�Ա���
//	sgbm->setSpeckleWindowSize(50);//�Ӳ���ͨ�������ص������ֵ
//	sgbm->setSpeckleRange(32);//�Ӳ���ͨ����/��ֵ
//	sgbm->setDisp12MaxDiff(1);//����һ���Լ��������������ֵ
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
//		capture_0.read(frame);///��ȡ��Ƶ֡����frame
//
//		//�ֳ�����		
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
//		//inRange(hsvImage, Scalar(minh, mins, minv), Scalar(maxh, maxs, maxv), imgThresholded);//��ֵ��
//		//morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//		//blur(imgThresholded, bf, Size(3, 3));
//		//Canny(bf, canny_output, g_nThresh, g_nThresh * 2, 3);
//		//GaussianBlur(canny_output, grayImage, Size(9, 9), 2, 2);
//		
//		GaussianBlur(imageL, grayImage, Size(9, 9), 2,2);
//		HoughCircles(grayImage, circles, HOUGH_GRADIENT, 2, 640, 100, 30, 20, 50);//�����������ɫ�ȽϺ�
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
//		std::cout << "    ���ĵ�3D����Ϊ: " << xyz.at<cv::Vec3f>(center) << std::endl;
//
//		circle(frame_0, center, 2, Scalar(0, 255, 0), -1, 8, 0);
//		//draw the circle outline 
//		circle(frame_0, center, radius, Scalar(0, 0, 255), 2, 8, 0);
//
//		imshow("circles", frame_0);
//		//}	
//
//		char c = waitKey(50); //�ȴ�ʱ��50ms
//		if (c == 27) {
//			break;
//		}
//	}
//	capture_0.release();
//
//	return 0;
//}

//===========���ļ��ж�ȡ�ٷ�ͼƬ-->�궨-->����ӳ��-->��������õ��ռ�����============

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
//	vector<vector<Point2f> > imagePoints[2];//����"[2]==>"2��, "Point2f==>"2ά����, ��̬������δ֪
//	vector<vector<Point3f> > objectPoints;//����1��3ά����
//	Size imageSize;
//
//	int i, j, k, nimages = (int)imagelist.size() / 2; //ֻ�����һ����ֵ
//
//	imagePoints[0].resize(nimages);//��չά��Ϊnimages������չ�ĳ�ʼ��Ϊ0
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
//			vector<Point2f>& corners = imagePoints[k][j];//�൱��ָ�룬��������û��pushback
//			for (int scale = 1; scale <= maxScale; scale++)
//			{
//				Mat timg;
//				if (scale == 1)
//					timg = img;
//				else
//					resize(img, timg, Size(), scale, scale);//img�������ţ������timg
//															//����.resize() <==��ͬ��==> resize()
//															//Alt + G �鿴
//															//���ŵ�Ŀ�ģ�δ���ҵ��ǵ�ʱ���Ŵ�ԭͼ����Ѱ��һ��
//				found = findChessboardCorners(timg, boardSize, corners,
//					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//				if (found)
//				{
//					if (scale > 1)
//					{
//						Mat cornersMat(corners); //����һ������Ϊcorners��cornersMat����
//						cornersMat *= 1. / scale;//���Ż�ȥ
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
//			//�����ؾ�ȷ���������find4QuadCornerSubpix��Ҫ�Ҷ�ͼ
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
//	objectPoints.resize(nimages);//��Ϊ������һ���ģ����Թ���һ��objectPoint
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
//	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// �õ���ʼ�������������K��3*3������֤�͵�Ŀ�궨�����һ��
//	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// ������С��һ��
//	Mat R, T, E, F;
//
//	//�ú�������Ҫ�ṩ��ʼ���������
//	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//		cameraMatrix[0], distCoeffs[0],
//		cameraMatrix[1], distCoeffs[1],
//		imageSize, R, T, E, F,
//		CALIB_FIX_ASPECT_RATIO + //�̶�fx/fy�ı�ֵ
//		CALIB_ZERO_TANGENT_DIST + //������������p1,p2��Ϊ��
//		CALIB_USE_INTRINSIC_GUESS + //ʹ��fx,fy,cx,cy����ֵ����Ϊ�����˱궨������Ҫʹ�ø�ģʽ��
//		CALIB_SAME_FOCAL_LENGTH + //ǿ�Ʊ�������������Ľ�����ͬ
//		CALIB_RATIONAL_MODEL + //���û���ϵ��k4,k5��k6������k1,k2,k3,p1,p2�ܹ�8��ϵ��
//		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//��Ӧ�ľ���������Ż��б��ֲ��䣬0�����Ż�
//		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//	cout << "done with RMS error=" << rms << endl;
//	///<��P150������֤��E = t^R������ F �� K2^-T * E * K1^-1 Ϊʲô������
//
//	///< ǰ�����Ŀ�����
//
//	// CALIBRATION QUALITY CHECK
//	// because the output fundamental matrix implicitly
//	// includes all the output information,
//	// we can check the quality of calibration using the
//	// epipolar geometry constraint�Լ�����Լ��: m2^t*F*m1=0
//	// �õ���ƥ�����������뻭���⼫�ߣ�����֤һ��ƥ��Ľ���Ƿ���ȷ��ֻ����֤���Ժ������û�й�ϵ
//
//	double err = 0;
//	int npoints = 0;
//	vector<Vec3f> lines[2];
//	for (i = 0; i < nimages; i++)
//	{
//		int npt = (int)imagePoints[0][i].size(); // imagePoints����ǽǵ����������
//		Mat imgpt[2];
//		for (k = 0; k < 2; k++)
//		{
//			imgpt[k] = Mat(imagePoints[k][i]);
//			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//δ���εĵ㣬������ĵ�
//			///<������ϵ���������������궨��ɺ󣬾�Ӧ�ý��л���Ľ������Դﵽ���������Ŀ��
//			///<��ʹ�õģ����Դ���վ���
//			///<imagePointsҲ�ᱻ�޸ģ�����
//			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//���濼���˻��䣬���������������������
//			//�����forѭ������F*p1��FT*p2��Ȼ�����forѭ���ٳ�p2��p1����OK��
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
//	// save intrinsic parameters�ڲ�
//  //******���(extrinsics)
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
//	//Ҫͨ������ͼ��������������Ϣ���ͱ���������ͼ����׼ȷ��ƥ�䵽ͬһ��㣬�������ܸ��ݸ����������ͼ���е�λ�ù�ϵ������������ȡ�
//	//Ϊ�˽���ƥ��ļ���������������ͷ�ĳ���ƽ��Ӧ����ͬһƽ��(��ת����)�����ǣ����������ϸ�İڷ�����ͷ���ﵽ���Ŀ����Ȼ��Щ���ѡ�
//	//����У���������ü���ͼ�α任��ϵ��ʹ��ԭ�Ȳ���������λ�ù�ϵ������ͼ�������������
//	//https://www.cnblogs.com/german-iris/p/5199203.html
//	stereoRectify(cameraMatrix[0], distCoeffs[0],
//		cameraMatrix[1], distCoeffs[1],
//		imageSize, R, T, R1, R2, P1, P2, Q,
//		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//	///<stereoRectify��������������н��������ֱ�ӽ�ͼƬ����������������ǵó��������������ͼ���Լ�������Ҫ����ת����R1,R2
//	///<���������������ϵ�µ�ͶӰ����P1, P2��3*4, ������ǰ�ı任����T�ˣ��������ǽ�3D�������ת����2ά��������:P*[X Y Z 1]' =[x y w] 
//	///<QΪ4*4����Ȳ���ӳ�������ͶӰ���󣬼�����Q���԰�2ά��������ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ����Ӳ�
//	//����֤ R1 = R2*R�� P1��P2ֻ����t�����������֪����Ϊ�Ѿ������ˣ� Q���Ǽ�����; �������ɣ������Լ�ʵ�ʲ���������
//	///<flags: 0(ˮƽ��ֱ���ƶ�ͼ����ʹ�������õķ�Χ���) �� CV_CALIB_ZERO_DISPARITY(��������У�����ͼ�����������ͬ����������)
//	///<alpha: ����������������Ϊ������ԣ������������졣�������Ϊ0����ôУ����ͼ��ֻ����Ч�Ĳ��ֻᱻ��ʾ��û�к�ɫ�Ĳ��֣����������Ϊ1����ô�ͻ���ʾ����ͼ������Ϊ0~1֮���ĳ��ֵ����Ч��Ҳ��������֮�䡣
//	///<imageSize��ʵ��newImageSize��У�����ͼ���С��һ���ԭͼ����ͬ
//	///<validPixROI1��validPixROI2-����������
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
//	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//���ˣ��Ժ���˵��
//
//																				  // COMPUTE AND DISPLAY RECTIFICATION�����涼����ʾ�������ͼ
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
//	else //�ڶ��ַ�ʽ��ͨ����Ӧ�������R1��R2��P1��P2
//		 // use intrinsic parameters of each camera, but
//		 // compute the rectification���� transformation�任 directly
//		 // from the fundamental matrix
//	{
//		vector<Point2f> allimgpt[2];
//		for (k = 0; k < 2; k++)
//		{
//			for (i = 0; i < nimages; i++)
//				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//		}
//		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//		Mat H1, H2;//��Ӧ����H��R��K֮���ϵ��slam P146
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
//	///<�ú��������Ǽ���������������У����ӳ��任������P1Ӧ�ô����Ǿ��������newCameraMatrix
//	///<��openCV���棬У����ļ��������newCameraMatrix�Ǹ�ͶӰ����Pһ�𷵻صġ�
//	///<�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//	//ʹ��SGBM�㷨��֤���,��һ������ƥ���㷨��׼ȷ�Ⱥ��ٶ����У������бȽϳ���
//	//��ʾ�Ӳ�ͼ�������ûҶ�ͼ��
//	Mat imageL = cv::imread("binocular_chessboard_pic/left01.bmp", 0);
//	Mat imageR = cv::imread("binocular_chessboard_pic/right01.bmp", 0);
//	cv::Mat rectifyImageL, rectifyImageR;
//	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
//	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = imageL.channels();    //
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //���Ϊ80���ӲΧ��������Ӳ�ֵ����С�Ӳ�ֵ֮��, ������16�ı��������Ժ���&-16
//
//	sgbm->setPreFilterCap(63);//ӳ���˲�����С
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1���㹫ʽ
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//��С�Ӳ�
//	sgbm->setNumDisparities(numberOfDisparities);//�Ӳ�������Χ
//	sgbm->setUniquenessRatio(10);//Ψһ�Ա���
//	sgbm->setSpeckleWindowSize(50);//�Ӳ���ͨ�������ص������ֵ
//	sgbm->setSpeckleRange(32);//�Ӳ���ͨ����/��ֵ
//	sgbm->setDisp12MaxDiff(1);//����һ���Լ��������������ֵ
//	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//
//	cv::Mat disp, disp8;
//
//	sgbm->compute(rectifyImageL, rectifyImageR, disp);//Ĭ�ϼ�����������Ӳ�ͼ��������ͼΪ�װ壬���Ӳ�Ϊ�Ҷ�ֵ
//	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));//��16λ�������ε��Ӳ����ת��Ϊ8λ�޷������ξ���???
//	cout << "��ʾ�Ӳ�ͼ" << endl;
//	cv::imshow("disparity8", disp8); //��ʾ�Ӳ�ͼ
//
//	cv::reprojectImageTo3D(disp, xyz, Q, true);
//	//xyz = xyz * 16; // xyz=[X/W Y/W Z/W]������16�õ���ʵ����
//	cv::setMouseCallback("disparity8", onMouse, 0);//��disparity8�Ӳ�ͼ�������ص�
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

//===============���߲ɼ�ͼƬ-->�궨-->����ӳ��-->ʵʱ��ʾ������3D����(HSV��ɫ����+����Բ����)=================

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
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
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
//		cout << "1.��ʼ��ȡ�궨ͼƬ:" << endl;
//		cout << "    -->��enter��ͼ, ��escֹͣ��ͼ" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//			capture_1.read(frame_1);///��ȡ��Ƶ֡����frame
//			imshow("camera_0", frame_0);
//			imshow("camera_1", frame_1);		
//
//			char c = waitKey(50); //�ȴ�ʱ��50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//��ɫ���ҶȾ���
//				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//
//				if (found1&&found2) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//���봫��Ҷ�ͼ
//					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//��ɫͼ�����ڹۿ�
//					drawChessboardCorners(frame_1, boardSize, corners2, found1);
//					imshow("camera_0_corner", frame_0);
//					imshow("camera_1_corner", frame_1);
//					cout << "    ��ͼ�ɹ���" << endl;
//					cout << "    �ǵ���ȡ�ɹ���" << endl;
//					cout << "    -->��enterȷ�ϱ���, ������������" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------��" << pic_counter << "��ͼƬ�ǵ���ȡ�ѱ�����----------->" << endl;
//						cout << "    -->��enter������ͼ, ��escֹͣ��ͼ" << endl;
//						imagePoints[0].push_back(corners1);
//						imagePoints[1].push_back(corners2);
//					}
//				}
//				else {
//					cout << "    δ���ҵ��ǵ㣬�����½�ȡ" << endl;
//				}
//			}
//			if (c == 27) {
//				cout << "��Ƶ��ȡ����" << endl << endl;
//				break; //esc
//			}
//		}
//
//		cout << "2.��ʼ�궨"<<endl;
//
//		vector<vector<Point3f> > objectPoints;
//		objectPoints.resize(pic_counter);//��������
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
//		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// �õ���ʼ�������������K��3*3������֤��ֵ�͵�Ŀ�궨�����һ��
//		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// ������С��һ��
//		Mat R, T, E, F;
//
//		//�ú�������Ҫ�ṩ��ʼ���������
//		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//			cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, E, F,
//			CALIB_FIX_ASPECT_RATIO + //�̶�fx/fy�ı�ֵ
//			CALIB_ZERO_TANGENT_DIST + //������������p1,p2��Ϊ��
//			CALIB_USE_INTRINSIC_GUESS + //ʹ��fx,fy,cx,cy����ֵ����Ϊ�����˱궨������Ҫʹ�ø�ģʽ��
//			CALIB_SAME_FOCAL_LENGTH + //ǿ�Ʊ�������������Ľ�����ͬ
//			CALIB_RATIONAL_MODEL + //���û���ϵ��k4,k5��k6������k1,k2,k3,p1,p2�ܹ�8��ϵ��
//			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//��Ӧ�ľ���������Ż��б��ֲ��䣬0�����Ż�
//			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//		cout << "    �궨���Ϊ��" << rms << endl;
//		///<��P150������֤��E = t^R������ F �� K2^-T * E * K1^-1 Ϊʲô������
//
//		///< ǰ�����Ŀ�����
//
//		// CALIBRATION QUALITY CHECK
//		// because the output fundamental matrix implicitly
//		// includes all the output information,
//		// we can check the quality of calibration using the
//		// epipolar geometry constraint�Լ�����Լ��: m2^t*F*m1=0
//		// �õ���ƥ�����������뻭���⼫�ߣ�����֤һ��ƥ��Ľ���Ƿ���ȷ��ֻ����֤���Ժ������û�й�ϵ
//
//		double err = 0;
//		int npoints = 0;
//		vector<Vec3f> lines[2];
//		for (int i = 0; i < pic_counter; i++)
//		{
//			int npt = (int)imagePoints[0][i].size(); // imagePoints����ǽǵ����������
//			Mat imgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				imgpt[k] = Mat(imagePoints[k][i]);
//				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//δ���εĵ㣬������ĵ�
//																											///<������ϵ���������������궨��ɺ󣬾�Ӧ�ý��л���Ľ������Դﵽ���������Ŀ��
//				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//���濼���˻��䣬���������������������
//				//�����forѭ������F*p1��FT*p2��Ȼ�����forѭ���ٳ�p2��p1����OK��
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
//		cout << "    ƽ���Լ�����" << err / npoints << endl;
//
//		// save intrinsic parameters�ڲ� ******���(extrinsics)
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
//		cout << "3.����������ӳ��"<< endl;
//		//Ҫͨ������ͼ��������������Ϣ���ͱ���������ͼ����׼ȷ��ƥ�䵽ͬһ��㣬�������ܸ��ݸ����������ͼ���е�λ�ù�ϵ������������ȡ�
//		//Ϊ�˽���ƥ��ļ���������������ͷ�ĳ���ƽ��Ӧ����ͬһƽ��(��ת����)�����ǣ����������ϸ�İڷ�����ͷ���ﵽ���Ŀ����Ȼ��Щ���ѡ�
//		//����У���������ü���ͼ�α任��ϵ��ʹ��ԭ�Ȳ���������λ�ù�ϵ������ͼ�������������
//		//https://www.cnblogs.com/german-iris/p/5199203.html
//		stereoRectify(cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, R1, R2, P1, P2, Q,
//			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//		///<stereoRectify��������������н��������ֱ�ӽ�ͼƬ����������������ǵó��������������ͼ���Լ�������Ҫ����ת����R1,R2
//		///<���������������ϵ�µ�ͶӰ����P1, P2��3*4, ������ǰ�ı任����T�ˣ��������ǽ�3D�������ת����2ά��������:P*[X Y Z 1]' =[x y w] 
//		///<QΪ4*4����Ȳ���ӳ�������ͶӰ���󣬼�����Q���԰�2ά��������ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ����Ӳ�
//		//����֤ R1 = R2*R�� P1��P2ֻ����t�����������֪����Ϊ�Ѿ������ˣ� Q���Ǽ�����; �������ɣ������Լ�ʵ�ʲ���������
//		///<flags: 0(ˮƽ��ֱ���ƶ�ͼ����ʹ�������õķ�Χ���) �� CV_CALIB_ZERO_DISPARITY(��������У�����ͼ�����������ͬ����������)
//		///<alpha: ����������������Ϊ������ԣ������������졣�������Ϊ0����ôУ����ͼ��ֻ����Ч�Ĳ��ֻᱻ��ʾ��û�к�ɫ�Ĳ��֣����������Ϊ1����ô�ͻ���ʾ����ͼ������Ϊ0~1֮���ĳ��ֵ����Ч��Ҳ��������֮�䡣
//		///<imageSize��ʵ��newImageSize��У�����ͼ���С��һ���ԭͼ����ͬ
//		///<validPixROI1��validPixROI2-����������
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
//		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//���ˣ��Ժ���˵��
//
//																					  // COMPUTE AND DISPLAY RECTIFICATION�����涼����ʾ�������ͼ
//		if (!showRectified)
//			return -1;
//
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useCalibrated)
//		{
//			// we already computed everything
//		}
//		// OR ELSE HARTLEY'S METHOD
//		else //�ڶ��ַ�ʽ��ͨ����Ӧ�������R1��R2��P1��P2
//			 // use intrinsic parameters of each camera, but
//			 // compute the rectification���� transformation�任 directly
//			 // from the fundamental matrix
//		{
//			vector<Point2f> allimgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				for (int i = 0; i < pic_counter; i++)
//					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//			}
//			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//			Mat H1, H2;//��Ӧ����H��R��K֮���ϵ��slam P146
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
//		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //������ʽ��yml����ȻҲ���Դ�xml����Ҫ����׺
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
//	//�ú��������Ǽ���������������У����ӳ��任������P1Ӧ�ô����Ǿ��������newCameraMatrix
//	//��openCV���棬У����ļ��������newCameraMatrix�Ǹ�ͶӰ����Pһ�𷵻صġ�
//	//�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
//	//ͶӰ�������������󵽵�ʲô��ϵ����
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//
//	////ʹ��SGBM�㷨��֤���,��һ������ƥ���㷨��׼ȷ�Ⱥ��ٶ����У������бȽϳ���
//	////��ʾ�Ӳ�ͼ�������ûҶ�ͼ��
//
//	cout << "4.Ŀ��׷��" << endl;
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = 1;//imageL.channels()
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //���Ϊ80���ӲΧ��������Ӳ�ֵ����С�Ӳ�ֵ֮��, ������16�ı��������Ժ���&-16
//
//	sgbm->setPreFilterCap(63);//ӳ���˲�����С
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1���㹫ʽ
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//��С�Ӳ�
//	sgbm->setNumDisparities(numberOfDisparities);//�Ӳ�������Χ
//	sgbm->setUniquenessRatio(10);//Ψһ�Ա���
//	sgbm->setSpeckleWindowSize(50);//�Ӳ���ͨ�������ص������ֵ
//	sgbm->setSpeckleRange(32);//�Ӳ���ͨ����/��ֵ
//	sgbm->setDisp12MaxDiff(1);//����һ���Լ��������������ֵ
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
//		capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//		capture_1.read(frame_1);///��ȡ��Ƶ֡����frame
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
//		inRange(hsvImage, Scalar(minh, mins, minv), Scalar(maxh, maxs, maxv), imgThresholded);//��ֵ��
//		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//		blur(imgThresholded, bf, Size(3, 3));
//		Canny(bf, canny_output, g_nThresh, g_nThresh * 2, 3);
//		GaussianBlur(canny_output, grayImage, Size(9, 9), 2, 2);		
//		HoughCircles(grayImage, circles, HOUGH_GRADIENT, 2, 640, 100, 30, 20, 50);//�����������ɫ�ȽϺ�
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
//		std::cout << "    ���ĵ�3D����Ϊ: " << xyz.at<cv::Vec3f>(center) << std::endl;
//
//		circle(frame_0, center, 2, Scalar(0, 255, 0), -1, 8, 0);
//		//draw the circle outline 
//		circle(frame_0, center, radius, Scalar(0, 0, 255), 2, 8, 0);
//
//		imshow("circles", frame_0);
//		//}	
//
//		char c = waitKey(50); //�ȴ�ʱ��50ms
//		if (c == 27) {
//			break;
//		}
//	}
//	capture_0.release();
//	capture_1.release();
//
//	return 0;
//}


//===============���߲ɼ�ͼƬ-->�궨-->����ӳ��-->���߽�ȡһ��ͼƬ����������������õ��ռ�����================

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
//	VideoCapture capture_0(0);///< ������ͷ��������Ҫ��һ�����ú�����opencv��ĺ�ǿ��
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
//		cout << "1.��ʼ��ȡ�궨ͼƬ:" << endl;
//		cout << "    -->��enter��ͼ, ��escֹͣ��ͼ" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//			capture_1.read(frame_1);///��ȡ��Ƶ֡����frame
//			imshow("camera_0", frame_0);
//			imshow("camera_1", frame_1);
//
//			char c = waitKey(50); //�ȴ�ʱ��50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				cvtColor(frame_1, img2, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//��ɫ���ҶȾ���
//				found2 = findChessboardCorners(frame_1, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//
//				if (found1&&found2) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//���봫��Ҷ�ͼ
//					cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//��ɫͼ�����ڹۿ�
//					drawChessboardCorners(frame_1, boardSize, corners2, found1);
//					imshow("camera_0_corner", frame_0);
//					imshow("camera_1_corner", frame_1);
//					cout << "    ��ͼ�ɹ���" << endl;
//					cout << "    �ǵ���ȡ�ɹ���" << endl;
//					cout << "    -->��enterȷ�ϱ���, ������������" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------��" << pic_counter << "��ͼƬ�ǵ���ȡ�ѱ�����----------->" << endl;
//						cout << "    -->��enter������ͼ, ��escֹͣ��ͼ" << endl;
//						imagePoints[0].push_back(corners1);
//						imagePoints[1].push_back(corners2);
//					}
//				}
//				else {
//					cout << "    δ���ҵ��ǵ㣬�����½�ȡ" << endl;
//				}
//			}
//			if (c == 27) {
//				cout << "��Ƶ��ȡ����" << endl << endl;
//				break; //esc
//			}
//		}
//
//		cout << "2.��ʼ�궨"<<endl;
//
//		vector<vector<Point3f> > objectPoints;
//		objectPoints.resize(pic_counter);//��������
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
//		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);// �õ���ʼ�������������K��3*3������֤�͵�Ŀ�궨�����һ��
//		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);// ������С��һ��
//		Mat R, T, E, F;
//
//		//�ú�������Ҫ�ṩ��ʼ���������
//		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
//			cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, E, F,
//			CALIB_FIX_ASPECT_RATIO + //�̶�fx/fy�ı�ֵ
//			CALIB_ZERO_TANGENT_DIST + //������������p1,p2��Ϊ��
//			CALIB_USE_INTRINSIC_GUESS + //ʹ��fx,fy,cx,cy����ֵ����Ϊ�����˱궨������Ҫʹ�ø�ģʽ��
//			CALIB_SAME_FOCAL_LENGTH + //ǿ�Ʊ�������������Ľ�����ͬ
//			CALIB_RATIONAL_MODEL + //���û���ϵ��k4,k5��k6������k1,k2,k3,p1,p2�ܹ�8��ϵ��
//			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//��Ӧ�ľ���������Ż��б��ֲ��䣬0�����Ż�
//			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
//		cout << "    �궨���Ϊ��" << rms << endl;
//		///<��P150������֤��E = t^R������ F �� K2^-T * E * K1^-1 Ϊʲô������
//
//		///< ǰ�����Ŀ�����
//
//		// CALIBRATION QUALITY CHECK
//		// because the output fundamental matrix implicitly
//		// includes all the output information,
//		// we can check the quality of calibration using the
//		// epipolar geometry constraint�Լ�����Լ��: m2^t*F*m1=0
//		// �õ���ƥ�����������뻭���⼫�ߣ�����֤һ��ƥ��Ľ���Ƿ���ȷ��ֻ����֤���Ժ������û�й�ϵ
//
//		double err = 0;
//		int npoints = 0;
//		vector<Vec3f> lines[2];
//		for (int i = 0; i < pic_counter; i++)
//		{
//			int npt = (int)imagePoints[0][i].size(); // imagePoints����ǽǵ����������
//			Mat imgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				imgpt[k] = Mat(imagePoints[k][i]);
//				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//δ���εĵ㣬������ĵ�
//																											///<������ϵ���������������궨��ɺ󣬾�Ӧ�ý��л���Ľ������Դﵽ���������Ŀ��
//				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);//���濼���˻��䣬���������������������
//				//�����forѭ������F*p1��FT*p2��Ȼ�����forѭ���ٳ�p2��p1����OK��
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
//		cout << "    ƽ���Լ�����" << err / npoints << endl;
//
//		// save intrinsic parameters�ڲ� ******���(extrinsics)
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
//		cout << "3.����������ӳ��"<< endl;
//		//Ҫͨ������ͼ��������������Ϣ���ͱ���������ͼ����׼ȷ��ƥ�䵽ͬһ��㣬�������ܸ��ݸ����������ͼ���е�λ�ù�ϵ������������ȡ�
//		//Ϊ�˽���ƥ��ļ���������������ͷ�ĳ���ƽ��Ӧ����ͬһƽ��(��ת����)�����ǣ����������ϸ�İڷ�����ͷ���ﵽ���Ŀ����Ȼ��Щ���ѡ�
//		//����У���������ü���ͼ�α任��ϵ��ʹ��ԭ�Ȳ���������λ�ù�ϵ������ͼ�������������
//		//https://www.cnblogs.com/german-iris/p/5199203.html
//		stereoRectify(cameraMatrix[0], distCoeffs[0],
//			cameraMatrix[1], distCoeffs[1],
//			imageSize, R, T, R1, R2, P1, P2, Q,
//			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
//		///<stereoRectify��������������н��������ֱ�ӽ�ͼƬ����������������ǵó��������������ͼ���Լ�������Ҫ����ת����R1,R2
//		///<���������������ϵ�µ�ͶӰ����P1, P2��3*4, ������ǰ�ı任����T�ˣ��������ǽ�3D�������ת����2ά��������:P*[X Y Z 1]' =[x y w] 
//		///<QΪ4*4����Ȳ���ӳ�������ͶӰ���󣬼�����Q���԰�2ά��������ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ����Ӳ�
//		//����֤ R1 = R2*R�� P1��P2ֻ����t�����������֪����Ϊ�Ѿ������ˣ� Q���Ǽ�����; �������ɣ������Լ�ʵ�ʲ���������
//		///<flags: 0(ˮƽ��ֱ���ƶ�ͼ����ʹ�������õķ�Χ���) �� CV_CALIB_ZERO_DISPARITY(��������У�����ͼ�����������ͬ����������)
//		///<alpha: ����������������Ϊ������ԣ������������졣�������Ϊ0����ôУ����ͼ��ֻ����Ч�Ĳ��ֻᱻ��ʾ��û�к�ɫ�Ĳ��֣����������Ϊ1����ô�ͻ���ʾ����ͼ������Ϊ0~1֮���ĳ��ֵ����Ч��Ҳ��������֮�䡣
//		///<imageSize��ʵ��newImageSize��У�����ͼ���С��һ���ԭͼ����ͬ
//		///<validPixROI1��validPixROI2-����������
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
//		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));//���ˣ��Ժ���˵��
//
//																					  // COMPUTE AND DISPLAY RECTIFICATION�����涼����ʾ�������ͼ
//		if (!showRectified)
//			return -1;
//
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useCalibrated)
//		{
//			// we already computed everything
//		}
//		// OR ELSE HARTLEY'S METHOD
//		else //�ڶ��ַ�ʽ��ͨ����Ӧ�������R1��R2��P1��P2
//			 // use intrinsic parameters of each camera, but
//			 // compute the rectification���� transformation�任 directly
//			 // from the fundamental matrix
//		{
//			vector<Point2f> allimgpt[2];
//			for (int k = 0; k < 2; k++)
//			{
//				for (int i = 0; i < pic_counter; i++)
//					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//			}
//			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//			Mat H1, H2;//��Ӧ����H��R��K֮���ϵ��slam P146
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
//		FileStorage fs1("binocular_chessboard_pic/extrinsics.yml", FileStorage::READ); //������ʽ��yml����ȻҲ���Դ�xml����Ҫ����׺
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
//	//�ú��������Ǽ���������������У����ӳ��任������P1Ӧ�ô����Ǿ��������newCameraMatrix
//	//��openCV���棬У����ļ��������newCameraMatrix�Ǹ�ͶӰ����Pһ�𷵻صġ�
//	//�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
//	//ͶӰ�������������󵽵�ʲô��ϵ����
//	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//
//	////ʹ��SGBM�㷨��֤���,��һ������ƥ���㷨��׼ȷ�Ⱥ��ٶ����У������бȽϳ���
//	////��ʾ�Ӳ�ͼ�������ûҶ�ͼ��	
//	
//	cout << "4.����������������õ��ռ�����" << endl;
//	Mat imageL, imageR;
//
//	cout << "    -->��enter��ͼ" << endl;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///��ȡ��Ƶ֡����frame
//		capture_1.read(frame_1);///��ȡ��Ƶ֡����frame
//		imshow("camera_0", frame_0);
//		imshow("camera_1", frame_1);
//
//		char c = waitKey(50); //�ȴ�ʱ��50ms
//		if (c == 13) {//enter		
//			cvtColor(frame_0, imageL, COLOR_BGR2GRAY);
//			cvtColor(frame_1, imageR, COLOR_BGR2GRAY);
//			cout << "    ��Ƶ��ȡ����!" << endl;
//			break;
//		}
//	}
//	capture_0.release();
//	capture_1.release();
//
//	cout << "    ��ʼ����!" << endl;
//	cv::Mat rectifyImageL, rectifyImageR;
//	cv::remap(imageL, rectifyImageL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
//	cv::remap(imageR, rectifyImageR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
//	//imshow("rectifyImageL", rectifyImageL);
//	//imshow("rectifyImageR", rectifyImageR);
//	///Q��remap����������ͼ--remap-->�����������ͼ---->�Ӳ�--Q-->3D����
//
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
//	int sgbmWinSize = 3;
//	int cn = imageL.channels();    //
//	int numberOfDisparities = ((imageSize.width / 8) + 15) & -16; //���Ϊ80���ӲΧ��������Ӳ�ֵ����С�Ӳ�ֵ֮��, ������16�ı��������Ժ���&-16
//
//	sgbm->setPreFilterCap(63);//ӳ���˲�����С
//	sgbm->setBlockSize(sgbmWinSize);//3*3
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);//P1���㹫ʽ
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);//��С�Ӳ�
//	sgbm->setNumDisparities(numberOfDisparities);//�Ӳ�������Χ
//	sgbm->setUniquenessRatio(10);//Ψһ�Ա���
//	sgbm->setSpeckleWindowSize(50);//�Ӳ���ͨ�������ص������ֵ
//	sgbm->setSpeckleRange(32);//�Ӳ���ͨ����/��ֵ
//	sgbm->setDisp12MaxDiff(1);//����һ���Լ��������������ֵ
//	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//
//	cv::Mat disp, disp8;
//
//	sgbm->compute(rectifyImageL, rectifyImageR, disp);//Ĭ�ϼ�����������Ӳ�ͼ��������ͼΪ�װ壬���Ӳ�Ϊ�Ҷ�ֵ
//	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));//��16λ�������ε��Ӳ����ת��Ϊ8λ�޷������ξ���???
//	cout << "    ��ʾ�Ӳ�ͼ" << endl;
//	cout << "    -->ͨ�������ͼ���ϵĵ���Եõ�3D����!!!" << endl;
//	cv::imshow("disparity8", disp8); //��ʾ�Ӳ�ͼ
//
//	cv::reprojectImageTo3D(disp, xyz, Q, true);//�Ӳ������Q��
//	xyz = xyz * 16; // xyz=[X/W Y/W Z/W]������16�õ���ʵ����
//	cv::setMouseCallback("disparity8", onMouse, 0);//��disparity8�Ӳ�ͼ�������ص�
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
//	cout << "    ��ʾ����ͼ" << endl;
//	cv::imshow("imageMatches", imageMatches);
//	cv::setMouseCallback("imageMatches", onMouse, 0);
//	
//	waitKey();
//
//	return 0;
//}
