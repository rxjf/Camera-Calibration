#include <windows.h> //写入时间，不能写在.h中

#include "monocular_chessboard.h"

//==============从文件读取图片-->标定-->矫正映射-->标定里的第一张图片矫正测试================

//int MonocularChess(void)
//{	
//	ifstream fin("monocular_chessboard_pic/calibdata.txt"); /* 标定所用图像文件的路径 */
//	ofstream fout("monocular_chessboard_pic/caliberation_result.txt");  /* 保存标定结果的文件 */
//											   //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
//	cout << "开始提取角点………………";
//	int image_count = 0;  /* 图像数量 */
//	Size image_size;  /* 图像的尺寸 */
//	Size board_size = Size(8, 5);    /* 标定板上每行、列的角点数(9,6) */
//	vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
//	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
//	string filename;
//	int count = -1;//用于存储角点个数。
//	while (getline(fin, filename))
//	{
//		image_count++;
//		// 用于观察检验输出
//		cout << "image_count = " << image_count << endl;
//		/* 输出检验*/
//		cout << "-->count = " << count;
//		Mat imageInput = imread(filename);
//		if (image_count == 1)  //读入第一张图片时获取图像宽高信息
//		{
//			image_size.width = imageInput.cols;
//			image_size.height = imageInput.rows;
//			cout << "image_size.width = " << image_size.width << endl;
//			cout << "image_size.height = " << image_size.height << endl;
//		}
//
//		///* 提取角点 */
//		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
//		{
//			cout << "can not find chessboard corners!\n"; //找不到角点
//			system("pause");
//			exit(1);
//		}
//		else
//		{
//			Mat view_gray;
//			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
//			/* 亚像素精确化 */
//			find4QuadCornerSubpix(view_gray, image_points_buf, Size(11, 11)); 
//			///<对粗提取的角点进行精确化
//			///<image_points_buf作为初始的角点坐标向量，同时作为亚像素坐标位置的输出
//			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));  
//			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
//														   /* 在图像上显示角点位置 */
//			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点
//			imshow("Camera Calibration", view_gray);//显示图片
//			waitKey(500);//暂停0.5S		
//		}
//	}
//	int total = image_points_seq.size();
//	cout << "total = " << total << endl;
//	int CornerNum = board_size.width*board_size.height;  //每张图片上总的角点数
//	for (int ii = 0; ii < total; ii++)
//	{
//		if (0 == ii%CornerNum)// 24 是每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看 
//		{
//			int i = -1;
//			i = ii / CornerNum;
//			int j = i + 1;
//			cout << "--> 第 " << j << "图片的数据 --> : " << endl;
//		}
//		if (0 == ii % 3)	// 此判断语句，格式化输出，便于控制台查看
//		{
//			cout << endl;
//		}
//		else
//		{
//			cout.width(10);
//		}
//		//输出所有的角点
//		cout << " -->" << image_points_seq[ii][0].x; //[0]只输出一个看看
//		cout << " -->" << image_points_seq[ii][0].y;
//	}
//	cout << "角点提取完成！\n";
//
//
//
//	//以下是摄像机标定
//	cout << "开始标定………………";
//	/*棋盘三维信息*/
//	Size square_size = Size(5, 5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
//	vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
//										   /*内外参数*/
//	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵，32位浮点，3*3 *///有各种风格的初始化方式
//	vector<int> point_counts;  // 每幅图像中角点的数量
//	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
//	vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
//	vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
//						  /* 初始化标定板上角点的三维坐标 */
//	int i, j, t;
//	for (t = 0; t < image_count; t++)
//	{
//		vector<Point3f> tempPointSet;
//		for (i = 0; i < board_size.height; i++)
//		{
//			for (j = 0; j < board_size.width; j++)
//			{
//				Point3f realPoint;
//				/* 假设标定板放在世界坐标系中z=0的平面上 */
//				realPoint.x = i*square_size.width;
//				realPoint.y = j*square_size.height;
//				realPoint.z = 0;
//				tempPointSet.push_back(realPoint);
//			}
//		}
//		object_points.push_back(tempPointSet);
//	}
//	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
//	for (i = 0; i < image_count; i++)
//	{
//		point_counts.push_back(board_size.width*board_size.height);
//	}
//	/* 开始标定 */
//	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
//	///<最后一个是标志位，不同的标志可能是零或以下值的组合：
//	///<①CV_CALIB_USE_INTRINSIC_GUESS ，使用该参数时，在cameraMatrix矩阵中应该有fx, fy, cx, cy的估计值。否则的话，将初始化(cx, cy）图像的中心点，使用最小二乘估算出fx，fy。如果内参数矩阵和畸变已知的话，应该标定模块中的solvePnP()函数计算外参数矩阵，而不用calibrateCamera。
//	///<②CV_CALIB_FIX_PRINCIPAL_POINT，在进行优化时会固定光轴点。当CV_CALIB_USE_INTRINSIC_GUESS参数被设置，光轴点将保持在中心或者某个输入的值。
//	///<③CV_CALIB_FIX_ASPECT_RATIO，固定fx / fy的比值，只将fy作为可变量，进行优化计算。当CV_CALIB_USE_INTRINSIC_GUESS没有被设置，fx和fy将会被忽略。只有fx / fy的比值在计算中会被用到。
//	///<④CV_CALIB_ZERO_TANGENT_DIST，设定切向畸变参数（p1, p2）为零
//	///<⑤CV_CALIB_FIX_K1，...，CV_CALIB_FIX_K6对应的径向畸变在优化中保持不变，如果设置了CV_CALIB_USE_INTRINSIC_GUESS参数。
//	///<⑥CV_CALIB_RATIONAL_MODEL启用畸变系数k4，k5和k6（那应该增多多项式）。为了提供向后兼容性，应该明确指定这个额外的标志，以使校准函数使用有理模型并返回8个系数（加上k1, k2, k3, p1, p2总共8个系数）。如果没有设置标志，则该功能仅计算并返回5个畸变系数。
//	///<⑦ CALIB_SAME_FOCAL_LENGTH，强制保持两个摄像机的焦距相同
//
//	cout << "标定完成！\n";
//	//对标定结果进行评价
//	cout << "开始评价标定结果………………\n";
//	double total_err = 0.0; /* 所有图像的平均误差的总和 */
//	double err = 0.0; /* 每幅图像的平均误差 */
//	vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
//	cout << "\t每幅图像的标定误差：\n";
//	fout << "每幅图像的标定误差：\n";
//	for (i = 0; i < image_count; i++)
//	{
//		vector<Point3f> tempPointSet = object_points[i];
//		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
//		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
//		/* 计算新的投影点和旧的投影点之间的误差*/
//		vector<Point2f> tempImagePoint = image_points_seq[i];
//		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);//32位浮点，双通道
//		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
//		for (int j = 0; j < tempImagePoint.size(); j++)
//		{
//			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
//			//cout << "image_points2  " << image_points2[j].x<<"   "<<image_points2[j].y << endl;
//			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
//			//cout << "tempImagePoint  " << tempImagePoint[j].x << "   " << tempImagePoint[j].y << endl;
//		}
//		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);//二范数，即平方和
//		total_err += err /= point_counts[i];
//		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
//		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
//	}
//	std::cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
//	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
//	std::cout << "评价完成！" << endl;
//	//保存定标结果  	
//	std::cout << "开始保存定标结果………………" << endl;
//	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
//	fout << "相机内参数矩阵：" << endl;
//	fout << cameraMatrix << endl << endl;
//	fout << "畸变系数：\n";
//	fout << distCoeffs << endl << endl << endl;
//	for (int i = 0; i < image_count; i++)
//	{
//		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
//		fout << tvecsMat[i] << endl;
//		/* 将旋转向量转换为相对应的旋转矩阵 */
//		Rodrigues(tvecsMat[i], rotation_matrix);
//		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
//		fout << rotation_matrix << endl;
//		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
//		fout << rvecsMat[i] << endl << endl;
//	}
//	std::cout << "完成保存" << endl;
//	fout << endl;
//	/************************************************************************
//	显示定标结果
//	*************************************************************************/
//	Mat mapx = Mat(image_size, CV_32FC1);
//	Mat mapy = Mat(image_size, CV_32FC1);
//	Mat R = Mat::eye(3, 3, CV_32F);
//	std::cout << "保存矫正图像" << endl;
//	string imageFileName;
//	std::stringstream StrStm;
//	for (int i = 0; i != image_count; i++)
//	{
//		std::cout << "Frame #" << i + 1 << "..." << endl;
//		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
//		StrStm.clear();
//		imageFileName.clear();
//		string filePath = "monocular_chessboard_pic/pic";
//		StrStm << i + 1;
//		StrStm >> imageFileName;
//		filePath += imageFileName;
//		filePath += ".bmp";
//		Mat imageSource = imread(filePath);
//		Mat newimage = imageSource.clone();
//		//另一种不需要转换矩阵的方式
//		//undistort(imageSource,newimage,cameraMatrix,distCoeffs);
//		remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);//双线性差值
//		StrStm.clear();
//		filePath.clear();
//		StrStm << i + 1;
//		StrStm >> imageFileName;
//		imageFileName += "_d.bmp";
//		imwrite(imageFileName, newimage);
//	}
//	std::cout << "保存结束" << endl;
//
//	return 0;
//}

//==============新的标定方法在线采集图片-->标定-->矫正映射-->在线截取一张图片矫正测试================

int MonocularChess(void)
{
	Size boardSize = Size(8, 5);//(9,6)
	const float squareSize = 20;

	bool useYmlData = false;
	bool release_object = false;

	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大

	if (!capture_0.isOpened()) {
		cout << "Could not open camera_0" << endl;
		return -1;
	}

	Mat frame_0;
	Mat map1, map2;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;

	if (!useYmlData) {
		int pic_counter = 0;
		bool found1 = false;

		vector<Point2f> corners1;// = imagePoints[k][j];
		vector<vector<Point2f> > imagePoints;
		Mat img1;

		cout << "1.开始截取标定图片:" << endl;
		cout << "    -->按enter截图, 按esc停止截图" << endl;

		while (true)
		{
			capture_0.read(frame_0);///读取视频帧放入frame
			imshow("camera_0", frame_0);

			char c = waitKey(50); //等待时间50ms
			if (c == 13) {//enter		
				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//彩色、灰度均可

				if (found1) {
					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
							30, 0.01));//必须传入灰度图
					drawChessboardCorners(frame_0, boardSize, corners1, found1);//彩色图，便于观看
					imshow("camera_0_corner", frame_0);

					cout << "    截图成功！" << endl;
					cout << "    角点提取成功！" << endl;
					cout << "    -->按enter确认保留, 按其他不保留" << endl;
					c = waitKey();
					if (c == 13) {
						pic_counter++;
						cout << "<-----------第" << pic_counter << "张图片角点提取已被保留----------->" << endl;
						cout << "    -->按enter继续截图, 按esc停止截图" << endl;
						imagePoints.push_back(corners1);
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

		cout << "2.开始标定" << endl;

		vector<vector<Point3f> > objectPoints(1);
		vector<Point3f> newObjPoints;

		for (int j = 0; j < boardSize.height; j++)
			for (int k = 0; k < boardSize.width; k++)
				objectPoints[0].push_back(Point3f(k*squareSize, j*squareSize, 0));

		float grid_width = squareSize * (boardSize.width - 1);
		objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
		newObjPoints = objectPoints[0];
		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		imageSize = frame_0.size();

		double rms;
		int iFixedPoint = -1;
		if (release_object)
			iFixedPoint = boardSize.width - 1;
		vector<Mat> rvecs, tvecs;

		rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
			cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
			CALIB_FIX_ASPECT_RATIO | CALIB_USE_LU);
		cout << "    标定误差为：" << rms << endl;

		// save intrinsic parameters内参 ******外参(extrinsics)
		FileStorage fs("monocular_chessboard_pic/trinsics.yml", FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "imageSize" << imageSize << "M" << cameraMatrix << "D" << distCoeffs << "R" << rvecs << "t" << tvecs;
			fs.release();
		}
		else
			cout << "    Error: can not save the trinsic parameters\n";

		ofstream fout;
		fout.open("monocular_chessboard_pic/caliberation_result.txt", ios::app);

		SYSTEMTIME sys;
		GetLocalTime(&sys);
		fout << sys.wYear << "-" << sys.wMonth << "-" << sys.wDay << " " << sys.wHour << ":" << sys.wMinute << ":" << sys.wSecond << endl << endl;
		fout << "相机内参数矩阵：" << endl;
		fout << cameraMatrix << endl << endl;
		fout << "畸变系数：" << endl;
		fout << distCoeffs << endl << endl;
		fout << "--------------------------------------------------" << endl << endl;
		fout.close();

		cout << endl;
	}
	else {
		FileStorage fs1("monocular_chessboard_pic/trinsics.yml", FileStorage::READ); //读的形式打开yml。当然也可以打开xml，主要看后缀

		if (fs1.isOpened())
		{
			fs1["imageSize"] >> imageSize;
			fs1["M"] >> cameraMatrix;
			fs1["D"] >> distCoeffs;
		}

		fs1.release();
	}

	cout << "3.求解矫正映射" << endl;
	initUndistortRectifyMap(
		cameraMatrix, distCoeffs, Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
		CV_16SC2, map1, map2);
	cout << endl;


	cout << "4.矫正测试与鼠标点击得到空间坐标" << endl;
	cout << "    -->按enter截图" << endl;

	while (true)
	{
		capture_0.read(frame_0);///读取视频帧放入frame
		imshow("camera_0", frame_0);

		char c = waitKey(50); //等待时间50ms
		if (c == 13) {//enter		
			cout << "    视频截取结束!" << endl;
			break;
		}
	}
	capture_0.release();

	cout << "    开始矫正!" << endl;
	cv::Mat remapImage;
	cv::remap(frame_0, remapImage, map1, map2, cv::INTER_LINEAR);
	imshow("remapImage", remapImage);
	waitKey();

	return 0;
}

//==============在线采集图片-->标定-->矫正映射-->在线截取一张图片矫正测试================

//int MonocularChess(void)
//{
//	Size boardSize = Size(8, 5);//(9,6)
//	const float squareSize = 25;
//
//	bool useYmlData = false;
//	bool release_object = false;
//
//	VideoCapture capture_0(0);///< 打开摄像头，最最主要的一个调用函数，opencv真的很强大
//
//	if (!capture_0.isOpened()) {
//		cout << "Could not open camera_0" << endl;
//		return -1;
//	}
//
//	Mat frame_0;
//	Mat map1, map2;
//	Mat cameraMatrix, distCoeffs;
//	Size imageSize;
//
//	if (!useYmlData) {
//		int pic_counter = 0;
//		bool found1 = false;
//
//		vector<Point2f> corners1;// = imagePoints[k][j];
//		vector<vector<Point2f> > imagePoints;
//		Mat img1;
//
//		cout << "1.开始截取标定图片:" << endl;
//		cout << "    -->按enter截图, 按esc停止截图" << endl;
//
//		while (true)
//		{
//			capture_0.read(frame_0);///读取视频帧放入frame
//			imshow("camera_0", frame_0);
//
//			char c = waitKey(50); //等待时间50ms
//			if (c == 13) {//enter		
//				cvtColor(frame_0, img1, COLOR_BGR2GRAY);
//				found1 = findChessboardCorners(frame_0, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//彩色、灰度均可
//
//				if (found1) {
//					cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
//						TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
//							30, 0.01));//必须传入灰度图
//					drawChessboardCorners(frame_0, boardSize, corners1, found1);//彩色图，便于观看
//					imshow("camera_0_corner", frame_0);
//
//					cout << "    截图成功！" << endl;
//					cout << "    角点提取成功！" << endl;
//					cout << "    -->按enter确认保留, 按其他不保留" << endl;
//					c = waitKey();
//					if (c == 13) {
//						pic_counter++;
//						cout << "<-----------第" << pic_counter << "张图片角点提取已被保留----------->" << endl;
//						cout << "    -->按enter继续截图, 按esc停止截图" << endl;
//						imagePoints.push_back(corners1);
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
//		vector<vector<Point3f> > objectPoints(1);
//		vector<Point3f> newObjPoints;
//
//		for (int j = 0; j < boardSize.height; j++)
//			for (int k = 0; k < boardSize.width; k++)
//				objectPoints[0].push_back(Point3f(k*squareSize, j*squareSize, 0));
//
//		//float grid_width = squareSize * (boardSize.width - 1);
//		//objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
//		//newObjPoints = objectPoints[0];
//		objectPoints.resize(imagePoints.size(), objectPoints[0]);
//
//		imageSize = frame_0.size();
//
//		double rms;
//		//int iFixedPoint = -1;
//		//if (release_object)
//			//iFixedPoint = boardSize.width - 1;
//		vector<Mat> rvecs, tvecs;
//
//		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
//		cout << "    标定误差为：" << rms << endl;
//
//		// save intrinsic parameters内参 ******外参(extrinsics)
//		FileStorage fs("monocular_chessboard_pic/trinsics.yml", FileStorage::WRITE);
//		if (fs.isOpened())
//		{
//			fs << "imageSize" << imageSize << "M" << cameraMatrix << "D" << distCoeffs << "R" << rvecs << "t" << tvecs;
//			fs.release();
//		}
//		else
//			cout << "    Error: can not save the trinsic parameters\n";
//
//		ofstream fout;
//		fout.open("monocular_chessboard_pic/caliberation_result.txt", ios::app);
//
//		SYSTEMTIME sys;
//		GetLocalTime(&sys);
//		fout << sys.wYear << "-" << sys.wMonth << "-" << sys.wDay << " " << sys.wHour << ":" << sys.wMinute << ":" << sys.wSecond << endl << endl;
//		fout << "相机内参数矩阵：" << endl;
//		fout << cameraMatrix << endl << endl;
//		fout << "畸变系数：" << endl;
//		fout << distCoeffs << endl << endl;
//		fout << "--------------------------------------------------" << endl << endl;
//		fout.close();
//
//		cout << endl;
//	}
//	else {
//		FileStorage fs1("monocular_chessboard_pic/trinsics.yml", FileStorage::READ); //读的形式打开yml。当然也可以打开xml，主要看后缀
//
//		if (fs1.isOpened())
//		{
//			fs1["imageSize"] >> imageSize;
//			fs1["M"] >> cameraMatrix;
//			fs1["D"] >> distCoeffs;
//		}
//
//		fs1.release();
//	}
//
//	cout << "3.求解矫正映射" << endl;
//	initUndistortRectifyMap(
//		cameraMatrix, distCoeffs, Mat(),
//		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
//		CV_16SC2, map1, map2);
//	cout << endl;
//
//
//	cout << "4.矫正测试" << endl;
//	cout << "    -->按enter截图" << endl;
//
//	while (true)
//	{
//		capture_0.read(frame_0);///读取视频帧放入frame
//		imshow("camera_0", frame_0);
//
//		char c = waitKey(50); //等待时间50ms
//		if (c == 13) {//enter		
//			cout << "    视频截取结束!" << endl;
//			break;
//		}
//	}
//	capture_0.release();
//
//	cout << "    开始矫正!" << endl;
//	cv::Mat remapImage;
//	cv::remap(frame_0, remapImage, map1, map2, cv::INTER_LINEAR);
//	imshow("remapImage", remapImage);
//	waitKey();
//
//	return 0;
//}