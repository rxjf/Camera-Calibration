#include <windows.h> //写入时间，不能写在.h中

#include "monocular_circle.h"

static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2, cv::NORM_L2));

		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
}

int MonocularCircle(void)
{
	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;

	string txt_name = "monocular_circle_pic/calibdata.txt";
	ifstream fin(txt_name);
	string filename;
	Size boardSize = Size(4, 11);//
	float squareSize = 2;
	vector<Point2f> pointBuf;
	bool found = false;
	Mat view;

	while (getline(fin, filename))
	{
		view = imread(filename);
		//imshow("Camera Calibration", view);
		//waitKey(500);
		found = findCirclesGrid(view, boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID); //非对称
		imagePoints.push_back(pointBuf);

		if (found)
		{
			//Draw the corners.
			//drawChessboardCorners(view, boardSize, pointBuf, found);
			//imshow("Camera Calibration", view);//显示图片
			//waitKey(500);//暂停0.5S
		}
		else {
			cout << filename << "提取圆心点失败" << endl;
			return -1;
		}
	}

	imageSize = view.size();

	if (imagePoints.size() > 0) {
		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;

		cameraMatrix = Mat::eye(3, 3, CV_64F);//matlab风格初始化
		distCoeffs = Mat::zeros(8, 1, CV_64F);

		vector<vector<Point3f> > objectPoints(1);

		calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);

		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		//Find intrinsic and extrinsic camera parameters
		double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs, 0);

		cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
			rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
		cout << (ok ? "Calibration succeeded" : "Calibration failed")
			<< ". avg re projection error = " << totalAvgErr<<endl;
		
		ifstream fin3(txt_name);
		int i = 0;
		while (getline(fin3, filename))
		{
			cout << filename << " reprojErrs:  " << reprojErrs[i] << endl;
			i++;
		}
	}

	ofstream fout;
	fout.open("monocular_circle_pic/caliberation_result.txt", ios::app);

	SYSTEMTIME sys;
	GetLocalTime(&sys);
	fout << sys.wYear << "-" << sys.wMonth << "-" << sys.wDay << " "<<sys.wHour<<":"<<sys.wMinute<<":"<<sys.wSecond << endl<<endl;
	fout << "相机内参数矩阵：" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "畸变系数："<<endl;
	fout << distCoeffs << endl << endl;
	fout << "--------------------------------------------------" << endl<<endl;
	fout.close();

	// -----------------------Show the undistorted image for the image list ------------------------	
	//Mat rview, map1, map2;
	//initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
	//	getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
	//	imageSize, CV_16SC2, map1, map2);

	//ifstream fin2(txt_name);
	//while (getline(fin2, filename))
	//{
	//	view = imread(filename);
	//	remap(view, rview, map1, map2, INTER_LINEAR);
	//	imshow("Image View", rview);
	//	char c = (char)waitKey();
	//	if (c == 27 || c == 'q' || c == 'Q')
	//		break;
	//}

	return 0;
}

