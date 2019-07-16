#include <iostream>

#include "collect_pic.h"
#include "monocular_chessboard.h"
#include "monocular_circle.h"
#include "binocular_chessboard.h"
#include "binocular_circle.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int Test(void) {
	
	

	waitKey(50);
	return 0;
}

class Student {
public:
	
	Student(void) {
		cout << age << endl;
	}
	int age=0;
};

int main(void) {
	//需修改：存放路径、文件名、txt文件内容
	//MonocularCollectChessPic();
	//MonocularCollectCirclePic();
	//BinocularCollectChessPic();
	//BinocularCollectCirclePic();

	//需修改：boardSize、squareSize、图片txt文件路径、结果txt存放路径
	//MonocularChess();
	//MonocularCircle();
	//BinocularChess();
	//BinocularCircle();

	Test();

	system("pause");
	return 0;
}