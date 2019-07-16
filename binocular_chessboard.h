#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <vector>
//#include <string>
//#include <algorithm>
#include <iostream>
#include <iterator>
//#include <stdio.h>
//#include <stdlib.h>
//#include <ctype.h>

using namespace std;
using namespace cv;

static void onMouse(int event, int x, int y, int, void*);

static bool readStringList(const string& filename, vector<string>& l);

int BinocularChess(void);