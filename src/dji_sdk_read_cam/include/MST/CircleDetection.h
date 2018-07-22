#ifndef GA_H
#define GA_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <time.h>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//#define SHOW_DETECTION_RESULT

#define MIN_RADIUS 4
#define MAX_RADIUS 18 

#define R_G_R_RED_CIRCLE 0.45
#define R_B_R_RED_CIRCLE 0.60

#define G_B_G_YELLOW_CIRCLE 0.2
#define R_B_R_YELLOW_CIRCLE 0.2

#define B_R_B_BLUE_CIRCLE 0.45
#define B_G_B_BLUE_CIRCLE 0.5

#define MAX_CANNY 100
#define MIN_CANNY 13

#define MIN_LINE 180
#define MAX_LINE 180

using namespace std;
using namespace cv;

double circleDistance(Vec3i A, Vec3i B);
void paintCircles(Mat img);
pair<vector<vector<double>>, vector<Vec3f>> circleDetection(Mat img);
pair<double,double> slopeAndIntercept(vector<Vec3f> circlesTemp, int A, int B, int max_line, int min_line);
pair<int,int> Y_Point_Max_Min(vector<Vec3f> circlesTemp);

#endif
