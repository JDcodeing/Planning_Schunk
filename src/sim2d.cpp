#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <functional>
#include <random>
using namespace std ;

using namespace cv;
Mat img ;
//g++ sim2d.cpp $(pkg-config --libs --cflags opencv) -o sim2d
void construct(Mat& img)
{
	img = cv::Mat::zeros(300,300, CV_64FC1);
	img.setTo(cv::Scalar(255,255,255));
	vector<Point2f> centers;
	vector<double> radius;
	centers.push_back(Point2f(50.0,50.0));
	radius.push_back(15);
	centers.push_back(Point2f(150.0,80.0));
	radius.push_back(25);
	centers.push_back(Point2f(200.0,150.0));
	radius.push_back(30);
	centers.push_back(Point2f(93.0,209.0));
	radius.push_back(27);


	for(size_t i = 0; i < centers.size(); i++)
	{
		circle(img, centers[i], radius[i], Scalar(0,0,0), CV_FILLED, 8);	
	}
	
	cv::imwrite("./image.png", img);
}

//pids.insert(make_pair(pid,dis));
		//right
		

int main()
{
	construct(img);
	return 0;
}