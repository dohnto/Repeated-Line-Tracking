#include "repeatedlinetracking.h"
#include <opencv2/highgui/highgui.hpp>

int main()
{
	cv::Mat finger = cv::imread("finger.png", CV_LOAD_IMAGE_GRAYSCALE);

	cv::Mat mask = cv::Mat::ones(finger.size(), CV_8U);         // Locus space

	cv::Mat result;
	RepeatedLineTracking(finger, result, mask, 200000, 1, 29);

	cv::imwrite("result.png", result);
}
