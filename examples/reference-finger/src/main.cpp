#include "repeatedlinetracking.h"
#include <opencv2/highgui/highgui.hpp>

int main()
{
	cv::Mat finger = cv::imread("finger.png", CV_LOAD_IMAGE_GRAYSCALE);

	cv::resize(finger, finger, cv::Size(), 0.5, 0.5);
	cv::Mat mask = cv::Mat::ones(finger.size(), CV_8U);         // Locus space

	cv::imshow("orig", finger);
	cv::Mat result, result2;
	RepeatedLineTracking(finger, result2, mask, 9000, 1, 19);

	cv::imshow("pes2", result2);
	cv::imwrite("orig2.png", result2);
	cv::waitKey(0);
}
