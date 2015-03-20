#ifndef REPEATED_LINE_TRACKING_H
#define REPEATED_LINE_TRACKING_H

#include <opencv2/imgproc/imgproc.hpp>

void RepeatedLineTracking(cv::InputArray src, cv::OutputArray dst, cv::InputArray mask, unsigned iterations, unsigned r, unsigned W);

#endif // REPEATED_LINE_TRACKING_H
