#include <iostream>
#include "repeatedlinetracking.h"

// DELETE
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/types_c.h>
#include <opencv2/core/operations.hpp>
#include <algorithm>    // std::random_shuffle

void RepeatedLineTracking(cv::InputArray _src, cv::OutputArray _dst, cv::InputArray _mask, unsigned iterations, unsigned r, unsigned W) {
	cv::Mat src = _src.getMat();

	src.convertTo(src, CV_64F, 1.0 / 255.0);
	cv::Mat mask = _mask.getMat();

	double p_lr = 0.5;                                                                                      // Probability of goin left or right
	double p_ud = 0.25;                                                                                     // Probability of going up or down

	cv::Mat Tr = cv::Mat::zeros(_src.size(), CV_8U);                                                        // Locus space

	cv::Mat bla = (cv::Mat_<char>(9, 2) <<
				   -1, -1,
				   -1, 0,
				   -1, 1,
				   0, -1,
				   0, 0,
				   0, 1,
				   1, -1,
				   1, 0,
				   1, 1);

	// check if W is even
	if (W % 2 == 0) {
		std::cerr << "FAIL: RepeatedLineTracking - W cannot be even" << std::endl;
		throw new std::exception;
	}

	int ro = cvRound(r * cv::sqrt(2) / 2);                                                         // r for oblique directions
	int hW = (W - 1) / 2;                                                                          // half width for horz. and vert. directions

	int hWo = cvRound(hW * cv::sqrt(2) / 2);                                                       // half width for oblique directions

	// Omit unreachable borders
	for (int x = 0; x < src.size().width; x++) {
		for (int y = 0; y <= r + hW; y++) {
			mask.at<uchar>(y, x) = 0;                                                              // top
			mask.at<uchar>(src.size().height - y - 1, x) = 0;                                      // bottom
		}
	}

	for (int y = 0; y < src.size().height; y++) {
		for (int x = 0; x <= r + hW; x++) {
			mask.at<uchar>(y, x) = 0;                                                              // left
			mask.at<uchar>(y, src.size().width - x - 1) = 0;                                       // right
		}
	}


	cv::RNG rng;

	// Uniformly distributed starting points
	std::vector<cv::Point> indices;
	for (int i = 0; i < iterations;) {
		int xRandom = rng.uniform(0, mask.cols);
		int yRandom = rng.uniform(0, mask.rows);
		cv::Point p(xRandom,yRandom);
		if (mask.at<uchar>(p)) {
			indices.push_back(p);
			i++;
		}
	}


	// Iterate through all starting points
	for (std::vector<cv::Point>::iterator startingPoint = indices.begin(); startingPoint != indices.end(); startingPoint++) {
		std::cout << *startingPoint << std::endl;
		int xc = startingPoint->x;                                                 //  Current tracking point, x
		int yc = startingPoint->y;                                                 //  Current tracking point, y

		// Determine the moving-direction attributes
		// Going left or right ?
		int Dlr = 1;                                                               // going right
		if (rng.uniform(0, 2)) {                                                           // returns 0 or 1
			Dlr = -1;                                                              // going left
		}

		int Dud = 1;                                                               // going down
//		if (1) {
		if (rng.uniform(0, 2)) {
			Dud = -1;                                                              // going up
		}

		// Initialize locus-positition table T
		cv::Mat Tc = cv::Mat::zeros(src.size(), CV_8U);


		double Vl = 1;
		while (Vl > 0) {
			// Determine the moving candidate point set Nc
			cv::Mat Nr = cv::Mat::zeros(cv::Size(3, 3), CV_8U);

			double random = rng.uniform(0, 101) / 100.0;
			if (random < p_lr) {
				// Going left or right
				Nr.at<uchar>(cv::Point(1 + Dlr, 0)) = 1;
				Nr.at<uchar>(cv::Point(1 + Dlr, 1)) = 1;
				Nr.at<uchar>(cv::Point(1 + Dlr, 2)) = 1;
			} else if (random > p_lr && random < (p_lr + p_ud)) {
				// Going up or down
				Nr.at<uchar>(cv::Point(0, 1 + Dud)) = 1;
				Nr.at<uchar>(cv::Point(1, 1 + Dud)) = 1;
				Nr.at<uchar>(cv::Point(2, 1 + Dud)) = 1;
			} else {
				// Going any direction
				Nr = cv::Mat::ones(cv::Size(3, 3), CV_8U);
				Nr.at<uchar>(cv::Point(1, 1)) = 0;
			}

			std::vector<cv::Point> Nc;
			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					int x = xc + dx;
					int y = yc + dy;
					if ((!Tc.at<uchar>(cv::Point(x, y))) && Nr.at<uchar>(cv::Point(dx + 1, dy + 1)) && mask.at<uchar>(cv::Point(x, y))) {
						int tmp = (dx + 1) * 3 + (dy + 1);
						Nc.push_back(cv::Point(xc + bla.at<char>(cv::Point(0, tmp)), yc + bla.at<char>(cv::Point(1, tmp))));
					}
				}
			}


			if (Nc.size() == 0) {
				Vl = -1;                                                              // TODO break?
				continue;
			}

			// Detect dark line direction near current tracking point
			std::vector<double> Vdepths(Nc.size());             // Valley depths

			for (int it = 0; it < Nc.size(); it++) {
				cv::Point Ncp = Nc[it];

				// Horizontal or vertical
				if (Ncp.y == yc) {
					// Horizontal plane
					int xp;
					int yp = Ncp.y;

					if (Ncp.x > xc) {
						// Right direction
						xp = Ncp.x + r;
					} else {
						// Left direction
						xp = Ncp.x - r;
					}

					Vdepths[it] =
						src.at<double>(cv::Point(xp, yp + hW))
						- 2 * src.at<double>(cv::Point(xp, yp))
						+ src.at<double>(cv::Point(xp, yp - hW))
						;

				} else if (Ncp.x == xc) {
					// Vertical plane
					int xp = Ncp.x;
					int yp;

					if (Ncp.y > yc) {
						// Down direction
						yp = Ncp.y + r;
					} else {
						// Up direction
						yp = Ncp.y - r;
					}

					Vdepths[it] =
						src.at<double>(cv::Point(xp + hW, yp))
						- 2 * src.at<double>(cv::Point(xp, yp))
						+ src.at<double>(cv::Point(xp - hW, yp))
						;
				}

				// Oblique directions
				else if ((Ncp.x > xc && Ncp.y < yc) || (Ncp.x < xc && Ncp.y > yc)) {
					// Diagonal, up /
					int xp;
					int yp;

					if (Ncp.x > xc && Ncp.y < yc) {
						// Top right
						xp = Ncp.x + ro;
						yp = Ncp.y - ro;
					} else {
						// Bottom left
						xp = Ncp.x - ro;
						yp = Ncp.y + ro;
					}

					Vdepths[it] =
						src.at<double>(cv::Point(xp - hWo, yp - hWo))
						- 2 * src.at<double>(cv::Point(xp, yp))
						+ src.at<double>(cv::Point(xp + hWo, yp + hWo))
						;
				} else {
					// Diagonal, down \.
					int xp;
					int yp;

					if (Ncp.x < xc && Ncp.y < yc) {
						// Top left
						xp = Ncp.x - ro;
						yp = Ncp.y - ro;
					} else {
						// Bottom right
						xp = Ncp.x + ro;
						yp = Ncp.y + ro;
					}

					Vdepths[it] =
						src.at<double>(cv::Point(xp - hWo, yp + hWo))
						- 2 * src.at<double>(cv::Point(xp, yp))
						+ src.at<double>(cv::Point(xp + hWo, yp - hWo))
						;

				}
			}             // End search of candidates


			Tc.at<uchar>(cv::Point(xc, yc)) = true;
			Tr.at<uchar>(cv::Point(xc, yc))++;

			int index = std::distance(Vdepths.begin(), std::max_element(Vdepths.begin(), Vdepths.end()));

			xc = Nc[index].x;
			yc = Nc[index].y;
			Vl = Vdepths[index];
		}
	}

	_dst.create(src.rows, src.cols, CV_8U);
	cv::Mat dst = _dst.getMat();
	Tr.copyTo(dst);
}
