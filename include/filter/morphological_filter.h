/*
 * morphological_filter.h
 *
 *  Created on: 26.03.2017
 *      Author: wadim mueller
 */

#ifndef INCLUDE_MORPHOLOGICAL_FILTER_H_
#define INCLUDE_MORPHOLOGICAL_FILTER_H_

#include "opencv2/opencv.hpp"

#define MORPH_FILTER_DX		10
#define MORPH_FILTER_DY		10

using namespace cv;

void morphological_filter(Mat imgThresholded);

#endif /* INCLUDE_MORPHOLOGICAL_FILTER_H_ */
