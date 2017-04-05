/*
 * morphological_filter.h
 *
 *  Created on: 26.03.2017
 *      Author: wadim mueller
 */

#ifndef INCLUDE_MORPHOLOGICAL_FILTER_H_
#define INCLUDE_MORPHOLOGICAL_FILTER_H_

#define MORPH_FILTER_DX		10
#define MORPH_FILTER_DY		10

#include "filter/filter.h"
#include "opencv2/opencv.hpp"

class SWMorphologicalFilter: public VideoFilterDevice
{
public:
	explicit SWMorphologicalFilter(int w, int h, int bpp);
    int run(cv::InputArray in, cv::OutputArray out);
};

#endif /* INCLUDE_MORPHOLOGICAL_FILTER_H_ */
