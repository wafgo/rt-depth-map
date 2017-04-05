/*
 * sgbm-sw.h
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#ifndef INCLUDE_BM_SGBM_SW_H_
#define INCLUDE_BM_SGBM_SW_H_

#include "stereo-matcher/stereo-matcher.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string.h>
#include "math.h"
#include "debug.h"

using namespace cv;

class SWSemiGlobalMatcher: public BlockMatcher
{
public:
	SWSemiGlobalMatcher(int blockSize, int minDisparity, int numOfDisparities, int uniquenessRatio,
			int speckleWindowSize, int speckleRange, int disp12MaxDiff);
	~SWSemiGlobalMatcher();
	int compute(InputArray left, InputArray right, OutputArray out);
private:
	Ptr<StereoSGBM> matcher;
};

#endif /* INCLUDE_BM_SGBM_SW_H_ */
