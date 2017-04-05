/*
 * sgbm_sw.cpp
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#include "stereo-matcher/sgbm-sw.h"

using namespace cv;

SWSemiGlobalMatcher::SWSemiGlobalMatcher(int blockSize, int minDisparity, int numOfDisparities, int uniquenessRatio,
		int speckleWindowSize, int speckleRange, int disp12MaxDiff)
{
	matcher = StereoSGBM::create(0, numOfDisparities, blockSize);

	matcher->setP1(8 * 3 * 5 * 5);
	matcher->setP2(32 * 3 * 5 * 5);
	matcher->setMinDisparity(minDisparity);
	matcher->setNumDisparities(numOfDisparities);
	matcher->setUniquenessRatio(uniquenessRatio);
	matcher->setSpeckleWindowSize(speckleWindowSize);
	matcher->setSpeckleRange(speckleRange);
	matcher->setDisp12MaxDiff(disp12MaxDiff);
}

SWSemiGlobalMatcher::~SWSemiGlobalMatcher()
{

}

int SWSemiGlobalMatcher::compute(InputArray left, InputArray right, OutputArray out)
{
	matcher->compute(left, right, out);

	return 0;
}
