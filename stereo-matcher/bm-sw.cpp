/*
 * bm_sw.cpp
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#include "stereo-matcher/bm-sw.h"

using namespace cv;

SWMatcherKonolige::SWMatcherKonolige(cv::Rect& roi1, cv::Rect& roi2, int preFilterCap, int blockSize, int minDisparity,
		int textureThreshold, int numOfDisparities, int maxDisparity, int uniquenessRatio, int speckleWindowSize,
		int speckleRange, int disp12MaxDiff)
{
	matcher = cv::StereoBM::create(numOfDisparities, blockSize);

	matcher->setROI1(roi1);
	matcher->setROI2(roi2);
	matcher->setPreFilterCap(preFilterCap);
	matcher->setMinDisparity(minDisparity);
	matcher->setNumDisparities(numOfDisparities);
	matcher->setTextureThreshold(textureThreshold);
	matcher->setUniquenessRatio(uniquenessRatio);
	matcher->setSpeckleWindowSize(speckleWindowSize);
	matcher->setSpeckleRange(speckleRange);
	matcher->setDisp12MaxDiff(disp12MaxDiff);
}

SWMatcherKonolige::~SWMatcherKonolige()
{

}

int SWMatcherKonolige::compute(InputArray left, InputArray right, OutputArray out)
{
	matcher->compute(left, right, out);

	return 0;
}
