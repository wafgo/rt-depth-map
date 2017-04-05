/*
 * morpho	logical_filter.cpp
 *
 *  Created on: 26.03.2017
 *      Author: sefo
 */

#include "filter/mf-sw.h"

SWMorphologicalFilter::SWMorphologicalFilter(int w, int h, int bpp)
{
	if (!video_in)
		video_in = new char[w * h * (bpp >> 3)];
	if (!video_out)
		video_out = new char[w * h * (bpp >> 3)];
	img_width = w;
	img_height = h;
	img_bpp = bpp;
}

int SWMorphologicalFilter::run(cv::InputArray in, cv::OutputArray out)
{
	/* morphological opening */
	cv::erode(in, out, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));
	cv::dilate(out, out, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));

	//morphological closing (fill small holes in the foreground)
	cv::dilate(out, out, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));
	cv::erode(out, out, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));
}
