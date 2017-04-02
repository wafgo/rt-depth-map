/*
 * morphological_filter.cpp
 *
 *  Created on: 26.03.2017
 *      Author: sefo
 */

#include "filter/morphological_filter.h"

void morphological_filter(Mat imgThresholded)
{
		erode(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));
		dilate(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));
		erode(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(MORPH_FILTER_DX, MORPH_FILTER_DY)));

}
