/*
 * bm.h
 *
 *  Created on: 02.04.2017
 *      Author: wadim mueller
 */

#ifndef INCLUDE_BM_H_
#define INCLUDE_BM_H_

#include "opencv2/opencv.hpp"

class BlockMatcher {
public:
	virtual int compute(cv::InputArray left, cv::InputArray right, cv::OutputArray out) = 0;
protected:
};


#endif /* INCLUDE_BM_H_ */
