/*
 * cmdline-parser.h
 *
 *  Created on: 20.05.2017
 *      Author: wadim.mueller
 */

#ifndef INCLUDE_CMDLINE_PARSER_H_
#define INCLUDE_CMDLINE_PARSER_H_

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

using namespace cv;

class RtDepthMapCmdLineParser {
public:
	RtDepthMapCmdLineParser(char** cmd_options, int num_options);

	int getHeight() const
	{
		return height;
	}

	int getWidth() const
	{
		return width;
	}

	bool isAdjustable() const
	{
		return adjustable;
	}

private:
	CommandLineParser* parser;
	int height, width;
	bool adjustable;
};



#endif /* INCLUDE_CMDLINE_PARSER_H_ */
