/*
 * cmdline-parser.cpp
 *
 *  Created on: 20.05.2017
 *      Author: wadim.mueller
 */
#include "utils/cmdline-parser.h"

EstimatorCmdLineParser::EstimatorCmdLineParser(char** cmd_options,
		int num_options)
{
	const String opts =
			"{help| |Prints this}"
					"{h height|720|video vertical resolution}"
					"{w width|1280|video horizontal resolution}"
					"{ad adjustable|0|allow adjust the detectable hsv colors through trackbars}"
					"{dp disparity-map|1|shows the disparity map}"
					"{lc left-camera-device|/dev/video0|set the video device for the left camera}"
					"{rc right-camera-device|/dev/video1|set the video device for the right camera}"
					"{i intrinsics-file-name|backup/640x480/intrinsics.yml|file name for the camera intrinisics}"
					"{e extrinsics-file-name|backup/640x480/extrinsics.yml|file name for the camera extrinsics}"
					"{nd number-of-disparities|192|maximum number of disparities to calculate, increasing this value improves the quality but also increases the execution time}"
					"{mos minimal-object-size|100|set the minimal detectable object size in pixel}"
					"{cu calibration-unit|25.0|set the calibration unit in millimeter for the reprojection, if calibrated with a chessboard patter, this should be set to the edge length of one chess field}"
			;

	parser = new CommandLineParser(num_options, cmd_options, opts);
	height = parser->get<int>("h");
	width = parser->get<int>("w");
	leftCameraDevice = parser->get<String>("lc");
	rightCameraDevice = parser->get<String>("rc");
	intrinsicsFileName = parser->get<String>("i");
	extrinsicsFileName = parser->get<String>("e");
	numOfDisparities = parser->get<int>("nd");
	minimalObjectSize = parser->get<int>("mos");
	calibrationUnit = parser->get<double>("cu");
#ifdef __ZYNQ__
	disparityMap = false;
	adjustable = false;
#else
	disparityMap = (bool)parser->get<int>("dp");
	adjustable = (bool)parser->get<int>("ad");
#endif
}
