/*
 * cmdline-parser.cpp
 *
 *  Created on: 20.05.2017
 *      Author: wadim.mueller
 */
#include "utils/cmdline-parser.h"


RtDepthMapCmdLineParser::RtDepthMapCmdLineParser(char** cmd_options,
		int num_options)
{
	const String opts = "{help| |Prints this}"
			"{h height|720|video vertical resolution}"
			"{w width|1280|video horizontal resolution}"
			"{a adjustable|0|allow adjust the detectable hsv colors through trackbars}"
			"{dp disparity-map|0|shows the disparity map}"
			;

	parser = new CommandLineParser(num_options, cmd_options, opts);
	height = parser->get<int>("h");
	width = parser->get<int>("w");
	adjustable = parser->get<bool>("a");
}
