/*
 * main.c
 *
 *  Created on: 27.01.2017
 *      Author: wadim mueller
 */

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <iostream>
#include <string.h>
#include "math.h"
#include <iomanip>
#include "debug.h"
#include <signal.h>
#include <unistd.h>
#include "decoder/mjpeg-decoder-sw.h"
#include "stream/v4l2-stream-stereo-device.h"
#include "include/filter/mf-sw.h"
#include "stereo-matcher/bm-sw.h"
#include "stereo-matcher/sgbm-sw.h"
#include "stereo-matcher/bm-hw-ip.h"
#include "filter/mf-sw.h"
#include "filter/mf-hw-ip.h"
#include "utils/cmdline-parser.h"
#include "helper.h"
#include "estimator.h"

static Estimator* distanceEstimator;

using namespace cv;
using namespace std;

struct hsv_object_ranges {
	String name;
	int h_low, h_high;
	int s_low, s_high;
	int v_low, v_high;
};

static struct hsv_object_ranges predefined_obj_colors[] = {
		{.name = "red", .h_low = 0, .h_high = 9, .s_low = 150, .s_high = 255, .v_low = 0, .v_high = 255},
		{.name = "blue", .h_low = 78, .h_high = 111, .s_low = 111, .s_high = 255, .v_low = 0, .v_high = 255},
		{.name = "green", .h_low = 61, .h_high = 92, .s_low = 100, .s_high = 255, .v_low = 0, .v_high = 255},
		{.name = "yellow", .h_low = 23, .h_high = 37, .s_low = 117, .s_high = 255, .v_low = 111, .v_high = 255},
		{.name = "orange", .h_low = 6, .h_high = 19, .s_low = 182, .s_high = 255, .v_low = 0, .v_high = 255},
};

static int get_rectified_remap_matrices(String intrinsics_file_name, String extrinsics_file_name, Size img_size,
		OutputArray left1, OutputArray left2, OutputArray right1, OutputArray right2, OutputArray Q, Rect* roi)
{
	Mat M1, D1, M2, D2;
	Mat R, T, R1, P1, R2, P2;
	Rect roi_left, roi_right;

	FileStorage intrinsics(intrinsics_file_name, FileStorage::READ);
	FileStorage extrinsics(extrinsics_file_name, FileStorage::READ);

	if (!intrinsics.isOpened() || !extrinsics.isOpened()) {
		printf("could not open intrinsics or extrinsics\n");
		return -1;
	}

	intrinsics["M1"] >> M1;
	intrinsics["D1"] >> D1;
	intrinsics["M2"] >> M2;
	intrinsics["D2"] >> D2;

	extrinsics["ROI1"] >> roi_left;
	extrinsics["ROI2"] >> roi_right;
	extrinsics["R"] >> R;
	extrinsics["T"] >> T;

	if (roi) {
		roi->x = max(roi_left.x, roi_right.x);
		roi->y = max(roi_left.y, roi_right.y);
		roi->width = min(roi_left.width, roi_right.width);
		roi->height = min(roi_left.height, roi_right.height);
	}

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi_left,
			&roi_right);

	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, left1, left2);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, right1, right2);
	return 0;
}

static void signal_handler(int signo)
{
	if (signo == SIGINT) {
		distanceEstimator->print_exec_time_stats();
		exit(1);
	}
}

int main(int argc, char** argv)
{
	Rect roif;
	Size imgSize;
	Mat remap_left1, remap_left2, remap_right1, remap_right2, reprojMat;
	EstimatorCmdLineParser parser(argv, argc);

	imgSize.width = parser.getWidth();
	imgSize.height = parser.getHeight();

	if (signal(SIGINT, signal_handler) == SIG_ERR) {
		cout << "can't catch SIGINT\n" << endl;
	}

	get_rectified_remap_matrices(parser.getIntrinsicsFileName(), parser.getExtrinsicsFileName(), imgSize, remap_left1, remap_left2, remap_right1,
			remap_right2, reprojMat, &roif);

	VideoStreamStereoDevice* videoDev = new V4LStreamStereoDevice(parser.getRightCameraDevice(), parser.getLeftCameraDevice(), imgSize.width , imgSize.height);
	DecoderDevice* mjpegDecoder = new MJPEGDecoderDevice;
	BlockMatcher* matcher = new SWMatcherKonolige(roif, roif, 31, 13, 0, 10, parser.getNumOfDisparities(),
			parser.getNumOfDisparities(), 10, 100, 32, 1);

#ifdef __ZYNQ__
	VideoFilterDevice* morphFilter = new HWMorphologicalFilterIPCore(roif.width, roif.height, 8);
#else
	VideoFilterDevice* morphFilter = new SWMorphologicalFilter(roif.width, roif.height, 8);
#endif

	distanceEstimator = new Estimator(matcher, morphFilter, videoDev, mjpegDecoder, &parser, remap_left1, remap_left2, remap_right1, remap_right2, reprojMat, roif);

	while (1) {
		distanceEstimator->run();
	}

	return 0;
}
