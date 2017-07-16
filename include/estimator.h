/*
 * estimator.h
 *
 *  Created on: 25.05.2017
 *      Author: sefo
 */

#ifndef INCLUDE_ESTIMATOR_H_
#define INCLUDE_ESTIMATOR_H_

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
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "helper.h"
#include "filter/filter.h"
#include "stereo-matcher/stereo-matcher.h"
#include "utils/cmdline-parser.h"

//#define ENABLE_POST_FILTER
//#define SHOW_DISPARITY_VALUE
#define ENABLE_EXECUTION_TIME_MEASUREMENT

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

struct exec_time_struct {
	String func_name;
	double tv;
	int num;
};

#ifdef ENABLE_EXECUTION_TIME_MEASUREMENT

#define MEASURE_EXECUTION_TIME_START \
do { \
		exec_time_func_idx = 0; \
} while(0);

#define MEASURE_EXECUTION_TIME(_fn_) \
do { \
	static double acc = 0; \
	static bool first_time = true; \
	if (first_time) { \
		String full_name = #_fn_; \
		size_t pos = full_name.find("("); \
		exec_times_tab[exec_time_func_idx].func_name = full_name.substr(0, pos); \
		first_time = false; \
	} \
	double __et = (double) getTickCount(); \
	_fn_; \
	__et = ((double) getTickCount() - __et) / getTickFrequency(); \
	acc += __et; \
	exec_times_tab[exec_time_func_idx].num++; \
	exec_times_tab[exec_time_func_idx].tv = acc / exec_times_tab[exec_time_func_idx].num; \
	exec_time_func_idx++; \
} while(0);

#define MEASURE_EXECUTION_TIME_END \
do { \
} while(0);

#else
	#define MEASURE_EXECUTION_TIME_START
	#define MEASURE_EXECUTION_TIME(_fn_) _fn_;
	#define MEASURE_EXECUTION_TIME_END
#endif

class Estimator {
public:
	void run();
	void print_exec_time_stats(void);
	Estimator(BlockMatcher* matcher, VideoFilterDevice* filter, VideoStreamStereoDevice* streamer, DecoderDevice* decoder, EstimatorCmdLineParser* cmdLineParser, Mat remap_left1, Mat remap_left2, Mat remap_right1,
			Mat remap_right2, Mat Q, Rect &relevantRoi);
	~Estimator();
private:
	int find_relevant_matching_region(vector<Rect>& bounds, Rect& roi);
	void fill_bounding_rects_of_contours(vector<vector<Point>> &contours, vector<Vec4i>& hierarchy, vector<Rect>& bounds, int minSize);
	void set_label(Mat& im, const string label, const Point & origin);
	void calc_depth(Mat xyz, Mat disparity_map, Mat mask, Mat img, vector<Rect>& regions, double calibrationUnit);
	void create_adjustment_track_bars(void);
	VideoFilterDevice* morphFilter;
	BlockMatcher* bm;
	VideoStreamStereoDevice* videoDevice;
	DecoderDevice* mjpegDecoder;
	Rect roif;
	Mat Q, img_rectified, raw_disp, left_disp, left_rect, right_rect, remap_left1, remap_left2, remap_right1, remap_right2, filter_in, filter_out;
	Mat img[2], left_gray, right_gray, right_disp, filtered_disparity_map;
	char* rgb[2];
	double calibration_unit, exec_time;
	Size imgSize;
	int numberOfDisparities, minObjSize;
	bool showDisparityMap;
	Mat xyz;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
	EstimatorCmdLineParser* parser;
	int exec_time_func_idx;
	struct exec_time_struct* exec_times_tab;
	const int num_exec_times_tab = 100;
#ifdef ENABLE_POST_FILTER
	Ptr<StereoMatcher> right_matcher
#endif
};



#endif /* INCLUDE_ESTIMATOR_H_ */
