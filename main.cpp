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
#include "decoder/mjpeg-decoder-sw.h"
#include "stream/v4l2-stream-stereo-device.h"
#include "include/filter/mf-sw.h"
#include "stereo-matcher/bm-sw.h"
#include "stereo-matcher/sgbm-sw.h"
#include "stereo-matcher/bm-hw-ip.h"
#include "filter/mf-sw.h"
#include "filter/mf-hw-ip.h"
#include "utils/cmdline-parser.h"
#include "opencv2/ximgproc/disparity_filter.hpp"

//#define ENABLE_POST_FILTER
#define SHOW_DISPARITY_MAP
//#define SHOW_DISPARITY_VALUE
//#define ADJUSTABLE_HSV_VALUES

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

#define NUM_DISPARITIES		(32 * 6)
#define CALIB_UNIT_MM		25.0
#define MIN_DISP_VALS		100

#ifdef ENABLE_POST_FILTER
Ptr<DisparityWLSFilter> wls_filter;
#endif

static int iLowH = 0;
static int iHighH = 9;
static int iLowS = 150;
static int iHighS = 255;
static int iLowV = 0;
static int iHighV = 255;

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

static void set_label(cv::Mat& im, const std::string label, const cv::Point & origin)
{
    int fontface = FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    Size text = getTextSize(label, fontface, scale, thickness, &baseline);
    rectangle(im, origin + Point(0, baseline), origin + Point(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);
    putText(im, label, origin, fontface, scale, CV_RGB(255,255,255), thickness, 8);
}

static void calculate_depth(Mat xyz, Mat disparity_map, Mat mask, Mat img, vector<Rect>& regions)
{
	Mat xyz_region;
	Mat mask_region;
	ostringstream distance_text;
#ifdef SHOW_DISPARITY_VALUE
	Mat tmp_disp_map;
#endif
	for(int i = 0 ; i < regions.size(); ++i) {
		Rect reg = regions[i];

		if (reg.area() < MIN_DISP_VALS)
			continue;

		double res = 0.0;
		double disp_mean = 0.0;
		int cnt = 0;
		const double max_z = 1.0e4;

		xyz_region = xyz(reg);
		mask_region = mask(reg);
#ifdef SHOW_DISPARITY_VALUE
		tmp_disp_map = disparity_map(*reg);
#endif
		for (int y = 0; y < xyz_region.rows; y++) {
			for (int x = 0; x < xyz_region.cols; x++) {
				Vec3f point = xyz_region.at<Vec3f>(y, x);
				Scalar mask_point = mask_region.at<uchar>(Point(x,y));
#ifdef SHOW_DISPARITY_VALUE
				uint16_t disp = tmp_disp_map.at<uint16_t>(y, x);
#endif
				if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z || mask_point.val[0] == 0)
					continue;
				res += point[2];
#ifdef SHOW_DISPARITY_VALUE
				disp_mean += disp;
#endif
				cnt++;
			}
		}

		if (cnt > 0) {
			res = res / cnt;
#ifdef SHOW_DISPARITY_VALUE
			disp_mean = disp_mean / cnt;
#endif
			rectangle(img, Point(reg.x, reg.y), Point(reg.x + reg.width, reg.y + reg.height), Scalar(255, 255, 255),
					1, LINE_8);
			res	= (res * CALIB_UNIT_MM / 10.0);
			distance_text.str("");
			distance_text << std::fixed << setprecision(0) << res << " cm"
#ifdef SHOW_DISPARITY_VALUE
			<< " disparity = " << disp_mean
#endif
			;
			set_label(img, distance_text.str(), Point(reg.x, reg.y - 5));
		}

	}
}

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

static void create_adjustment_track_bars(void)
{
	createTrackbar("hue low", "depth", &iLowH, 255, NULL);
	createTrackbar("hue high", "depth", &iHighH, 255, NULL);

	createTrackbar("saturation low", "depth", &iLowS, 255, NULL);
	createTrackbar("saturation high", "depth", &iHighS, 255, NULL);

	createTrackbar("value low", "depth", &iLowV, 255, NULL);
	createTrackbar("value high", "depth", &iHighV, 255, NULL);
}

static int find_all_contour_boxes(vector<vector<Point>> &contours, vector<Vec4i>& hierarchy, vector<Rect>& all_bound_boxes, Rect& roi)
{
	int max_x = -1e6;
	int max_y = -1e6;
	int min_x = 1e6;
	int min_y = 1e6;

	for(int i = 0 ; i >= 0; i = hierarchy[i][0] ) {
		Rect reg = boundingRect(Mat(contours[i]));
		all_bound_boxes.push_back(reg);
		if (reg.x < min_x)
			min_x = reg.x;
		if (reg.y < min_y)
			min_y = reg.y;
		if (reg.x + reg.width > max_x)
			max_x = reg.x + reg.width;
		if (reg.y + reg.height > max_y)
			max_y = reg.y + reg.height ;
	}
	roi.x = min_x;
	roi.y = min_y;
	roi.width = max_x - min_x;
	roi.height = max_y - min_y;

	return 0;
}

int main(int argc, char** argv)
{
	Rect roif;
	Mat Q;
	char* rgb[2];
	double exec_time;
	Mat filter_in, filter_out;
	Size imgSize;
	Mat remap_left1, remap_left2, remap_right1, remap_right2;
	Mat left_rect, right_rect;
	Mat left_disp;
	int numberOfDisparities = NUM_DISPARITIES;
	Mat xyz;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat img_rectified;
	Mat raw_disp;

	RtDepthMapCmdLineParser parser(argv, argc);

	imgSize.width = parser.getWidth();
	imgSize.height = parser.getHeight();

	VideoStreamStereoDevice* videoDev = new V4LStreamStereoDevice("/dev/video1", "/dev/video0", imgSize.width, imgSize.height);
	DecoderDevice* mjpegDecoder = new MJPEGDecoderDevice;

#ifdef ENABLE_POST_FILTER
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
	wls_filter = createDisparityWLSFilter(left_matcher);
#endif

	get_rectified_remap_matrices("intrinsics.yml", "extrinsics.yml", imgSize, remap_left1, remap_left2, remap_right1,
			remap_right2, Q, &roif);

	SWMatcherKonolige* matcher = new SWMatcherKonolige(roif, roif, 31, 13, 0, 10, numberOfDisparities,
			numberOfDisparities, 10, 100, 32, 1);
#ifdef __ZYNQ__
	VideoFilterDevice* morphFilter = new HWMorphologicalFilterIPCore(roif.width, roif.height, 8);
#else
	VideoFilterDevice* morphFilter = new SWMorphologicalFilter(roif.width, roif.height, 8);
#endif

	/* SGBM Version*/
	//SWSemiGlobalMatcher* matcher = new SWSemiGlobalMatcher(9, 0, numberOfDisparities, 10, 100, 32, 1);
	/* Disparity Coprocessor Version */
	//HWMatcherDisparityCoprocessor* matcher = new HWMatcherDisparityCoprocessor("disp-coproc", IMG_WIDTH, IMG_HEIGHT);
	//setWindowProperty("left", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	Mat img[2];
	Mat left_gray, right_gray;
	Mat right_disp, filtered_disparity_map;

	namedWindow("depth", 1);
	namedWindow("debug", 1);

	if (parser.isAdjustable())
		create_adjustment_track_bars();

	for (int i = 0; i < 2; ++i) {
		rgb[i] = new char[videoDev->getHeight() * videoDev->getWidth() * 3];
		debug("%d: original width = %d ; height = %d\n", i, videoDev->getWidth(), videoDev->getHeight());
		img[i] = Mat(videoDev->getHeight(), videoDev->getWidth(), CV_8UC3, rgb[i]);
	}

	filter_in = Mat(roif.height, roif.width, CV_8UC1, morphFilter->getVideoInBuffer());
	filter_out = Mat(roif.height, roif.width, CV_8UC1, morphFilter->getVideoOutBuffer());

	debug("undistorted roi width = %d, height = %d\n", roif.width, roif.height);

	for (;;) {
		Mat imgHSV;
		Mat imgThresholded;
		Mat contInput;
		struct videoStreamBuffer videoBufferLeft, videoBufferRight;

		exec_time = (double) getTickCount();
		videoDev->grabOneFrame();
		videoDev->getBuffers(&videoBufferLeft, &videoBufferRight);

		mjpegDecoder->decode(videoBufferLeft.data, videoBufferLeft.len, videoDev->getWidth(), videoDev->getHeight(),
				rgb[0]);
		mjpegDecoder->decode(videoBufferRight.data, videoBufferRight.len, videoDev->getWidth(), videoDev->getHeight(),
				rgb[1]);

		cvtColor(img[0], left_gray, CV_RGB2GRAY);
		cvtColor(img[1], right_gray, CV_RGB2GRAY);

		remap(left_gray, left_rect, remap_left1, remap_left2, INTER_LINEAR);
		left_rect = left_rect(roif);

		remap(right_gray, right_rect, remap_right1, remap_right2, INTER_LINEAR);
		right_rect = right_rect(roif);

		remap(img[0], img_rectified, remap_left1, remap_left2, INTER_LINEAR);
		img_rectified = img_rectified(roif);

		cvtColor(img_rectified, img_rectified, COLOR_RGB2BGR);
		cvtColor(img_rectified, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), filter_in); //Threshold the image
		morphFilter->run(filter_in, filter_out);
		filter_out.copyTo(contInput);
		findContours(contInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		if (contours.size() > 0) {
			Rect stereo_calc_roi;

			vector<Rect> obj_boundings;
			find_all_contour_boxes(contours, hierarchy, obj_boundings, stereo_calc_roi);
			//matcher->setROI1(stereo_calc_roi);
			/*FIXME: set ROI2 too*/
			matcher->compute(left_rect, right_rect, left_disp);
#ifdef ENABLE_POST_FILTER
			right_matcher->compute(right_rect,left_rect, right_disp);

			wls_filter->setLambda(8000.0);
			wls_filter->setSigmaColor(1.5);

			wls_filter->filter(left_disp, left_rect, filtered_disparity_map, right_disp);
			getDisparityVis(left_disp,raw_disp);

			Mat filtered_disp_vis;
			getDisparityVis(filtered_disparity_map,filtered_disp_vis);

			imshow("disp", filtered_disp_vis);
#endif
#ifdef SHOW_DISPARITY_MAP
			getDisparityVis(left_disp, raw_disp);
			imshow("disparity", raw_disp);
#endif
			left_disp /= 16.;
			reprojectImageTo3D(left_disp, xyz, Q, true, CV_32F);
			calculate_depth(xyz, left_disp, filter_out, img_rectified, obj_boundings);
		}
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Overall Time: %lf s\n", exec_time);
		imshow("debug", filter_out);
		imshow("depth", img_rectified);
		waitKey(10);
	}

	return 0;
}
