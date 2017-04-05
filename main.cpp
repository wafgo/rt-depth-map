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
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string.h>
#include "math.h"
#include <iomanip>
#include "debug.h"
#include "decoder/mjpeg-decoder-sw.h"
#include "stream/v4l2-stream-stereo-device.h"
#include "decoder/mjpeg-decoder-sw.h"
#include "include/filter/mf-sw.h"
#include "stereo-matcher/bm-sw.h"
#include "stereo-matcher/sgbm-sw.h"
#include "stereo-matcher/bm-hw-ip.h"
#include "filter/mf-sw.h"
#include "filter/mf-hw-ip.h"

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

//#define ENABLE_POST_FILTER
#define NUM_DISPARITIES		(64 * 3)
#define IMG_WIDTH			1280
#define IMG_HEIGHT			720

#define CALIB_UNIT_MM		25.0

#define MORPH_FILTER_DX		10
#define MORPH_FILTER_DY		10

#ifdef ENABLE_POST_FILTER
Ptr<DisparityWLSFilter> wls_filter;
#endif

static int iLowH = 0;
static int iHighH = 9;
static int iLowS = 69;
static int iHighS = 255;
static int iLowV = 0;
static int iHighV = 255;

#define MIN_DISP_VALS		100

static float calculate_depth(Mat xyz, Mat img, vector<Rect>& region)
{
	Mat tmp_xyz;

	for (int i = 0; i < region.size(); ++i) {
		Rect* reg = &region[i];

		if (reg->area() < MIN_DISP_VALS)
			continue;

		double res = 0.0;
		int cnt = 0;
		const double max_z = 1.0e4;
		ostringstream distance_text;

		rectangle(img, Point(reg->x, reg->y), Point(reg->x + reg->width, reg->y + reg->height), Scalar(255, 255, 255),
				1, LINE_8);

		tmp_xyz = xyz(*reg);

		for (int y = 0; y < tmp_xyz.rows; y++) {
			for (int x = 0; x < tmp_xyz.cols; x++) {
				Vec3f point = tmp_xyz.at<Vec3f>(y, x);
				if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
					continue;
				res += point[2];
				cnt++;
			}
		}

		if (cnt > 0)
			res = res / cnt;

		distance_text << std::fixed << setprecision(1) << res * CALIB_UNIT_MM / 10.0 << " cm";
		putText(img, String(distance_text.str().c_str()), Point(reg->x, reg->y - 5), FONT_HERSHEY_SIMPLEX, 0.5,
				Scalar(255, 255, 255), 1, LINE_8);
	}
	return 0.0f;
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

int main(int, char**)
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

	VideoStreamStereoDevice* videoDev = new V4LStreamStereoDevice("/dev/video1", "/dev/video0", IMG_WIDTH, IMG_HEIGHT);
	DecoderDevice* mjpegDecoder = new MJPEGDecoderDevice;

#ifdef ENABLE_POST_FILTER
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
#endif
#ifdef ENABLE_POST_FILTER
	wls_filter = createDisparityWLSFilter(left_matcher);
#endif

	imgSize.width = IMG_WIDTH;
	imgSize.height = IMG_HEIGHT;

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

	for (int i = 0; i < 2; ++i) {
		rgb[i] = new char[videoDev->getHeight() * videoDev->getWidth() * 3];
		debug("%d: original width = %d ; height = %d\n", i, videoDev->getWidth(), videoDev->getHeight());
		img[i] = Mat(videoDev->getHeight(), videoDev->getWidth(), CV_8UC3, rgb[i]);
	}

	filter_in = Mat(roif.height, roif.width, CV_8UC1, morphFilter->getVideoInBuffer());
	filter_out = Mat(roif.height, roif.width, CV_8UC1, morphFilter->getVideoOutBuffer());

	debug("after roi width = %d, height = %d\n", roif.width, roif.height);

	for (;;) {
		Mat imgHSV;
		Mat imgThresholded;
		vector<Rect> b_roi;
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
		findContours(filter_out, contours, hierarchy, CV_RETR_FLOODFILL, CV_LINK_RUNS, Point(0, 0));

		for (int i = 0; i < contours.size(); i++) {
			Rect br = boundingRect(Mat(contours[i]));
			Mat mask;
			drawContours(mask, contours, i, Scalar(255));
			Scalar mv = mean(filter_out(br), mask);
			if (mv.val[0] >= 120)
				b_roi.push_back(br);
		}

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

		left_disp /= 16.;

		reprojectImageTo3D(left_disp, xyz, Q, true, CV_32F);
		calculate_depth(xyz, img_rectified, b_roi);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Overall Time: %lf s\n", exec_time);
		imshow("left", img_rectified);
		waitKey(10);
	}

	return 0;
}
