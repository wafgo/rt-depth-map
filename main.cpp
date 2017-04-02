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
#include <unistd.h>
#include <linux/videodev2.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "math.h"
#include "debug.h"
#include "decoder/mjpeg-decoder-sw.h"
#include "filter/morphological_filter.h"
#include "stream/v4l2-stream-stereo-device.h"
#include "decoder/mjpeg-decoder-sw.h"

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

#define NUM_DISPARITIES		(64*4)
#define IMG_WIDTH			1280
#define IMG_HEIGHT			720

#define CALIB_UNIT_MM	25.0

//#define ENABLE_POST_FILTER

#define MORPH_FILTER_DX		10
#define MORPH_FILTER_DY		10

#define BM			0
#define SGBM		1

#define BM_TYPE		BM

Rect mouse_selected;

#ifdef ENABLE_POST_FILTER
Ptr<DisparityWLSFilter> wls_filter;
#endif

static char* converted[2];
static int iLowH = 0;
static int iHighH = 9;
static int iLowS = 69;
static int iHighS = 255;
static int iLowV = 0;
static int iHighV = 255;


#define MIN_DISP_VALS		100

static float calculate_depth(Mat xyz, Mat img, vector<Rect>& region, vector<Point2f> center_of_object)
{
	Mat tmp_xyz;
	region.push_back(mouse_selected);

	for (int i = 0; i < region.size(); ++i) {
		Rect* reg = &region[i];

		if (reg->area() < MIN_DISP_VALS)
			continue;

		double aspect_ratio = (double) reg->width / (double) reg->height;
		Point2f* center = &center_of_object[i];
		Rect disparity_calculation_region;
		double res = 0.0;
		double x, y, w, h;
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
#if 0
		/* note the aspect ratio */
		h = max<double>(1.0, reg->height / 10.0);
		w = aspect_ratio * h;

		while (cnt < MIN_DISP_VALS) {
			if (w >= reg->width || h >= reg->height)
				break;

			x = (double) center->x - (w / 2.0);
			y = (double) center->y - (h / 2.0);
			disparity_calculation_region.width = (int) w;
			disparity_calculation_region.height = (int) h;
			disparity_calculation_region.x = (int) x;
			disparity_calculation_region.y = (int) y;

			try {
				tmp_xyz = xyz(disparity_calculation_region);
			} catch (const exception& e) {
				cout << "exeception caught" << e.what() << endl;
				break;
			}
			for (int y = 0; y < tmp_xyz.rows; y++) {
				for (int x = 0; x < tmp_xyz.cols; x++) {
					Vec3f point = tmp_xyz.at<Vec3f>(y, x);
					if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
						continue;
					res += point[2];
					cnt++;
				}
			}
			if (cnt < MIN_DISP_VALS) {
				h = h + 1.0;
				w += aspect_ratio;
			}

		}
		if (cnt >= MIN_DISP_VALS) {
#ifdef DEBUG
			rectangle(img, Point(disparity_calculation_region.x, disparity_calculation_region.y),
					Point(disparity_calculation_region.x + disparity_calculation_region.width,
							disparity_calculation_region.y + disparity_calculation_region.height),
					Scalar(255, 255, 255), 1, LINE_8);
#endif
#endif
			if (cnt > 0)
				res = res / cnt;

			distance_text << std::fixed << setprecision(1) << res * CALIB_UNIT_MM / 10.0 << " cm";
			putText(img, String(distance_text.str().c_str()), Point(reg->x, reg->y - 5), FONT_HERSHEY_SIMPLEX, 0.5,
					Scalar(255, 255, 255), 1, LINE_8);
		//}
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
	Size imgSize;
	Mat remap_left1, remap_left2, remap_right1, remap_right2;
	Mat left_rect, right_rect;
	Mat framel, framer;
	Mat left, right;
	Mat left_disp, raw_disp;
	int numberOfDisparities = NUM_DISPARITIES;
	Mat xyz;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat img_rectified;
	VideoStreamStereoDevice* videoDev = new V4LStreamStereoDevice("/dev/video1", "/dev/video0", IMG_WIDTH, IMG_HEIGHT);
	MJPEGDecoderDevice* mjpegDecoder = new MJPEGDecoderDevice;
	
	double exec_time;
	mouse_selected = Rect();
#if BM_TYPE == BM
	Ptr<StereoBM> left_matcher = StereoBM::create(numberOfDisparities, 9);
#elif BM_TYPE == SGBM
	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, numberOfDisparities, 9);
#endif

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

#if BM_TYPE == SGBM
	left_matcher->setP1(8 * 3 * 5 * 5);
	left_matcher->setP2(32 * 3 * 5 * 5);
	left_matcher->setMinDisparity(0);
	left_matcher->setNumDisparities(numberOfDisparities);
	left_matcher->setUniquenessRatio(10);
	left_matcher->setSpeckleWindowSize(100);
	left_matcher->setSpeckleRange(32);
	left_matcher->setDisp12MaxDiff(1);
#elif BM_TYPE == BM
	left_matcher->setROI1(roif);
	left_matcher->setROI2(roif);
	left_matcher->setPreFilterCap(31);
	left_matcher->setBlockSize(13);
	left_matcher->setMinDisparity(0);
	left_matcher->setNumDisparities(numberOfDisparities);
	left_matcher->setTextureThreshold(10);
	left_matcher->setUniquenessRatio(10);
	left_matcher->setSpeckleWindowSize(100);
	left_matcher->setSpeckleRange(32);
	left_matcher->setDisp12MaxDiff(1);
#endif

	//setWindowProperty("left", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	Mat img[2];
	Mat left_gray, right_gray;
	Mat right_disp, filtered_disparity_map;

	for (int i = 0; i < 2; ++i) {
		converted[i] = new char[videoDev->getHeight() * videoDev->getWidth() * 3];
		debug("%d: original width = %d ; height = %d\n", i, width[i], height[i]);
		img[i] = Mat(videoDev->getHeight(), videoDev->getWidth(), CV_8UC3, converted[i]);
	}


	debug("after roi width = %d, height = %d\n", roif.width, roif.height);

	for (;;) {
		Mat imgHSV;
		Mat imgThresholded;
		vector<Rect> b_roi;

		struct videoStreamBuffer __bl, __br;
		videoDev->grabOneFrame();
		videoDev->getBuffers(&__bl, &__br);

		exec_time = (double) getTickCount();
		mjpegDecoder->decode(__bl.data, __bl.len, videoDev->getWidth(), videoDev->getHeight(), converted[0]);//(__bl.data, __bl.len, videoDev->getWidth(), videoDev->getHeight(), converted[0]);
		mjpegDecoder->decode(__br.data, __br.len, videoDev->getWidth(), videoDev->getHeight(), converted[1]);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Jpeg2RGB Time: %lf s\n", exec_time);

		exec_time = (double) getTickCount();
		cvtColor(img[0], left_gray, CV_RGB2GRAY);
		cvtColor(img[1], right_gray, CV_RGB2GRAY);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("cvtColor Time: %lf s\n", exec_time);

		exec_time = (double) getTickCount();
		remap(left_gray, left_rect, remap_left1, remap_left2, INTER_LINEAR);
		left_rect = left_rect(roif);

		remap(right_gray, right_rect, remap_right1, remap_right2, INTER_LINEAR);
		right_rect = right_rect(roif);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("remap Time: %lf s\n", exec_time);

		remap(img[0], img_rectified, remap_left1, remap_left2, INTER_LINEAR);
		img_rectified = img_rectified(roif);

		cvtColor(img_rectified, img_rectified, COLOR_RGB2BGR);
		exec_time = (double) getTickCount();
		cvtColor(img_rectified, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		//morphological opening (remove small objects from the foreground)
		morphological_filter(imgThresholded);
		findContours(imgThresholded, contours, hierarchy, CV_RETR_FLOODFILL, CV_LINK_RUNS, Point(0, 0));
		//imshow("th", imgThresholded);

		vector<Point2f> mc(contours.size());

		for (int i = 0; i < contours.size(); i++) {
			Moments mu = moments(contours[i], true);
			mc[i] = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
			Rect br = boundingRect(Mat(contours[i]));
			Mat mask;
			drawContours(mask, contours, i,  Scalar(255));
			Scalar mv = mean(imgThresholded(br), mask);
			if ( mv.val[0] >= 120 )
				b_roi.push_back(br);
			//circle(img_rectified, mc[i], 4, Scalar(255, 255, 255), -1, 8, 0);
		}

		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Contours Time: %lf s\n", exec_time);
		exec_time = (double) getTickCount();
		GaussianBlur(left_rect, left_rect, Size(7, 7), 1.5, 1.5);
		GaussianBlur(right_rect, right_rect, Size(7, 7), 1.5, 1.5);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Gaussian Blur Time: %lf s\n", exec_time);

		exec_time = (double) getTickCount();

		left_matcher->compute(left_rect, right_rect, left_disp);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Matching Time: %lf s\n", exec_time);
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
		getDisparityVis(left_disp, raw_disp);

		left_disp /= 16.;

		exec_time = (double) getTickCount();
		reprojectImageTo3D(left_disp, xyz, Q, true, CV_32F);
		exec_time = ((double) getTickCount() - exec_time) / getTickFrequency();
		debug("Reproject Time: %lf s\n", exec_time);
		calculate_depth(xyz, img_rectified, b_roi, mc);
		b_roi.clear();
		imshow("left", img_rectified);
		//imshow("raw_disp", raw_disp);
		waitKey(10);

		//Canny(edges, edges, 0, 30, 3);
	}

	return 0;
}
