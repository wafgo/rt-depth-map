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
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "video_capture.h"
#include "math.h"
#include "debug.h"

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

#define CALIB_UNIT_MM	25.0

//#define ENABLE_POST_FILTER

#define BM			0
#define SGBM		1

#define BM_TYPE		BM


struct v4l2_capability cap[2];
int fd[2];
char* buffer[2];
int width[2], height[2];
struct v4l2_buffer bufferinfo[2];
Rect roi_box;

#ifdef ENABLE_POST_FILTER
Ptr<DisparityWLSFilter> wls_filter;
#endif

char* converted[2];
static bool new_roi_available = false;
static vector<vector<Point> > contours;
static vector<Vec4i> hierarchy;
static int iLowH = 0;
static int iHighH = 9;
static int iLowS = 69;
static int iHighS = 255;
static int iLowV = 0;
static int iHighV = 255;

enum roi_capture_state {
	WAIT_FOR_START_POINT,
	WAIT_FOR_END_POINT,
	WAIT_FOR_UNLOCK,

};

static void mouse_callback(int event, int x, int y, int flags, void* cookie)
{
	static int cap_state = WAIT_FOR_START_POINT;
	Mat* disp = (Mat*) cookie;

	switch (cap_state) {
	case WAIT_FOR_START_POINT:
		if (event == EVENT_LBUTTONDOWN) {
			roi_box.x = x;
			roi_box.y = y;
			cap_state = WAIT_FOR_END_POINT;
			debug("captured start point at x = %d y = %d\n", x, y);
		}
		break;
	case WAIT_FOR_END_POINT:
		if (event == EVENT_LBUTTONUP) {
			roi_box.width = abs(x - roi_box.x);
			roi_box.height = abs(y - roi_box.y);

			if (x < roi_box.x) {
				roi_box.x = x;
			}
			if (y < roi_box.y) {
				roi_box.y = y;
			}
			new_roi_available = true;
			debug("captured roi at x = %d y = %d with width = %d and height %d\n", roi_box.x, roi_box.y, roi_box.width, roi_box.height);
			cap_state = WAIT_FOR_UNLOCK;
		}
		break;
	case WAIT_FOR_UNLOCK:
		if (!new_roi_available)
			cap_state = WAIT_FOR_START_POINT;
		break;
	default:
		printf("invalid state %d. Mouse callback called with event %d, x = %d, y = %d, flags = %d\n", cap_state, event, x, y, flags);
		break;
	}

	debug("mouse callback called with event %d, x = %d, y = %d, flags = %d\n", event, x, y, flags);
}

static void v4l2_init(int i, const char* dev_name)
{
	struct v4l2_format format;
	struct v4l2_requestbuffers bufrequest;

	if ((fd[i] = open(dev_name, O_RDWR)) < 0) {
		perror("open");
		return;
	}

	if (ioctl(fd[i], VIDIOC_QUERYCAP, &cap) < 0) {
		perror("VIDIOC_QUERYCAP");
		exit(1);
	}

	if ((cap[i].capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		cout << "The device can handle single-planar video capture.\n" << endl;
	}

	if ((cap[i].capabilities & V4L2_CAP_STREAMING)) {
		cout << "The device can stream.\n" << endl;
	}

	if ((cap[i].capabilities & V4L2_CAP_READWRITE)) {
		cout << "The device can handle read/write syscalls.\n" << endl;
	}

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	format.fmt.pix.width = 1280;
	format.fmt.pix.height = 720;

	if (ioctl(fd[i], VIDIOC_S_FMT, &format) < 0) {
		perror("VIDIOC_S_FMT");
		cout << "Cold not open correct format.\n" << endl;
		return;
	}

	width[i] = format.fmt.pix.width;
	height[i] = format.fmt.pix.height;

	converted[i] = (char*) malloc(width[i] * height[i] * 3);

	if (!converted[i])
		cout << "could not alloc mem" << endl;

	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;

	if (ioctl(fd[i], VIDIOC_REQBUFS, &bufrequest) < 0) {
		perror("VIDIOC_REQBUFS");
		return;
	}

	memset(&bufferinfo[i], 0, sizeof(bufferinfo[i]));

	bufferinfo[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufferinfo[i].memory = V4L2_MEMORY_MMAP;
	bufferinfo[i].index = 0;

	if (ioctl(fd[i], VIDIOC_QUERYBUF, &bufferinfo[i]) < 0) {
		perror("VIDIOC_QUERYBUF");
		return;
	}

	buffer[i] = (char*) mmap(
			NULL, bufferinfo[i].length,
			PROT_READ | PROT_WRITE,
			MAP_SHARED, fd[i], bufferinfo[i].m.offset);

	if (buffer[i] == MAP_FAILED) {
		perror("mmap");
		return;
	}

	memset(buffer[i], 0, bufferinfo[i].length);

	memset(&bufferinfo[i], 0, sizeof(bufferinfo[i]));

	bufferinfo[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufferinfo[i].memory = V4L2_MEMORY_MMAP;
	bufferinfo[i].index = 0;

	int type = bufferinfo[i].type;
	if (ioctl(fd[i], VIDIOC_STREAMON, &type) < 0) {
		cout << "VIDIOC_STREAMON" << " " << errno << endl;
		return;
	}
}



static float calculate_depth(Mat xyz, Mat disp, Rect& region)
{
	rectangle(disp, Point(region.x, region.y), Point(region.x + region.width, region.y + region.height), Scalar(255,255,255), 1, LINE_8);
	ostringstream distance_text;

	double res = 0.0;
	int cnt = 0;
	const double max_z = 1.0e4;
	xyz = xyz(region);

	for (int y = 0; y < xyz.rows; y++) {
		for (int x = 0; x < xyz.cols; x++) {
			Vec3f point = xyz.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				continue;
			res += point[2];
			cnt++;
		}
	}

	if (cnt > 0)
		res = res/cnt;
//	distance_text.fixed;
//	distance_text.precision(1);
	distance_text << res * CALIB_UNIT_MM/10.0 << " cm";
	putText(disp, String(distance_text.str().c_str()), Point(region.x, region.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1, LINE_8);
	new_roi_available = false;
	return 0.0f;
}

int main(int, char**)
{
	Rect roi1, roi2;
	Rect roif;
	Mat M1, D1, M2, D2;
	Mat R, T, R1, P1, R2, P2, Q;
	Size imgSize;
	Mat map11, map12, map21, map22;
	Mat left_rect, right_rect;
	Mat framel, framer;
	Mat left, right;
	Mat left_disp, raw_disp;
	int numberOfDisparities = 64 * 4;
	Mat xyz;
	Rect b_roi;
	double exec_time;
	roi_box = Rect();
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
	imgSize.width = 1280;
	imgSize.height = 720;

	FileStorage intrinsics("intrinsics.yml", FileStorage::READ);
	FileStorage extrinsics("extrinsics.yml", FileStorage::READ);

	if (!intrinsics.isOpened() || !extrinsics.isOpened()) {
		printf("could not open intrinsics or extrinsics\n");
		return -1;
	}

	intrinsics["M1"] >> M1;
	intrinsics["D1"] >> D1;
	intrinsics["M2"] >> M2;
	intrinsics["D2"] >> D2;

	extrinsics["ROI1"] >> roi1;
	extrinsics["ROI2"] >> roi2;
	extrinsics["R"] >> R;
	extrinsics["T"] >> T;

	roif.x = max(roi1.x, roi2.x);
	roif.y = max(roi1.y, roi2.y);
	roif.width = min(roi1.width, roi2.width);
	roif.height = min(roi1.height, roi2.height);

	stereoRectify(M1, D1, M2, D2, imgSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgSize, &roi1, &roi2);

	initUndistortRectifyMap(M1, D1, R1, P1, imgSize, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imgSize, CV_16SC2, map21, map22);

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
	left_matcher->setROI1(roi1);
	left_matcher->setROI2(roi2);
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

	v4l2_init(0, "/dev/video0");
	v4l2_init(1, "/dev/video1");

	namedWindow("left", 1);
	//namedWindow("right", 1);
	namedWindow("raw_disp", 0);
	setMouseCallback("raw_disp", mouse_callback, &raw_disp);
	Mat img[2];
	Mat left_gray, right_gray;
	Mat right_disp, filtered_disparity_map;

	for (int i = 0; i < 2; ++i) {
		debug("%d: original width = %d ; height = %d\n", i, width[i], height[i]);
		img[i] = Mat( height[i], width[i], CV_8UC3, converted[i]);
	}

	debug("after roi width = %d, height = %d\n", roif.width, roif.height);

	for (;;) {
		Mat imgHSV;
		Mat imgThresholded;

		if (ioctl(fd[0], VIDIOC_QBUF, &bufferinfo[0]) < 0) {
			printf("VIDIOC_QBUF error %d\n", errno);
			break;
		}
		if (ioctl(fd[1], VIDIOC_QBUF, &bufferinfo[1]) < 0) {
			printf("VIDIOC_QBUF error %d\n", errno);
			break;
		}

		if (ioctl(fd[0], VIDIOC_DQBUF, &bufferinfo[0]) < 0) {
			printf("VIDIOC_DQBUF error %d\n", errno);
			break;
		}
		if (ioctl(fd[1], VIDIOC_DQBUF, &bufferinfo[1]) < 0) {
			printf("VIDIOC_DQBUF error %d\n", errno);
			break;
		}

		exec_time = (double)getTickCount();
		mjpeg2rgb(buffer[0], bufferinfo[0].bytesused, width[0], height[0], converted[0]);
		mjpeg2rgb(buffer[1], bufferinfo[1].bytesused, width[1], height[1], converted[1]);
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "Jpeg2RGB Time: " << exec_time<<" s" << endl;

		exec_time = (double)getTickCount();
		cvtColor(img[0], left_gray, CV_RGB2GRAY);
		cvtColor(img[1], right_gray, CV_RGB2GRAY);
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "cvtColor Time: " << exec_time<<" s" << endl;

		exec_time = (double)getTickCount();
		remap(left_gray, left_rect, map11, map12, INTER_LINEAR);
		left_rect = left_rect(roif);

		remap(right_gray, right_rect, map21, map22, INTER_LINEAR);
		right_rect = right_rect(roif);
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "remap Time: " << exec_time<<" s" << endl;

		Mat img_rectified;

		remap(img[0], img_rectified, map11, map12, INTER_LINEAR);
		img_rectified = img_rectified(roif);

		cvtColor(img_rectified, img_rectified, COLOR_RGB2BGR);
		exec_time = (double)getTickCount();
		cvtColor(img_rectified, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
		findContours(imgThresholded, contours, hierarchy, CV_RETR_FLOODFILL, CV_LINK_RUNS, Point(0, 0));

		for (int i = 0; i < contours.size(); i++) {
			Scalar color = Scalar(255, 255, 255);
			b_roi = boundingRect(Mat(contours[i]));
			rectangle(img_rectified, b_roi, Scalar(0,0,0));
		}
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "Contours Time: " << exec_time<<" s" << endl;
		imshow("left", img_rectified);
		//imshow("right", right_rect);

		exec_time = (double)getTickCount();
		GaussianBlur(left_rect, left_rect, Size(7,7), 1.5, 1.5);
		GaussianBlur(right_rect, right_rect, Size(7,7), 1.5, 1.5);
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "Gaussian Blur Time: " << exec_time<<" s" << endl;

		exec_time = (double)getTickCount();

		left_matcher->compute(left_rect, right_rect, left_disp);
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "Matching Time: " << exec_time<<" s" << endl;
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

		exec_time = (double)getTickCount();
		reprojectImageTo3D(left_disp, xyz, Q, true, CV_32F);
		exec_time = ((double)getTickCount() - exec_time)/getTickFrequency();
		cout << "Reproject Time: " << exec_time<<" s" << endl;

		calculate_depth(xyz, raw_disp, b_roi);
		imshow("raw_disp", raw_disp);
		waitKey(10);

		//Canny(edges, edges, 0, 30, 3);
	}

	return 0;
}
