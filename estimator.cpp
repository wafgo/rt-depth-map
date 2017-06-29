/*
 * estimator.cpp
 *
 *  Created on: 25.05.2017
 *      Author: sefo
 */

#include "estimator.h"

void Estimator::run()
{
	Mat imgHSV;
	Mat imgThresholded;
	Mat contInput;

	struct videoStreamBuffer videoBufferLeft, videoBufferRight;

	while (1) {
		exec_time = (double) getTickCount();
		MEASURE_EXECUTION_TIME_START;
		MEASURE_EXECUTION_TIME(videoDevice->grabOneFrame());
		videoDevice->getBuffers(&videoBufferLeft, &videoBufferRight);

		MEASURE_EXECUTION_TIME(mjpegDecoder->decode(videoBufferLeft.data, videoBufferLeft.len, videoDevice->getWidth(), videoDevice->getHeight(),
				rgb[0]));
		MEASURE_EXECUTION_TIME(mjpegDecoder->decode(videoBufferRight.data, videoBufferRight.len, videoDevice->getWidth(), videoDevice->getHeight(),
				rgb[1]));

		MEASURE_EXECUTION_TIME(cvtColor(img[0], left_gray, CV_RGB2GRAY));
		MEASURE_EXECUTION_TIME(cvtColor(img[1], right_gray, CV_RGB2GRAY));

		MEASURE_EXECUTION_TIME(remap(left_gray, left_rect, remap_left1, remap_left2, INTER_LINEAR));
		left_rect = left_rect(roif);

		MEASURE_EXECUTION_TIME(remap(right_gray, right_rect, remap_right1, remap_right2, INTER_LINEAR));
		right_rect = right_rect(roif);

		MEASURE_EXECUTION_TIME(remap(img[0], img_rectified, remap_left1, remap_left2, INTER_LINEAR));
		img_rectified = img_rectified(roif);
		MEASURE_EXECUTION_TIME(cvtColor(img_rectified, img_rectified, COLOR_RGB2BGR));

		MEASURE_EXECUTION_TIME(cvtColor(img_rectified, imgHSV, COLOR_BGR2HSV)); //Convert the captured frame from BGR to HSV
		MEASURE_EXECUTION_TIME(inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), filter_in)); //Threshold the image

		MEASURE_EXECUTION_TIME(morphFilter->run(filter_in, filter_out));
		MEASURE_EXECUTION_TIME(filter_out.copyTo(contInput));
		MEASURE_EXECUTION_TIME(findContours(contInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)));
		if (contours.size() > 0) {
			Rect matching_roi;

			vector<Rect> obj_boundings;
			MEASURE_EXECUTION_TIME(fill_bounding_rects_of_contours(contours, hierarchy, obj_boundings, minObjSize));
			MEASURE_EXECUTION_TIME(find_relevant_matching_region(obj_boundings, matching_roi));
			bm->setROI1(matching_roi);
			/*FIXME: set ROI2 too*/
			MEASURE_EXECUTION_TIME(bm->compute(left_rect, right_rect, left_disp));
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
			if (showDisparityMap) {
				getDisparityVis(left_disp, raw_disp);
				imshow("disparity", raw_disp);
			}
			left_disp /= 16.;
			MEASURE_EXECUTION_TIME(reprojectImageTo3D(left_disp, xyz, Q, true, CV_32F));
			MEASURE_EXECUTION_TIME(calc_depth(xyz, left_disp, filter_out, img_rectified, obj_boundings, calibration_unit));
		}
		//debug("Overall Time: %lf s\n", ((double) getTickCount() - exec_time) / getTickFrequency());
		imshow("depth", img_rectified);
		waitKey(10);
	}
}

Estimator::~Estimator()
{
	delete(exec_times_tab);
	for (int i = 0; i < 2; ++i) {
		delete(rgb[i]);
	}
}


Estimator::Estimator(BlockMatcher* matcher, VideoFilterDevice* filter, VideoStreamStereoDevice* streamer, DecoderDevice* decoder, EstimatorCmdLineParser* cmdLineParser, Mat remapl1, Mat remapl2, Mat remapr1,
		Mat remapr2, Mat reprojMatrix, Rect &relevantRoi)
{
	this->bm = matcher;
	this->morphFilter = filter;
	this->videoDevice = streamer;
	this->mjpegDecoder = decoder;
	this->parser = cmdLineParser;
	exec_time_func_idx = 0;
	exec_times_tab = new struct exec_time_struct[num_exec_times_tab];
	memset(exec_times_tab, 0, sizeof(struct exec_time_struct) * num_exec_times_tab);

#ifdef ENABLE_POST_FILTER
	right_matcher = createRightMatcher(this->bm);
	wls_filter = createDisparityWLSFilter(this->bm);
#endif
	iLowH = 0;
	iHighH = 9;
	iLowS = 150;
	iHighS = 255;
	iLowV = 0;
	iHighV = 255;

	imgSize.width = videoDevice->getWidth();
	imgSize.height = videoDevice->getHeight();
	showDisparityMap = parser->isDisparityMap();
	calibration_unit = parser->getCalibrationUnit();
	numberOfDisparities = parser->getNumOfDisparities(imgSize.width, imgSize.height);
	minObjSize = parser->getMinimalObjectSize(imgSize.width, imgSize.height);


	remap_left1 = remapl1;
	remap_left2 = remapl2;
	remap_right1 = remapr1;
	remap_right2 = remapr2;
	Q = reprojMatrix;
	roif = relevantRoi;

	if (parser->isAdjustable())
		create_adjustment_track_bars();

	for (int i = 0; i < 2; ++i) {
		rgb[i] = new char[videoDevice->getHeight() * videoDevice->getWidth() * 3];
		debug("%d: original width = %d ; height = %d\n", i, videoDevice->getWidth(), videoDevice->getHeight());
		img[i] = Mat(videoDevice->getHeight(), videoDevice->getWidth(), CV_8UC3, rgb[i]);
	}

	filter_in = Mat(roif.height, roif.width, CV_8UC1, morphFilter->getVideoInBuffer());
	filter_out = Mat(roif.height, roif.width, CV_8UC1, morphFilter->getVideoOutBuffer());

	debug("undistorted roi width = %d, height = %d\n", roif.width, roif.height);

	namedWindow("depth", CV_WINDOW_NORMAL);
#ifdef __ZYNQ__
	setWindowProperty("depth", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
#endif
}

void Estimator::set_label(Mat& im, const string label, const Point& origin)
{
    int fontface = FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    Size text = getTextSize(label, fontface, scale, thickness, &baseline);
    rectangle(im, origin + Point(0, baseline), origin + Point(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);
    putText(im, label, origin, fontface, scale, CV_RGB(255,255,255), thickness, 8);
}

void Estimator::fill_bounding_rects_of_contours(
		vector<vector<Point> >& contours, vector<Vec4i>& hierarchy,
		vector<Rect>& bounds, int minSize)
{
	for(int i = 0 ; i >= 0; i = hierarchy[i][0] ) {
		Rect region = boundingRect(Mat(contours[i]));
		if (region.area() < minSize)
			continue;
		bounds.push_back(region);
	}
}

int Estimator::find_relevant_matching_region(vector<Rect>& bounds,
		Rect& roi)
{
	int max_x = -1e6;
	int max_y = -1e6;
	int min_x = 1e6;
	int min_y = 1e6;

	for(int i = 0 ; i < bounds.size(); ++i ) {
		Rect reg = bounds[i];
		int reg_x2 = reg.x + reg.width;
		int reg_y2 = reg.y + reg.height;

		if (reg.x < min_x)
			min_x = reg.x;
		if (reg.y < min_y)
			min_y = reg.y;
		if (reg_x2 > max_x)
			max_x = reg_x2;
		if (reg_y2 > max_y)
			max_y = reg_y2;
	}
	roi.x = min_x;
	roi.y = min_y;
	roi.width = max_x - min_x;
	roi.height = max_y - min_y;

	return 0;
}

void Estimator::calc_depth(Mat xyz, Mat disparity_map, Mat mask, Mat img,
		vector<Rect>& regions, double calibrationUnit)
{
	Mat xyz_region;
	Mat mask_region;
	ostringstream distance_text;
#ifdef SHOW_DISPARITY_VALUE
	Mat tmp_disp_map;
#endif
	for(int i = 0 ; i < regions.size(); ++i) {
		Rect reg = regions[i];

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
			res	= (res * calibrationUnit / 10.0);
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

void Estimator::print_exec_time_stats(void)
{
#ifdef ENABLE_EXECUTION_TIME_MEASUREMENT
	size_t max_str_size = 0;
	int i;
	double overall_exec_time = 0.0;

	for (i = 0 ; i < num_exec_times_tab ; ++i) {
		struct exec_time_struct* et = &exec_times_tab[i];
		if (et->func_name.length() > max_str_size)
			max_str_size = et->func_name.length();
	}

	printf( "----------------------------------------------------------------------------------------------------------\n"
			"-----------------------------------------Measurement Statistics-------------------------------------------\n"
			"----------------------------------------------------------------------------------------------------------\n"
			);
	for (i = 0 ; i < num_exec_times_tab; ++i) {
		struct exec_time_struct* et = &exec_times_tab[i];

		if (et->num) {
			overall_exec_time += et->tv;
			printf("[%*.*s] average execution time = %lfs measured over %d periods\n", (int)max_str_size, (int)max_str_size, et->func_name.c_str(), et->tv, et->num);
		}
	}
	printf("-> average overall execution time = %lfs\n", overall_exec_time);
#endif
}

void Estimator::create_adjustment_track_bars(void)
{
	createTrackbar("hue low", "depth", &iLowH, 255, NULL);
	createTrackbar("hue high", "depth", &iHighH, 255, NULL);

	createTrackbar("saturation low", "depth", &iLowS, 255, NULL);
	createTrackbar("saturation high", "depth", &iHighS, 255, NULL);

	createTrackbar("value low", "depth", &iLowV, 255, NULL);
	createTrackbar("value high", "depth", &iHighV, 255, NULL);
}
