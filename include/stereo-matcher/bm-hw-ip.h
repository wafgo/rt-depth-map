/*
 * bm-hw-ip.h
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#ifndef INCLUDE_BM_BM_HW_IP_H_
#define INCLUDE_BM_BM_HW_IP_H_


#include "stereo-matcher/stereo-matcher.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string.h>
#include "math.h"
#include "debug.h"
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#include <sys/stat.h>
#include <math.h>
#include <cstdint>
#include "uio.h"

using namespace cv;

struct dcx_config
{
	uint16_t id;
	uint32_t ctrl_mem_base;
	uint32_t axis_mem_base;
};

struct dcx_device
{
	intptr_t ctrl_mem_base;
	intptr_t axis_mem_base;
	intptr_t left_mem_base;
	intptr_t right_mem_base;
	intptr_t out_mem_base;
};

struct dcx_uio_map
{
	uint32_t addr;
	uint32_t size;
};

struct dcx_uio_info
{
	int uio_fd;
	int uio_num;
	char name[MAX_UIO_NAME_SIZE];
	char version[MAX_UIO_NAME_SIZE];
	struct dcx_uio_map maps[MAX_UIO_MAPS];
};

class HWMatcherDisparityCoprocessor: public BlockMatcher
{
public:
	HWMatcherDisparityCoprocessor(const char* uio_name, int width, int height);
	~HWMatcherDisparityCoprocessor();
	int compute(InputArray left, InputArray right, OutputArray out);
	void setROI1(cv::Rect roi1) {}
	void setROI2(cv::Rect roi2) {}
private:
	int line_from_file(char* filename, char* linebuf);
	int uio_info_read_map_size(struct dcx_uio_info* info, int n);
	int uio_info_read_map_addr(struct dcx_uio_info* info, int n);
	int uio_info_read_version(dcx_uio_info* info);
	int uio_info_read_name(struct dcx_uio_info* info);
	struct dcx_uio_info* dcx_info;
	struct dcx_device* dcx_dev;
};


#endif /* INCLUDE_BM_BM_HW_IP_H_ */
