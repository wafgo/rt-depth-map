/*
 * mf-hw-ip.h
 *
 *  Created on: 02.04.2017
 *      Author: wadim mueller
 */

#ifndef INCLUDE_FILTER_MF_HW_IP_H_
#define INCLUDE_FILTER_MF_HW_IP_H_


#include "filter/filter.h"
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "opencv2/opencv.hpp"
#include "uio.h"
#include "io.h"

struct vdma_transfer_dim {
	int dx;
	int dy;
};

typedef struct
{
	uint32_t addr;
	uint32_t size;
} xmorph_filter_uio_map;

struct xmorph_filter_uio_info
{
	int uio_fd;
	int uio_num;
	char name[MAX_UIO_NAME_SIZE];
	char version[MAX_UIO_NAME_SIZE];
	xmorph_filter_uio_map maps[MAX_UIO_MAPS];
};

class HWMorphologicalFilterIPCore: public VideoFilterDevice
{
public:
	explicit HWMorphologicalFilterIPCore(int w, int h, int bpp);
	~HWMorphologicalFilterIPCore();
    int run(cv::InputArray in, cv::OutputArray out);

private:
	int line_from_file(char* filename, char* linebuf);
	int uio_info_read_map_size(struct xmorph_filter_uio_info* info, int n);
	int uio_info_read_map_addr(struct xmorph_filter_uio_info* info, int n);
	int uio_info_read_version(struct xmorph_filter_uio_info* info);
	int uio_info_read_name(struct xmorph_filter_uio_info* info);
	int init_morphological_filter_ip(void);
	struct xmorph_filter_uio_info* info;
	__io void* xmorph_ctrl_regs;
	int fd;
};


#endif /* INCLUDE_FILTER_MF_HW_IP_H_ */
