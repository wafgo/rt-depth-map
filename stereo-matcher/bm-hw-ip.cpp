/*
 * bm_hw_ip.cpp
 *
 *  Created on: 02.04.2017
 *      Author: wadim mueller
 */

#include "stereo-matcher/bm-hw-ip.h"
#include <stdlib.h>
#include "debug.h"
#include "string.h"
#include "io.h"


#define MAX_DISP				12
// CTRL_BUS
#define CTRL_BPP          		0x10
#define CTRL_XDIM         		0x18
#define CTRL_YDIM         		0x20
#define CTRL_CURRENT_ROW  		0x28
#define CTRL_CURRENT_ROW_CTRL  	0x2c
#define CTRL_MAXDISPARITY 		0x30

// AXILiteS_BUS
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
#define AXILITES_AP_CTRL        0x00
#define AXILITES_GIE            0x04
#define AXILITES_IER            0x08
#define AXILITES_ISR            0x0c
#define AXILITES_AP_RETURN      0x10

#define AXILITES_LEFT   		0x18
#define AXILITES_RIGHT  		0x20
#define AXILITES_OUTPUT 		0x28

#define AP_EN_BIT				0U
#define AP_DONE_BIT				1U

int HWMatcherDisparityCoprocessor::uio_info_read_name(struct dcx_uio_info* info)
{
	char file[MAX_UIO_PATH_SIZE];
	sprintf(file, "/sys/class/uio/uio%d/name", info->uio_num);
	return line_from_file(file, info->name);
}

int HWMatcherDisparityCoprocessor::uio_info_read_version(dcx_uio_info* info)
{
	char file[MAX_UIO_PATH_SIZE];
	sprintf(file, "/sys/class/uio/uio%d/version", info->uio_num);
	return line_from_file(file, info->version);
}

int HWMatcherDisparityCoprocessor::uio_info_read_map_addr(struct dcx_uio_info* info, int n)
{
	int ret;
	char file[MAX_UIO_PATH_SIZE];
	info->maps[n].addr = UIO_INVALID_ADDR;
	sprintf(file, "/sys/class/uio/uio%d/maps/map%d/addr", info->uio_num, n);
	FILE* fp = fopen(file, "r");
	if (!fp)
		return -1;
	ret = fscanf(fp, "0x%x", &info->maps[n].addr);
	fclose(fp);
	if (ret < 0)
		return -2;
	return 0;
}

int HWMatcherDisparityCoprocessor::uio_info_read_map_size(struct dcx_uio_info* info, int n)
{
	int ret;
	char file[MAX_UIO_PATH_SIZE];
	sprintf(file, "/sys/class/uio/uio%d/maps/map%d/size", info->uio_num, n);
	FILE* fp = fopen(file, "r");
	if (!fp)
		return -1;
	ret = fscanf(fp, "0x%x", &info->maps[n].size);
	fclose(fp);
	if (ret < 0)
		return -2;
	return 0;
}

int HWMatcherDisparityCoprocessor::line_from_file(char* filename, char* linebuf)
{
	char* s;
	int i;
	FILE* fp = fopen(filename, "r");
	if (!fp)
		return -1;
	s = fgets(linebuf, MAX_UIO_NAME_SIZE, fp);
	fclose(fp);
	if (!s)
		return -2;
	for (i = 0; (*s) && (i < MAX_UIO_NAME_SIZE); i++) {
		if (*s == '\n')
			*s = 0;
		s++;
	}
	return 0;
}

HWMatcherDisparityCoprocessor::HWMatcherDisparityCoprocessor(const char* uio_name, int width,
		int height)
{
	dcx_info = new struct dcx_uio_info;
	dcx_dev = new struct dcx_device;

	struct dirent **namelist;
	int i, n;
	char* s;
	char file[MAX_UIO_PATH_SIZE];
	char name[MAX_UIO_NAME_SIZE];
	int flag = 0;

	n = scandir("/sys/class/uio", &namelist, 0, alphasort);
	if (n < 0) {
		printf("could not find any disparty coporocessor in System, shutting down the app\n");
		exit(1);
	}
	for (i = 0; i < n; i++) {
		debug("found dir %s\n", namelist[i]->d_name);
		strcpy(file, "/sys/class/uio/");
		strcat(file, namelist[i]->d_name);
		strcat(file, "/name");
		if ((line_from_file(file, name) == 0) && (strcmp(name, uio_name ? uio_name : "disp-coproc") == 0)) {
			flag = 1;
			s = namelist[i]->d_name;
			s += 3; // "uio"
			dcx_info->uio_num = atoi(s);
			break;
		}
	}
	if (flag == 0) {
		printf("could not find the requested disparty coporocessor, shutting down the app\n");
		exit(1);
	}

	uio_info_read_name(dcx_info);
	uio_info_read_version(dcx_info);
	for (n = 0; n < MAX_UIO_MAPS; ++n) {
		uio_info_read_map_addr(dcx_info, n);
		uio_info_read_map_size(dcx_info, n);
	}

	sprintf(file, "/dev/uio%d", dcx_info->uio_num);
	if ((dcx_info->uio_fd = open(file, O_RDWR)) < 0) {
		printf("could not open the requested disparty coporocessor, shutting down the app\n");
		exit(1);
	}

	// NOTE: slave interface 'Axilites' should be mapped to uioX/map1
	dcx_dev->axis_mem_base = (intptr_t) mmap(NULL, dcx_info->maps[0].size, PROT_READ | PROT_WRITE, MAP_SHARED,
			dcx_info->uio_fd, 0);
	dcx_dev->ctrl_mem_base = dcx_dev->axis_mem_base + (dcx_info->maps[0].size >> 1);
	dcx_dev->left_mem_base = (intptr_t) mmap(NULL, dcx_info->maps[1].size, PROT_READ | PROT_WRITE, MAP_SHARED,
			dcx_info->uio_fd, 1 * getpagesize());
	dcx_dev->right_mem_base = (intptr_t) mmap(NULL, dcx_info->maps[2].size, PROT_READ | PROT_WRITE, MAP_SHARED,
			dcx_info->uio_fd, 2 * getpagesize());
	dcx_dev->out_mem_base = (intptr_t) mmap(NULL, dcx_info->maps[3].size, PROT_READ | PROT_WRITE, MAP_SHARED,
			dcx_info->uio_fd, 3 * getpagesize());

	__io uint32_t* bpp_reg = (__io uint32_t*) ((uint8_t*) dcx_dev->ctrl_mem_base + CTRL_BPP);
	__io uint32_t* xdim_reg = (__io uint32_t*) ((uint8_t*) dcx_dev->ctrl_mem_base  + CTRL_XDIM);
	__io uint32_t* ydim_reg = (__io uint32_t*) ((uint8_t*) dcx_dev->ctrl_mem_base  + CTRL_YDIM);
	__io uint32_t* max_disp_reg = (__io uint32_t*) ((uint8_t*) dcx_dev->ctrl_mem_base  + CTRL_MAXDISPARITY);

	/* set the user space mapped register handling */
	*bpp_reg = 8;
	*xdim_reg = width;
	*ydim_reg = height;
	*max_disp_reg = MAX_DISP;

}

HWMatcherDisparityCoprocessor::~HWMatcherDisparityCoprocessor()
{

}

int HWMatcherDisparityCoprocessor::compute(InputArray left, InputArray right, OutputArray out)
{
	/* do the user space mapped register handling */
	__io uint32_t* control_register = (__io uint32_t*) dcx_dev->axis_mem_base;
	__io uint32_t* current_row_register = (__io uint32_t*) ((uint8_t*) dcx_dev->ctrl_mem_base + CTRL_CURRENT_ROW);

	*control_register |= (1 << AP_EN_BIT);

	/*FIXME: is still far too slow to use in real time application. I need to tweak the HDL. For example by using OpenCL with Vivado HLS*/
	while (!(*control_register & (1 << AP_DONE_BIT))) {
		debug("current row is %d\n", *current_row_register);
		sleep(1);
	}

	return 0;
}
