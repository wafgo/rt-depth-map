/*
 * morphological_filter_hw_ip.cpp
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#include "filter/mf-hw-ip.h"

#define XMORPH_FILTER_MAJOR					10
#define XMORPH_FILTER_MINOR					235

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//IOCTL calls for the filter
#define XMORPH_IOCTL_BASE					'S'
#define XMORPH_START						_IO(XMORPH_IOCTL_BASE, 0)
#define XMORPH_STOP							_IO(XMORPH_IOCTL_BASE, 1)
#define XMORPH_SET_DIM						_IO(XMORPH_IOCTL_BASE, 2)

HWMorphologicalFilterIPCore::~HWMorphologicalFilterIPCore()
{
	close(fd);
}

HWMorphologicalFilterIPCore::HWMorphologicalFilterIPCore(int w, int h, int bpp)
{
    struct vdma_transfer_dim dim;
	info = new struct xmorph_filter_uio_info;

	img_bpp = bpp;
	dim.dy = img_height = h;
    dim.dx = img_width = w;

	int err = init_morphological_filter_ip();

	if (err != 0) {
		printf("Could not open xmorph filter %d\n", err);
	} else {
		printf("successfully opened xmoprh filter ip core\n");
	}

	/* if device file does not exist, create it */
	if (access("/dev/xmorph0", F_OK) == -1) {
		if (mknod("/dev/xmorph0", S_IFCHR | S_IRUSR | S_IWUSR, makedev(XMORPH_FILTER_MAJOR, XMORPH_FILTER_MINOR))) {
			perror("mknod() error");
			exit(1);
		} else {
			printf("successfully created xmorph char device\n");
		}
	}

	if ((fd = open("/dev/xmorph0", O_RDWR)) < 0) {
		printf("Cannot open device node xfilter, shutting down the app\n");
		exit(1);
	} else {
		printf("successfully opened device node xfilter\n");
	}

	if (ioctl(fd, XMORPH_SET_DIM, &dim) < 0) {
		printf("Failed to set filter dimension, shutting down the app\n");
		exit(1);
	} else {
		printf("successfully set filter dimensions\n");
	}
}

int HWMorphologicalFilterIPCore::init_morphological_filter_ip(void)
{
	struct dirent **namelist;
	int i, n;
	char* s;
	char file[MAX_UIO_PATH_SIZE];
	char name[MAX_UIO_NAME_SIZE];
	int flag = 0;
	char* xmorph_name = (char*) "xmorph-dev";

	n = scandir("/sys/class/uio", &namelist, 0, alphasort);
	if (n < 0)
		return -1;
	for (i = 0; i < n; i++) {
		printf("found dir %s\n", namelist[i]->d_name);
		strcpy(file, "/sys/class/uio/");
		strcat(file, namelist[i]->d_name);
		strcat(file, "/name");
		if ((line_from_file(file, name) == 0) && (strcmp(name, xmorph_name) == 0)) {
			flag = 1;
			s = namelist[i]->d_name;
			s += 3; // "uio"
			info->uio_num = atoi(s);
			break;
		}
	}
	if (flag == 0)
		return -1;

	uio_info_read_name(info);
	uio_info_read_version(info);
	for (n = 0; n < MAX_UIO_MAPS; ++n) {
		uio_info_read_map_addr(info, n);
		uio_info_read_map_size(info, n);
	}

	sprintf(file, "/dev/uio%d", info->uio_num);
	if ((info->uio_fd = open(file, O_RDWR)) < 0) {
		return -1;
	}

	// NOTE: slave interface 'Axilites' should be mapped to uioX/map1
	xmorph_ctrl_regs = (void*) mmap(NULL, info->maps[0].size, PROT_READ | PROT_WRITE, MAP_SHARED, info->uio_fd, 0);

	video_in = (char*) mmap(NULL, info->maps[1].size, PROT_READ | PROT_WRITE, MAP_SHARED, info->uio_fd,
			1 * getpagesize());

	video_out = (char*) mmap(NULL, info->maps[2].size, PROT_READ | PROT_WRITE, MAP_SHARED, info->uio_fd,
			2 * getpagesize());

	return 0;
}

int HWMorphologicalFilterIPCore::run(cv::InputArray in, cv::OutputArray out)
{
	int err = 0;

	if (ioctl(fd, XMORPH_START, NULL) < 0) {
		printf("Failed to start Filter driver\n");
		err = -1;
	}

	return err;
}

int HWMorphologicalFilterIPCore::line_from_file(char* filename, char* linebuf)
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

int HWMorphologicalFilterIPCore::uio_info_read_name(struct xmorph_filter_uio_info* info)
{
	char file[MAX_UIO_PATH_SIZE];
	sprintf(file, "/sys/class/uio/uio%d/name", info->uio_num);
	return line_from_file(file, info->name);
}

int HWMorphologicalFilterIPCore::uio_info_read_version(struct xmorph_filter_uio_info* info)
{
	char file[MAX_UIO_PATH_SIZE];
	sprintf(file, "/sys/class/uio/uio%d/version", info->uio_num);
	return line_from_file(file, info->version);
}

int HWMorphologicalFilterIPCore::uio_info_read_map_addr(struct xmorph_filter_uio_info* info, int n)
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

int HWMorphologicalFilterIPCore::uio_info_read_map_size(struct xmorph_filter_uio_info* info, int n)
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
