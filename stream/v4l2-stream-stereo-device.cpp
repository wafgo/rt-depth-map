/*
 * v4l2-stream-device.cpp
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#include "stream/v4l2-stream-stereo-device.h"
#include <stdlib.h>

using namespace std;

void V4LStereoStreamDevice::initialize_v4l2_device(std::string dev_name, int* fd, struct v4l2_buffer* buf_info,  struct v4l2_capability* caps, char** buff, int width, int height)
{
	struct v4l2_format format;
	struct v4l2_requestbuffers bufrequest;

	if ((*fd = open(dev_name.c_str(), O_RDWR)) < 0) {
		perror("open");
		return;
	}

	if (ioctl(*fd, VIDIOC_QUERYCAP, caps) < 0) {
		perror("VIDIOC_QUERYCAP");
		exit(1);
	}

	if ((caps->capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		cout << "The device can handle single-planar video capture.\n" << endl;
	}

	if ((caps->capabilities & V4L2_CAP_STREAMING)) {
		cout << "The device can stream.\n" << endl;
	}

	if ((caps->capabilities & V4L2_CAP_READWRITE)) {
		cout << "The device can handle read/write syscalls.\n" << endl;
	}

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;

	if (ioctl(*fd, VIDIOC_S_FMT, &format) < 0) {
		perror("VIDIOC_S_FMT");
		cout << "Cold not open correct format.\n" << endl;
		return;
	}


	this->width = format.fmt.pix.width;
	this->height = format.fmt.pix.height;

	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;

	if (ioctl(*fd, VIDIOC_REQBUFS, &bufrequest) < 0) {
		perror("VIDIOC_REQBUFS");
		return;
	}

	memset(buf_info, 0, sizeof(*buf_info));

	buf_info->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf_info->memory = V4L2_MEMORY_MMAP;
	buf_info->index = 0;

	if (ioctl(*fd, VIDIOC_QUERYBUF, buf_info) < 0) {
		perror("VIDIOC_QUERYBUF");
		return;
	}

	*buff = (char*) mmap(
	NULL, buf_info->length,
	PROT_READ | PROT_WRITE,
	MAP_SHARED, *fd, buf_info->m.offset);

	if (*buff == MAP_FAILED) {
		perror("mmap");
		return;
	}

	memset(*buff, 0, buf_info->length);

	memset(buf_info, 0, sizeof(*buf_info));

	buf_info->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf_info->memory = V4L2_MEMORY_MMAP;
	buf_info->index = 0;

	int type = buf_info->type;
	if (ioctl(*fd, VIDIOC_STREAMON, &type) < 0) {
		cout << "VIDIOC_STREAMON" << " " << errno << endl;
		return;
	}
}

V4LStereoStreamDevice::V4LStereoStreamDevice(std::__cxx11::string vdev_right, std::__cxx11::string vdev_left, int width, int height)
{
	initialize_v4l2_device(vdev_right, &fd_right, &bufferinfo_right, &cap_right, &buffer_right, width, height);
	initialize_v4l2_device(vdev_left, &fd_left, &bufferinfo_left, &cap_left, &buffer_left, width, height);
}

V4LStereoStreamDevice::~V4LStereoStreamDevice()
{
	close(fd_left);
	close(fd_right);
}

int V4LStereoStreamDevice::grabOneFrame()
{
	if (ioctl(fd_left, VIDIOC_QBUF, &bufferinfo_left) < 0) {
		printf("VIDIOC_QBUF error %d\n", errno);
	}

	if (ioctl(fd_right, VIDIOC_QBUF, &bufferinfo_right) < 0) {
		printf("VIDIOC_QBUF error %d\n", errno);
	}

	if (ioctl(fd_left, VIDIOC_DQBUF, &bufferinfo_left) < 0) {
		printf("VIDIOC_DQBUF error %d\n", errno);
	}

	if (ioctl(fd_right, VIDIOC_DQBUF, &bufferinfo_right) < 0) {
		printf("VIDIOC_DQBUF error %d\n", errno);
	}

	this->buffer_right_len = bufferinfo_right.bytesused;
	this->buffer_left_len = bufferinfo_left.bytesused;
	return 0;
}

int V4LStereoStreamDevice::connectToDevice()
{
	return 0;
}

