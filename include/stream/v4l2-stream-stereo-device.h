/*
 * v4l2-stream-device.h
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#ifndef INCLUDE_V4L2_STREAM_DEVICE_H_
#define INCLUDE_V4L2_STREAM_DEVICE_H_

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
#include <string.h>
#include "video-stream-stereo-device.h"

class V4LStreamStereoDevice : public VideoStreamStereoDevice
{
public:
    V4LStreamStereoDevice(std::string vdev_right = "/dev/video0", std::string vdev_left = "/dev/video1", int width = 640, int height = 480);
    ~V4LStreamStereoDevice();
    int grabOneFrame();
    int connectToDevice();
private:
    void initialize_v4l2_device(std::string dev_name, int* fd, struct v4l2_buffer* buf_info,  struct v4l2_capability* caps, char** buff, int width, int height);
    struct v4l2_capability cap;
    int fd, fd_right;
    struct v4l2_buffer bufferinfo_left, bufferinfo_right;
    struct v4l2_capability cap_left, cap_right;
};




#endif /* INCLUDE_V4L2_STREAM_DEVICE_H_ */
