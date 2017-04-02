#include "stream/video-stream-stereo-device.h"

VideoStreamStereoDevice::VideoStreamStereoDevice()
{

}

int VideoStreamStereoDevice::getFrameSize() const
{
	return (width * height);
}

int VideoStreamStereoDevice::getWidth() const
{
    return width;
}

void VideoStreamStereoDevice::setWidth(int value)
{
    width = value;
}

int VideoStreamStereoDevice::getHeight() const
{
    return height;
}

void VideoStreamStereoDevice::setHeight(int value)
{
    height = value;
}

void VideoStreamStereoDevice::getBuffers(struct videoStreamBuffer* left, struct videoStreamBuffer* right)
{
	if (left && right) {
		left->data = buffer_left;
		right->data = buffer_right;

		left->len = buffer_left_len;
		right->len = buffer_right_len;
	}
}

