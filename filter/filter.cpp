/*
 * filter.cpp
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#include "filter/filter.h"

int VideoFilterDevice::getFrameSize() const
{
	return img_width * img_height * (img_bpp >> 3);
}

int VideoFilterDevice::getBpp() const
{
	return img_bpp;
}

void VideoFilterDevice::setBpp(int bpp)
{
	img_bpp = bpp;
}

int VideoFilterDevice::getWidth() const
{
	return img_width;
}

void VideoFilterDevice::setWidth(int w)
{
	img_width = w;
}

int VideoFilterDevice::getHeight() const
{
	return img_height;
}

void VideoFilterDevice::setHeight(int h)
{
	img_height = h;
}

char* VideoFilterDevice::getVideoInBuffer()
{
	return video_in;
}

char* VideoFilterDevice::getVideoOutBuffer()
{
	return video_out;
}
