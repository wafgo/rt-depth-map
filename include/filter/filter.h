/*
 * filter.h
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#ifndef INCLUDE_FILTER_FILTER_H_
#define INCLUDE_FILTER_FILTER_H_

#include "opencv2/opencv.hpp"

class VideoFilterDevice
{
public:
    int getFrameSize() const;

    int getBpp() const;
    void setBpp(int bpp);

    int getWidth() const;
    void setWidth(int value);

    int getHeight() const;
    void setHeight(int value);

    char* getVideoInBuffer();
    char* getVideoOutBuffer();
    virtual int run(cv::InputArray in, cv::OutputArray out) = 0;
protected:
    int img_width, img_height, img_bpp;
    char* video_in;
    char* video_out;
};


#endif /* INCLUDE_FILTER_FILTER_H_ */
