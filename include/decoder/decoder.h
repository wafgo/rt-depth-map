/*
 * decoder.h
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */


#include "opencv2/opencv.hpp"

class DecoderDevice
{
public:
    explicit DecoderDevice();

    virtual int decode(char* in, int len, int width, int height, char* out) = 0;
};
