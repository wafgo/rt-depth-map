/*
 * mjpeg.h
 *
 *  Created on: 26.03.2017
 *      Author: wadim mueller
 */

#ifndef INCLUDE_MJPEG_H_
#define INCLUDE_MJPEG_H_

#include "decoder/decoder.h"
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <jpeglib.h>
#include <setjmp.h>

class MJPEGDecoderDevice: public DecoderDevice
{
public:
    explicit MJPEGDecoderDevice();
    ~MJPEGDecoderDevice();
    int decode(char* in, int len, int width, int height, char* out);
private:
    int mjpeg2rgb(char *in, int len, int width, int height, char *out);
    static void insert_huff_tables(j_decompress_ptr dinfo);
};

//int mjpeg2rgb(char *in, int len, int width, int height, char *out);

#endif /* INCLUDE_MJPEG_H_ */
