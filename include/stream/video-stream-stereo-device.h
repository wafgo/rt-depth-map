/*
 * video-stream-device.h
 *
 *  Created on: 02.04.2017
 *      Author: sefo
 */

#ifndef INCLUDE_VIDEO_STREAM_DEVICE_H_
#define INCLUDE_VIDEO_STREAM_DEVICE_H_

struct videoStreamBuffer {
	char* data;
	int len;
};

class VideoStreamStereoDevice
{
public:
    explicit VideoStreamStereoDevice();

    int getFrameSize() const;
    int getWidth() const;
    void setWidth(int value);

    int getHeight() const;
    void setHeight(int value);

    void getBuffers(struct videoStreamBuffer* left, struct videoStreamBuffer* right);

    virtual int grabOneFrame() = 0;
    virtual int connectToDevice() = 0;
protected:
    int width;
    int height;
    char* buffer_right;
    char* buffer_left;
    int buffer_right_len;
    int buffer_left_len;
};


#endif /* INCLUDE_VIDEO_STREAM_DEVICE_H_ */
