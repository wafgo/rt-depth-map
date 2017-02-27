/*
 * streaming_source.h
 *
 *  Created on: 06.01.2017
 *      Author: sefo
 */

#ifndef INCLUDE_STREAMING_SOURCE_H_
#define INCLUDE_STREAMING_SOURCE_H_

#include <stdint.h>

struct video_streaming_device;

struct video_streaming_ops {
	void (*init)(struct video_streaming_device*);
	int (*decode)(struct video_streaming_device*, uint32_t);
	void (*connect)(struct video_streaming_device*);
	void* (*get_one_frame)(struct video_streaming_device*);
	uint32_t (*get_frame_width)(struct video_streaming_device*);
	uint32_t (*get_frame_height)(struct video_streaming_device*);
	uint32_t (*get_frame_bpp)(struct video_streaming_device*);
	int (*start_streaming)(struct video_streaming_device*);
	int (*stop_streaming)(struct video_streaming_device*);
};

struct video_streaming_device {
	struct video_streaming_ops* ops;
	char device_name[32];

	void* priv;
};

static inline uint32_t video_streaming_get_frame_width(struct video_streaming_device* sdev)
{
	if (sdev->ops->get_frame_width)
		return sdev->ops->get_frame_width(sdev);
	return 0;
}

static inline uint32_t video_streaming_get_frame_height(struct video_streaming_device* sdev)
{
	if (sdev->ops->get_frame_height)
		return sdev->ops->get_frame_height(sdev);
	return 0;
}

static inline uint32_t video_streaming_get_frame_bpp(struct video_streaming_device* sdev)
{
	if (sdev->ops->get_frame_bpp)
		return sdev->ops->get_frame_bpp(sdev);
	return 0;
}
static inline void video_streaming_start(struct video_streaming_device* sdev)
{
	/* dont need to check, already done during registration */
	sdev->ops->start_streaming(sdev);
}

static inline void video_streaming_stop(struct video_streaming_device* sdev)
{
	/* dont need to check, already done during registration */
	sdev->ops->stop_streaming(sdev);
}

uint32_t video_streaming_get_frame_size(struct video_streaming_device* dev);
struct video_streaming_device* video_streaming_get_device_by_index(int idx);
struct video_streaming_device* video_streaming_get_device_by_name(char* name);
int video_streaming_get_device_name_by_index(char** name, int idx);
int register_video_streaming_source(struct video_streaming_device* device);

#endif /* INCLUDE_STREAMING_SOURCE_H_ */
