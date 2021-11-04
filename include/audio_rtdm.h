// SPDX-License-Identifier: GPL-2.0
/*
 * @copyright 2017-2021 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#ifndef AUDIO_RTDM_H
#define AUDIO_RTDM_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define AUDIO_IOC_MAGIC		'r'

#define RTAUDIO_PROFILE_VER	1

/* ioctl request to wait on dma callback */
#define AUDIO_IRQ_WAIT			_IO(AUDIO_IOC_MAGIC, 1)
/* This ioctl not used anymore but kept for backwards compatibility */
#define AUDIO_IMMEDIATE_SEND		_IOW(AUDIO_IOC_MAGIC, 2, int)
/* ioctl request to start receiving audio callbacks */
#define AUDIO_PROC_START		_IO(AUDIO_IOC_MAGIC, 3)
/* ioctl to inform the driver the user space process has completed */
#define AUDIO_USERPROC_FINISHED		_IOW(AUDIO_IOC_MAGIC, 4, int)
/* ioctl to stop receiving audio callbacks */
#define AUDIO_PROC_STOP			_IO(AUDIO_IOC_MAGIC, 5)
/* ioctl for xmos firmware transfer over spi */
#define AUDIO_FW_TRANSFER		_IO(AUDIO_IOC_MAGIC, 6)

enum codec_sample_format {
	INT24_LJ = 1, // 24 bit samples left justified. Format : 0xXXXXXX00
	INT24_I2S, // 24 bit samples I2S format..
	INT24_RJ, // 24 bit samples right justified. Format : 0x00XXXXXX
	INT24_32RJ, // 24 bit samples converted into 32 bit samples
	INT32, // 32 bit samples
};

enum platform_type {
	NATIVE_AUDIO = 1,
	SYNC_WITH_UC_AUDIO,
	ASYNC_WITH_UC_AUDIO,
};

// Special error codes for xmos related issues.
enum device_error_codes {
	DEVICE_INACTIVE = 140,
	INVALID_FIRMWARE_VER,
	INVALID_BUFFER_SIZE,
};

#endif
