// SPDX-License-Identifier: GPL-2.0
/*
 * @brief Initial version of real-time audio driver for rpi
 * @copyright 2017-2021 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#ifndef __audio_rtdm_spi_h__
#define __audio_rtdm_spi_h__

#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#define RESERVED_BUFFER_SIZE_IN_PAGES 30

struct audio_spi_dev {
	struct dma_chan			*dma_tx;
	struct dma_chan			*dma_rx;
	void				*tx_buf[2];
	void				*rx_buf[2];
	struct spi_message 		*msg[2];
	struct spi_transfer 		*transfer[2];
	dma_addr_t			rx_phys_addr;
	dma_addr_t			tx_phys_addr;
	struct spi_device 		*spi_device;
	enum dma_status 		prev_dma_status;
};

extern struct audio_spi_dev *audio_spidev_init(void);
extern void audio_spi_dev_exit(struct audio_spi_dev *spi_dev);
extern void audio_spi_start_transfer(struct spi_device *spi,
					 struct spi_message *msg);
extern int audio_buffers_init(struct audio_spi_dev *spi_dev,
				int spi_num_of_words);
extern int audio_xmos_fw_transfer(struct audio_spi_dev *audio_spidev);
extern int audio_spi_alloc_rt_resources(struct spi_master *master);
#endif
