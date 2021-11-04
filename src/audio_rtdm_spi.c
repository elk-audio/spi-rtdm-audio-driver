// SPDX-License-Identifier: GPL-2.0
/*
 * @brief Initial version of real-time audio driver for rpi
 * @copyright 2017-2021 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#include <linux/slab.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma/bcm2835-dma.h>
#include <../drivers/spi/spi-bcm2835.h>

#include "../include/audio_rtdm_spi.h"
#include "../elkpi_stereo_xmos_firmware/elkpi_stereo_firmware.h"

#define SPI_DEFAULT_SPEED_HZ 12500000
#define SPI_NUM_OF_MSGS 2
#define SPI_BITS_PER_WORD 8
#define SPI_BUS_NUM 0
#define XMOS_RESET_GPIO_NUM 25
#define NUM_PAGES_FOR_FIRMWRE 30

static void reverse_bits_in_buffer(uint8_t *txbuf, const uint8_t *data,
				const int len)
{
	int i;
	for (i = 0; i < len; i++) {
		uint8_t b = *data;
		*txbuf++ = ((b * 0x80200802ULL) & 0x0884422110ULL) *
				0x0101010101ULL >> 32;
		data++;
	}
}

static int send_fw_to_xmos(uint8_t *txbuf, struct spi_device *spi,
			struct spi_message *msg)
{
	int res = 0;
	reverse_bits_in_buffer(txbuf, elkpi_stereo_firmware,
			elkpi_stereo_firmware_len);

	gpio_free(XMOS_RESET_GPIO_NUM);
	res = gpio_request(XMOS_RESET_GPIO_NUM, "hw_reset_gpio");
	if (res < 0) {
		printk(KERN_ERR "gpio_request: Error ");
		return res;
	}

	res = gpio_direction_output(XMOS_RESET_GPIO_NUM, 0);
	if (res < 0) {
		printk(KERN_ERR "gpio_direction_output: Error ");
		return res;
	}

	msleep(50);

	res = gpio_direction_output(XMOS_RESET_GPIO_NUM, 1);
	if (res < 0) {
		printk(KERN_ERR "gpio_direction_output: Error ");
		return res;
	}

	msleep(1);

	if (spi_sync(spi, msg)) {
		printk("spi_async: spi_async failed\n");
		return -1;
	}

	return 0;
}

static int audio_spi_map_buf(struct spi_master *master, struct device *dev,
			     struct sg_table *sgt, void *buf, size_t len,
			     enum dma_data_direction dir)
{
	const bool vmalloced_buf = is_vmalloc_addr(buf);
	unsigned int max_seg_size = dma_get_max_seg_size(dev);
	int desc_len;
	int sgs;
	struct page *vm_page;
	void *sg_buf;
	size_t min;
	int i, ret;

	if (vmalloced_buf) {
		desc_len = min_t(int, max_seg_size, PAGE_SIZE);
		sgs = DIV_ROUND_UP(len + offset_in_page(buf), desc_len);
	} else if (virt_addr_valid(buf)) {
		desc_len = min_t(int, max_seg_size, master->max_dma_len);
		sgs = DIV_ROUND_UP(len, desc_len);
	} else {
		return -EINVAL;
	}

	ret = sg_alloc_table(sgt, sgs, GFP_KERNEL);
	if (ret != 0)
		return ret;

	for (i = 0; i < sgs; i++) {
		if (vmalloced_buf) {
			min = min_t(size_t, len,
				    desc_len - offset_in_page(buf));
			vm_page = vmalloc_to_page(buf);
			if (!vm_page) {
				sg_free_table(sgt);
				return -ENOMEM;
			}
			sg_set_page(&sgt->sgl[i], vm_page, min,
				    offset_in_page(buf));
		} else {
			min = min_t(size_t, len, desc_len);
			sg_buf = buf;
			sg_set_buf(&sgt->sgl[i], sg_buf, min);
		}
		buf += min;
		len -= min;
	}

	ret = dma_map_sg(dev, sgt->sgl, sgt->nents, dir);
	if (!ret)
		ret = -ENOMEM;
	if (ret < 0) {
		sg_free_table(sgt);
		return ret;
	}

	sgt->nents = ret;

	return 0;
}

static void audio_spi_unmap_buf(struct spi_master *master, struct device *dev,
				struct sg_table *sgt,
				enum dma_data_direction dir)
{
	if (sgt->orig_nents) {
		dma_unmap_sg(dev, sgt->sgl, sgt->orig_nents, dir);
		sg_free_table(sgt);
	}
}

static int audio_spi_map_msg(struct spi_master *master, struct spi_message *msg)
{
	struct device *tx_dev, *rx_dev;
	struct spi_transfer *xfer;
	int ret;

	if (!master->can_dma)
		return -1;

	tx_dev = master->dma_tx->device->dev;
	rx_dev = master->dma_rx->device->dev;

	list_for_each_entry (xfer, &msg->transfers, transfer_list) {
		ret = audio_spi_map_buf(master, tx_dev, &xfer->tx_sg,
					(void *)xfer->tx_buf, xfer->len,
					DMA_MEM_TO_DEV);
		if (ret != 0)
			return ret;
		ret = audio_spi_map_buf(master, rx_dev, &xfer->rx_sg,
					xfer->rx_buf, xfer->len,
					DMA_DEV_TO_MEM);
		if (ret != 0) {
			audio_spi_unmap_buf(master, tx_dev, &xfer->tx_sg,
					    DMA_DEV_TO_MEM);
			return ret;
		}
	}
	master->cur_msg_mapped = true;
	return 0;
}

int audio_spi_alloc_rt_resources(struct spi_master *master)
{
	if (bcm2835_dma_alloc_rtdm_resources(master->dma_tx, DMA_MEM_TO_DEV)) {
		printk(KERN_INFO "Failed to allocate RTDM \
		resources for dma tx\n");
		return -1;
	}

	if (bcm2835_dma_alloc_rtdm_resources(master->dma_rx, DMA_DEV_TO_MEM)) {
		printk(KERN_INFO "Failed to allocate RTDM \
		resources for dma rx\n");
		return -1;
	}
	return 0;
}

static int __audio_spi_unmap_msg(struct spi_master *master,
				 struct spi_message *msg)
{
	struct spi_transfer *xfer;
	struct device *tx_dev, *rx_dev;

	master->cur_msg = NULL;
	master->cur_msg_prepared = false;
	tx_dev = master->dma_tx->device->dev;
	rx_dev = master->dma_rx->device->dev;

	list_for_each_entry (xfer, &msg->transfers, transfer_list) {
		if (!master->can_dma(master, msg->spi, xfer))
			continue;

		audio_spi_unmap_buf(master, rx_dev, &xfer->rx_sg,
				    DMA_FROM_DEVICE);
		audio_spi_unmap_buf(master, tx_dev, &xfer->tx_sg,
				    DMA_TO_DEVICE);
	}

	return 0;
}

static int audio_spi_unmap_msg(struct spi_master *master,
			       struct spi_message *msg)
{
	struct spi_transfer *xfer;

	list_for_each_entry (xfer, &msg->transfers, transfer_list) {
		if (xfer->tx_buf == master->dummy_tx)
			xfer->tx_buf = NULL;
		if (xfer->rx_buf == master->dummy_rx)
			xfer->rx_buf = NULL;
	}

	return __audio_spi_unmap_msg(master, msg);
}

void audio_spi_start_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_master *master = spi->master;
	master->cur_msg = msg;
	bcm2835_spi_transfer_one_message(master, master->cur_msg);
}

static int audio_spi_prep_messages(struct spi_device *spi,
				   struct spi_message *msg)
{
	int ret;
	struct spi_master *master = spi->master;

	if (!master->running) {
		printk("spi_rtdm: prep_messages failed, master not running\n");
		return -ESHUTDOWN;
	}

	master->cur_msg = msg;

	ret = audio_spi_map_msg(master, master->cur_msg);

	if (ret) {
		printk("spi_rtdm: prep_messages: audio_spi_map_msg failed ! \n");
		return -EINVAL;
	}
	master->prepare_message(master, msg);
	return ret;
}

static void audio_spi_complete(void *arg)
{
	/* struct spi_dev_context *dev_context = (struct spi_dev_context *) arg;
	dev_context->prev_dma_status = DMA_COMPLETE; */
}

struct audio_spi_dev *audio_spidev_init(void)
{
	int status, i;
	struct spi_master *master;
	struct audio_spi_dev *spi_dev;
	struct spi_board_info spi_device_info = {
		.modalias = "spi_rtdm_dev",
		.max_speed_hz = SPI_DEFAULT_SPEED_HZ,
		.bus_num = SPI_BUS_NUM,
		.chip_select = 0,
		.mode = 3,
	};

	master = spi_busnum_to_master(spi_device_info.bus_num);
	if (!master) {
		printk("audio_spi_init: SPI master not found\n");
		return NULL;
	}

	spi_dev = kcalloc(1, sizeof(struct audio_spi_dev), GFP_KERNEL);
	if (!spi_dev) {
		printk("audio_spi_init: failed to alloc audio_spi_dev\n");
		return NULL;
	}

	spi_dev->spi_device = spi_new_device(master, &spi_device_info);
	if (!spi_dev->spi_device) {
		printk("audio_spi_init: failed to create slave\n");
		return NULL;
	}

	spi_dev->spi_device->bits_per_word = SPI_BITS_PER_WORD;
	spi_dev->spi_device->chip_select = 0;
	master->cur_msg_mapped = false;

	status = spi_setup(spi_dev->spi_device);

	if (status) {
		printk("audio_spi_init: failed to setup slave\n");
		spi_unregister_device(spi_dev->spi_device);
		return NULL;
	}

	spi_dev->rx_buf[0] =
		dma_alloc_coherent(master->dma_rx->device->dev,
				    RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE,
				    &spi_dev->rx_phys_addr, GFP_KERNEL);

	if (!spi_dev->rx_buf[0]) {
		printk("audio_spi_init: dma_alloc_coherent rx failed\n");
		return NULL;
	}
	spi_dev->tx_buf[0] =
		dma_alloc_coherent(master->dma_tx->device->dev,
				    RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE,
				    &spi_dev->tx_phys_addr, GFP_KERNEL);
	if (!spi_dev->tx_buf[0]) {
		printk("audio_spi_init: dma_alloc_coherent tx failed\n");
		return NULL;
	}
	for (i = 0; i < SPI_NUM_OF_MSGS; i++) {
		spi_dev->msg[i] =
			kcalloc(1, sizeof(struct spi_message), GFP_KERNEL);
		if (!spi_dev->msg[i]) {
			return NULL;
		}

		spi_dev->transfer[i] =
			kcalloc(1, sizeof(struct spi_transfer), GFP_KERNEL);
		if (!spi_dev->transfer[i]) {
			return NULL;
		}
	}
	return spi_dev;
}

int audio_buffers_init(struct audio_spi_dev *spi_dev, int spi_num_of_words)
{
	int i, status;

	spi_dev->rx_buf[1] =
		spi_dev->rx_buf[0] + spi_num_of_words * sizeof(int32_t);
	for (i = 0; i < 2; i++) {
		spi_dev->tx_buf[i] =
			spi_dev->rx_buf[0] +
			(2 + i) * spi_num_of_words * sizeof(int32_t);
	}

	for (i = 0; i < SPI_NUM_OF_MSGS; i++) {
		spi_message_init(spi_dev->msg[i]);
		spi_dev->msg[i]->complete = audio_spi_complete;
		spi_dev->msg[i]->context = &spi_dev;
		spi_dev->msg[i]->spi = spi_dev->spi_device;
		spi_dev->transfer[i]->bits_per_word = SPI_BITS_PER_WORD;
		spi_dev->transfer[i]->speed_hz = SPI_DEFAULT_SPEED_HZ;
		spi_dev->transfer[i]->delay_usecs = 0;
		spi_dev->transfer[i]->tx_buf = spi_dev->tx_buf[i];
		spi_dev->transfer[i]->rx_buf = spi_dev->rx_buf[i];
		spi_dev->transfer[i]->len = sizeof(int32_t) * spi_num_of_words;

		spi_message_add_tail(spi_dev->transfer[i], spi_dev->msg[i]);

		status = audio_spi_prep_messages(spi_dev->spi_device,
						 spi_dev->msg[i]);
		if (status) {
			printk("audio_spi_init: prep_messages failed\n");
			return -1;
		}
	}
	printk("spi_num_of_words = %d\n", spi_num_of_words);
	return 0;
}

int audio_xmos_fw_transfer(struct audio_spi_dev *audio_spidev)
{
	int len, i, ret = 0;
	struct spi_message *msg;
	struct spi_transfer *transfers;
	int dlen = elkpi_stereo_firmware_len;
	struct spi_device *spi = audio_spidev->spi_device;
	void *tx_buf = audio_spidev->tx_buf[0];
	void *rx_buf = audio_spidev->rx_buf[0];
	struct spi_master *master = audio_spidev->spi_device->master;
	int num_transfers = DIV_ROUND_UP(dlen, PAGE_SIZE);

	printk("SPI_FIRMWARE_TRANSFER: dlen = %d num_transfers = %d\n", dlen,
	       num_transfers);

	msg = kcalloc(1, sizeof(struct spi_message), GFP_KERNEL);
	if (!msg) {
		printk("!message");
		return -1;
	}
	transfers = kcalloc(NUM_PAGES_FOR_FIRMWRE, sizeof(struct spi_transfer),
			    GFP_KERNEL);
	if (!transfers) {
		printk("!transfers");
		return -1;
	}

	spi_message_init(msg);

	for (i = 0; i < num_transfers; i++) {
		transfers[i].bits_per_word = SPI_BITS_PER_WORD;
		transfers[i].speed_hz = SPI_DEFAULT_SPEED_HZ;
		transfers[i].delay_usecs = 0;
		transfers[i].tx_buf = tx_buf + i * PAGE_SIZE;
		transfers[i].rx_buf = rx_buf + i * PAGE_SIZE;
		len = (dlen - i * PAGE_SIZE) >= PAGE_SIZE ?
			      PAGE_SIZE :
			      (dlen - i * PAGE_SIZE);
		transfers[i].len = len;
		spi_message_add_tail(&transfers[i], msg);
	}

	ret = send_fw_to_xmos((uint8_t *)tx_buf, spi, msg);
	if (ret) {
		printk(KERN_ERR "Sending fw to xmos failed\n");
		return ret;
	}
	kfree(msg);
	kfree(transfers);
	dma_free_coherent(master->dma_tx->device->dev,
			  RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE,
			  audio_spidev->tx_buf[0], audio_spidev->tx_phys_addr);
	return ret;
}

static void audio_spi_dev_unmap_msg(struct audio_spi_dev *spi_dev)
{
	int i;
	struct spi_master *master = spi_dev->spi_device->master;
	for (i = 0; i < SPI_NUM_OF_MSGS; i++) {
		audio_spi_unmap_msg(master, spi_dev->msg[i]);
		kfree(spi_dev->transfer[i]);
		kfree(spi_dev->msg[i]);
	}
}

void audio_spi_dev_exit(struct audio_spi_dev *spi_dev)
{
	struct spi_master *master = spi_dev->spi_device->master;
	printk(KERN_INFO "audio_spi_exit: Deregistering spi\n");

	gpio_free(XMOS_RESET_GPIO_NUM);
	audio_spi_dev_unmap_msg(spi_dev);
	spi_unregister_device(spi_dev->spi_device);

	if (bcm2835_dma_free_rtdm_resources(master->dma_tx, DMA_MEM_TO_DEV)) {
		printk(KERN_INFO "Failed to free rtdm dma resources\n");
	}
	if (bcm2835_dma_free_rtdm_resources(master->dma_rx, DMA_DEV_TO_MEM)) {
		printk(KERN_INFO "Failed to free rtdm dma resources\n");
	}
	dma_free_coherent(master->dma_rx->device->dev,
			  RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE,
			  spi_dev->rx_buf[0], spi_dev->rx_phys_addr);
}
