// SPDX-License-Identifier: GPL-2.0
/*
 * @brief Initial version of real-time audio driver for rpi
 * @copyright 2017-2021 Modern Ancient Instruments Networked AB, dba Elk,
 * Stockholm
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include <rtdm/driver.h>
#include <rtdm/rtdm.h>
#include <linux/ipipe.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include <linux/ipipe_domain.h>

/*driver specific header*/
#include "../include/version.h"
#include "../include/audio_rtdm.h"
#include "../include/audio_rtdm_spi.h"
#include "../audio-control-protocol/include/audio_control_protocol/device_control_protocol.h"
#include "../audio-control-protocol/include/audio_control_protocol/device_packet_helper.h"
#include "../audio-control-protocol/include/audio_control_protocol/audio_control_protocol.h"

#define NUM_INPUT_CHANNELS 4
#define NUM_OUTPUT_CHANNELS 4
#define NUM_CODEC_CHANNELS 4
#define DEFAULT_AUDIO_BUFFER_SIZE 32
#define AUDIO_SAMPLING_RATE_HZ 48000
#define NUM_AUDIO_BUFFER_SETS 2
#define NUM_VERSION_CHECK_RETRIES 5
#define CODEC_FORMAT INT24_LJ
#define PLATFORM_TYPE SYNC_WITH_UC_AUDIO
#define SUPPORTED_BUFFER_SIZES 32, 64, 128
#define AUDIO_DEVICE_NAME "audio_rtdm"
#define FW_DEVICE_NAME "spi_fw_transfer"
#define WAIT_FOR_XMOS_BOOT_MS 500
#define TX_DEV_PKT_NUM_INTERRUPTS_TO_WAIT 100

MODULE_DESCRIPTION("RTDM audio driver for RPI Mini");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nitin Kulkarni & Sharan Yagneswar");
MODULE_VERSION("0.2.0");

/**
 * initialize module parameters
 */
static uint audio_buffer_size = DEFAULT_AUDIO_BUFFER_SIZE;
module_param(audio_buffer_size, uint, 0644);

static int session_under_runs = 0;
module_param(session_under_runs, int, 0644);

static char *audio_hat = "elkpi-stereo";
module_param(audio_hat, charp, 0644);

/**
 * Module constants - defines the charecteristics of the underlying board
 */
static const unsigned long audio_sampling_rate = AUDIO_SAMPLING_RATE_HZ;
static const uint audio_rtdm_ver_maj = ELKPI_MINI_AUDIO_RTDM_VERSION_MAJ;
static const uint audio_rtdm_ver_min = ELKPI_MINI_AUDIO_RTDM_VERSION_MIN;
static const uint audio_rtdm_ver_rev = ELKPI_MINI_AUDIO_RTDM_VERSION_REV;
static const uint audio_input_channels = NUM_INPUT_CHANNELS;
static const uint audio_output_channels = NUM_OUTPUT_CHANNELS;
static const uint audio_format = CODEC_FORMAT;
static const uint platform_type = PLATFORM_TYPE;
static const int supported_buffer_sizes[] = { SUPPORTED_BUFFER_SIZES };
static unsigned buffer_idx = 0;
static unsigned long user_proc_completions = 0;
static unsigned long kernel_interrupts = 0;
static uint8_t board_version = 1;
static uint codec_input_gain_10dB = 0;
static uint input_gain_changed_flag = 0;

/**
 * Global variables
 */

static bool device_active;

static bool version_check_success;

struct audio_dev_context {
	rtdm_event_t irq_event;
	ipipe_spinlock_t lock;
	uint64_t periodic_wake_up_ns;
	uint64_t period_ns;
	int delta_ns;
	rtdm_task_t *audio_task;
	bool transfer_in_progress;
	struct audio_spi_dev *audio_spidev;
};

static struct audio_spi_dev *audio_spidev_drv_data;

static ssize_t audio_buffer_size_show(struct class *cls,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_buffer_size);
}

static ssize_t audio_buffer_size_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t size)
{
	unsigned long bs;
	ssize_t result;
	result = sscanf(buf, "%lu", &bs);
	if (result != 1)
		return -EINVAL;
	audio_buffer_size = bs;
	return size;
}

static ssize_t audio_hat_show(struct class *cls, struct class_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%s\n", audio_hat);
}

static ssize_t audio_sampling_rate_show(struct class *cls,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", audio_sampling_rate);
}

static ssize_t audio_rtdm_ver_maj_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_rtdm_ver_maj);
}

static ssize_t audio_rtdm_ver_min_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_rtdm_ver_min);
}

static ssize_t audio_rtdm_ver_rev_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_rtdm_ver_rev);
}

static ssize_t audio_input_channels_show(struct class *cls,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", audio_input_channels);
}

static ssize_t audio_output_channels_show(struct class *cls,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", audio_output_channels);
}

static ssize_t audio_format_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_format);
}

static ssize_t platform_type_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", platform_type);
}

static ssize_t board_version_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", board_version);
}

static ssize_t codec_input_gain_10dB_show(struct class *cls,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", codec_input_gain_10dB);
}


static ssize_t codec_input_gain_10dB_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t size)
{
	unsigned long ig;
	ssize_t result;
	result = sscanf(buf, "%lu", &ig);
	if (result != 1)
		return -EINVAL;

	// Check to prevent input gain from being set for board version 1
	if (board_version == 1 && ig == 1) {
		printk(KERN_ERR "audio_rtdm : error : Cannot boost codec input"
				" gain by 10dB. Its not supported in board"
				" version %d", board_version);
		return -EINVAL;
	}

	if (ig == 1) {
		printk(KERN_INFO "Enabling 10dB boost to codec input gain");
	}
	else {
		printk(KERN_INFO "Disabling 10dB boost to codec input gain");
	}

	codec_input_gain_10dB = ig;

	// set a flag so that interrupt handler knows that the val has changed
	input_gain_changed_flag = 1;
	return size;
}

static CLASS_ATTR_RW(audio_buffer_size);
static CLASS_ATTR_RO(audio_hat);
static CLASS_ATTR_RO(audio_sampling_rate);
static CLASS_ATTR_RO(audio_rtdm_ver_maj);
static CLASS_ATTR_RO(audio_rtdm_ver_min);
static CLASS_ATTR_RO(audio_rtdm_ver_rev);
static CLASS_ATTR_RO(audio_input_channels);
static CLASS_ATTR_RO(audio_output_channels);
static CLASS_ATTR_RO(audio_format);
static CLASS_ATTR_RO(platform_type);
static CLASS_ATTR_RW(codec_input_gain_10dB);
static CLASS_ATTR_RO(board_version);

static struct attribute *audio_rtdm_class_attrs[] = {
	&class_attr_audio_buffer_size.attr,
	&class_attr_audio_hat.attr,
	&class_attr_audio_sampling_rate.attr,
	&class_attr_audio_rtdm_ver_maj.attr,
	&class_attr_audio_rtdm_ver_min.attr,
	&class_attr_audio_rtdm_ver_rev.attr,
	&class_attr_audio_input_channels.attr,
	&class_attr_audio_output_channels.attr,
	&class_attr_audio_format.attr,
	&class_attr_platform_type.attr,
	&class_attr_codec_input_gain_10dB.attr,
	&class_attr_board_version.attr,
	NULL,
};
ATTRIBUTE_GROUPS(audio_rtdm_class);

struct class audio_rtdm_class = {
	.name = "audio_rtdm",
	.class_groups = audio_rtdm_class_groups,
};

/**
 * @brief Verifies the firmware of xmos
 */
static void verify_xmos_firmware(struct audio_spi_dev *spi_dev)
{
	struct device_ctrl_pkt *device_pkt_tx =
		(struct device_ctrl_pkt *)spi_dev->tx_buf[0];
	struct device_ctrl_pkt *device_pkt_rx =
		(struct device_ctrl_pkt *)spi_dev->rx_buf[0];
	int num_retries = 0;
	bool received_reply = false;

	version_check_success = false;
	device_active = false;
	prepare_version_check_query_pkt(device_pkt_tx);
	clear_device_ctrl_pkt(device_pkt_rx);

	// Send version check packet repeatedly to xmos.
	while (num_retries < NUM_VERSION_CHECK_RETRIES && !received_reply) {
		clear_device_ctrl_pkt(device_pkt_rx);
		audio_spi_start_transfer(spi_dev->spi_device, spi_dev->msg[0]);
		msleep(1);
		if (check_device_pkt_for_magic_words(device_pkt_rx)) {
			device_active = true;
			if (check_for_version_check_cmd(device_pkt_rx)) {
				received_reply = true;
				break;
			}
		}
		num_retries++;
		msleep(10);
	}

	if (!device_active) {
		printk("audio_rtdm : error : No response from device!");
		return;
	}

	// test with a time out count and return a specific error code for xmos not running and xmos different version
	if (!check_if_fw_vers_matches(device_pkt_rx, XMOS_FIRMWARE_VERSION_MAJ,
				      XMOS_FIRMWARE_VERSION_MIN)) {
		printk(KERN_ERR "audio_rtdm : error : xmos version mismatch, "
				"expected %d.%d.x\n",
			XMOS_FIRMWARE_VERSION_MAJ, XMOS_FIRMWARE_VERSION_MIN);
		return;
	}

	board_version = get_board_vers(device_pkt_rx);

	printk(KERN_INFO "audio_rtdm : xmos firmware verified : %d.%d.x.\n",
		XMOS_FIRMWARE_VERSION_MAJ, XMOS_FIRMWARE_VERSION_MIN);

	printk(KERN_INFO "audio_rtdm : Board version : %d\n", board_version);

	version_check_success = true;

	clear_device_ctrl_pkt(device_pkt_rx);
	return;
}

/**
 * @brief Sends a start message audio control packet by writing to the
 *        initial tx packet address and then triggering the mu interrupt. Also
 *        sends buffer size as payload
 *
 */
static void send_start_message_to_xmos(struct audio_spi_dev *audio_spidev)
{
	struct device_ctrl_pkt *tx_pkt =
		(struct device_ctrl_pkt *)audio_spidev->tx_buf[0];

	prepare_start_cmd_pkt(tx_pkt, audio_buffer_size);
	audio_spi_start_transfer(audio_spidev->spi_device,
				 audio_spidev->msg[0]);
	clear_device_ctrl_pkt(tx_pkt);
}

/**
 * @brief Sends a stop message audio control packet by writing to the
 *        initial tx packet address and then triggering the mu interrupt
 *
 */
static void send_stop_message_to_xmos(struct audio_spi_dev *audio_spidev)
{
	struct device_ctrl_pkt *tx_pkt =
		(struct device_ctrl_pkt *)audio_spidev->tx_buf[0];
	prepare_stop_cmd_pkt(tx_pkt);
	audio_spi_start_transfer(audio_spidev->spi_device,
				 audio_spidev->msg[0]);
	clear_device_ctrl_pkt(tx_pkt);
}

/**
 * @brief Helper to generate any runtime tx device control packets if
 *        required. If there are none, then the memory location is cleared.
 *
 * @param tx_pkt pointer to a device packet where a new packet will be filled.
 */
static void get_runtime_tx_device_ctrl_pkt(struct device_ctrl_pkt *tx_pkt)
{
	if (kernel_interrupts < TX_DEV_PKT_NUM_INTERRUPTS_TO_WAIT) {
		return;
	}

	if (input_gain_changed_flag) {
		if (codec_input_gain_10dB) {
			prepare_enable_input_gain_cmd_pkt(tx_pkt);
		}
		else {
			prepare_disable_input_gain_cmd_pkt(tx_pkt);
		}

		input_gain_changed_flag = 0;
		return;
	}

	// cleared if nothing is to be sent
	clear_device_ctrl_pkt(tx_pkt);
}

static void audio_intr_handler(void *ctx)
{
	unsigned long flags;
	struct audio_dev_context *dev_context = ctx;
	int err;
	uint64_t next_wake_up_ns;
	struct spi_device *spi = dev_context->audio_spidev->spi_device;
	struct audio_spi_dev *audio_spidev = dev_context->audio_spidev;
	struct device_ctrl_pkt *tx_device_pkt;

	dev_context->periodic_wake_up_ns = rtdm_clock_read_monotonic();

	while (!rtdm_task_should_stop()) {
		audio_spi_start_transfer(spi, audio_spidev->msg[buffer_idx]);

		raw_spin_lock_irqsave(&dev_context->lock, flags);
		buffer_idx = ~(buffer_idx)&0x1;
		kernel_interrupts++;
		raw_spin_unlock_irqrestore(&dev_context->lock, flags);

		rtdm_event_signal(&dev_context->irq_event);

		/* generate any required tx device packets before starting
		a new transfer */
		tx_device_pkt = (struct device_ctrl_pkt *)
					audio_spidev->tx_buf[buffer_idx];

		get_runtime_tx_device_ctrl_pkt(tx_device_pkt);

		dev_context->periodic_wake_up_ns += dev_context->period_ns;
		next_wake_up_ns = dev_context->periodic_wake_up_ns +
				  dev_context->delta_ns;
		if (dev_context->delta_ns)
			dev_context->periodic_wake_up_ns = next_wake_up_ns;

		err = rtdm_task_sleep_abs(next_wake_up_ns,
					  RTDM_TIMERMODE_ABSOLUTE);
		if (err) {
			printk(KERN_ERR
			       "rtdm_task_sleep_abs failed Error code: %d\n",
			       err);
			break;
		}
	}
}

static int audio_driver_open(struct rtdm_fd *fd, int oflags)
{
	struct audio_dev_context *dev_context =
		(struct audio_dev_context *)rtdm_fd_to_private(fd);
	int spi_num_of_words = DEVICE_CTRL_PKT_SIZE_WORDS +
				AUDIO_CTRL_PKT_SIZE_WORDS +
				(NUM_CODEC_CHANNELS * audio_buffer_size);
	int i;
	bool buffer_size_valid = false;

	// check if xmos booted
	if (!device_active) {
		return -DEVICE_INACTIVE;
	}

	// check if xmos has the right firmware
	if (!version_check_success) {
		// Error code for RASPA firmware check failed
		return -INVALID_FIRMWARE_VER;
	}

	/* set input gain changed flag on purpose so that a gain setting on the xmos
	   is refreshed */
	input_gain_changed_flag = 1;

	// check buffer size
	for (i = 0; i < sizeof(supported_buffer_sizes) / sizeof(int); i++) {
		if (audio_buffer_size == supported_buffer_sizes[i]) {
			buffer_size_valid = true;
		}
	}
	if (!buffer_size_valid) {
		printk(KERN_ERR
		       "audio_rtdm: invalid buffer size %d specified\n",
		       audio_buffer_size);
		return -INVALID_BUFFER_SIZE;
	}

	dev_context->audio_spidev = audio_spidev_drv_data;
	dev_context->periodic_wake_up_ns = 0;
	dev_context->period_ns =
		audio_buffer_size * (1000000000 / audio_sampling_rate);
	dev_context->delta_ns = 0;
	dev_context->audio_task = NULL;
	session_under_runs = 0;
	user_proc_completions = 0;
	kernel_interrupts = 0;
	buffer_idx = 0;
	if (audio_buffers_init(audio_spidev_drv_data, spi_num_of_words)) {
		printk("audio_buffers_init failed\n");
		return -1;
	}
	send_start_message_to_xmos(dev_context->audio_spidev);

	printk(KERN_INFO "audio_rtdm: buffer size = %d\n", audio_buffer_size);
	rtdm_event_init(&dev_context->irq_event, 0);

	return 0;
}

static void audio_driver_close(struct rtdm_fd *fd)
{
	struct audio_dev_context *dev_context =
		(struct audio_dev_context *)rtdm_fd_to_private(fd);
	dev_context->audio_spidev = audio_spidev_drv_data;
	if (dev_context->audio_task) {
		rtdm_event_destroy(&dev_context->irq_event);
		rtdm_task_destroy(dev_context->audio_task);
		kfree(dev_context->audio_task);
	}
	send_stop_message_to_xmos(dev_context->audio_spidev);
	printk(KERN_INFO "audio_rtdm: audio_rtdm dev closed\n");
}

static int audio_driver_mmap_nrt(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct audio_dev_context *dev_context =
		(struct audio_dev_context *)rtdm_fd_to_private(fd);
	struct spi_master *master =
		dev_context->audio_spidev->spi_device->master;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return dma_mmap_coherent(master->dma_rx->device->dev, vma,
				 dev_context->audio_spidev->rx_buf[0],
				 dev_context->audio_spidev->rx_phys_addr,
				 RESERVED_BUFFER_SIZE_IN_PAGES * PAGE_SIZE);
}

static int audio_driver_ioctl_rt(struct rtdm_fd *fd, unsigned int request,
				 void __user *arg)
{
	struct audio_dev_context *dev_context =
		(struct audio_dev_context *)rtdm_fd_to_private(fd);

	int result = 0, under_runs = 0;
	int delta_delay_ns;
	unsigned long flags;

	switch (request) {
	case AUDIO_IRQ_WAIT: {
		if ((result = rtdm_event_wait(&dev_context->irq_event)) < 0) {
			return result;
		} else {
			return buffer_idx;
		}
	} break;

	case AUDIO_PROC_START: {
		return -ENOSYS;
	}

	case AUDIO_PROC_STOP: {
		return -ENOSYS;
	}

	case AUDIO_USERPROC_FINISHED: {
		raw_spin_lock_irqsave(&dev_context->lock, flags);
		user_proc_completions++;
		under_runs = kernel_interrupts - user_proc_completions;
		if (under_runs >= 0) {
			session_under_runs += under_runs;
		}
		raw_spin_unlock_irqrestore(&dev_context->lock, flags);

		result = rtdm_safe_copy_from_user(fd, &delta_delay_ns, arg,
						  sizeof(int));

		if (result < 0) {
			raw_spin_unlock_irqrestore(&dev_context->lock, flags);
			printk(KERN_INFO
			       "audio_rtdm: AUDIO_USERPROC_FINISH failed to get user arg\n");
			return result;
		}

		dev_context->delta_ns = delta_delay_ns;

		return under_runs;
	} break;

	default:
		printk(KERN_WARNING "audio_rtdm : audio_ioctl_rt: invalid value"
				    " %d\n",
		       request);
		return -EINVAL;
	}
	return result;
}

static int audio_driver_ioctl_nrt(struct rtdm_fd *fd, unsigned int request,
				  void __user *arg)
{
	struct audio_dev_context *dev_context =
		(struct audio_dev_context *)rtdm_fd_to_private(fd);
	int result = 0;

	switch (request) {
	case AUDIO_PROC_START: {
		dev_context->audio_task =
			kcalloc(1, sizeof(rtdm_task_t), GFP_KERNEL);
		result = rtdm_task_init(dev_context->audio_task,
					"audio_driver_task", audio_intr_handler,
					dev_context, 95, 0);
		if (result) {
			printk(KERN_ERR "audio_rtdm: rtdm_task_init failed\n");
			return -EINVAL;
		}
	} break;

	case AUDIO_PROC_STOP: {
		if (dev_context->audio_task) {
			rtdm_task_destroy(dev_context->audio_task);
			rtdm_event_destroy(&dev_context->irq_event);
			kfree(dev_context->audio_task);
			dev_context->audio_task = NULL;
		}
	} break;

	default:
		return -EINVAL;
	}
	return result;
}

static struct rtdm_driver audio_driver = {

	.profile_info =
		RTDM_PROFILE_INFO(gpio, RTDM_CLASS_EXPERIMENTAL,
				  RTDM_SUBCLASS_GENERIC, RTAUDIO_PROFILE_VER),
	.device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count = 1,
	.context_size = sizeof(struct audio_dev_context),
	.ops =
		{
			.open = audio_driver_open,
			.close = audio_driver_close,
			.mmap = audio_driver_mmap_nrt,
			.ioctl_rt = audio_driver_ioctl_rt,
			.ioctl_nrt = audio_driver_ioctl_nrt,
		},
};

static struct rtdm_device audio_device = {
	.driver = &audio_driver,
	.label = AUDIO_DEVICE_NAME,
};

static int __init audio_rtdm_driver_init(void)
{
	int result;
	struct spi_master *master;
	int spi_num_of_words = DEVICE_CTRL_PKT_SIZE_WORDS +
				AUDIO_CTRL_PKT_SIZE_WORDS +
				(NUM_CODEC_CHANNELS * audio_buffer_size);

	result = class_register(&audio_rtdm_class);
	if (result < 0)
		return result;

	if (!realtime_core_enabled()) {
		printk(KERN_ERR "audio_rtdm: rt core not enabled !\n");
		return -ENODEV;
	}

	audio_spidev_drv_data = audio_spidev_init();

	if (audio_xmos_fw_transfer(audio_spidev_drv_data)) {
		printk(KERN_ERR "audio_rtdm: xmos FW transfer failed\n");
		return -1;
	}

	master = audio_spidev_drv_data->spi_device->master;
	if (audio_spi_alloc_rt_resources(master)) {
		printk(KERN_ERR "audio_rtdm: alloc rt resources failed\n");
		return -1;
	}
	if (audio_buffers_init(audio_spidev_drv_data, spi_num_of_words)) {
		printk(KERN_ERR "audio_rtdm: audio buffers init failed\n");
		return -1;
	}

	msleep(WAIT_FOR_XMOS_BOOT_MS);

	verify_xmos_firmware(audio_spidev_drv_data);

	result = rtdm_dev_register(&audio_device);
	if (result) {
		rtdm_dev_unregister(&audio_device);
		printk(KERN_INFO "audio_rtdm: audio_device init failed\n");
		return result;
	}

	printk(KERN_INFO "audio_rtdm: driver version v%d.%d.%d\n",
	       ELKPI_MINI_AUDIO_RTDM_VERSION_MAJ,
	       ELKPI_MINI_AUDIO_RTDM_VERSION_MIN,
	       ELKPI_MINI_AUDIO_RTDM_VERSION_REV);
	printk(KERN_INFO "audio_rtdm: num of ch = %d\n", NUM_CODEC_CHANNELS);
	printk(KERN_INFO "audio_rtdm: driver initialized\n");

	return 0;
}

static void __exit audio_rtdm_driver_exit(void)
{
	printk(KERN_INFO "audio_rtdm: driver exiting...\n");
	class_unregister(&audio_rtdm_class);
	rtdm_dev_unregister(&audio_device);
	audio_spi_dev_exit(audio_spidev_drv_data);
}

module_init(audio_rtdm_driver_init) module_exit(audio_rtdm_driver_exit)
