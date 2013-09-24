/*
 * Copyright 2013 Con Kolivas
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include "miner.h"
#include "driver-bitfury.h"

struct device_drv bitfury_drv;

static void bitfury_open(struct cgpu_info *bitfury)
{
	/* Magic open sequence */
	usb_transfer(bitfury, 0x21, 0x22, 0x0003, 0, C_BFO_OPEN);
}

static void bitfury_close(struct cgpu_info *bitfury)
{
	/* Magic close sequence */
	usb_transfer(bitfury, 0x21, 0x22, 0, 0, C_BFO_CLOSE);
}

static bool bitfury_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *bitfury;
	char buf[512];
	int amount;

	bitfury = usb_alloc_cgpu(&bitfury_drv, 1);

	if (!usb_init(bitfury, dev, found)) {
		bitfury = usb_free_cgpu(bitfury);
		goto out;
	}
	applog(LOG_INFO, "%s%d: Found at %s", bitfury->drv->name,
	       bitfury->device_id, bitfury->device_path);

	usb_buffer_enable(bitfury);
	bitfury_open(bitfury);
	usb_write(bitfury, "I", 1, &amount, C_BFO_REQINFO);
	usb_read(bitfury, buf, 14, &amount, C_BFO_GETINFO);
	if (amount != 14) {
		applog(LOG_WARNING, "%s%d: Getinfo received %d",
		       bitfury->drv->name, bitfury->device_id, amount);
		goto out_close;
	}
	applog(LOG_INFO, "%s%d: Getinfo returned %s", bitfury->drv->name,
	       bitfury->device_id, buf);

	//return true;
out_close:
	bitfury_close(bitfury);
out:
	return false;
}

static void bitfury_detect(void)
{
	usb_detect(&bitfury_drv, bitfury_detect_one);
}

static bool bitfury_prepare(struct thr_info __maybe_unused *thr)
{
	return false;
}

static bool bitfury_fill(struct cgpu_info __maybe_unused *bitfury)
{
	return true;
}

static int64_t bitfury_scanhash(struct thr_info __maybe_unused *thr)
{
	return 0;
}

static void bitfury_flush_work(struct cgpu_info __maybe_unused *bitfury)
{
}

static struct api_data *bitfury_api_stats(struct cgpu_info __maybe_unused *cgpu)
{
	return NULL;
}

static void get_bitfury_statline_before(char __maybe_unused *buf, size_t __maybe_unused bufsiz,
					struct cgpu_info __maybe_unused *bitfury)
{
}

static void bitfury_init(struct cgpu_info __maybe_unused *bitfury)
{
}

static void bitfury_shutdown(struct thr_info __maybe_unused *thr)
{
	struct cgpu_info *bitfury = thr->cgpu;

	bitfury_close(bitfury);
}

/* Currently hardcoded to BF1 devices */
struct device_drv bitfury_drv = {
	.drv_id = DRIVER_BITFURY,
	.dname = "bitfury",
	.name = "BFO",
	.drv_detect = bitfury_detect,
	.thread_prepare = bitfury_prepare,
	.hash_work = hash_queued_work,
	.queue_full = bitfury_fill,
	.scanwork = bitfury_scanhash,
	.flush_work = bitfury_flush_work,
	.get_api_stats = bitfury_api_stats,
	.get_statline_before = get_bitfury_statline_before,
	.reinit_device = bitfury_init,
	.thread_shutdown = bitfury_shutdown
};