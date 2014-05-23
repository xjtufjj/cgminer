/*
 * Copyright 2014 Jinjun feng <xjtufjj@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

/*
 * These part code is leveraged from BitmineA1 board and icarus board. 
 * Btcmine-be200 baord will use the USE uart as the basic connect device.
 * The following is the commands used for communation between the btcmine Be200 
 * and the cgminer:
 * please check the github for the document 
 * Assumption: 
 * 1. Use the PL2303 as the basic USBUART chip, will add other serial-2-USB chip support 
 */


#include <float.h>
#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>
#include <sys/time.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include "logging.h"
#include "miner.h"
#include "util.h"
#include "config.h"

#ifdef WIN32
#include <windows.h>
#endif

#include "compat.h"
#include "miner.h"
#include "usbutils.h"

#define BTCMINE_BE200_IO_SPEED 115200

/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
#define DISABLE_CHIP_FAIL_THRESHOLD 3
#define BTCMINE_BE200_MAGIC "AMBE200"
enum BTCMINE_BE200_Command {

    BTCMINE_BE200_SELF_TEST = 0x01,
    BTCMINE_BE200_SELF_TEST_RSP = 0x81,

    BTCMINE_BE200_RESET_CHIP = 0x02,
    BTCMINE_BE200_RESET_CHIP_RSP = 0x82,

    BTCMINE_BE200_TWEAK_CHIP = 0x03,
    BTCMINE_BE200_TWEAK_CHIP_RSP = 0x83,

    BTCMINE_BE200_SEND_CLEAN_JOB = 0x04,
    BTCMINE_BE200_SEND_CLEAN_JOB_RSP = 0x84,

    BTCMINE_BE200_SEND_PRIMTIVE_JOB = 0x05,
    BTCMINE_BE200_SEND_PRIMTIVE_JOB_RSP = 0x85,

    BTCMINE_BE200_GET_NONCE = 0x06,
    BTCMINE_BE200_GET_NONCE_RSP = 0x86,

    BTCMINE_BE200_DETECT = 0x07,
    BTCMINE_BE200_DETECT_RSP = 0x87,

    BTCMINE_BE200_IDLE_CHIP = 0x08,
    BTCMINE_BE200_IDLE_CHIP_RSP = 0x88,

};

struct be200_chip {
	int num_cores;
	int last_queued_id;
	struct work *work[4];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;
};

#define AM_BE200_DATALEN    44
#define AM_BE200_DIFF_LEN   4
#define AM_BE200_CMD_OVERHEAD   2
#define MAX_CMD_LENGTH (AM_BE200_DATALEN + AM_BE200_DIFF_LEN + AM_BE200_CMD_OVERHEAD)

struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

struct be200_chain {
	enum sub_ident ident;
	int intinfo;

	struct cgpu_info *cgpu;
	int num_chips;
	int num_cores;
	int num_active_chips;
	int chain_skew;
	uint8_t txbuf[MAX_CMD_LENGTH];
	uint8_t rxbuf[MAX_CMD_LENGTH];
	struct be200_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;

	uint8_t temp;
	int last_temp_time;
};

static int option_offset = -1;

/*********************************************************************************
 *    USB  read write wrapper region
 * ********************************************************************************/
#define BE200_USB_OPERATION_TIMEOUT -2
#define BE200_USB_OPERATION_ERROR -1
#define BE200_USB_OPERATION_OK 0
static int usb_read_wrapper(struct cgpu_info *btcmine_be200, unsigned char *buf, size_t readsize)
{
	struct be200_chain *info = (struct be200_chain *)(btcmine_be200->device_data);
	int err, amt;

	if (btcmine_be200->usbinfo.nodev)
		return BE200_USB_OPERATION_ERROR;

	err = usb_read_ii_timeout_cancellable(btcmine_be200, info->intinfo, (char *)buf,
					      readsize, &amt, 100, C_GETRESULTS);

	if (err < 0 && err != LIBUSB_ERROR_TIMEOUT) {
		applog(LOG_ERR, "%s%i: Comms error (rerr=%d amt=%d)", btcmine_be200->drv->name,
		       btcmine_be200->device_id, err, amt);
		dev_error(btcmine_be200, REASON_DEV_COMMS_ERROR);
		return BE200_USB_OPERATION_ERROR;
	}

	if (amt >= readsize)
		return BE200_USB_OPERATION_OK;

	if (amt > 0)
		applog(LOG_DEBUG, "Btcmine Read: Some data read, but not all data has been read out");
	else
		applog(LOG_DEBUG, "Btcmine Read: No data");
	return BE200_USB_OPERATION_TIMEOUT;
}

static int usb_write_wrapper(struct cgpu_info *btcmine_be200, unsigned char *buf, size_t writesize)
{
    int err, amount;
	struct be200_chain *info = (struct be200_chain *)(btcmine_be200->device_data);

    err = usb_write_ii(btcmine_be200, info->intinfo,
            (char *)buf, writesize, &amount, C_SENDWORK);
	if (err != LIBUSB_SUCCESS || amount != writesize) {
		applog(LOG_ERR, "Btcmine Read: Write Data to remote fails");
        return BE200_USB_OPERATION_ERROR;
    }
    return BE200_USB_OPERATION_OK;
}

static void _transfer(struct cgpu_info *btcmine_be200, uint8_t request_type, 
        uint8_t bRequest, uint16_t wValue, uint16_t wIndex, 
        uint32_t *data, int siz, enum usb_cmds cmd)
{
	int err;

	err = usb_transfer_data(btcmine_be200, request_type, bRequest, wValue, wIndex, data, siz, cmd);
	applog(LOG_DEBUG, "%s: cgid %d %s got err %d",
			btcmine_be200->drv->name, btcmine_be200->cgminer_id,
			usb_cmdname(cmd), err);
    return;
}

#define transfer(btcmine_be200, request_type, bRequest, wValue, wIndex, cmd) \
		_transfer(btcmine_be200, request_type, bRequest, wValue, wIndex, NULL, 0, cmd)

/****************************************************************************
 *    Detect region
 ****************************************************************************/
#define BTCMINE_BE200_ASIC_CORE_NUMBER 32
static bool check_chip(struct be200_chain *btcmine_be200, int i)
{
	int chip_id = i;
	btcmine_be200->chips[i].num_cores = BTCMINE_BE200_ASIC_CORE_NUMBER;
	btcmine_be200->num_cores += btcmine_be200->chips[i].num_cores;
	applog(LOG_WARNING, "Be200 chip %d has %d cores", chip_id, btcmine_be200->chips[i].num_cores);
	return true;
}
void exit_be200_chain(struct be200_chain *btcmine_be200)
{
	if (btcmine_be200 == NULL)
		return;
	free(btcmine_be200->chips);  /*even if chips is NULL*/
	btcmine_be200->chips = NULL;
	free(btcmine_be200);
}
#define BTCMINE_BE200_ASIC_COUNT 16
static uint8_t *cmd_BTCMINE_BE200_DETECT(struct be200_chain *btcmine_be200, uint8_t *resp_len);
struct be200_chain *init_be200_chain(struct cgpu_info* cgpu)
{
	int i;
	struct be200_chain *btcmine_be200 = malloc(sizeof(*btcmine_be200));
    uint8_t rsp_len; 
    uint8_t *ret;

	assert(btcmine_be200 != NULL);

	applog(LOG_DEBUG, "Init Btcmine be200 board");
	memset(btcmine_be200, 0, sizeof(*btcmine_be200));

    /*bridge the cgpu and chain*/
    cgpu->device_data = btcmine_be200;
    btcmine_be200->cgpu = cgpu; 

    /* we can use the command now */
	ret = cmd_BTCMINE_BE200_DETECT(btcmine_be200, &rsp_len);
    if (!ret) {
		goto failure;
    }

    /*Check the output */
    if (strncmp(ret, BTCMINE_BE200_MAGIC, strlen(BTCMINE_BE200_MAGIC)) != 0) {
		goto failure;
    }
    /*
     * Get the ASIC chip count, currently will use the fixed number, will change based on the detect  TODO
     */
	btcmine_be200->num_chips = BTCMINE_BE200_ASIC_COUNT;

	applog(LOG_WARNING, "Btcmine be200 has %d be200 Asic chips", btcmine_be200->num_chips);

	btcmine_be200->chips = calloc(btcmine_be200->num_active_chips, sizeof(struct be200_chip));
	assert(btcmine_be200->chips != NULL);


	for (i = 0; i < btcmine_be200->num_active_chips; i++) {
		check_chip(btcmine_be200, i);
    }

	applog(LOG_WARNING, "found %d chips with total %d active cores",
	       btcmine_be200->num_active_chips, btcmine_be200->num_cores);

	mutex_init(&btcmine_be200->lock);
	INIT_LIST_HEAD(&btcmine_be200->active_wq.head);

	return btcmine_be200;

failure:
	exit_be200_chain(btcmine_be200);
	return NULL;
}

static struct cgpu_info *detect_btcmine_be200_inline (void)
{
    struct cgpu_info *cgpu = usb_alloc_cgpu(&btcmine_be200_drv, 1);
    assert(cgpu != NULL);

    memset(cgpu, 0, sizeof(*cgpu));
    cgpu->drv = &btcmine_be200_drv;
    cgpu->name = "Btcmine_Be200";
    cgpu->threads = 1;
    add_cgpu(cgpu);

    return cgpu;
}

static void btcmine_be200_usb_interface_init(struct cgpu_info *btcmine_be200, int baud)
{
	struct be200_chain *info = (struct be200_chain *)(btcmine_be200->device_data);
	uint16_t wValue, wIndex;
	enum sub_ident ident;
	int interface;

	if (btcmine_be200->usbinfo.nodev)
		return;

	interface = _usb_interface(btcmine_be200, info->intinfo);
	ident = usb_ident(btcmine_be200);

	switch (ident) {
		case IDENT_BE200:
			// Set Data Control
			transfer(btcmine_be200, PL2303_CTRL_OUT, PL2303_REQUEST_CTRL, PL2303_VALUE_CTRL,
				 interface, C_SETDATA);

			if (btcmine_be200->usbinfo.nodev)
				return;

			// Set Line Control
			uint32_t ica_data[2] = { PL2303_VALUE_LINE0, PL2303_VALUE_LINE1 };
			_transfer(btcmine_be200, PL2303_CTRL_OUT, PL2303_REQUEST_LINE, PL2303_VALUE_LINE,
				 interface, &ica_data[0], PL2303_VALUE_LINE_SIZE, C_SETLINE);

			if (btcmine_be200->usbinfo.nodev)
				return;

			// Vendor
			transfer(btcmine_be200, PL2303_VENDOR_OUT, PL2303_REQUEST_VENDOR, PL2303_VALUE_VENDOR,
				 interface, C_VENDOR);
			break;
		default:
			quit(1, "icarus_intialise() called with invalid %s cgid %i ident=%d",
				btcmine_be200->drv->name, btcmine_be200->cgminer_id, ident);
	}
}

static struct cgpu_info *btcmine_be200_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct be200_chain *btcmine_be200_chain;
	int baud = BTCMINE_BE200_IO_SPEED;
	struct cgpu_info *btcmine_be200;

    /*init the cgpu first */
	btcmine_be200 = detect_btcmine_be200_inline();

    /* init the cgpu usb info */
	if (!usb_init(btcmine_be200, dev, found))
		goto shin;
    /*init the end device */
    btcmine_be200_usb_interface_init(btcmine_be200, baud);
    
    /* init the be200 */
    btcmine_be200_chain = init_be200_chain(btcmine_be200);
    if (btcmine_be200_chain == NULL) {
	    applog(LOG_ERR, "Cannot init the BE200 boards");
		goto shin;
    }

    /* Init the ident */
	btcmine_be200_chain->ident = usb_ident(btcmine_be200);

    return btcmine_be200;

unshin:
	usb_uninit(btcmine_be200);
shin:
	btcmine_be200 = usb_free_cgpu(btcmine_be200);
	return NULL;
}

static void btcmine_be200_detect(bool __maybe_unused hotplug)
{
	usb_detect(&btcmine_be200_drv, btcmine_be200_detect_one);
}

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}

/********** temporary helper for hexdumping*/
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

/* BTCmine Be200 commands region  */
static uint8_t *exec_cmd(struct be200_chain *btcmine_be200,
			  uint8_t cmd, uint8_t *data, uint8_t len,
			  uint8_t *resp_len)
{   
	uint8_t tx_len;
    uint8_t rx_len;
    int err;
    
    tx_len = len + AM_BE200_CMD_OVERHEAD;
	memset(btcmine_be200->txbuf, 0, tx_len);


    /*Set the command and */
	btcmine_be200->txbuf[0] = cmd & 0x7F;
	btcmine_be200->txbuf[1] = len;

	if (data != NULL)
		memcpy(btcmine_be200->txbuf + AM_BE200_CMD_OVERHEAD, data, len);

    /* write the command into USB */
	hexdump("send: TX", btcmine_be200->txbuf, tx_len);
    err = usb_write_wrapper(btcmine_be200->cgpu, btcmine_be200->txbuf, tx_len);
    if (err != BE200_USB_OPERATION_OK) {
		applog(LOG_ERR, "Cannot write command[%d] into btcmine be200 boards", cmd);
        return NULL;
    }

    /* Try to receive from the boards */
    err = usb_read_wrapper(btcmine_be200->cgpu, btcmine_be200->rxbuf, AM_BE200_CMD_OVERHEAD);  /* Read the TL*/
    if (err != BE200_USB_OPERATION_OK) {
		applog(LOG_ERR, "Cannot read response from btcmine be200 boards for command[%d]", cmd);
        return NULL;
    }

    if (btcmine_be200->rxbuf[0] != (cmd | 0x80)) {
		applog(LOG_ERR, "Wrong command response received from be200 boards, expect[%d], real[%d]", 
                cmd | 0x80, btcmine_be200->rxbuf[0]);
        return NULL;
    }

    /*Get the len and read it again */
    rx_len = btcmine_be200->rxbuf[1];
    if (rx_len > 0) {
        err = usb_read_wrapper(btcmine_be200->cgpu, btcmine_be200->rxbuf + AM_BE200_CMD_OVERHEAD, rx_len);  /* Read the TL*/
        if (err != BE200_USB_OPERATION_OK) {
            applog(LOG_ERR, "Cannot read response from btcmine be200 boards for data, len[%d]", rx_len);
            return NULL;
        }
    }

	hexdump("send: RX", btcmine_be200->rxbuf, AM_BE200_CMD_OVERHEAD + rx_len);

    /*set the resp len */
    *resp_len =  rx_len;

	return btcmine_be200->rxbuf + AM_BE200_CMD_OVERHEAD;
}

static uint8_t *cmd_BTCMINE_BE200_SELF_TEST(struct be200_chain *btcmine_be200)
{
    uint8_t *ret;
    uint8_t resp_len;

    ret = exec_cmd(btcmine_be200, BTCMINE_BE200_SELF_TEST, NULL, 0, &resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}

static uint8_t *cmd_BTCMINE_BE200_RESET_CHIP(struct be200_chain *btcmine_be200, uint8_t *chip_id, int chip_num)
{
    uint8_t *ret;
    uint8_t resp_len;

    if (!chip_id) {
		applog(LOG_ERR, "%s: chip_id parameter is NULL", __func__);
        return NULL;
    }

    ret = exec_cmd(btcmine_be200, BTCMINE_BE200_RESET_CHIP, chip_id, chip_num, &resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}

static uint8_t *cmd_BTCMINE_BE200_TWEAK_CHIP(struct be200_chain *btcmine_be200, uint8_t chip_id, int freq)
{
    uint8_t *ret;
    uint8_t resp_len;
    uint8_t data[2];

    data[0] = chip_id;
    data[1] = freq;
    ret = exec_cmd(btcmine_be200, BTCMINE_BE200_RESET_CHIP, data, 2, &resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}

static uint8_t *cmd_BTCMINE_BE200_SEND_JOB_inline(struct be200_chain *btcmine_be200, 
        uint8_t chip_id, uint8_t job_id, 
        uint8_t *datain, uint8_t len, 
        uint8_t is_clean)
{
    uint8_t *ret;
    uint8_t resp_len;
    uint8_t data[MAX_CMD_LENGTH];
    int cmd = BTCMINE_BE200_SEND_PRIMTIVE_JOB;

    data[0] = chip_id;
    data[1] = job_id;

    memcpy(&data[2], datain, len);

    if (is_clean) {
        cmd = BTCMINE_BE200_SEND_CLEAN_JOB;
    }

    ret = exec_cmd(btcmine_be200, cmd, data, len + 2, &resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}

#define cmd_BTCMINE_BE200_SEND_CLEAN_JOB(btcmine_be200, cid, jid, data) \
    cmd_BTCMINE_BE200_SEND_JOB_inline(btcmine_be200, cid, jid, data, 44, 1)
#define cmd_BTCMINE_BE200_SEND_PRIMTIVE_JOB(btcmine_be200, cid, jid, data) \
    cmd_BTCMINE_BE200_SEND_JOB_inline(btcmine_be200, cid, jid, data, 44, 0)

static uint8_t *cmd_BTCMINE_BE200_GET_NONCE(struct be200_chain *btcmine_be200)
{
    uint8_t *ret;
    uint8_t resp_len;

    ret = exec_cmd(btcmine_be200, BTCMINE_BE200_GET_NONCE, NULL, 0, &resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}

static uint8_t *cmd_BTCMINE_BE200_IDLE_CHIP(struct be200_chain *btcmine_be200, uint8_t *resp_len)
{
    uint8_t *ret;

    ret = exec_cmd(btcmine_be200, BTCMINE_BE200_IDLE_CHIP, NULL, 0, resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}


static uint8_t *cmd_BTCMINE_BE200_DETECT(struct be200_chain *btcmine_be200, uint8_t *resp_len)
{
    uint8_t *ret;

    ret = exec_cmd(btcmine_be200, BTCMINE_BE200_DETECT, NULL, 0, resp_len);
    if (!ret) {
		applog(LOG_ERR, "%s failed", __func__);
    }
    return ret;
}


/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

static bool is_chip_disabled(struct be200_chain *btcmine_be200, uint8_t chip_id)
{
	struct be200_chip *chip = &btcmine_be200->chips[chip_id - 1];
	return chip->disabled || chip->cooldown_begin != 0;
}

/* check and disable chip, remember time */
static void disable_chip(struct be200_chain *btcmine_be200, uint8_t chip_id)
{
	struct be200_chip *chip = &btcmine_be200->chips[chip_id - 1];
	if (is_chip_disabled(btcmine_be200, chip_id)) {
		applog(LOG_WARNING, "chip %d already disabled", chip_id);
		return;
	}
	applog(LOG_WARNING, "temporary disabling chip %d", chip_id);
	chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct be200_chain *btcmine_be200)
{
	int i;
	for (i = 0; i < btcmine_be200->num_active_chips; i++) {
		int chip_id = i + 1;
		struct be200_chip *chip = &btcmine_be200->chips[i];
		if (!is_chip_disabled(btcmine_be200, chip_id))
			continue;
		/* do not re-enable fully disabled chips */
		if (chip->disabled)
			continue;
		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
			continue;
        /*
		if (!cmd_READ_REG(btcmine_be200, chip_id)) {
			chip->fail_count++;
			applog(LOG_WARNING, "chip %d not yet working - %d",
			       chip_id, chip->fail_count);
			if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) {
				applog(LOG_WARNING,
				       "completely disabling chip %d at %d",
				       chip_id, chip->fail_count);
				chip->disabled = true;
				btcmine_be200->num_cores -= chip->num_cores;
				continue;
			}
			chip->cooldown_begin = get_current_ms();
			continue;
		}
            */
		applog(LOG_WARNING, "chip %d is working again", chip_id);
		chip->cooldown_begin = 0;
		chip->fail_count = 0;
	}
}

/********** job creation and result evaluation */
uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
	static uint8_t job[AM_BE200_DATALEN] = {
		/* midstate */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* wdata */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
	};

	uint8_t *midstate = work->midstate;
	uint8_t *wdata = work->data + 64;

	uint32_t *p1 = (uint32_t *) &job[32];
	uint32_t *p2 = (uint32_t *) wdata;

	swab256(job, midstate);

	p1[0] = bswap_32(p2[0]);
	p1[1] = bswap_32(p2[1]);
	p1[2] = bswap_32(p2[2]);

	return job;
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct be200_chain *btcmine_be200, uint8_t chip_id, struct work *work)
{
	struct be200_chip *chip = &btcmine_be200->chips[chip_id - 1];
	bool retval = false;

	int job_id = chip->last_queued_id + 1;

	applog(LOG_INFO, "queuing chip %d with job_id %d", chip_id, job_id);

	if (chip->work[chip->last_queued_id] != NULL) {
		work_completed(btcmine_be200->cgpu, chip->work[chip->last_queued_id]);
		chip->work[chip->last_queued_id] = NULL;
		retval = true;
	}
	uint8_t *jobdata = create_job(chip_id, job_id, work);
	if (!cmd_BTCMINE_BE200_SEND_CLEAN_JOB(btcmine_be200, chip_id, job_id, jobdata)) {
		/* give back work */
		work_completed(btcmine_be200->cgpu, work);

		applog(LOG_ERR, "failed to set work for chip %d.%d",
		       chip_id, job_id);
		disable_chip(btcmine_be200, chip_id);
	} else {
		chip->work[chip->last_queued_id] = work;
		chip->last_queued_id++;
		chip->last_queued_id &= 3;
	}
	return retval;
}

static bool get_nonce(struct be200_chain *btcmine_be200, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	uint8_t *ret = cmd_BTCMINE_BE200_GET_NONCE(btcmine_be200);

	if (ret == NULL)
		return false;

	*job_id = ret[1];
	*chip = ret[0];
	memcpy(nonce, ret + 2, 4);
	return true;
}

/* reset input work queues in chip chain */
static bool abort_work(struct be200_chain *btcmine_be200)
{
    int i;
    uint8_t data[MAX_CMD_LENGTH];
    int cidx = 0;

    for (i = 0; i < btcmine_be200->num_chips; i++) {
        if (is_chip_disabled(btcmine_be200, i)) {
            continue;
        }
        data[cidx++] = i;
    }
	/* drop jobs already queued: reset strategy 0xed */
	if (cmd_BTCMINE_BE200_RESET_CHIP(btcmine_be200, data, cidx) != NULL) {
        return true;
    }
    return false;
}

#define TEMP_UPDATE_INT_MS	2000
static int64_t Btcmine_Be200_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct be200_chain *btcmine_be200 = cgpu->device_data;
	int32_t nonce_ranges_processed = 0;

	applog(LOG_DEBUG, "Btcmine Be200 running scanwork");

	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;
	bool work_updated = false;
    int err;
    uint8_t *chips;
    uint8_t chip_num;

	mutex_lock(&btcmine_be200->lock);

	if (btcmine_be200->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms()) {
		btcmine_be200->last_temp_time = get_current_ms();
	}
	/* poll queued results */
	while (true) {
		if (!get_nonce(btcmine_be200, (uint8_t*)&nonce, &chip_id, &job_id))
			break;
		nonce = bswap_32(nonce);
		work_updated = true;
		if (chip_id < 1 || chip_id > btcmine_be200->num_active_chips) {
			applog(LOG_WARNING, "wrong chip_id %d", chip_id);
			continue;
		}
		if (job_id < 1 && job_id > 4) {
			applog(LOG_WARNING, "chip %d: result has wrong job_id %d", chip_id, job_id);
			continue;
		}

		struct be200_chip *chip = &btcmine_be200->chips[chip_id - 1];
		struct work *work = chip->work[job_id - 1];
		if (work == NULL) {
			/* already been flushed => stale */
			applog(LOG_WARNING, "chip %d: stale nonce 0x%08x", chip_id, nonce);
			chip->stales++;
			continue;
		}
		if (!submit_nonce(thr, work, nonce)) {
			applog(LOG_WARNING, "chip %d: invalid nonce 0x%08x", chip_id, nonce);
			chip->hw_errors++;
			/* add a penalty of a full nonce range on HW errors */
			nonce_ranges_processed--;
			continue;
		}
		applog(LOG_DEBUG, "chip %d / job_id %d: nonce 0x%08x", chip_id, job_id, nonce);
		chip->nonces_found++;
	}

	/* check for completed works */

    chips = cmd_BTCMINE_BE200_IDLE_CHIP(btcmine_be200, &chip_num);
    if (chips) {
        for (i = chip_num - 1; i >= 0; i--) {
		    uint8_t c = chips[i];
            if (is_chip_disabled(btcmine_be200, c))
                continue;
		    struct work *work;
		    struct be200_chip *chip = &btcmine_be200->chips[c]; //TODO C-1?

			work = wq_dequeue(&btcmine_be200->active_wq);
			if (work == NULL) {
				applog(LOG_ERR, "chip %d: work underflow", c);
				break;
			}
			if (set_work(btcmine_be200, c, work)) {
				chip->nonce_ranges_done++;
				nonce_ranges_processed++;
			}
			applog(LOG_DEBUG, "chip %d: job done: %d/%d/%d/%d", c,
			       chip->nonce_ranges_done, chip->nonces_found,
			       chip->hw_errors, chip->stales);
        }
    }
	check_disabled_chips(btcmine_be200);
	mutex_unlock(&btcmine_be200->lock);

	if (nonce_ranges_processed < 0)
		nonce_ranges_processed = 0;

	if (nonce_ranges_processed != 0) {
		applog(LOG_DEBUG, "nonces processed %d", nonce_ranges_processed);
	}

	return (int64_t)nonce_ranges_processed << 32;
}


/* queue two work items per chip in chain */
static bool Btcmine_Be200_queue_full(struct cgpu_info *cgpu)
{
	struct be200_chain *btcmine_be200 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&btcmine_be200->lock);
	applog(LOG_DEBUG, "Btcmine be200 running queue_full: %d/%d",
	       btcmine_be200->active_wq.num_elems, btcmine_be200->num_active_chips);

	if (btcmine_be200->active_wq.num_elems >= btcmine_be200->num_active_chips * 2)
		queue_full = true;
	else
		wq_enqueue(&btcmine_be200->active_wq, get_queued(cgpu));

	mutex_unlock(&btcmine_be200->lock);

	return queue_full;
}

static void Btcmine_Be200_flush_work(struct cgpu_info *cgpu)
{
	struct be200_chain *btcmine_be200 = cgpu->device_data;

	applog(LOG_DEBUG, "Btcmine be200 running flushwork");

	int i;

	mutex_lock(&btcmine_be200->lock);
	/* stop chips hashing current work */
	if (!abort_work(btcmine_be200)) {
		applog(LOG_ERR, "failed to abort work in chip chain!");
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < btcmine_be200->num_active_chips; i++) {
		int j;
		struct be200_chip *chip = &btcmine_be200->chips[i];
		for (j = 0; j < 4; j++) {
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "flushing chip %d, work %d: 0x%p", i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}
		chip->last_queued_id = 0;
	}
	/* flush queued work */
	applog(LOG_DEBUG, "flushing queued work...");
	while (btcmine_be200->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&btcmine_be200->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&btcmine_be200->lock);
}

struct device_drv btcmine_be200_drv = {
	.drv_id = DRIVER_btcmine_be200,
	.dname = "Btcmine_Be200",
	.name = "Btcmine_Be200",
	.drv_detect = btcmine_be200_detect,

	.hash_work = hash_queued_work,
	.scanwork = Btcmine_Be200_scanwork,
	.queue_full = Btcmine_Be200_queue_full,
	.flush_work = Btcmine_Be200_flush_work,
};
