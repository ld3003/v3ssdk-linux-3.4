/**
 * Hillcrest SensorHub driver
 *
 * Copyright (C) 2013-15 Hillcrest Labs, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinconf-sunxi.h>
#include <linux/pinctrl/consumer.h>
#include <mach/sys_config.h>

#include "sensorhub-core.h"
#include "sensorhub-i2c.h"
#include "sensorhub-gpio.h"
#include <linux/sensorhub.h> 

#include "sensorhub-gpio.h"

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define CHECK_LEN(req) \
	if (len < req) return -ETOOSMALL;

#define SENSORHUB_SYSFS_DEBUG

#define GET_REPORT_BUFSIZE (64)
#define SET_REPORT_BUFSIZE (64)
#define MAX_INPUT_REPORT_BUFSIZE (64)

 // Duration for SH-1 app to launch and be ready to serve hid desc
#define APP_STARTUP_MAX_DELAY_MS       1000
#define APP_STARTUP_CHECK_INTERVAL_MS  25

// Duration for keeping reset low per reset
#define RESET_TOGGLE_US 1000

inline struct sensorhub_i2c_data * __i2c(struct sensorhub_drv_data * _ddata) {
	return container_of(_ddata, struct sensorhub_i2c_data, ddata);
}

// Write u16 val to u8 pointer with offset *idx
inline void cpu_to_le16_u8p(u16 val, u8 * p, size_t * idx) {
	val = cpu_to_le16(val);
	*((u16 *) (p + *idx)) = val;
	*idx += 2;
}

#define set_debug_gpio(val)\
	if (ddata->pdata.gpio_debug) { \
		gpio_set_value(ddata->pdata.gpio_debug, val); \
	}

// Forward declarations:
static int sensorhub_i2c_test_and_read_locked(struct sensorhub_i2c_data * i2cdata);
/**
 * Reset the SH
 * @param  ddata     driver data
 * @param  reps      number of times to toggle reset line
 * @param  toggle_us duration to keep reset low
 * @param  rep_ms    duration to wait before repeating reset
 * @param  bootmode  gpio value for boot mode, -1 if not needed
 * @return           0 on success or -errno
 */
static int sensorhub_i2c_toggle_reset(struct sensorhub_drv_data * ddata,
									unsigned int reps,
									unsigned int toggle_us,
									unsigned int rep_ms,
									int bootmode);

static int sensorhub_i2c_poll_handshake(struct sensorhub_i2c_data * i2cdata);
static int sensorhub_i2c_init_app(struct sensorhub_i2c_data * i2cdata);

#define GPIO_SENSORHUB_IRQ 138 
#define GPIO_SENSORHUB_RESET 136 
#define GPIO_SENSORHUB_WAKEUP 139 
#define GPIO_SENSORHUB_DEBUG 140 
#define SENSORHUB_DFU_ATMEL_V1 1 
#define GPIO_SENSORHUB_GPIO_TO_IRQ 298
/* Addresses to scan */
static const unsigned short normal_i2c[] = {SENSORHUB_I2C_ADDR, I2C_CLIENT_END};
static int i2c_num = 0;
static const unsigned short i2c_address[2] = {SENSORHUB_I2C_ADDR,0x49};

static struct gpio sensorhub_gpios[] = {
	{ GPIO_SENSORHUB_IRQ,    GPIOF_IN,            "sensorhub-data"   },
	{ GPIO_SENSORHUB_RESET,  GPIOF_OUT_INIT_HIGH, "sensorhub-reset"  },
	{ GPIO_SENSORHUB_DEBUG,  GPIOF_OUT_INIT_LOW,  "sensorhub-debug"  },
	{ GPIO_SENSORHUB_WAKEUP, GPIOF_OUT_INIT_HIGH, "sensorhub-wakeup" },
};
		//-------------board info
	//#if 0
static	struct sensorhub_platform_data sensorhub_pdata = { 
		//.dfu_type = SENSORHUB_DFU_ATMEL_V1, 
		//.dfu_type = SENSORHUB_DFU_BNO_V1,//fea error
		.dfu_type = SENSORHUB_DFU_NONE,
		//.irq = OMAP_GPIO_IRQ(GPIO_SENSORHUB_IRQ), 
		.irq = GPIO_SENSORHUB_GPIO_TO_IRQ, 
		.gpio_interrupt = GPIO_SENSORHUB_IRQ, 
		.gpio_reset = GPIO_SENSORHUB_RESET, 
		.gpio_debug = GPIO_SENSORHUB_DEBUG, 
		.gpio_wakeup = GPIO_SENSORHUB_WAKEUP, 
	}; 
static	struct i2c_board_info sensorhub_i2c_board_info[] = { 
		{ 
			I2C_BOARD_INFO(SENSORHUB_I2C_ID, SENSORHUB_I2C_ADDR), 
			.platform_data = &sensorhub_pdata, 
		}, 
	};
	//#endif
	//-------------

#ifdef SENSORHUB_SYSFS_DEBUG
static const struct attribute_group sensorhub_i2c_attr_group;
#endif

#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
#define ATMEL_BL_I2C_ADDR_BASE          0x26
#define ATMEL_BL_STS_INIT_MASK          0xC0
#define ATMEL_BL_STS_WAIT_CMD_1         0xC0
#define ATMEL_BL_STS_APP_CRC_FAIL       0x40
#define ATMEL_BL_STS_WAIT_FRM_1_MASK    0x80
#define ATMEL_BL_STS_FRM_CRC_CHK_1      0x02
#define ATMEL_BL_STS_FRM_CRC_FAIL       0x03
#define ATMEL_BL_STS_FRM_CRC_PASS       0x04
#define ATMEL_BL_MAX_FRAME_SIZE          532

// Duration for bootloader to assert the chg pin
// The max is set for a timeout long enough to calculate the app crc at startup
#define BOOTLOADER_READY_MAX_DELAY_MS       100
#define BOOTLOADER_READY_CHECK_INTERVAL_MS  5

// Number of times the board has to be reset to launch bootloader
#define BOOTLOADER_RESET_REPS 10
// Bootloader execution min time
#define BOOTLOADER_CYCLE_DELAY_MS 50

// Timeout on waiting for a bootloader state change
#define DFU_STATE_CHANGE_TIMEOUT_MS 5000
// Number of times a frame is re-sent on a CRC error
#define DFU_FRAME_SEND_NUM_RETRY 2

static const u8 ATMEL_BL_UNLOCK_CMD[] = {0xDC, 0xAA};

#endif
static int                                 Log_level = DEBUG_MSG;
#define SH_MSG(format, ...)             if(DEBUG_MSG&Log_level){printk(KERN_ERR SH_TAG format "\n", ## __VA_ARGS__);}
#define SH_ERR(format, ...)             	if(DEBUG_ERR&Log_level){printk(KERN_ERR SH_TAG format "\n", ## __VA_ARGS__);}
static unsigned char 			twi_id = 0;

static int sensorhub_i2c_aux_base_write(struct sensorhub_i2c_data * i2cdata, u8 baseaddr, const u8 * buf, size_t len);
static int sensorhub_i2c_aux_base_read(struct sensorhub_i2c_data * i2cdata, u8 baseaddr, u8 * buf, size_t len);


// Prepare a command buf with a cmd addr + reset command
static int sensorhub_i2c_prepare_request_reset(struct sensorhub_i2c_data * i2cdata,
	u8 * buf, size_t len) {

	size_t i = 0;

	CHECK_LEN(4);
	cpu_to_le16_u8p(i2cdata->desc.wCommandRegister, buf, &i);
	cpu_to_le16_u8p(SH_HID_OP_CODE_RESET, buf, &i);

	return i;
}

// Prepare a command buf with a cmd addr + set power command
static int sensorhub_i2c_prepare_request_set_power(struct sensorhub_i2c_data * i2cdata,
	enum sensorhub_i2c_hid_power_state s,
	u8 * buf, size_t len) {

	size_t i = 0;

	if (s != SH_HID_PWR_ON && s != SH_HID_PWR_SLEEP) {
		return -EINVAL;
	}

	CHECK_LEN(4);
	cpu_to_le16_u8p(i2cdata->desc.wCommandRegister, buf, &i);
	cpu_to_le16_u8p(SH_HID_OP_CODE_SET_POWER | s, buf, &i);

	return i;
}

// Prepare a command buf with a cmd addr + get/set report command
static int sensorhub_i2c_prepare_request_getset(struct sensorhub_i2c_data * i2cdata,
	enum sensorhub_i2c_hid_op_code op, enum sensorhub_i2c_hid_report_type type, u8 id,
	u8 * buf, size_t len) {

	size_t i = 0;
	u16 cmd;

	if (op == SH_HID_OP_CODE_GET_REPORT) {
		if (type != SH_HID_REPORT_TYPE_INPUT && type != SH_HID_REPORT_TYPE_FEATURE) {
			return -EINVAL;
		}
	} else if (op == SH_HID_OP_CODE_SET_REPORT) {
		if (type != SH_HID_REPORT_TYPE_OUTPUT && type != SH_HID_REPORT_TYPE_FEATURE) {
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	CHECK_LEN(6);
	cpu_to_le16_u8p(i2cdata->desc.wCommandRegister, buf, &i);
	if (id < SH_HID_OP_CODE_REPORT_ID_SENTINEL) {
		cmd = op | type | id;
		cpu_to_le16_u8p(cmd, buf, &i);
	} else {
		CHECK_LEN(7);
		cmd = op | type | SH_HID_OP_CODE_REPORT_ID_SENTINEL;
		cpu_to_le16_u8p(cmd, buf, &i);
		buf[i++] = id;
	}

	cpu_to_le16_u8p(i2cdata->desc.wDataRegister, buf, &i);
	return i;
}

static int wakeup_delay_us = 25;
module_param(wakeup_delay_us, int, S_IRUGO | S_IWUSR);
static int wakeup_clear_delay_us = 25;
module_param(wakeup_clear_delay_us, int, S_IRUGO | S_IWUSR);

// Perform a write operation containing a command cmd of length cmdlen,
// followed by a read to data of length datalen
// returns 0 on success
static int sensorhub_i2c_command_read_locked(struct sensorhub_i2c_data * i2cdata,
	u8 * cmd, size_t cmdlen,
	u8 * data, size_t datalen) {

	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	struct i2c_client * const client = i2cdata->client;
	struct i2c_adapter * const adap = client->adapter;
	struct i2c_msg msg[2];
	int num_msgs = 1;
	int rc;
	bool retry;

	// print_hex_dump(KERN_DEBUG, "CMD: ", DUMP_PREFIX_OFFSET, 16, 1, cmd, cmdlen, true);
	if (ddata->pdata.gpio_wakeup) {
		gpio_set_value(ddata->pdata.gpio_wakeup, 0);
		udelay(wakeup_delay_us);
		retry = true;
	} else {
		retry = false;
	}

do_transfer:
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = cmdlen;
	msg[0].buf = cmd;

	if (data) {
#ifdef DEBUG
		memset(data, 0xcc, datalen);
#endif
		msg[1].addr = client->addr;
		msg[1].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
		msg[1].len = datalen;
		msg[1].buf = data;
		num_msgs = 2;
	}

	rc = i2c_transfer(adap, msg, num_msgs);
	if (rc < 0) {
		if (retry) {
			// dummy write and retry
			char dummy[] = {0, 0};
			++ddata->stats.retries;
			udelay(wakeup_clear_delay_us);
			msg[0].addr = client->addr;
			msg[0].flags = client->flags & I2C_M_TEN;
			msg[0].len = 2;
			msg[0].buf = dummy;
			retry = false;
			i2c_transfer(adap, msg, 1);
			goto do_transfer;
		}

		++ddata->stats.errorCount;
		goto exit;
	}

	if (rc != num_msgs) {
		++ddata->stats.errorCount;
		rc = -EIO;
		goto exit;
	}

	ddata->stats.sent++;
	ddata->stats.sentBytes += cmdlen;
	if (data) {
		ddata->stats.received++;
		ddata->stats.receivedBytes += datalen;
	}
	rc = 0;
	// fall-thru
exit:
	if (ddata->pdata.gpio_wakeup) {
		gpio_set_value(ddata->pdata.gpio_wakeup, 1);
	}
	return rc;
}

// Perform a (16-bit address) register read
static int sensorhub_i2c_reg_read_locked(struct sensorhub_i2c_data * i2cdata,
										 u16 reg, u8 * buf, size_t len) {
	reg = cpu_to_le16(reg);
	return sensorhub_i2c_command_read_locked(i2cdata, (u8 *) &reg, sizeof(reg), buf, len);
}

// Get a feature report from the device
static int sensorhub_i2c_get_report(struct sensorhub_drv_data * ddata,
						enum sensorhub_report_type type, u8 id,
						u8 * report, size_t len) {
	int ret;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	u8 command[7];
	u8 buf[GET_REPORT_BUFSIZE];
	size_t responseSize = len + 2;
#ifdef DEBUG
	memset(command, 0xaa, sizeof(command));
	memset(buf, 0xcc, sizeof(buf));
#endif

	if (type != SENSORHUB_REPORT_FEATURE) {
		// don't support get input report yet
		return -EINVAL;
	}

	if (WARN_ON(responseSize > sizeof(buf))) {
		return -ENOBUFS;
	}

	if (test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
		return -ENXIO;
	}

	ret = sensorhub_i2c_prepare_request_getset(
		i2cdata, SH_HID_OP_CODE_GET_REPORT, SH_HID_REPORT_TYPE_FEATURE, id,
		command, sizeof(command)
	);

	if (WARN_ON(ret < 0)) {
		return ret;
	}

	mutex_lock(&i2cdata->hw_lock);
	ret = sensorhub_i2c_command_read_locked(i2cdata, command, ret, buf, responseSize);
	mutex_unlock(&i2cdata->hw_lock);
	if (ret < 0) {
		dev_err(ddata->dev, "i2c transfer failed for get feature request - %d\n", ret);
		return ret;
	}

	ret = le16_to_cpup((u16 *) &buf[0]);
	if (ret < 0 || ret > 0xffff ||
		ret == 1  /* len has to be 0, or at least 2 */ ) {
		dev_dbg(ddata->dev, "invalid length received - %d\n", ret);
		++ddata->stats.errorCount;
		return -EPROTO;
	}

	if (ret == 0) {
		// hub has no knowledge of this report
		return -ENOENT;
	}

	if (ret - 2 > len) {
		// the output buffer is too small to hold the report
		return -EMSGSIZE;
	}

	// copy only the data to the output buffer
	memcpy(report, buf + 2, ret - 2);
	return ret - 2;
}

// Set a report
// type - feature/output
// id - the report id
// report - repory payload excluding id
// reportLen - length of payload
static ssize_t sensorhub_i2c_set_report(struct sensorhub_drv_data * ddata,
								 enum sensorhub_report_type type, u8 id,
								 const u8 * report, size_t reportLen) {
	int ret;
	u8 cmd[SET_REPORT_BUFSIZE];
	size_t offset = 0;
	enum sensorhub_i2c_hid_report_type _type;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

#ifdef DEBUG
	memset(cmd, 0xcc, sizeof(cmd));
#endif

	if (type == SENSORHUB_REPORT_OUTPUT) {
		_type = SH_HID_REPORT_TYPE_OUTPUT;
	} else if (type == SENSORHUB_REPORT_FEATURE) {
		_type = SH_HID_REPORT_TYPE_FEATURE;
	} else {
		return -EINVAL;
	}

	if (test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
		return -ENXIO;
	}

	ret = sensorhub_i2c_prepare_request_getset(
		i2cdata, SH_HID_OP_CODE_SET_REPORT, _type, id,
		cmd, sizeof(cmd)
	);

	if (WARN_ON(ret < 0)) {
		return ret;
	}

	offset += ret;
	if (reportLen > (sizeof(cmd) - offset - 2)) {
		return -EMSGSIZE;
	}

	cpu_to_le16_u8p((u16) reportLen + 2, cmd, &offset);
	/* copy report in to cmd bufer */
	memcpy(cmd + offset, report, reportLen);
	offset += reportLen;
	mutex_lock(&i2cdata->hw_lock);
	ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, offset, NULL, 0);
	mutex_unlock(&i2cdata->hw_lock);
	if (ret == 0) {
		// success
		return reportLen;
	}

	return ret;
}

// Reset the hub
static int sensorhub_i2c_reset(struct sensorhub_drv_data * ddata, enum sensorhub_reset reset) {
	u8 cmd[4];
	int ret = 0;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	if (reset == SENSORHUB_RESET_HARD) {
		return sensorhub_i2c_toggle_reset(ddata, 1, RESET_TOGGLE_US, 0,
										  i2cdata->bootmode_state_app);
	}

	if (reset != SENSORHUB_RESET_SOFT) {
		return -EINVAL;
	}

	ret = sensorhub_i2c_prepare_request_reset(i2cdata, cmd, sizeof(cmd));
	WARN_ON(ret < 0);
	mutex_lock(&i2cdata->hw_lock);
	ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, sizeof(cmd), NULL, 0);
	mutex_unlock(&i2cdata->hw_lock);

	return ret;
}


static int sensorhub_i2c_setpower_locked(struct sensorhub_drv_data * ddata,
										 enum sensorhub_i2c_hid_power_state s) {
	u8 cmd[4];
	int ret = 0;
	int suspend = (s == SH_HID_PWR_SLEEP);
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	ret = sensorhub_i2c_prepare_request_set_power(i2cdata, s, cmd, sizeof(cmd));
	WARN_ON(ret < 0);

	if (suspend) {
		set_bit(SENSOR_HUB_SUSPENDED, &ddata->flags);
		ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, sizeof(cmd), NULL, 0);
	} else {
		ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, sizeof(cmd), NULL, 0);
		clear_bit(SENSOR_HUB_SUSPENDED, &ddata->flags);
		sensorhub_i2c_test_and_read_locked(__i2c(ddata));
	}

	return ret;
}

static int sensorhub_i2c_setpower(struct sensorhub_drv_data * ddata,
								  enum sensorhub_i2c_hid_power_state s) {
	int ret;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	mutex_lock(&i2cdata->hw_lock);
	ret = sensorhub_i2c_setpower_locked(ddata, s);
	mutex_unlock(&i2cdata->hw_lock);

	return ret;
}

// Get a input report from the device
static int sensorhub_i2c_getinput_locked(struct sensorhub_i2c_data * i2cdata, u64 host_int_timestamp) {

	int rc;
	u16 len = 0xffff;
	const size_t read_size = i2cdata->desc.wMaxInputLength;
	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	u8 buf[MAX_INPUT_REPORT_BUFSIZE];
	struct sensorhub_input_event event;

	if (test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
		return -ENXIO;
	}

	if (ddata->state != SENSORHUB_STATE_APP) {
		return -ENODEV;
	}

#ifdef DEBUG
	memset(buf, 0xcc, sizeof(buf));
#endif
	set_debug_gpio(1);
	rc = i2c_master_recv(i2cdata->client, buf, read_size);
	set_debug_gpio(0);

	if (unlikely(rc < 0)) {
		++ddata->stats.errorCount;
		goto exit;
	}

	if (WARN_ON(rc != read_size)) {
		++ddata->stats.errorCount;
		goto exit;
	}

	len = le16_to_cpup((u16 *) &buf[0]);
	if (len > 0 && len <= sizeof(buf)) {
		// a complete message was read in, send event to clients
		event.timestamp = host_int_timestamp;
		event.len = len - 2;
		rc = sensorhub_core_notify_input(ddata, &event, buf + 2, len - 2);
		
		//count_int++;
		//int i;
		//if((count_int%100)==0)
		//{
		//for(i=0;i<event.len;i++)
		//	printk("%d: 0x%x\n",i,event.msg[i]);
		//}
		
	} else if (len == 0) {
		// power on message recieved
		rc = sensorhub_core_notify_reset(ddata);
	} else {
		dev_err(ddata->dev, "message too long: msg len = %d\n", (int) len);
		rc = -ENOBUFS;
		goto exit;
	}

	wake_up_interruptible(&ddata->wait);

exit:
	return rc;
}

// primary isr, take a timestamp and queue process isr to run
static irqreturn_t sensorhub_i2c_isr_primary(int irq, void * irqdata) {
	struct sensorhub_i2c_data * i2cdata = (struct sensorhub_i2c_data *) irqdata;
	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	if (ddata->state == SENSORHUB_STATE_APP) {
		i2cdata->irq.timestamp = sensorhub_get_time_ns();
	}

	return IRQ_WAKE_THREAD;
}

// data ready interrupt service routine
static irqreturn_t sensorhub_i2c_isr(int irq, void * irqdata) {
	struct sensorhub_i2c_data * i2cdata = (struct sensorhub_i2c_data *) irqdata;
	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	++ddata->stats.interrupts;

	if (ddata->state == SENSORHUB_STATE_APP) {
		if (!test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
			mutex_lock(&i2cdata->hw_lock);
			sensorhub_i2c_getinput_locked(i2cdata, i2cdata->irq.timestamp);
			mutex_unlock(&i2cdata->hw_lock);
		}
	}

	else if (test_bit(SENSOR_HUB_DFU_COMPL, &ddata->flags)) {
		set_debug_gpio(1);
		complete(&i2cdata->bl.compl);
	}

	return IRQ_HANDLED;
}

// checks the interrupt gpio and services if low
static int sensorhub_i2c_test_and_read_locked(struct sensorhub_i2c_data * i2cdata) {
	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	if (gpio_get_value(i2cdata->ddata.pdata.gpio_interrupt) == 0) {
		dev_dbg(ddata->dev, "GPIO is low, getting input\n");
		return sensorhub_i2c_getinput_locked(i2cdata, sensorhub_get_time_ns());
	}

	dev_dbg(ddata->dev, "GPIO is high, no data to read\n");
	return 1;
}

// Bring up Sensorhub device
static int sensorhub_i2c_handshake(struct sensorhub_i2c_data * i2cdata) {

	int rc;
	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	struct device * dev = ddata->dev;

	BUILD_BUG_ON(sizeof(i2cdata->desc) != SH_HID_DESC_V1_LEN);
	mutex_lock(&i2cdata->hw_lock);
	rc = sensorhub_i2c_reg_read_locked(
			i2cdata, SH_HID_REG_HID_DESCRIPTOR,
			(uint8_t *) &i2cdata->desc, sizeof(i2cdata->desc)
	);

	if (rc < 0) {
		goto exit;
	}

	dev_dbg(dev,
			"I2C Hid Descriptor:\n"
			"    wHIDDescLength            = %04x\n"
			"    bcdVersion                = %04x\n"
			"    wReportDescriptorLength   = %04x\n"
			"    wReportDescriptorRegister = %04x\n"
			"    wInputRegister            = %04x\n"
			"    wMaxInputLength           = %04x\n"
			"    wOutputRegister           = %04x\n"
			"    wMaxOutputLength          = %04x\n"
			"    wCommandRegister          = %04x\n"
			"    wDataRegister             = %04x\n"
			"    wVendorID                 = %04x\n"
			"    wProductID                = %04x\n"
			"    wVersionID                = %04x\n"
			, (unsigned int) i2cdata->desc.wHIDDescLength
			, (unsigned int) i2cdata->desc.bcdVersion
			, (unsigned int) i2cdata->desc.wReportDescriptorLength
			, (unsigned int) i2cdata->desc.wReportDescriptorRegister
			, (unsigned int) i2cdata->desc.wInputRegister
			, (unsigned int) i2cdata->desc.wMaxInputLength
			, (unsigned int) i2cdata->desc.wOutputRegister
			, (unsigned int) i2cdata->desc.wMaxOutputLength
			, (unsigned int) i2cdata->desc.wCommandRegister
			, (unsigned int) i2cdata->desc.wDataRegister
			, (unsigned int) i2cdata->desc.wVendorID
			, (unsigned int) i2cdata->desc.wProductID
			, (unsigned int) i2cdata->desc.wVersionID
	);

	i2cdata->desc.wHIDDescLength            = le16_to_cpu(i2cdata->desc.wHIDDescLength);
	i2cdata->desc.bcdVersion                = le16_to_cpu(i2cdata->desc.bcdVersion);
	i2cdata->desc.wReportDescriptorLength   = le16_to_cpu(i2cdata->desc.wReportDescriptorLength);
	i2cdata->desc.wReportDescriptorRegister = le16_to_cpu(i2cdata->desc.wReportDescriptorRegister);
	i2cdata->desc.wInputRegister            = le16_to_cpu(i2cdata->desc.wInputRegister);
	i2cdata->desc.wMaxInputLength           = le16_to_cpu(i2cdata->desc.wMaxInputLength);
	i2cdata->desc.wOutputRegister           = le16_to_cpu(i2cdata->desc.wOutputRegister);
	i2cdata->desc.wMaxOutputLength          = le16_to_cpu(i2cdata->desc.wMaxOutputLength);
	i2cdata->desc.wCommandRegister          = le16_to_cpu(i2cdata->desc.wCommandRegister);
	i2cdata->desc.wDataRegister             = le16_to_cpu(i2cdata->desc.wDataRegister);
	i2cdata->desc.wVendorID                 = le16_to_cpu(i2cdata->desc.wVendorID);
	i2cdata->desc.wProductID                = le16_to_cpu(i2cdata->desc.wProductID);
	i2cdata->desc.wVersionID                = le16_to_cpu(i2cdata->desc.wVersionID);

	if (i2cdata->desc.wHIDDescLength != SH_HID_DESC_V1_LEN) {
		dev_err(dev, "Invalid descriptor length = %d, expecting %d\n",
			i2cdata->desc.wHIDDescLength, SH_HID_DESC_V1_LEN);
		rc = -ENODEV;
		goto exit;
	}

	if (i2cdata->desc.bcdVersion != SH_HID_DESC_V1_BCD) {
		dev_err(dev, "Incompatible bcdVersion = %4x, expecting %4x\n",
			i2cdata->desc.bcdVersion, SH_HID_DESC_V1_BCD);
		rc = -ENODEV;
		goto exit;
	}

	if (i2cdata->desc.wMaxInputLength > MAX_INPUT_REPORT_BUFSIZE) {
		dev_warn(dev, "Input message size exceeds max length = %4x.\n",
			i2cdata->desc.wMaxInputLength);
		rc = -ENODEV;
		goto exit;
	}

	if (!i2cdata->desc.wReportDescriptorLength ||
		 i2cdata->desc.wReportDescriptorLength > SENSORHUB_MAX_REPORT_DESCRIPTOR_SIZE) {
		dev_err(dev, "Unexpected reportDescriptorLength=%d\n",
			i2cdata->desc.wReportDescriptorLength);
		rc = -ENODEV;
		goto exit;
	}

#ifdef CONFIG_SENSOR_HUB_I2C_ENABLE_REPORT_DESCRIPTOR
	if (ddata->report_descriptor.buf) {
		kfree(ddata->report_descriptor.buf);
		ddata->report_descriptor.len = 0;
	}

	ddata->report_descriptor.buf = kzalloc(i2cdata->desc.wReportDescriptorLength, GFP_KERNEL);
	ddata->report_descriptor.len = i2cdata->desc.wReportDescriptorLength;

	if (!ddata->report_descriptor.buf) {
		dev_err(dev, "Failed allocating report descriptor\n");
		rc = -ENOMEM;
		goto exit;
	}

	rc = sensorhub_i2c_reg_read_locked(
		i2cdata, i2cdata->desc.wReportDescriptorRegister,
		ddata->report_descriptor.buf, i2cdata->desc.wReportDescriptorLength
	);
	if (rc) {
		dev_err(dev, "Failed reading report descriptor, rc=%d\n", rc);
		kfree(ddata->report_descriptor.buf);
		ddata->report_descriptor.buf = 0;
		ddata->report_descriptor.len = 0;
		goto exit;
	}
#endif

	ddata->vendorID  = i2cdata->desc.wVendorID;
	ddata->productID = i2cdata->desc.wProductID;
	ddata->versionID = i2cdata->desc.wVersionID;
	sensorhub_set_state(ddata, SENSORHUB_STATE_APP);

	sensorhub_i2c_test_and_read_locked(i2cdata);
	rc = 0;
	goto exit;

exit:
	mutex_unlock(&i2cdata->hw_lock);
	return rc;
}

// try to establish comms with the device every APP_STARTUP_CHECK_INTERVAL_MS
// and give up after APP_STARTUP_MAX_DELAY_MS
static int sensorhub_i2c_poll_handshake(struct sensorhub_i2c_data * i2cdata) {
	struct timespec end, now;
	end = CURRENT_TIME;
	now = end;

	end = CURRENT_TIME;
	timespec_add_ns(&end, APP_STARTUP_MAX_DELAY_MS * 1000ull * 1000ull);
	while (timespec_compare(&now, &end) < 0) {
		if (sensorhub_i2c_handshake(i2cdata) == 0) {
			return 0;
		}
		now = CURRENT_TIME;
		msleep(APP_STARTUP_CHECK_INTERVAL_MS);
	}
	return -ETIMEDOUT;
}

static int sensorhub_i2c_toggle_reset(struct sensorhub_drv_data * ddata,
									unsigned int reps,
									unsigned int toggle_us,
									unsigned int rep_ms,
									int bootmode) {
	if (ddata->pdata.gpio_reset == 0) {
		return -EOPNOTSUPP;
	}

	if (bootmode >= 0) {
		if (ddata->pdata.gpio_bootmode == 0) {
			return -EOPNOTSUPP;
		}

		gpio_set_value(ddata->pdata.gpio_bootmode, bootmode);
	}

	if (reps == 1) {
		gpio_set_value(ddata->pdata.gpio_reset, 0);
		udelay(toggle_us);
		gpio_set_value(ddata->pdata.gpio_reset, 1);
	} else {
		while (reps--) {
			gpio_set_value(ddata->pdata.gpio_reset, 0);
			udelay(toggle_us);
			gpio_set_value(ddata->pdata.gpio_reset, 1);
			if (reps) { // exit immediately after last
				msleep(rep_ms);
			}
		}
	}

	return 0;
}

#if defined(CONFIG_SENSOR_HUB_I2C_ATMEL_DFU) || \
	defined(CONFIG_SENSOR_HUB_I2C_STM32_DFU) || \
	defined(CONFIG_SENSOR_HUB_I2C_BNO_DFU)

struct sensorhub_dfu_stats {
	struct timespec start_time;

	size_t len;
	size_t sent;
	int num_frames;

	size_t next_print;
};

static void sensorhub_dfu_stats_init(struct sensorhub_dfu_stats * stats, size_t len) {
	memset(stats, 0, sizeof(*stats));
	stats->start_time = CURRENT_TIME;
	stats->len = len;
}

static void sensorhub_dfu_stats_show(struct sensorhub_drv_data * ddata,
									 struct sensorhub_dfu_stats * stats) {


	struct timespec elapsed = timespec_sub(CURRENT_TIME, stats->start_time);
	dev_dbg(ddata->dev, "dfu: sent %3d%% %6zu/%zu bytes %4d frames %2d.%03d s elapsed\n",
			stats->sent * 100 / stats->len,
			stats->sent, stats->len, stats->num_frames,
			(int) elapsed.tv_sec, (int)(elapsed.tv_nsec / 1000 / 1000));
}

static void sensorhub_dfu_stats_update(struct sensorhub_drv_data * ddata,
								   struct sensorhub_dfu_stats * stats,
								   size_t sent) {
	stats->sent += sent;
	stats->num_frames++;

	if (stats->sent >= stats->next_print) {
		size_t incr;
		sensorhub_dfu_stats_show(ddata, stats);
		// at least every 4k or 8 times
		incr = (stats->len >> 3) > 4096 ? (stats->len >> 3) : 4096;
		stats->next_print += incr;
	}
}

/* read from arbitrary address */
static int sensorhub_i2c_aux_read(struct sensorhub_i2c_data * i2cdata, u8 addr, u8 * buf, size_t len) {
	struct i2c_msg msg[1];
	int rc;

	struct i2c_client * const client = i2cdata->client;
	struct i2c_adapter * const adap = client->adapter;

	msg[0].addr = addr;
	msg[0].flags = (i2cdata->client->flags & I2C_M_TEN) | I2C_M_RD;
	msg[0].len = len;
	msg[0].buf = buf;

	rc = i2c_transfer(adap, msg, 1);
	return rc == 1 ? 0 : rc;
}

/* write to arbitrary address */
static int sensorhub_i2c_aux_write(struct sensorhub_i2c_data * i2cdata, u8 addr, const u8 * buf, size_t len) {
	struct i2c_msg msg[1];
	int rc;

	struct i2c_client * const client = i2cdata->client;
	struct i2c_adapter * const adap = client->adapter;

	msg[0].addr = addr;
	msg[0].flags = (i2cdata->client->flags & I2C_M_TEN);
	msg[0].len = len;
	msg[0].buf = (u8 *) buf; // discard const

	rc = i2c_transfer(adap, msg, 1);
	return rc == 1 ? 0 : rc;
}

/**
 * read from an arbitrary address
 * use the given address, unless the SH is on the alternate address
 */
static int sensorhub_i2c_aux_base_read(struct sensorhub_i2c_data * i2cdata,
	u8 baseaddr, u8 * buf, size_t len) {
	return sensorhub_i2c_aux_read(i2cdata,
		baseaddr | (i2cdata->client->addr & 0x1), buf, len);
}

/**
 * write to an arbitrary address
 * use the given address, unless the SH is on the alternate address
 */
static int sensorhub_i2c_aux_base_write(struct sensorhub_i2c_data * i2cdata,
	u8 baseaddr, const u8 * buf, size_t len) {
	return sensorhub_i2c_aux_write(i2cdata,
		baseaddr | (i2cdata->client->addr & 0x1), buf, len);
}
#endif

#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU

inline int sensorhub_i2c_dfu_bl_read(struct sensorhub_i2c_data * i2cdata, u8 * buf, size_t len) {
	return sensorhub_i2c_aux_base_read(
		i2cdata, ATMEL_BL_I2C_ADDR_BASE, buf, len);
}

inline int sensorhub_i2c_dfu_bl_write(struct sensorhub_i2c_data * i2cdata, const u8 * buf, size_t len) {
	return sensorhub_i2c_aux_base_write(
		i2cdata, ATMEL_BL_I2C_ADDR_BASE, buf, len);
}

// poll the chg interrupt until it asserts (low) or a timeout occurs
// return 0 if chg is asserted or -ETIMEDOUT
static int sensorhub_i2c_bl_poll_chg(struct sensorhub_drv_data * ddata) {
	struct timespec end, now;;
	end = CURRENT_TIME;
	now = end;
	timespec_add_ns(&end, BOOTLOADER_READY_MAX_DELAY_MS * 1000 * 1000);
	while (timespec_compare(&now, &end) < 0) {
		if (gpio_get_value(ddata->pdata.gpio_interrupt) == 0) {
			return 0;
		}
		now = CURRENT_TIME;
		msleep(BOOTLOADER_READY_CHECK_INTERVAL_MS);
	}
	return -ETIMEDOUT;
}

static int sensorhub_i2c_atmel_dfu_enterbl(struct sensorhub_drv_data * ddata) {
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	int rc;
	char buf[3];

	sensorhub_i2c_toggle_reset(ddata, BOOTLOADER_RESET_REPS, RESET_TOGGLE_US, BOOTLOADER_CYCLE_DELAY_MS, -1);
	if (sensorhub_i2c_bl_poll_chg(ddata)) {
		dev_dbg(ddata->dev, "reset2bl: bootloader did not respond (timeout waiting for CHG)\n");
		return -ENODEV;
	}

	rc = sensorhub_i2c_dfu_bl_read(i2cdata, buf, sizeof(buf));
	if (rc != 0) {
		dev_err(ddata->dev, "reset2bl: bootloader read failed\n");
		return -ENODEV;
	}

	dev_dbg(ddata->dev, "reset2bl: bootloader status=%02x id=%02x ver=%02x\n",
						buf[0], buf[1], buf[2]);
	if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_APP_CRC_FAIL) {
		dev_warn(ddata->dev, "reset2bl: bootloader reports app CRC fail\n");
		sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER_NOIMAGE);
		return 0;
	}

	if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_WAIT_CMD_1) {
		dev_info(ddata->dev, "reset2bl: bootloader ready\n");
		sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER);
		return 0;
	}
	dev_err(ddata->dev, "reset2bl: bootloader response with unexpected status=%02x\n", buf[0]);
	return -ENODEV;
}

// return negative number on error or the status as a u8
static int sensorhub_i2c_dfu_wait_status(struct sensorhub_i2c_data * i2cdata,
										 unsigned long timeout) {

	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	u8 status;

	long rc = wait_for_completion_interruptible_timeout(&i2cdata->bl.compl, timeout);
	set_debug_gpio(0);
	if (rc < 0) {
		dev_err(ddata->dev, "interrupted while waiting for status\n");
		return -EINTR;
	} else if (rc == 0) {
		dev_err(ddata->dev, "time out waiting for status\n");
		return -ETIMEDOUT;
	}

	rc = sensorhub_i2c_dfu_bl_read(i2cdata, &status, 1);
	if (rc < 0) {
		dev_err(ddata->dev, "error reading status: %ld\n", rc);
		return rc;
	}

	return status;
}

static int sensorhub_i2c_atmel_dfu_fw_validate(struct sensorhub_drv_data * ddata,
											   const u8 * fw, size_t len,
											   const char * metadata) {

	const u8 * p = fw;
	const u8 * const end = p + len;

	while (p < end) {
		size_t frame_len = p[1] | (p[0] << 8);
		size_t xfer_len = frame_len + 2; // + Frame CRC
		if (frame_len > ATMEL_BL_MAX_FRAME_SIZE) {
			dev_err(ddata->dev, "frame exceeds max size: %zu/%zu\n",
					frame_len, ATMEL_BL_MAX_FRAME_SIZE);
			return -EINVAL;
		}

		if ((p + xfer_len) > (u8 *) end) {
			dev_err(ddata->dev, "frame exceed fw image bounds\n");
			return -EINVAL;
		}

		p += xfer_len;
	}

	// the last frame should have aligned p with the footer
	return (p != end) ? -EINVAL : 0;
}

static void sensorhub_i2c_atmel_dfu_abort(struct sensorhub_drv_data * ddata) {
	sensorhub_i2c_init_app(__i2c(ddata));
}

static int sensorhub_i2c_atmel_dfu_perform(struct sensorhub_drv_data * ddata, const u8 * fw, size_t len) {

	int rc;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	const u8 * const fw_end = fw + len - 4;
	const u8 * p = fw;

	int frame_retry = DFU_FRAME_SEND_NUM_RETRY;
	struct sensorhub_dfu_stats stats;

	sensorhub_dfu_stats_init(&stats, len);

	// initialize our context
	init_completion(&i2cdata->bl.compl);
	set_bit(SENSOR_HUB_DFU_COMPL, &ddata->flags);

	// send the unlock command
	rc = sensorhub_i2c_dfu_bl_write(i2cdata,
					ATMEL_BL_UNLOCK_CMD, ARRAY_SIZE(ATMEL_BL_UNLOCK_CMD));
	if (rc) {
		goto exit;
	}

	while (p < (fw_end - 1)) {
		// extra two bytes for CRC
		size_t xfer_len = (p[1] | (p[0] << 8)) + 2;
		long t;

		rc = sensorhub_i2c_dfu_wait_status(i2cdata, DFU_STATE_CHANGE_TIMEOUT_MS);
		if (rc < 0) {
			goto exit;
		}

		if (!(rc & ATMEL_BL_STS_WAIT_FRM_1_MASK)) {
			dev_err(ddata->dev, "unexpected status=%02x Expect WAIT_FRAME_1\n", rc);
			rc = EINVAL;
			goto exit;
		}

		// CHG is toggled after the previous read
		t = wait_for_completion_interruptible_timeout(&i2cdata->bl.compl, DFU_STATE_CHANGE_TIMEOUT_MS);
		set_debug_gpio(0);
		if (t < 0) {
			dev_err(ddata->dev, "interrupted while waiting for bogus status\n");
			rc = -EINTR;
			goto exit;
		}
		// try the read after the timeout anyway

		rc = sensorhub_i2c_dfu_bl_write(i2cdata, p, xfer_len);
		if (rc) {
			dev_err(ddata->dev, "error sending frame: %d\n", rc);
			goto exit;
		}

		rc = sensorhub_i2c_dfu_wait_status(i2cdata, DFU_STATE_CHANGE_TIMEOUT_MS);
		if (rc < 0) {
			goto exit;
		}
		if (rc != ATMEL_BL_STS_FRM_CRC_CHK_1) {
			dev_err(ddata->dev, "unexpected status=%02x expect CRC_CHK_1\n", rc);
			rc = EINVAL;
			goto exit;
		}

		rc = sensorhub_i2c_dfu_wait_status(i2cdata, DFU_STATE_CHANGE_TIMEOUT_MS);
		if (rc == ATMEL_BL_STS_FRM_CRC_PASS) {
			p += xfer_len;
			frame_retry = DFU_FRAME_SEND_NUM_RETRY;
			sensorhub_dfu_stats_update(ddata, &stats, xfer_len);
		} else if (rc == ATMEL_BL_STS_FRM_CRC_FAIL) {
			if (frame_retry--) {
				dev_warn(ddata->dev, "dfu: status=CRC failed; frame=%d offset=%zu; retrying.\n",
						 stats.num_frames, stats.sent);
			} else {
				dev_err(ddata->dev, "dfu: failed CRC check\n");
				rc = -EINVAL;
				goto exit;
			}
		} else {
			dev_err(ddata->dev, "dfu: unexpected status from device: %02x\n", rc);
			rc = -EINVAL;
			goto exit;
		}
	}

	dev_dbg(ddata->dev, "dfu: download finished\n");

	if ((rc = sensorhub_i2c_poll_handshake(i2cdata))) {
		char buf[3];
		dev_warn(ddata->dev, "DFU: application did not start after successful download.\n");

		// try bootloader
		rc = sensorhub_i2c_dfu_bl_read(i2cdata, buf, sizeof(buf));
		if (rc == 0) {
			if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_APP_CRC_FAIL) {
				sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER_NOIMAGE);
				dev_err(ddata->dev, "DFU: download succeeded, but bootloader reports app CRC fail\n");
			} else {
				sensorhub_set_state(ddata, SENSORHUB_STATE_DOWNLOAD_FAILED);
				dev_dbg(ddata->dev, "DFU: got bootloader response after DFU. status=%02x id=%02x ver=%02x\n",
									buf[0], buf[1], buf[2]);
			}

			rc = -EREMOTEIO;
		} else {
			dev_warn(ddata->dev, "DFU: resetting and trying again\n");
			sensorhub_i2c_reset(ddata, SENSORHUB_RESET_HARD);
			if ((rc = sensorhub_i2c_poll_handshake(i2cdata))) {
				sensorhub_set_state(ddata, SENSORHUB_STATE_DOWNLOAD_FAILED);
				if ((rc = ddata->transport->dfu_enterbl(ddata))) {
					dev_err(ddata->dev, "DFU: download succeeded, but no response from app or bootloader now\n");
					rc = -ENODEV;
				} else {
					dev_err(ddata->dev, "DFU: download succeeded, but no response from app\n");
					rc = -EREMOTEIO;
				}
			}
		}
	}
exit:
	clear_bit(SENSOR_HUB_DFU_COMPL, &ddata->flags);
	return rc;
}


static int sensorhub_atmel_bl_detect_and_exit(struct sensorhub_i2c_data * i2cdata) {
	char buf[3];
	char reset_cmd[] = {0x00, 0x00};
	int rc;

	struct sensorhub_drv_data * ddata = &i2cdata->ddata;

	if (sensorhub_i2c_dfu_bl_read(i2cdata, buf, sizeof(buf)) != 0) {
		// bootloader did not respond
		return 1;
	}
	dev_dbg(ddata->dev, "init: bootloader status=%02x id=%02x ver=%02x\n",
						 buf[0], buf[1], buf[2]);

	// got a response from the bootloader
	if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_APP_CRC_FAIL) {
		dev_warn(ddata->dev, "init: app crc failure\n");
		// no way to exit from bl when crc is failed
		return 1;
	}

	// waiting for unlock?
	if ((buf[0] & ATMEL_BL_STS_INIT_MASK) != ATMEL_BL_STS_WAIT_CMD_1) {
		dev_warn(ddata->dev, "init: unexpected bootloader status: %02x\n", buf[0]);
		return -ENODEV;
	}

	dev_dbg(ddata->dev, "init: bootloader state=waiting; try bootloader exit\n");
	// send the unlock command
	rc = sensorhub_i2c_dfu_bl_write(i2cdata,
					ATMEL_BL_UNLOCK_CMD, ARRAY_SIZE(ATMEL_BL_UNLOCK_CMD));
	if (rc) {
		dev_dbg(ddata->dev, "init: bootloader unlock command write failed (%d)\n", rc);
		return rc;
	}

	if (sensorhub_i2c_bl_poll_chg(ddata) != 0) {
		set_debug_gpio(1);
		dev_err(ddata->dev, "init: bootloader timedout\n");
		return -ENODEV;
	}

	if (sensorhub_i2c_dfu_bl_read(i2cdata, buf, 1) != 0) {
		dev_dbg(ddata->dev, "init: no response from bootloader after unlock\n");
		return -ENODEV;

	}

	if (!(buf[0] & ATMEL_BL_STS_WAIT_FRM_1_MASK)) {
		dev_warn(ddata->dev, "init: unexpected bootloader status: %02x\n", buf[0]);
		return -ENODEV;
	}

	// wait for CHG to assert (again)
	if (sensorhub_i2c_bl_poll_chg(ddata) != 0) {
		dev_err(ddata->dev, "init: bootloader timedout\n");
		return -ENODEV;
	}

	// the bootloader should be unlocked now.
	// a zero length frame _should_ cause it to reset
	dev_dbg(ddata->dev, "init: bootloader unlocked. sending reset command\n");
	rc = sensorhub_i2c_dfu_bl_write(i2cdata, reset_cmd, sizeof(reset_cmd));
	if (rc) {
		dev_dbg(ddata->dev, "init: bootloader unlock reset write failed (%d)\n", rc);
		return rc;
	}

	return rc;
}
#endif // CONFIG_SENSOR_HUB_I2C_ATMEL_DFU

#ifdef CONFIG_SENSOR_HUB_I2C_STM32_DFU

#define SENSORHUB_STM32_I2C_ADDR	(0x39)

#define SENSORHUB_STM32_BL_GET_CMD	(0x00)
#define SENSORHUB_STM32_BL_GET_ID	(0x02)
#define SENSORHUB_STM32_BL_READ_MEMORY	(0x11)
#define SENSORHUB_STM32_BL_WRITE_MEMORY	(0x31)
#define SENSORHUB_STM32_BL_ERASE	(0x44)
#define SENSORHUB_STM32_BL_WRITE_PROTECT (0x63)
#define SENSORHUB_STM32_BL_WRITE_UNPROTECT (0x73)

#define SENSORHUB_STM32_ACK		(0x79)
#define SENSORHUB_STM32_NACK		(0x1F)
#define SENSORHUB_STM32_BUSY		(0x76)

#define SENSORHUB_STM32_BL_MAX_FRAME_SIZE 256

static int sensorhub_stm32_dfu_enterbl(struct sensorhub_drv_data * ddata);
static int sensorhub_stm32_get_command(struct sensorhub_i2c_data * i2cdata);
static int sensorhub_stm32_writememory(struct sensorhub_i2c_data * i2cdata, const u8 * pData);
static int sensorhub_stm32_erasememory(struct sensorhub_i2c_data * i2cdata, const u8 * pData);


static void sensorhub_stm32_dfu_abort(struct sensorhub_drv_data * ddata) {
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);
	sensorhub_i2c_toggle_reset(ddata, 1, 1000, 0, 0);
	sensorhub_i2c_init_app(i2cdata);
}

static int sensorhub_stm32_dfu_perform(struct sensorhub_drv_data * ddata, const u8 * fw, size_t len)
{
	int rc = 0;
	struct sensorhub_i2c_data * i2cdata = container_of(ddata, struct sensorhub_i2c_data, ddata);

	const u8 * const fw_end = fw + len;
	const u8 * p = fw;
	size_t xfer_len;
	size_t payloadLength;
	size_t numberOfSectors;
	struct sensorhub_dfu_stats stats;

	sensorhub_dfu_stats_init(&stats, len);

	// Erases memory sectors
	// Number of sectors to be erased = N (2 bytes)
	// (Sector code (2 bytes) + checksum (1 byte) ) * N
	dev_info(ddata->dev, "DFU: erase flash memory\n");
	if ((rc = sensorhub_stm32_erasememory(i2cdata, p)) < 0)
	{
		sensorhub_update_state(ddata, SENSORHUB_STATE_DOWNLOAD_FAILED);
		goto exit;
	}

	numberOfSectors = (p[0] << 8) + p[1];
	payloadLength = len - (2 + (numberOfSectors) * 3);
	p += 2 + (numberOfSectors) * 3;

	// downloads new firmware into stm32 memory
	dev_info(ddata->dev, "DFU: start download\n");
	while (p < (fw_end - 1))
	{
		// Address (4 bytes) + checksum (1 byte)
		// Number of data bytes N (1 byte) + data bytes (N + 1 bytes) + checksum (1 byte)
		xfer_len = p[5] + 8;

		if ((rc = sensorhub_stm32_writememory(i2cdata, p)) < 0)
		{
			sensorhub_update_state(ddata, SENSORHUB_STATE_DOWNLOAD_FAILED);
			dev_warn(ddata->dev, "DFU: unable to write memory. Download aborted.\n");
			break;
		}

		sensorhub_dfu_stats_update(ddata, &stats, xfer_len);
		p += xfer_len;
	}

	dev_dbg(ddata->dev, "dfu: download finished\n");

	// normal mode
	sensorhub_i2c_toggle_reset(&i2cdata->ddata, 1, 1000, 0, 0);

	if (rc < 0)
		goto exit;

	if ((rc = sensorhub_i2c_poll_handshake(i2cdata)) < 0)
	{
		dev_warn(ddata->dev, "DFU: application did not start after successful download.\n");
		dev_warn(ddata->dev, "DFU: resetting and trying again\n");

		if (ddata->transport->dfu_enterbl(ddata) < 0) {
			dev_err(ddata->dev, "DFU: download succeeded, but no response from app or bootloader now\n");
			rc = -ENODEV;
		} else {
			dev_err(ddata->dev, "DFU: download succeeded, but no response from app\n");
			rc = -EREMOTEIO;
		}

		sensorhub_update_state(ddata, SENSORHUB_STATE_BOOTLOADER_NOIMAGE);
	} else {
		sensorhub_update_state(ddata, SENSORHUB_STATE_APP);
	}

exit:
	if (rc) {
		sensorhub_i2c_toggle_reset(&i2cdata->ddata, 1, 1000, 0, 0);
	}
	return rc;
}

static int sensorhub_stm32_dfu_fw_validate(struct sensorhub_drv_data * ddata,
										   const u8 * fw, size_t len,
										   const char * metadata) {

	const u8 * p = fw;
	const u8 * const end = p + len;
	size_t numberOfSectors;

	numberOfSectors = (p[0] << 8) + p[1];
	p += 2 + (numberOfSectors) * 3;

	while (p < end) {
		size_t frame_len = p[5] + 1;
		// 4 bytes address + checksum byte
		// length byte + data bytes + checksum byte
		size_t xfer_len = frame_len + 7;
		if (frame_len > SENSORHUB_STM32_BL_MAX_FRAME_SIZE) {
			dev_err(ddata->dev, "frame exceeds max size: %zu/%zu\n",
					frame_len, SENSORHUB_STM32_BL_MAX_FRAME_SIZE);
			return -EINVAL;
		}

		if ((p + xfer_len) > (u8 *) end) {
			dev_err(ddata->dev, "frame exceed fw image bounds\n");
			return -EINVAL;
		}

		p += xfer_len;
	}

	// the last frame should have aligned p with the end of the image
	return (p != end) ? -EINVAL : 0;
}

static int sensorhub_stm32_dfu_enterbl(struct sensorhub_drv_data * ddata) {
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);
	int rc = - ENODEV;
	int i;

	sensorhub_i2c_toggle_reset(&i2cdata->ddata, 1, 1000, 0, 1);

	for (i = 0; i < 10; i++) {
		msleep(100);
		if (sensorhub_stm32_get_command(i2cdata) >= 0) {
			rc = 0;
			sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER);
			break;
		}
	}

	return rc;
}

inline int sensorhub_stm32_receivedata(struct sensorhub_i2c_data * i2cdata, u8 * buf, size_t len) {
	return sensorhub_i2c_aux_read(i2cdata, SENSORHUB_STM32_I2C_ADDR, buf, len);
}

inline int sensorhub_stm32_senddata(struct sensorhub_i2c_data * i2cdata, const u8 * buf, size_t len) {
	return sensorhub_i2c_aux_write(i2cdata, SENSORHUB_STM32_I2C_ADDR, buf, len);
}

static int sensorhub_stm32_getack(struct sensorhub_i2c_data * i2cdata) {

	int ret;
	u8 buf;

	ret = sensorhub_stm32_receivedata(i2cdata, &buf, 1);
	if (ret < 0) {
		return ret;
	}

	switch(buf) {
		case SENSORHUB_STM32_NACK:
		default:
			ret = -EAGAIN;
		break;
		case SENSORHUB_STM32_BUSY:
			ret = -EBUSY;
		break;
		case SENSORHUB_STM32_ACK:
			// ok, return ret.
			break;
	}

	return ret;
}

static int sensorhub_stm32_cmd(struct sensorhub_i2c_data * i2cdata, u8 command) {
	u8 buf[2];

	buf[0] = command;
	buf[1] = ~command;

	return sensorhub_stm32_senddata(i2cdata, buf, sizeof(buf));
}

static int sensorhub_stm32_get_command(struct sensorhub_i2c_data * i2cdata)
{
	u8 buf[13];
	int rc;

	memset(buf, 0, sizeof(buf));

	rc = sensorhub_stm32_cmd(i2cdata, SENSORHUB_STM32_BL_GET_CMD);
	if(rc < 0)
		goto exit;

	rc = sensorhub_stm32_getack(i2cdata);
	if(rc < 0)
		goto exit;

	rc = sensorhub_stm32_receivedata(i2cdata, buf, sizeof(buf));
	if(rc < 0)
		goto exit;

	switch(buf[1]) {
		case 0x10:
		case 0x11:
			dev_info(i2cdata->ddata.dev, "STM32 Bootloader revision %02x\n", buf[1]);
		break;
		default:
			dev_info(i2cdata->ddata.dev, "Not supported STM32 Bootloader revision %02x\n", buf[1]);
			rc = -EPROTONOSUPPORT;
		break;
	}

	rc = sensorhub_stm32_getack(i2cdata);
	if(rc < 0)
		goto exit;

exit:
	return rc == 1 ? 0 : rc;
}


static int sensorhub_stm32_writememory(struct sensorhub_i2c_data * i2cdata, const u8 * pData)
{
	int rc;

	// Sends command + checksum
	rc = sensorhub_stm32_cmd(i2cdata, SENSORHUB_STM32_BL_WRITE_MEMORY);
	if(rc < 0)
		goto exit;

	rc = sensorhub_stm32_getack(i2cdata);
	if(rc < 0)
		goto exit;

	// Sends 4 bytes address + checksum
	rc = sensorhub_stm32_senddata(i2cdata, (u8 *) pData, 5);
	if(rc < 0)
		goto exit;

	rc = sensorhub_stm32_getack(i2cdata);
	if(rc < 0)
		goto exit;

	// Sends number of bytes + data bytes + checksum
	rc = sensorhub_stm32_senddata(i2cdata, (u8 *) pData + 5, pData[5] + 3);
	if(rc < 0)
		goto exit;

	rc = sensorhub_stm32_getack(i2cdata);
	if(rc < 0)
		goto exit;

exit:
	return rc == 1 ? 0 : rc;
}


static int sensorhub_stm32_erasememory(struct sensorhub_i2c_data * i2cdata, const u8 * pData)
{
	int rc;
	size_t numberOfSectors;
	size_t i;
	u8 subCmd[3];
	size_t offset;
	int retry;

	rc = 1;
	numberOfSectors = (pData[0] << 8) + pData[1];

	if (numberOfSectors == 0)
		goto exit;

	// Sectors are erased one by one
	subCmd[0] = 0;
	subCmd[1] = 0;
	subCmd[2] = 0;

	for (i = 0, offset = 2; i < numberOfSectors; i++, offset += 3)
	{
		// Sends erase memory command
		if ((rc = sensorhub_stm32_cmd(i2cdata, SENSORHUB_STM32_BL_ERASE)) < 0)
			goto exit;

		if ((rc = sensorhub_stm32_getack(i2cdata)) < 0)
			goto exit;

		// Sends the number of sectors
		if ((rc = sensorhub_stm32_senddata(i2cdata, subCmd, sizeof(subCmd))) < 0)
			goto exit;

		if ((rc = sensorhub_stm32_getack(i2cdata)) < 0)
			goto exit;

		// Sends the sector code
		if ((rc = sensorhub_stm32_senddata(i2cdata, (u8 *) pData + offset, 3)) < 0)
			goto exit;

		mdelay(900);
		if (sensorhub_stm32_getack(i2cdata) < 0)
		{
			// The microcontroller did not have enough time to erase flash
			for (retry = 0; retry < 10; retry++)
			{
				if ((rc = sensorhub_stm32_get_command(i2cdata)) >= 0)
					break;

				msleep(1000);
			}
		}

		dev_dbg(i2cdata->ddata.dev, "dfu: erase %d/%d sectors\n", i + 1, numberOfSectors);
	}

exit:
	return rc == 1 ? 0 : rc;
}

static int sensorhub_stm32_bl_detect_and_exit(struct sensorhub_i2c_data * i2cdata) {
	// check value of boot mode pin
	if (gpio_get_value(i2cdata->ddata.pdata.gpio_bootmode) == 1) {
		// if high, clear boot mode and reset
		sensorhub_i2c_toggle_reset(&i2cdata->ddata, 1, 1000, 0, 0);
	}

	return 0;
}

#endif // CONFIG_SENSOR_HUB_I2C_STM32_DFU
#ifdef CONFIG_SENSOR_HUB_I2C_BNO_DFU

#define SH_BNO_BL_ADDR_BASE        0x28
#define SH_BNO_DFU_MAX_PKT_SIZE    64
#define SH_BNO_DFU_PKT_DEF_SIZE    SH_BNO_DFU_MAX_PKT_SIZE
#define SH_BNO_DFU_DATA_CRC16_POLY 0x1021
#define SH_BNO_DFU_CMD_CRC16_POLY  0x0811
#define SH_BNO_BOOT_MODE_APP       1
#define SH_BNO_DFU_ACK             's'
#define SH_BNO_DFU_NAK             'n'
#define SH_BNO_FW_FORMAT           "BNO_V1"

inline int sensorhub_bno_bl_read(struct sensorhub_i2c_data * i2cdata, u8 * buf, size_t len) {
	return sensorhub_i2c_aux_base_read(i2cdata, SH_BNO_BL_ADDR_BASE, buf, len);
}

// 16 bit CRC-CCITT modified to not XOR the final result with 0xffff
// per BNO070 bootloader documentation. (In the docs, they say to NOT
// the CRC-CCITT output, but CRC-CCITT contains a NOT already so we can
// just get rid of it)
static int sensorhub_bno_crc16(u16 poly, const u8 * buf, size_t len) {
	int i, j;
	u16 crc = 0xffff;
	for (i = 0; i < len; i++) {
		u16 x = buf[i] << 8;
		for (j = 0; j < 8; j++) {
			if ((crc ^ x) & 0x8000) {
				crc = (crc << 1) ^ poly;
			} else {
				crc = crc << 1;
			}
			x = x << 1;
		}
	}
	return crc;
}

#if 0
static void sensorhub_bno_crc16_test(void) {
	u16 crc1, crc2;
	const char * str1 = "A";
	const char * str2 = "123456789";
	crc1 = sensorhub_bno_crc16(SH_BNO_DFU_DATA_CRC16_POLY, str1, 1);
	crc2 = sensorhub_bno_crc16(SH_BNO_DFU_DATA_CRC16_POLY, str2, 9);

	pr_info(" %10s | %#4x | %1phN\n", str1, crc1, str1);
	pr_info(" %10s | %#4x | %9phN\n", str2, crc2, str2);
}
#endif

// adds crc to buf, send data and check status
// handles retry on NAKs
static int sensorhub_bno_bl_sendcommon(struct sensorhub_i2c_data * i2cdata,
                                       u16 poly, const u8 * pkt, size_t len,
                                       int retries) {

	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;

	int rc;
	int i;
	u8 status;
	u8 buf[SH_BNO_DFU_MAX_PKT_SIZE + 2]; // +2 for CRC
	u16 crc;

	bool send = true;

	if (len > sizeof(buf)) {
		return -EMSGSIZE;
	}

	memcpy(buf, pkt, len);
	crc = sensorhub_bno_crc16(poly, buf, len);
	buf[len++] = crc >> 8;
	buf[len++] = crc & 0xff;

	rc = -EINVAL;
	for (i = 0; i < retries + 1; i++) {
		if (i) { // retry
			msleep(10);
		}

		if (send) {
			rc =sensorhub_i2c_aux_base_write(i2cdata,
				SH_BNO_BL_ADDR_BASE, buf, len);
			if (rc) {
				dev_err(ddata->dev, "dfu: error writing to device, retrying\n");
				continue;
			}
			send = false;
		}

		rc = sensorhub_bno_bl_read(i2cdata, &status, 1);
		if (rc) {
			dev_err(ddata->dev, "dfu: error reading status\n");
			continue; // retry
		}

		if (status == SH_BNO_DFU_NAK) {
			send = true; // resend
			dev_err(ddata->dev, "dfu: NAK received, retrying\n");
			continue;
		}

		if (status != SH_BNO_DFU_ACK) {
			dev_err(ddata->dev, "dfu: unexpected status=%#02x/'%c'\n", (int) status, status);
			rc = -EINVAL;
			break;
		}

		// all good
		rc = 0;
		break;
	}

	return rc;
}

static int sensorhub_bno_bl_senddata(struct sensorhub_i2c_data * i2cdata,
									 const u8 * buf, size_t len) {
	return sensorhub_bno_bl_sendcommon(i2cdata, SH_BNO_DFU_DATA_CRC16_POLY, buf, len, 2);
}

static void sensorhub_bno_dfu_abort(struct sensorhub_drv_data * ddata) {
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);
	sensorhub_i2c_toggle_reset(ddata, 1, 1000, 0, 1); // set boot mode to app
	sensorhub_i2c_init_app(i2cdata);
	if (ddata->state != SENSORHUB_STATE_APP) {
		// we are effectively in this state because we corrupted the app
		sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER_NOIMAGE);
	}
}

static int sensorhub_bno_dfu_perform(struct sensorhub_drv_data * ddata, const u8 * fw, size_t len) {

	struct sensorhub_i2c_data * const i2cdata = __i2c(ddata);
	const u8 * p;
	const u8 * end;
	int rc;

	u32 fw_sz = htonl(len);
	const u8 pkt_sz = SH_BNO_DFU_PKT_DEF_SIZE;
	u8 tmp[4] = {0, 0, 0, 0};

	struct sensorhub_dfu_stats stats;
	sensorhub_dfu_stats_init(&stats, len);

	// reset in to bootloader and wait until it's ready
	// there's no way to query the status, and entering the bootloader causes
	// the firmware to be lost
	sensorhub_i2c_toggle_reset(ddata, 1, 1000, 0, 0);
	msleep(25);

	rc = sensorhub_bno_bl_senddata(i2cdata, (u8 *) &fw_sz, sizeof(fw_sz));
	if (rc) {
		dev_err(ddata->dev, "dfu: error sending fw size\n");
		goto exit_abort;
	}

	rc = sensorhub_bno_bl_senddata(i2cdata, &pkt_sz, sizeof(pkt_sz));
	if (rc) {
		dev_err(ddata->dev, "dfu: error packet fw size\n");
		goto exit_abort;
	}

	for (p = fw, end = fw + len; p < end; p += pkt_sz) {
		size_t avail = end - p;
		size_t l;
		const u8 * buf = p;

		if (avail < pkt_sz) {
			// need to ensure at least 4 bytes
			if (avail < 4) {
				int j;
				for (j = 0; j < avail; j++) {
					tmp[j] = *(p + j);
				}
				buf = tmp;
				l = 4;
			} else {
				l = avail;
			}
		} else {
			l = pkt_sz;
		}

		set_debug_gpio(1);
		rc = sensorhub_bno_bl_senddata(i2cdata, buf, l);
		set_debug_gpio(0);

		if (rc) {
			goto exit_abort;
		}

		sensorhub_dfu_stats_update(ddata, &stats, l);
	}
	gpio_set_value(ddata->pdata.gpio_bootmode, 1); // clear boot mode pin
	dev_dbg(ddata->dev, "dfu: download finished\n");

	if ((rc = sensorhub_i2c_poll_handshake(i2cdata)) == 0) {
		sensorhub_update_state(ddata, SENSORHUB_STATE_APP);
	} else {
		dev_warn(ddata->dev, "dfu: application did not start after successful download.\n");
		sensorhub_update_state(ddata, SENSORHUB_STATE_BOOTLOADER_NOIMAGE);
	}

	return 0;

exit_abort:
	return rc;
}

static int sensorhub_bno_dfu_fw_validate(struct sensorhub_drv_data * ddata,
										 const u8 * fw, size_t len,
										 const char * metadata) {
	char value[32];

	int rc = sensorhub_core_dfu_metadata_get_prop(metadata, "FW-Format",
												  value, sizeof(value));
	if (rc) {
		return -EINVAL;
	}

	if (!sysfs_streq(value, SH_BNO_FW_FORMAT)) {
		dev_warn(ddata->dev, "dfu: invalid fw type '%s'; expect '%s'\n",
			     value, SH_BNO_FW_FORMAT);
		return -EINVAL;
	}

	// nothing to validate in the payload
	return 0;
}

static int sensorhub_bno_bl_detect_and_exit(struct sensorhub_i2c_data * i2cdata) {\
	// there is no way to query BNO's bl status. just say 'ok' below
	return 0;
}

static int sensorhub_bno_dfu_enterbl(struct sensorhub_drv_data * ddata) {
	// don't actually reset in to the bootloader until we are sure we are ready
	// to download firmware
	// just acknowledge the state change
	sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER);
	return 0;
}
#endif // CONFIG_SENSOR_HUB_I2C_BNO_DFU

// Contact the sensorhub application and determine if it's running
// Peform any first time intiialization
// return 0 if application is started (and running) or -ENODEV
static int sensorhub_i2c_init_app(struct sensorhub_i2c_data * i2cdata) {
	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	int rc;

	if (sensorhub_i2c_handshake(i2cdata) == 0) {
		// all is good, no need for anything fancy
		return 0;
	}

	dev_info(ddata->dev, "init: app did not respond, retrying after reset\n");
	sensorhub_i2c_reset(ddata, SENSORHUB_RESET_HARD);
	if (sensorhub_i2c_poll_handshake(i2cdata) == 0) {
		return 0;
	}

	// no response from the app, maybe it's stuck in the bootloader?
	rc = i2cdata->bl_detect_and_exit ? i2cdata->bl_detect_and_exit(i2cdata) : 1;

	if (rc < 0) {
		// something went wrong
		return rc;
	}

	if (rc > 0) {
		// an unrecovarable error occred; we can't boot in to application
		return -ENODEV;
	}

	// we exited from a stuck bootloader
	// allow the app to startup one last time
	return sensorhub_i2c_poll_handshake(i2cdata);
}

#ifdef DEBUG
static inline void _gpio_export(int gpio) {
	if (gpio > 0) {
		gpio_export(gpio, 0);
	}
}

static inline void _gpio_unexport(int gpio) {
	if (gpio > 0) {
		gpio_unexport(gpio);
	}
}
#else
static inline void _gpio_export(int gpio) {}
static inline void _gpio_unexport(int gpio) {}
#endif // DEBUG

#ifdef CONFIG_OF
static int sensorhub_i2c_of_request_gpio(struct i2c_client * client,
							const char * gpio_name, // e.g. sensorhub-xxxx (must start with sensorhub)
							int gpio_flags, bool required, int *out) {
	int gpio;
	int rc;
	char of_name[32];
	// create dtb propery name from gpio name like: sensorhub-xxxxx
	scnprintf(of_name, sizeof(of_name), "sensorhub,gpio_%s", gpio_name + sizeof("sensorhub"));

	rc = of_get_named_gpio(client->dev.of_node, of_name, 0);
	if (rc < 0) {
		if (required) {
			dev_err(&client->dev, "error getting device tree config item %s: (%d)",
									of_name, rc);
			return rc;
		}
		return 0;
	}
	if (rc == 0) {
		if (required) {
			dev_err(&client->dev, "invalid gpio (0) %s", of_name);
			return -ENOENT;
		}
		dev_warn(&client->dev, "invalid gpio (0) %s", of_name);
		return 0;
	}

	gpio = rc;
	rc = gpio_request_one(gpio, gpio_flags, gpio_name);
	if (rc) {
		dev_err(&client->dev, "error requesting gpio %s: (%d)", gpio_name, rc);
		return rc;
	}
	*out = gpio;
	return 0;
}

// load platform configuration from device tree
static int sensorhub_i2c_of_probe(struct sensorhub_i2c_data * i2cdata) {

	struct i2c_client * client = i2cdata->client;
	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	bool need_reset = false;
	bool need_bootmode = false;
	const char * dfu_type;

	int rc;

	rc = sensorhub_i2c_of_request_gpio(client, "sensorhub-interrupt",
						  GPIOF_IN, true, &ddata->pdata.gpio_interrupt);
	if (rc) {
		return rc;
	}

	ddata->pdata.irq = gpio_to_irq(ddata->pdata.gpio_interrupt);
	if (ddata->pdata.irq <= 0) {
		dev_err(&client->dev, "could not convert gpio to irq, %d=>%d",
				ddata->pdata.gpio_interrupt, ddata->pdata.irq);
		return -EINVAL ;
	}

	if (of_property_read_string(client->dev.of_node, "sensorhub,dfu_type", &dfu_type) == 0) {
		if (strcmp(dfu_type, "ATMEL_V1") == 0) {
			ddata->pdata.dfu_type = SENSORHUB_DFU_ATMEL_V1;
		} else if (strcmp(dfu_type, "STM32F_V1") == 0) {
			ddata->pdata.dfu_type = SENSORHUB_DFU_STM32F_V1;
			need_bootmode = true;
		} else if (strcmp(dfu_type, "BNO_V1") == 0) {
			ddata->pdata.dfu_type = SENSORHUB_DFU_BNO_V1;
			need_bootmode = true;
		} else {
			dev_err(&client->dev, "unknown dfu_type: %s", dfu_type);
			return -EINVAL;
		}
		need_reset = true;
	} else {
		dev_dbg(&client->dev, "no firmware upgrade support (dfu_type not specified)");
	}

	rc = sensorhub_i2c_of_request_gpio(client, "sensorhub-reset",
						  GPIOF_OUT_INIT_HIGH, need_reset, &ddata->pdata.gpio_reset);
	if (rc) return rc;

	rc = sensorhub_i2c_of_request_gpio(client, "sensorhub-wakeup",
						  GPIOF_OUT_INIT_HIGH, false, &ddata->pdata.gpio_wakeup);
	if (rc) return rc;

	rc = sensorhub_i2c_of_request_gpio(client, "sensorhub-debug",
						  GPIOF_OUT_INIT_LOW, false, &ddata->pdata.gpio_debug);
	if (rc) return rc;

	rc = sensorhub_i2c_of_request_gpio(client, "sensorhub-bootmode",
						  GPIOF_OUT_INIT_LOW, need_bootmode, &ddata->pdata.gpio_bootmode);


	return rc;
}

static inline void _gpio_free(int gpio) {
	if (gpio > 0) {
		gpio_free(gpio);
	}
}

// unload resource requested during of probe
static void sensorhub_i2c_of_remove(struct sensorhub_i2c_data * i2cdata) {
	_gpio_free(i2cdata->ddata.pdata.gpio_bootmode);
	_gpio_free(i2cdata->ddata.pdata.gpio_debug);
	_gpio_free(i2cdata->ddata.pdata.gpio_wakeup);
	_gpio_free(i2cdata->ddata.pdata.gpio_reset);
	_gpio_free(i2cdata->ddata.pdata.gpio_interrupt);
}

#else // CONFIG_OF

static int sensorhub_i2c_of_probe(struct sensorhub_i2c_data * i2cdata) {
	return -ENOSYS;
}

static void sensorhub_i2c_of_remove(struct sensorhub_i2c_data * i2cdata) {
	// this space intentionally left blank
}

#endif // CONFIG_OF

// Allocate Driver instance
static int sensorhub_i2c_probe(struct i2c_client * client,
	const struct i2c_device_id * id) {

	struct sensorhub_i2c_data * i2cdata;
	struct sensorhub_drv_data * ddata;
	int err = 0;

	dev_info(&client->dev, "i2c-client %s on %s #%d\n",
			 client->name, client->adapter->name, client->adapter->nr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	i2cdata = kzalloc(sizeof(struct sensorhub_i2c_data), GFP_KERNEL);
	if (!i2cdata) {
		dev_err(&client->dev, "could not allocate memory for driver data\n");
		err = -ENOMEM;
		goto exit;
	}

	i2cdata->client = client;
	ddata = &i2cdata->ddata;
	if (client->dev.of_node) {
		if ((err = sensorhub_i2c_of_probe(i2cdata))) {
			goto exit_free_data;
		}
	} else if (client->dev.platform_data != NULL) {
		ddata->pdata = *((struct sensorhub_platform_data *) client->dev.platform_data);
	} else {
		dev_err(&client->dev, "No platform_data\n");
		err = -EINVAL;
		goto exit_free_data;
	}

	dev_info(&client->dev, "driver version: " SENSORHUB_VERSION);
	dev_dbg(&client->dev,
			"I2C Platform Configuration:\n"
			"    dfu_type       = %d\n"
			"    irq            = %d\n"
			"    gpio_interrupt = %d\n"
			"    gpio_reset     = %d\n"
			"    gpio_debug     = %d\n"
			"    gpio_wakeup    = %d\n"
			"    gpio_bootmode  = %d\n"
			"    setup          = %-pF\n"
			, (int) ddata->pdata.dfu_type
			, ddata->pdata.irq
			, ddata->pdata.gpio_interrupt
			, ddata->pdata.gpio_reset
			, ddata->pdata.gpio_debug
			, ddata->pdata.gpio_wakeup
			, ddata->pdata.gpio_bootmode
			, ddata->pdata.setup
	);

	i2c_set_clientdata(client, ddata);
	mutex_init(&i2cdata->hw_lock);
	if (!ddata->pdata.gpio_interrupt || !ddata->pdata.irq) {
		dev_err(&client->dev, "irq/gpio not set in platform config\n");
		err = -EINVAL;
		goto exit_free_data;
	}
	set_debug_gpio(0);

	if (ddata->pdata.setup) {
		err = ddata->pdata.setup(&client->dev, 1);
		if (err) {
			dev_err(&client->dev, "platform setup failed\n");
			goto exit_free_data;
		}
	}
#ifdef DEBUG
	_gpio_export(ddata->pdata.gpio_interrupt);
	_gpio_export(ddata->pdata.gpio_reset);
	_gpio_export(ddata->pdata.gpio_debug);
	_gpio_export(ddata->pdata.gpio_wakeup);
	_gpio_export(ddata->pdata.gpio_bootmode);
#endif

	i2cdata->transports.get_report = sensorhub_i2c_get_report;
	i2cdata->transports.set_report = sensorhub_i2c_set_report;
	i2cdata->transports.reset      = sensorhub_i2c_reset;

	if (ddata->pdata.dfu_type == SENSORHUB_DFU_NONE) {
		i2cdata->transports.dfu_abort    = NULL;
		i2cdata->transports.dfu_perform  = NULL;
		i2cdata->transports.dfu_validate = NULL;
		i2cdata->bootmode_state_app = -1;
	}
#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
	else if (ddata->pdata.dfu_type == SENSORHUB_DFU_ATMEL_V1) {
		i2cdata->transports.dfu_abort    = sensorhub_i2c_atmel_dfu_abort;
		i2cdata->transports.dfu_perform  = sensorhub_i2c_atmel_dfu_perform;
		i2cdata->transports.dfu_validate = sensorhub_i2c_atmel_dfu_fw_validate;
		i2cdata->transports.dfu_enterbl  = sensorhub_i2c_atmel_dfu_enterbl;
		i2cdata->bl_detect_and_exit = sensorhub_atmel_bl_detect_and_exit;
		i2cdata->bootmode_state_app = -1;
	}
#endif
#ifdef CONFIG_SENSOR_HUB_I2C_STM32_DFU
	else if (ddata->pdata.dfu_type == SENSORHUB_DFU_STM32F_V1) {
		i2cdata->transports.dfu_abort    = sensorhub_stm32_dfu_abort;
		i2cdata->transports.dfu_perform  = sensorhub_stm32_dfu_perform;
		i2cdata->transports.dfu_validate = sensorhub_stm32_dfu_fw_validate;
		i2cdata->transports.dfu_enterbl  = sensorhub_stm32_dfu_enterbl;
		i2cdata->bl_detect_and_exit = sensorhub_stm32_bl_detect_and_exit;
		i2cdata->bootmode_state_app = 0;
	}
#endif
#ifdef CONFIG_SENSOR_HUB_I2C_BNO_DFU
	else if (ddata->pdata.dfu_type == SENSORHUB_DFU_BNO_V1) {
		i2cdata->transports.dfu_abort    = sensorhub_bno_dfu_abort;
		i2cdata->transports.dfu_perform  = sensorhub_bno_dfu_perform;
		i2cdata->transports.dfu_validate = sensorhub_bno_dfu_fw_validate;
		i2cdata->transports.dfu_enterbl  = sensorhub_bno_dfu_enterbl;
		i2cdata->bl_detect_and_exit = sensorhub_bno_bl_detect_and_exit;
		i2cdata->bootmode_state_app = 1;
	}
#endif
	else {
		dev_err(&client->dev, "dfu_type %d unsupported.\n", (int) ddata->pdata.dfu_type);
		err = -ENOSYS;
		goto exit_free_data;
	}

	if (i2cdata->transports.dfu_perform && !ddata->pdata.gpio_reset) {
		dev_err(&client->dev, "reset gpio not set in platform config\n");
		err = -EINVAL;
		goto exit_free_data;
	}

	if ((ddata->pdata.dfu_type == SENSORHUB_DFU_STM32F_V1 ||
		 ddata->pdata.dfu_type == SENSORHUB_DFU_BNO_V1)
		 && !ddata->pdata.gpio_bootmode) {
		dev_err(&client->dev, "bootmode gpio not set in platform config\n");
		err = -EINVAL;
		goto exit_free_data;
	}

	if (ddata->pdata.gpio_bootmode && i2cdata->bootmode_state_app >= 0) {
		// init gpio for 'app' mode
		gpio_set_value(ddata->pdata.gpio_bootmode, i2cdata->bootmode_state_app);
	}

	ddata->transport = &i2cdata->transports;

	/* register with core */
	//ddata->name = SENSORHUB_I2C_ID;
	//ddata->pdata.irq = gpio_to_irq(GPIO_SENSORHUB_IRQ);
	printk("ddata->pdata.gpio_interrupt = %d, ddata->pdata.irq = %d\n", ddata->pdata.gpio_interrupt,ddata->pdata.irq);
	err = sensorhub_core_register(ddata);
	if (err) {
		goto exit_power_down;
	}

	err = request_threaded_irq(
			ddata->pdata.irq,
			sensorhub_i2c_isr_primary,
			sensorhub_i2c_isr,
			IRQF_TRIGGER_FALLING,
			"sensorhub-hid",
			i2cdata
		);
	if (err < 0) {
		dev_dbg(ddata->dev, "request_irq failed\n");
		goto exit_unregister;
	}

	err = sensorhub_i2c_init_app(i2cdata);

	if (err) {
		if (i2cdata->transports.dfu_perform) {
			if (!(err = ddata->transport->dfu_enterbl(ddata))) {
				if (ddata->state == SENSORHUB_STATE_BOOTLOADER) {
					sensorhub_set_state(ddata, SENSORHUB_STATE_BOOTLOADER_HUNG_APP);
				}
			}
		}
		if (err) {
			dev_err(ddata->dev, "error initializing i2c device: %d\n", err);
			goto exit_free_irq;
		}
	}

	err = device_init_wakeup(ddata->dev, true);
	if (err) {
		dev_err(ddata->dev, "failed initializing device wakeup\n");
	}

#ifdef SENSORHUB_SYSFS_DEBUG
	err = sysfs_create_group(&ddata->dev->kobj, &sensorhub_i2c_attr_group);
	if (err) {
		dev_err(ddata->dev, "error initializing i2c sysfs attrs: %d\n", err);
		goto exit_free_irq;
	}
#endif

	sensorhub_notify_state(ddata);
	dev_info(ddata->dev, "successfully initialized %s (misc=%d)\n",
					ddata->name, ddata->misc.minor);
	return err;

exit_free_irq:
	free_irq(ddata->pdata.irq, i2cdata);
exit_unregister:
	sensorhub_core_unregister(ddata);
exit_power_down:
	if (ddata->pdata.setup) {
		ddata->pdata.setup(&client->dev, 0);
	}
exit_free_data:
	if (i2cdata->client->dev.of_node) {
		sensorhub_i2c_of_remove(i2cdata);
	}
	kfree(i2cdata);
exit:
	return err;
}

// Remove a driver instance
static int sensorhub_i2c_remove(struct i2c_client * client) {
	struct sensorhub_drv_data * const ddata = i2c_get_clientdata(client);
	struct sensorhub_i2c_data * const i2cdata = __i2c(ddata);

	dev_info(ddata->dev, "remove\n");
	while (test_bit(SENSOR_HUB_DFU_IN_FLIGHT, &ddata->flags)) {
		dev_dbg(&client->dev, "waiting for DFU to complete - flags=%lx\n",
				(unsigned long) ddata->flags);
		msleep(500);
	}

#ifdef DEBUG
	_gpio_unexport(ddata->pdata.gpio_bootmode);
	_gpio_unexport(ddata->pdata.gpio_wakeup);
	_gpio_unexport(ddata->pdata.gpio_debug);
	_gpio_unexport(ddata->pdata.gpio_reset);
	_gpio_unexport(ddata->pdata.gpio_interrupt);
#endif

#ifdef SENSORHUB_SYSFS_DEBUG
	sysfs_remove_group(&ddata->dev->kobj,  &sensorhub_i2c_attr_group);
#endif
	sensorhub_core_unregister(ddata);
	if (ddata->report_descriptor.buf) {
		kfree(ddata->report_descriptor.buf);
		ddata->report_descriptor.buf = NULL;
	}
	free_irq(ddata->pdata.irq, i2cdata);
	if (i2cdata->client->dev.of_node) {
		sensorhub_i2c_of_remove(i2cdata);
	}
	if (ddata->pdata.setup) {
		ddata->pdata.setup(&client->dev, 0);
	}
	kfree(i2cdata);

	return 0;
}

#ifdef CONFIG_PM

static int sensorhub_i2c_resume(struct device *dev) {
	struct sensorhub_drv_data * const ddata = dev_get_drvdata(dev);

	if (device_may_wakeup(ddata->dev)) {
		int rc = disable_irq_wake(ddata->pdata.irq);
		if (rc) {
			dev_err(ddata->dev, "disable_irq_wake failed with %d", rc);
		}
	}

	if (sensorhub_i2c_setpower(ddata, SH_HID_PWR_ON)) {
		++ddata->stats.errorCount;
	}

	return 0;
}

static int sensorhub_i2c_suspend(struct device *dev) {
	struct sensorhub_drv_data * const ddata = dev_get_drvdata(dev);

	++ddata->stats.suspendCount;
	if (sensorhub_i2c_setpower(ddata, SH_HID_PWR_SLEEP)) {
		++ddata->stats.errorCount;
	}

	if (device_may_wakeup(ddata->dev)) {
		int rc = enable_irq_wake(ddata->pdata.irq);
		if (rc) {
			dev_err(ddata->dev, "enable_irq_wake failed with %d", rc);
		}
	} else {
		dev_dbg(ddata->dev, "AP wakeup disabled");
	}

	return 0;
}

static const struct dev_pm_ops sensorhub_drv_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sensorhub_i2c_suspend, sensorhub_i2c_resume)
};

#define SH_PM_OPS (&sensorhub_drv_pmops)

#else

#define SH_PM_OPS NULL

#endif /* CONFIG_PM */

#ifdef SENSORHUB_SYSFS_DEBUG
inline struct sensorhub_drv_data * __misc(struct device *dev) {
	struct miscdevice * misc = (struct miscdevice *) dev_get_drvdata(dev);
	return container_of(misc, struct sensorhub_drv_data, misc);
}

static ssize_t sensorhub_i2c_sysfs_store_suspend(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {

	struct sensorhub_drv_data * ddata = __misc(dev);
	int rc = 0;
	if (sysfs_streq("1", buf)) {
		sensorhub_i2c_setpower(ddata, SH_HID_PWR_SLEEP);
	} else {
		sensorhub_i2c_setpower(ddata, SH_HID_PWR_ON);
	}

	return rc ? rc : count;
}

static DEVICE_ATTR(suspend, S_IWUSR, NULL, sensorhub_i2c_sysfs_store_suspend);
static struct attribute * sensorhub_i2c_drv_attrs[] = {
	&dev_attr_suspend.attr,
	NULL /* keep last*/
};

static const struct attribute_group sensorhub_i2c_attr_group = {
	.attrs = sensorhub_i2c_drv_attrs,
};
#endif


/*----------------------------------------------------------------------------*/
static int sensorhub_detect(struct i2c_client *new_client,
		       struct i2c_board_info *info)
{
      struct i2c_adapter *adapter = new_client->adapter;
      SH_MSG("%s:bus[%d] addr[0x%x]\n",__func__, adapter->nr,new_client->addr);
      if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if(twi_id == adapter->nr){		
		for (i2c_num = 0; i2c_num < (sizeof(i2c_address)/sizeof(i2c_address[0]));i2c_num++) {
		new_client->addr = i2c_address[i2c_num];
		SH_MSG("%s:addr= 0x%x,i2c_num:%d\n",__func__,new_client->addr,i2c_num);
		int ret = i2c_smbus_read_byte_data(new_client,HID_REG);
		SH_MSG("Read ID value is :0x%x,ret = 0x%x, ret = %d\n",ret&0x180000,ret,ret);
		if ((ret&0x180000) == VENDOR_ID) {//20-21byte
			SH_MSG("sensorhub detected!\n" );
			strlcpy(info->type, SENSORHUB_I2C_ID, I2C_NAME_SIZE);
			return 0;
		} 
		strlcpy(info->type, SENSORHUB_I2C_ID, I2C_NAME_SIZE);
	}
	return 0;
	}

       return  -ENODEV;
}

//static const unsigned short normal_i2c[] = { 0x27, I2C_CLIENT_END };

static int gsensor_fetch_sysconfig_para(void)
{
	int ret = -1;
	int device_used = -1;
	script_item_u	val;
	script_item_value_type_e  type;
	type = script_get_item("gsensor_para", "gsensor_used", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT  != type) {
	       SH_ERR("%s: gsensor not used [%d] !!\n",__func__,val.val); 		   
		goto script_get_err;
	}
	device_used = val.val;
	if (1 == device_used) {
		type = script_get_item("gsensor_para", "gsensor_twi_id", &val);	
		if(SCIRPT_ITEM_VALUE_TYPE_INT != type){
			SH_ERR("%s: gsensor has no i2c bus [%d] \n",__func__,val.val);
			goto script_get_err;
		}
		twi_id = val.val;		
		SH_MSG( "%s: twi_id is %d. \n", __func__, twi_id);
		ret = 0;	
	} else {
		ret = -1;
	}
	return ret;
	
script_get_err:
	SH_ERR("script_parser_fetch_err!!\n");
	return ret;
}

//---------------------------------------------------

struct sensorhub_dev *sensorhub_devp;  
static sensorhub_gpio_set_t sensorhub_gpio[4];

__s32  sensorhub_gpio_read_one_pin_value(u32 p_handler, const char *gpio_name)
{
	if(p_handler) {		
		return __gpio_get_value(p_handler);
	}	
	printk("sensorhub_gpio_read_one_pin_value, hdl is NULL\n");	
	return -1;
}

__s32  sensorhub_gpio_write_one_pin_value(u32 p_handler, __u32 value_to_gpio, const char *gpio_name)
{
	if(p_handler) {	
		__gpio_set_value(p_handler, value_to_gpio);	
	} else {	
	printk("sensorhub_gpio_write_one_pin_value, hdl is NULL\n");	
	}	

	return 0;
}

int sensorhub_sys_script_get_item(char *main_name, char *sub_name, int value[], int count)
{
    script_item_u   val;
	script_item_value_type_e  type;
	int ret = -1;

	type = script_get_item(main_name, sub_name, &val);
	if(SCIRPT_ITEM_VALUE_TYPE_INT == type) {
		ret = 0;
		*value = val.val;
		printk("%s.%s=%d\n", main_name, sub_name, *value);
	}	else if(SCIRPT_ITEM_VALUE_TYPE_PIO == type) {
		sensorhub_gpio_set_t *gpio_info = (sensorhub_gpio_set_t *)value;

		ret = 0;
		gpio_info->gpio = val.gpio.gpio;
		gpio_info->mul_sel = val.gpio.mul_sel;
		gpio_info->pull = val.gpio.pull;
		gpio_info->drv_level = val.gpio.drv_level;
		gpio_info->data = val.gpio.data;
		memcpy(gpio_info->gpio_name, sub_name, strlen(sub_name)+1);
		//printk("%s.%s gpio=%d,mul_sel=%d,data:%d\n",main_name, sub_name, gpio_info->gpio, gpio_info->mul_sel, gpio_info->data);
	}	else if(SCIRPT_ITEM_VALUE_TYPE_STR == type) {
		memcpy((void*)value, (void*)val.str, strlen(val.str)+1);
		//printk("%s.%s=%s\n",main_name, sub_name, val.str);
	} else {
		ret = -1;
		printk("fetch script data %s.%s fail\n", main_name, sub_name);
	}

	return type;

}

int sensorhub_sys_gpio_request(sensorhub_gpio_set_t *gpio_list, u32 group_count_max)
{
	int ret = 0;
	struct gpio_config pin_cfg;
	char   pin_name[32];
	u32 config;

	if(gpio_list == NULL)
		return 0;

	pin_cfg.gpio = gpio_list->gpio;
	pin_cfg.mul_sel = gpio_list->mul_sel;
	pin_cfg.pull = gpio_list->pull;
	pin_cfg.drv_level = gpio_list->drv_level;
	pin_cfg.data = gpio_list->data;
	ret = gpio_request(pin_cfg.gpio, NULL);
	if(0 != ret) {
		printk("%s failed, gpio_name=%s, gpio=%d, ret=%d\n", __func__, gpio_list->gpio_name, gpio_list->gpio, ret);
		return ret;
	} 
	ret = pin_cfg.gpio;

	if (!IS_AXP_PIN(pin_cfg.gpio)) {
		/* valid pin of sunxi-pinctrl,
		* config pin attributes individually.
		*/
		sunxi_gpio_to_name(pin_cfg.gpio, pin_name);
		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, pin_cfg.mul_sel);
		pin_config_set(SUNXI_PINCTRL, pin_name, config);
		if (pin_cfg.pull != GPIO_PULL_DEFAULT) {
			config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD, pin_cfg.pull);
			pin_config_set(SUNXI_PINCTRL, pin_name, config);
		}
		if (pin_cfg.drv_level != GPIO_DRVLVL_DEFAULT) {
			config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV, pin_cfg.drv_level);
			pin_config_set(SUNXI_PINCTRL, pin_name, config);
		}
		if (pin_cfg.data != GPIO_DATA_DEFAULT) {
			config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT, pin_cfg.data);
			pin_config_set(SUNXI_PINCTRL, pin_name, config);
		}
	} else if (IS_AXP_PIN(pin_cfg.gpio)) {
		/* valid pin of axp-pinctrl,
		* config pin attributes individually.
		*/
		sunxi_gpio_to_name(pin_cfg.gpio, pin_name);
		if (pin_cfg.data != GPIO_DATA_DEFAULT) {
			config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT, pin_cfg.data);
			pin_config_set(AXP_PINCTRL, pin_name, config);
		}
	} else {
		printk("invalid pin [%d] from sys-config\n", pin_cfg.gpio);
	}
	return ret;
}

int sensorhub_sys_gpio_release(int p_handler, s32 if_release_to_default_status)
{
	if(p_handler) {
		gpio_free(p_handler);
	} else {
		printk("OSAL_GPIO_Release, hdl is NULL\n");
	}
	return 0;
}


//----------------------------------------------------------

// TODO -- is id table useful? what to do with it?
static const struct i2c_device_id sensorhub_i2c_id[] = {
	{ SENSORHUB_I2C_ID, 1 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sensorhub_i2c_id);

static struct i2c_driver sensorhub_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSORHUB_I2C_ID,
		.pm    = SH_PM_OPS,
	},

	.id_table = sensorhub_i2c_id,
	.probe    = sensorhub_i2c_probe,
	.remove   = sensorhub_i2c_remove,
	.detect	  = sensorhub_detect, 
	.address_list	= normal_i2c,
};

void sensorhub_gpio_init()
{  
    int err =  -1;
    sensorhub_devp =(struct sensorhub_dev*)kmalloc(sizeof(struct sensorhub_dev),GFP_KERNEL);	
	if(!sensorhub_devp) {  
			printk(" kmalloc sensorhub_devp fail\n");	
			return -1;
	}  
	memset(sensorhub_devp, 0 ,sizeof(struct sensorhub_dev));
    err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_sensor_en",//for gpio_sensor_en
						(int *)&sensorhub_gpio[0],sizeof(sensorhub_gpio_set_t)/sizeof(__u32));
    sensorhub_devp->gpio_sensor_en_handeler = sensorhub_sys_gpio_request(&sensorhub_gpio[0], 1);
	if(!sensorhub_devp->gpio_sensor_en_handeler) {
		printk("sensorhub_devp->gpio_sensor_en_handeler ERROR!!! %d\n");
		return -1;
	}
	sensorhub_gpio_write_one_pin_value(sensorhub_devp->gpio_sensor_en_handeler, 1, NULL);

    err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_bootn",//for gpio_bootn
						(int *)&sensorhub_gpio[1],sizeof(sensorhub_gpio_set_t)/sizeof(__u32));
	sensorhub_devp->gpio_bootn_handeler = sensorhub_sys_gpio_request(&sensorhub_gpio[1], 1);
	if(!sensorhub_devp->gpio_bootn_handeler) {
		printk("sensorhub_devp->gpio_bootn_handeler ERROR!!! %d\n");
		return -1;
	}
	sensorhub_gpio_write_one_pin_value(sensorhub_devp->gpio_bootn_handeler, 1, NULL);
	
	err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_sensor_rstn",//for gpio_sensor_rstn
						(int *)&sensorhub_gpio[2],sizeof(sensorhub_gpio_set_t)/sizeof(__u32));
	sensorhub_devp->gpio_sensor_rstn_handeler = sensorhub_sys_gpio_request(&sensorhub_gpio[2], 1);
	if(!sensorhub_devp->gpio_sensor_rstn_handeler) {
		printk("sensorhub_devp->gpio_sensor_rstn_handeler ERROR!!! %d\n");
		return -1;
	}
	sensorhub_gpio_write_one_pin_value(sensorhub_devp->gpio_sensor_rstn_handeler, 0, NULL);
	mdelay(100);
	sensorhub_gpio_write_one_pin_value(sensorhub_devp->gpio_sensor_rstn_handeler, 1, NULL);
	
	err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_sensor_int",//for gpio_sensor_rstn
						(int *)&sensorhub_gpio[3],sizeof(sensorhub_gpio_set_t)/sizeof(__u32));
	sensorhub_devp->gpio_sensor_int_handeler = sensorhub_sys_gpio_request(&sensorhub_gpio[3], 1);
	if(!sensorhub_devp->gpio_sensor_int_handeler) {
		printk("sensorhub_devp->gpio_sensor_int_handeler ERROR!!! %d\n");
		return -1;
	}
	sensorhub_gpio_write_one_pin_value(sensorhub_devp->gpio_sensor_int_handeler, 0, NULL);

	//update sensorhub_platform_data: gpio_interrupt & irq
	sensorhub_pdata.gpio_interrupt = sensorhub_gpio[3].gpio;
	sensorhub_pdata.irq = gpio_to_irq( sensorhub_gpio[3].gpio);
}
EXPORT_SYMBOL(sensorhub_gpio_init);
//---------------------------------------------------
static int __init sensorhub_board_info_init(void) { 
	int ret = i2c_register_board_info(1, sensorhub_i2c_board_info, ARRAY_SIZE(sensorhub_i2c_board_info));
	printk("ret = i2c_register_board_info = %d \n",ret);
	return ret;
} 
static int sensorhub_i2c_init(void) {
	if(gsensor_fetch_sysconfig_para()){
	printk("sensorhub_i2c:%s: err.\n", __func__);
	return -1;
	} 
	sensorhub_gpio_init();
	SH_MSG( "%s:sensorhub_i2c: init\n", __func__);
	 printk("sensorhub_i2c : sensorhub_i2c inited!\n");
	return i2c_add_driver(&sensorhub_i2c_driver);
}

static void sensorhub_i2c_exit(void) {
	i2c_del_driver(&sensorhub_i2c_driver);
}

arch_initcall(sensorhub_board_info_init);

module_init(sensorhub_i2c_init);
module_exit(sensorhub_i2c_exit);

MODULE_DESCRIPTION("Freespace SensorHub Driver");
MODULE_AUTHOR("Khalid Zubair <kzubair@hcrest.com>");
MODULE_VERSION(SENSORHUB_VERSION);
MODULE_LICENSE("GPL");
