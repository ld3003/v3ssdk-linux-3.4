/*
 * Platform interfaces for XRadio drivers
 * 
 * Implemented by platform vendor(such as AllwinnerTech).
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/ioport.h>

#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <mach/sys_config.h>

#include "xradio.h"
#include "platform.h"
#include "sbus.h"

extern void wifi_pm_power(int on);
extern int rf_module_power(int onoff);
/* default 0, set 1 keep PMU power on when wlan is closed. */
#define PMU_POWER_WLAN_RETAIN  0
/* default 1, set 0 if you don't use sysconfig. */
#define PLATFORM_SYSCONFIG     1

#define MAX_POWER_NUM          3

#if (PLATFORM_SYSCONFIG)
/* WLAN platform configure.*/
#define WIFI_CONFIG        "wifi_para"
static char * axp_name[MAX_POWER_NUM] = {0};
static u32 power_level[MAX_POWER_NUM] = {0};

static u32 wlan_reset_gpio = 0;
static u32 wlan_irq_gpio   = 0;
static int wlan_bus_id     = 0;
static u32 lpo_use_apclk   = 0;

#else 
/* If no sysconfig, configure param of your platform.*/
static char * axp_name[MAX_POWER_NUM] = {
	"axp22_dldo1",
	"axp22_eldo1",
	NULL,
};
static u32 power_level[MAX_POWER_NUM] = {
	3300000,
	1800000,
	0
};
static u32 wlan_reset_gpio = GPIOL(8);
static u32 wlan_irq_gpio   = GPIOL(9);
static int wlan_bus_id     = 1;
static u32 lpo_use_apclk   = 1;
#endif

/********************* platform Interfaces *********************/
#define xradio_msg(...)    do {printk("[xradio_plat]: "__VA_ARGS__);} while(0)
#if (PLATFORM_SYSCONFIG)
#define SYS_CONFIG_INT(name, value, ret) do { \
	type = script_get_item(WIFI_CONFIG, name, &val); \
	if (SCIRPT_ITEM_VALUE_TYPE_INT == type) { \
		value = val.val; \
		xradio_msg("%s=%d\n", name, value); \
	} else { \
		ret = -1; \
		xradio_msg("%s not config.\n", name); \
	} \
} while(0)

#define SYS_CONFIG_STR(name, ptr, ret) do { \
	type = script_get_item(WIFI_CONFIG, name, &val); \
	if (SCIRPT_ITEM_VALUE_TYPE_STR == type && \
		*(char *)val.str != '\0') { \
		ptr = val.str; \
		xradio_msg("%s=%s\n", name, ptr); \
	} else { \
		ret = -1; \
		xradio_msg("%s not config.\n", name); \
	} \
} while(0)

#define SYS_CONFIG_PIO(name, pio, ret) do { \
	type = script_get_item(WIFI_CONFIG, name, &val); \
	if (SCIRPT_ITEM_VALUE_TYPE_PIO == type) { \
		pio = val.gpio.gpio; \
		xradio_msg("%s=%d\n", name, pio); \
	} else { \
		ret = -1; \
		xradio_msg("%s not config.\n", name); \
	} \
} while(0)
	
static int plat_get_syscfg(void)
{
	int ret = 0;
	script_item_u val;
	script_item_value_type_e type;

	/* Get SDIO/USB config. */
#if defined(CONFIG_XRADIO_SDIO)
	SYS_CONFIG_INT("wifi_sdc_id", wlan_bus_id, ret);
#elif defined(CONFIG_XRADIO_USB)
	SYS_CONFIG_INT("wifi_usbc_id", wlan_bus_id, ret);
#endif
	if (ret)
		return ret;

	/* Get GPIO config. */
	SYS_CONFIG_PIO("wl_host_wake", wlan_irq_gpio, ret);
	if (ret)
		return ret;
	return 0;
}
#endif

/*********************Interfaces called by xradio wlan. *********************/
#ifdef CONFIG_XRADIO_USE_GPIO_IRQ
static u32 gpio_irq_handle = 0;
#ifdef PLAT_ALLWINNER_SUNXI
static irqreturn_t xradio_gpio_irq_handler(int irq, void *sbus_priv)
{
	struct sbus_priv *self = (struct sbus_priv *)sbus_priv;
	unsigned long flags;

	SYS_BUG(!self);
	spin_lock_irqsave(&self->lock, flags);
	if (self->irq_handler)
		self->irq_handler(self->irq_priv);
	spin_unlock_irqrestore(&self->lock, flags);
	return IRQ_HANDLED;
}

int xradio_request_gpio_irq(struct device *dev, void *sbus_priv)
{
	int ret = -1;
	if(!gpio_irq_handle) {
		gpio_request(wlan_irq_gpio, "xradio_irq");
		gpio_direction_input(wlan_irq_gpio);
		gpio_irq_handle = gpio_to_irq(wlan_irq_gpio);
		ret = devm_request_irq(dev, gpio_irq_handle, 
		                      (irq_handler_t)xradio_gpio_irq_handler,
		                       IRQF_TRIGGER_RISING, "xradio_irq", sbus_priv);
		if (IS_ERR_VALUE(ret)) {
			gpio_irq_handle = 0;
		}
	} else {
		xradio_dbg(XRADIO_DBG_ERROR, "%s: error, irq exist already!\n", __func__);
	}

	if (gpio_irq_handle) {
		xradio_dbg(XRADIO_DBG_NIY, "%s: request_irq sucess! irq=0x%08x\n", 
		           __func__, gpio_irq_handle);
		ret = 0;
	} else {
		xradio_dbg(XRADIO_DBG_ERROR, "%s: request_irq err: %d\n", __func__, ret);
		ret = -1;
	}
	return ret;
}
void xradio_free_gpio_irq(struct device *dev, void *sbus_priv)
{
	struct sbus_priv *self = (struct sbus_priv *)sbus_priv;
	if(gpio_irq_handle) {
		//for linux3.4
		devm_free_irq(dev, gpio_irq_handle, self);
		gpio_free(wlan_irq_gpio);
		gpio_irq_handle = 0;
	}
}
#else //PLAT_ALLWINNER_SUN6I
static u32 xradio_gpio_irq_handler(void *sbus_priv)
{
	struct sbus_priv *self = (struct sbus_priv *)sbus_priv;
	unsigned long flags;

	SYS_BUG(!self);
	spin_lock_irqsave(&self->lock, flags);
	if (self->irq_handler)
		self->irq_handler(self->irq_priv);
	spin_unlock_irqrestore(&self->lock, flags);
	return 0;
}
int xradio_request_gpio_irq(struct device *dev, void *sbus_priv)
{
	int ret = -1;
	if(!gpio_irq_handle) {
		gpio_irq_handle = sw_gpio_irq_request(wlan_irq_gpio, TRIG_EDGE_POSITIVE, 
			                                        (peint_handle)xradio_gpio_irq_handler, sbus_priv);
	} else {
		xradio_dbg(XRADIO_DBG_ERROR, "%s: error, irq exist already!\n", __func__);
	}

	if (gpio_irq_handle) {
		xradio_dbg(XRADIO_DBG_NIY, "%s: request_irq sucess! irq=0x%08x\n", 
		           __func__, gpio_irq_handle);
		ret = 0;
	} else {
		xradio_dbg(XRADIO_DBG_ERROR, "%s: request_irq err: %d\n", __func__, ret);
		ret = -1;
	}
	return ret;
}
void xradio_free_gpio_irq(struct device *dev, void *sbus_priv)
{
	struct sbus_priv *self = (struct sbus_priv *)sbus_priv;
	if(gpio_irq_handle) {
		sw_gpio_irq_free(gpio_irq_handle);
		gpio_irq_handle = 0;
	}
}
#endif
#endif /* CONFIG_XRADIO_USE_GPIO_IRQ */

int xradio_sdio_detect(int enable)
{
	int insert = enable;
	MCI_RESCAN_CARD(wlan_bus_id, insert);
	xradio_dbg(XRADIO_DBG_ALWY, "%s SDIO card %d\n", 
	           enable?"Detect":"Remove", wlan_bus_id);
	mdelay(10);
	return 0;
}

int xradio_wlan_power(int on)
{
	if (on) {  //power up.
#if (!PMU_POWER_WLAN_RETAIN)
		rf_module_power(1);
		mdelay(50);
#endif
		wifi_pm_power(1);
		mdelay(50);
		wifi_pm_power(0);
		mdelay(2);
		wifi_pm_power(1);
		mdelay(50);
	} else { //power down.
		wifi_pm_power(0);
#if (!PMU_POWER_WLAN_RETAIN)
		rf_module_power(0);
#endif
	}
	return 0;
}

int  xradio_plat_init(void)
{
	int ret = 0;

#if (PLATFORM_SYSCONFIG)
	ret = plat_get_syscfg();
#endif

#if (PMU_POWER_WLAN_RETAIN)
	ret = rf_module_power(1);
#endif
	return ret;
}

void xradio_plat_deinit(void)
{
#if (PMU_POWER_WLAN_RETAIN)
	rf_module_power(0);
#endif
}

/******************************************************************************************/