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

#ifndef __SENSORHUB_GPIO__
#define __SENSORHUB_GPIO__ __FILE__

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
#include <mach/sys_config.h>

#include "sensorhub-core.h"
#include "sensorhub-i2c.h"
#include <linux/sensorhub.h> 
#define read32(addr)       (*((volatile unsigned int  *)(addr)))  
#define write32(addr,value) do{*(volatile unsigned int *)(addr) = value;}while(0)

struct sensorhub_dev
{
	unsigned int gpio_sensor_en_handeler;
	unsigned int gpio_bootn_handeler;
	unsigned int gpio_sensor_rstn_handeler;
	unsigned int gpio_sensor_int_handeler;
};  

typedef struct
{
	char  gpio_name[32];
	int port;
	int port_num;
	int mul_sel;
	int pull;
	int drv_level;
	int data;
	int gpio;
} sensorhub_gpio_set_t;

void sensorhub_gpio_init();

#endif
