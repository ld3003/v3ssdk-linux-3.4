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


#include "sensorhub-gpio.h"

struct sensorhub_dev *sensorhub_devp;  

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
	__wrn("sensorhub_gpio_write_one_pin_value, hdl is NULL\n");	
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
		__inf("%s.%s=%d\n", main_name, sub_name, *value);
	}	else if(SCIRPT_ITEM_VALUE_TYPE_PIO == type) {
		ir_cut_gpio_set_t *gpio_info = (ir_cut_gpio_set_t *)value;

		ret = 0;
		gpio_info->gpio = val.gpio.gpio;
		gpio_info->mul_sel = val.gpio.mul_sel;
		gpio_info->pull = val.gpio.pull;
		gpio_info->drv_level = val.gpio.drv_level;
		gpio_info->data = val.gpio.data;
		memcpy(gpio_info->gpio_name, sub_name, strlen(sub_name)+1);
		__inf("%s.%s gpio=%d,mul_sel=%d,data:%d\n",main_name, sub_name, gpio_info->gpio, gpio_info->mul_sel, gpio_info->data);
	}	else if(SCIRPT_ITEM_VALUE_TYPE_STR == type) {
		memcpy((void*)value, (void*)val.str, strlen(val.str)+1);
		__inf("%s.%s=%s\n",main_name, sub_name, val.str);
	} else {
		ret = -1;
		__inf("fetch script data %s.%s fail\n", main_name, sub_name);
	}

	return type;

}

int sensorhub_sys_gpio_request(ir_cut_gpio_set_t *gpio_list, u32 group_count_max)
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
		__wrn("%s failed, gpio_name=%s, gpio=%d, ret=%d\n", __func__, gpio_list->gpio_name, gpio_list->gpio, ret);
		return ret;
	} else {
		__inf("%s, gpio_name=%s, gpio=%d, ret=%d\n", __func__, gpio_list->gpio_name, gpio_list->gpio, ret);
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
		pr_warn("invalid pin [%d] from sys-config\n", pin_cfg.gpio);
	}

	return ret;
}

int sensorhub_sys_gpio_release(int p_handler, s32 if_release_to_default_status)
{
	if(p_handler) {
		gpio_free(p_handler);
	} else {
		__wrn("OSAL_GPIO_Release, hdl is NULL\n");
	}
	return 0;
}


//----------------------------------------------------------

void sensorhub_gpio_init()
{  
    int err =  -1;
    err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_sensor_en",
						(int *)&ir_cut_gpio[0],sizeof(ir_cut_gpio_set_t)/sizeof(__u32));
	
    sensorhub_devp->ir_cut_gpio_handler[0] = sensorhub_sys_gpio_request(&ir_cut_gpio[0], 1);
	if(!sensorhub_devp->ir_cut_gpio_handler[0]) {
		sensorhub_gpio_write_one_pin_value(sensorhub_devp->ir_cut_gpio_handler[0], 1, NULL);
	}
	printk(" step 1:init sensorhub gpio\n");

    err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_bootn",
						(int *)&ir_cut_gpio[1],sizeof(ir_cut_gpio_set_t)/sizeof(__u32));
	
	sensorhub_devp->ir_cut_gpio_handler[1] = sensorhub_sys_gpio_request(&ir_cut_gpio[1], 1);
	
	if(!sensorhub_devp->ir_cut_gpio_handler[1]) {
		sensorhub_gpio_write_one_pin_value(sensorhub_devp->ir_cut_gpio_handler[1], 1, NULL);
	}
	printk(" step 2:init sensorhub gpio\n");
	
	 err = sensorhub_sys_script_get_item("gsensor_sensorhub", "gpio_sensor_rstn",
						(int *)&ir_cut_gpio[2],sizeof(ir_cut_gpio_set_t)/sizeof(__u32));
	
	sensorhub_devp->ir_cut_gpio_handler[2] = sensorhub_sys_gpio_request(&ir_cut_gpio[2], 1);
	
	if(!sensorhub_devp->ir_cut_gpio_handler[2]) {
	printk(" step 3:init sensorhub gpio\n");
		sensorhub_gpio_write_one_pin_value(sensorhub_devp->ir_cut_gpio_handler[2], 0, NULL);
		mdelay(100);
		sensorhub_gpio_write_one_pin_value(sensorhub_devp->ir_cut_gpio_handler[2], 1, NULL);
	}
	printk(" step 4:init sensorhub gpio\n");
}
EXPORT_SYMBOL(sensorhub_gpio_init);