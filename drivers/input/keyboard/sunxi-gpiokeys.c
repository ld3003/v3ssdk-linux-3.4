#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/sys_config.h>
#include <linux/gpio.h>

static struct input_dev *sunxigk_dev;
static struct timer_list *s_timer;

struct gpiokey_pd {
	char name[32];
	struct gpio_config gpio_key;
	int key_code;
	int key_value;
	int key_default_state;
};

enum {
	SHUTDOWN_KEY = 0x74,
	UPDATE_KEY = 0x72,
	PHOTO_KEY = 0x8b,
	SUPPLY_KEY = 0x66,
};

enum {
	DEBUG_INIT = 1U << 0,
	DEBUG_INT = 1U << 1,
	DEBUG_DATA_INFO = 1U << 2,
	DEBUG_SUSPEND = 1U << 3,
};
static u32 debug_mask = 0;
#define dprintk(level_mask, fmt, arg...)	if (unlikely(debug_mask & level_mask)) \
	printk(KERN_DEBUG fmt , ## arg)

module_param_named(debug_mask, debug_mask, int, 0644);

static struct gpiokey_pd mul_gpiokey_io[4];
static int sunxi_gpiokey_script_init(void)
{
	script_item_u val;
	script_item_value_type_e type;

	type = script_get_item("gpio_key", "key_used", &val);

	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		pr_err("%s: gpio_key para failed\n", __func__);
		return -1;
	}

	if (!val.val) {
		pr_err("this modules is not used\n");
		return -1;
	}
	/*power key*/
	type = script_get_item("gpio_key", "power_key", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		pr_err("%s: get power key io err! \n", __func__);
		return -1;
	}

	if (gpio_request(val.gpio.gpio, NULL)) {
		pr_err("%s: request power_key io err! \n", __func__);
		return -1;
	}
	gpio_direction_input(val.gpio.gpio);
	strcpy(mul_gpiokey_io[0].name, "power_key");
	mul_gpiokey_io[0].gpio_key.gpio = val.gpio.gpio;
	mul_gpiokey_io[0].key_code = SHUTDOWN_KEY;
	mul_gpiokey_io[0].key_value = __gpio_get_value(val.gpio.gpio);
	mul_gpiokey_io[0].key_default_state = 0;

	/*update key*/
	type = script_get_item("gpio_key", "update_key", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		pr_err("%s: get update key io err! \n", __func__);
		return -1;
	}
	if (gpio_request(val.gpio.gpio, NULL)) {
		pr_err("%s: request update_key io err! \n", __func__);
		return -1;
	}
	gpio_direction_input(val.gpio.gpio);
	strcpy(mul_gpiokey_io[1].name, "update_key");
	mul_gpiokey_io[1].gpio_key.gpio = val.gpio.gpio;
	mul_gpiokey_io[1].key_code = UPDATE_KEY;
	mul_gpiokey_io[1].key_value = __gpio_get_value(val.gpio.gpio);
	mul_gpiokey_io[1].key_default_state = 1;

	/*phone key*/
	type = script_get_item("gpio_key", "photo_key", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		pr_err("%s: get photo_key io err! \n", __func__);
		return -1;
	}
	if (gpio_request(val.gpio.gpio, NULL)) {
		pr_err("%s: request photo_key io err! \n", __func__);
		return -1;
	}
	gpio_direction_input(val.gpio.gpio);
	strcpy(mul_gpiokey_io[2].name, "photo_key");
	mul_gpiokey_io[2].gpio_key.gpio = val.gpio.gpio;
	mul_gpiokey_io[2].key_code = PHOTO_KEY;
	mul_gpiokey_io[2].key_value = __gpio_get_value(val.gpio.gpio);
	mul_gpiokey_io[2].key_default_state = 1;

	/*supply key*/
	type = script_get_item("gpio_key", "supply_key", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		pr_err("%s: get supply key io err! \n", __func__);
		return -1;
	}
	if (gpio_request(val.gpio.gpio, NULL)) {
		pr_err("%s: request supply_key io err! \n", __func__);
		return -1;
	}
	gpio_direction_input(val.gpio.gpio);
	strcpy(mul_gpiokey_io[3].name, "supply_key");
	mul_gpiokey_io[3].gpio_key.gpio = val.gpio.gpio;
	mul_gpiokey_io[3].key_code = SUPPLY_KEY;
	mul_gpiokey_io[3].key_value = __gpio_get_value(val.gpio.gpio);
	mul_gpiokey_io[3].key_default_state = 0;

	return 0;
}

static void gpiokey_timer_handle(unsigned long arg)
{
	int gpio_data = 0;
	int i;

	/*check power key*/
	for (i = 0; i < 4; i++) {
		gpio_data = __gpio_get_value(mul_gpiokey_io[i].gpio_key.gpio);

		if (gpio_data != mul_gpiokey_io[i].key_value) {
			if (gpio_data ^ mul_gpiokey_io[i].key_default_state) {
				input_report_key(sunxigk_dev, mul_gpiokey_io[i].key_code, 1);
				input_sync(sunxigk_dev);
				dprintk(DEBUG_INT, "report data: power key down,code=0x%x\n", mul_gpiokey_io[i].key_code);
			} else {
				input_report_key(sunxigk_dev, mul_gpiokey_io[i].key_code, 0);
				input_sync(sunxigk_dev);
				dprintk(DEBUG_INT, "report data: key up,code=0x%x\n", mul_gpiokey_io[i].key_code);
			}
			mul_gpiokey_io[i].key_value = gpio_data;
		}
	}

	mod_timer(s_timer, jiffies + (HZ/100));
}

static int __init sunxi_gpiokey_init(void)
{
	int err = 0;

	pr_info("sunxi_gpiokey_init init\n");
	if (sunxi_gpiokey_script_init()) {
		err = -EFAULT;
		goto fail1;
	}

	sunxigk_dev = input_allocate_device();
	if (!sunxigk_dev) {
		pr_err("sunxigk:not enough memory for input device\n");
		err = -ENOMEM;
		goto fail1;
	}

	sunxigk_dev->name = "sunxi-gpiokeys";
	sunxigk_dev->phys = "sunxikbd/input0";
	sunxigk_dev->id.bustype = BUS_HOST;
	sunxigk_dev->id.vendor = 0x0001;
	sunxigk_dev->id.product = 0x0001;
	sunxigk_dev->id.version = 0x0100;

#ifdef REPORT_REPEAT_KEY_BU_INPUT_CORE
	sunxigk_dev->evkit[0] = BIT_MASK(EV_KEY)|BIT_MASK(EV_REP);
	printk(KERN_DEBUG "REPORT_REPEAT_KEY_BY_INPUT_CORE is defined, support report repeat key value. \n");
#else
	sunxigk_dev->evbit[0] = BIT_MASK(EV_KEY);
#endif
	set_bit(SHUTDOWN_KEY, sunxigk_dev->keybit);
	set_bit(UPDATE_KEY, sunxigk_dev->keybit);
	set_bit(PHOTO_KEY, sunxigk_dev->keybit);
	set_bit(SUPPLY_KEY, sunxigk_dev->keybit);

	err = input_register_device(sunxigk_dev);
	if (err)
		goto fail2;

	s_timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!s_timer) {
		err = -ENOMEM;
		pr_err("sunxigk: request timer failed\n");
		goto fail3;
	}
	init_timer(s_timer);
	s_timer->function = &gpiokey_timer_handle;
	mod_timer(s_timer, jiffies + (HZ/1));
	return 0;

fail3:
	input_unregister_device(sunxigk_dev);
fail2:
	input_free_device(sunxigk_dev);
fail1:
	printk(KERN_DEBUG "sunxi_gpiokey_init failed\n");
	return err;
}

static void __exit sunxi_gpiokey_exit(void)
{
	script_item_u val;
	script_item_value_type_e type;
	pr_info("sunxi_gpiokey_exit\n");

	type = script_get_item("gpio_key", "key_used", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		pr_err("%s: gpio_key para failed\n", __func__);
		return;
	}

	if (!val.val) {
		pr_err("this modules is not used\n");
		return ;
	}

	input_unregister_device(sunxigk_dev);
}

module_init(sunxi_gpiokey_init);
module_exit(sunxi_gpiokey_exit);

MODULE_AUTHOR("<@>");
MODULE_DESCRIPTION("sunxi gpio key event driver");
MODULE_LICENSE("GPL");
