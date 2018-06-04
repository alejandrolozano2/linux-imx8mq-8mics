/*
 * Cirrus Logic CS42448/CS42888 Audio CODEC DAI I2C driver
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Author: Nicolin Chen <Guangyu.Chen@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>

#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_data/adau1977.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "cs4244.h"

static int cs4244_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	pr_info("Roxana cs4244_i2c_probe\n");
	u32 ret = cs4244_probe(&i2c->dev,
			devm_regmap_init_i2c(i2c, &cs4244_regmap_config));
	if (ret)
		return ret;

	pm_runtime_enable(&i2c->dev);
	pm_request_idle(&i2c->dev);

	return 0;
}

static int cs4244_i2c_remove(struct i2c_client *i2c)
{
	pr_info("Roxanacs4244_i2c_remove \n");
	snd_soc_unregister_codec(&i2c->dev);
	pm_runtime_disable(&i2c->dev);

	return 0;
}

static struct i2c_device_id cs4244_i2c_id[] = {
	{"cs4244", (kernel_ulong_t)&cs4244_data},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs4244_i2c_id);

static struct i2c_driver cs4244_i2c_driver = {
	.driver = {
		.name = "cs4244",
		.owner = THIS_MODULE,
		.pm = &cs4244_pm,
	},
	.probe = cs4244_i2c_probe,
	.remove = cs4244_i2c_remove,
	.id_table = cs4244_i2c_id,
};

module_i2c_driver(cs4244_i2c_driver);

MODULE_DESCRIPTION("Cirrus Logic CS42448/CS42888 ALSA SoC Codec I2C Driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
