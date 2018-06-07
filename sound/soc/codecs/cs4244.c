/*
 * Cirrus Logic CS42448/CS42888 Audio CODEC Digital Audio Interface (DAI) driver
 *
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc.
 *
 * Author: Nicolin Chen <Guangyu.Chen@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
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

#include "cs4244.h"

#define CS4244_NUM_SUPPLIES 2
static const char *const cs4244_supply_names[CS4244_NUM_SUPPLIES] = {
	"VA",
	"VD",
};

#define CS4244_FORMATS	(SNDRV_PCM_FMTBIT_S32_LE)
static int cs4244_fill_defaults(struct regmap *regmap,  struct reg_default * reg_vals);


struct work_data {

	struct work_struct work;
	int data;
};

/* codec private data */
struct cs4244_priv {
	struct work_data work_regmap;
	struct regulator_bulk_data supplies[CS4244_NUM_SUPPLIES];
	const struct cs4244_driver_data *drvdata;
	struct regmap *regmap;
	struct clk *clk;

	bool slave_mode;
	unsigned long sysclk;
	int rate[2];
	u32 tx_channels;
	/*GPIO Reset*/
	struct gpio_desc *                            gpio_nreset;
	struct workqueue_struct * wq; 
};



static const struct reg_default cs4244_reg[] = {
	{ 0x06, 0x04  }, /*Clock & SP Sel*/
	{ 0x07, 0xff  }, /*Sample Widh*/
	{ 0x08, 0x48  }, /*SP Control*/
	{ 0x09, 0x01  }, /*SP Data Sel*/
	{ 0x0a, 0xff  }, /*Reserved*/
	{ 0x0b, 0xff  }, /*Reserved*/ 
	{ 0x0c, 0xff  }, /*Reserved*/
	{ 0x0d, 0xff  }, /*Reserved*/
	{ 0x0e, 0x00  }, /*Reserved*/
	{ 0x0f, 0xc0  }, /*ADC Control 1*/
	{ 0x10, 0xff  }, /*ADC Control 2*/
	{ 0x11, 0xe0  }, /*Reserved*/
	{ 0x12, 0xe0  }, /*DAC Control 1*/
	{ 0x13, 0xe0  }, /*DAC Control 2*/
	{ 0x14, 0xbf  }, /*DAC Control 3*/
	{ 0x15, 0x1f  }, /*DAC Control 4*/
	{ 0x16, 0x87  }, /*Volume Mode*/
	{ 0x17, 0x10  }, /*MASTER Volume*/
	{ 0x18, 0x10  }, /*DAC1 Volume*/
	{ 0x19, 0x10  }, /*DAC2 Volume*/
	{ 0x1a, 0x10  }, /*DAC3 Volume*/
	{ 0x1b, 0x10  }, /*DAC4 Volume*/
	{ 0x1c, 0x10  }, /*Reserved*/
	{ 0x1d, 0xba  }, /*Reserved*/
	{ 0x1e, 0x40  }, /*Interrupt Control*/
	{ 0x1f, 0x10  }, /*Interrupt Mask 1*/
	{ 0x20, 0x20  }, /*Interrupt Mask 2*/
	{ 0x21, 0x00  }, /*Interrupt Notification 1*/
	{ 0x22, 0x00  }, /*Interrupt Notification 1*/

};

/* -90dB to 0dB with step of 0.38dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -8955, 38, 0);

static const char *const cs4244_adc_single[] = { "Differential", "Single-Ended" };
static const char *const cs4244_szc[] = { "Immediate Change","Soft Ramp"};

static const struct soc_enum dac_szc_enum =
	SOC_ENUM_SINGLE(CS4244_DACCTL3, 6, 2, cs4244_szc);

static const struct snd_kcontrol_new cs4244_snd_controls[] = {
	SOC_SINGLE_TLV("Master Volume", CS4244_VOLAOUT, 0,255, 1,dac_tlv),
	SOC_DOUBLE_R_TLV("DAC1 Playback Volume", CS4244_VOLAOUT1,
			 CS4244_VOLAOUT2, 0, 0xff, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("DAC2 Playback Volume", CS4244_VOLAOUT3,
			 CS4244_VOLAOUT4, 0, 0xff, 1, dac_tlv),
	SOC_DOUBLE("DAC1 Invert Switch", CS4244_DACCTL2, 0, 1, 1, 0),
	SOC_DOUBLE("DAC2 Invert Switch", CS4244_DACCTL2, 2, 3, 1, 0),
	SOC_SINGLE("DAC De-emphasis Switch", CS4244_DACCTL1, 4, 1, 0),

	SOC_DOUBLE("ADC1 Invert Switch", CS4244_ADCCTL1, 0, 1, 1, 0),
	SOC_DOUBLE("ADC2 Invert Switch", CS4244_ADCCTL1, 2, 3, 1, 0),
};


static const struct snd_soc_dapm_widget cs4244_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC1", "Playback", CS4244_DACCTL4, 1, 1),
	SND_SOC_DAPM_DAC("DAC2", "Playback", CS4244_DACCTL4, 2, 1),
	SND_SOC_DAPM_DAC("DAC3", "Playback", CS4244_DACCTL4, 3, 1),
	SND_SOC_DAPM_DAC("DAC4", "Playback", CS4244_DACCTL4, 4, 1),

	SND_SOC_DAPM_OUTPUT("AOUT1L"),
	SND_SOC_DAPM_OUTPUT("AOUT1R"),
	SND_SOC_DAPM_OUTPUT("AOUT2L"),
	SND_SOC_DAPM_OUTPUT("AOUT2R"),
	SND_SOC_DAPM_OUTPUT("AOUT3L"),
	SND_SOC_DAPM_OUTPUT("AOUT3R"),
	SND_SOC_DAPM_OUTPUT("AOUT4L"),
	SND_SOC_DAPM_OUTPUT("AOUT4R"),

	SND_SOC_DAPM_ADC("ADC1", "Capture", CS4244_ADCCTL2, 5, 1),
	SND_SOC_DAPM_ADC("ADC2", "Capture", CS4244_ADCCTL2, 6, 1),
	
	SND_SOC_DAPM_ADC("ADC3", "Capture", CS4244_ADCCTL2, 7, 1),
	SND_SOC_DAPM_ADC("ADC4", "Capture", CS4244_ADCCTL2, 8, 1),
	
	SND_SOC_DAPM_INPUT("AIN1L"),
	SND_SOC_DAPM_INPUT("AIN1R"),
	SND_SOC_DAPM_INPUT("AIN2L"),
	SND_SOC_DAPM_INPUT("AIN2R"),
	
	SND_SOC_DAPM_INPUT("AIN3L"),
	SND_SOC_DAPM_INPUT("AIN3R"),
	SND_SOC_DAPM_INPUT("AIN4L"),
	SND_SOC_DAPM_INPUT("AIN4R"),
	
};

static const struct snd_soc_dapm_route cs4244_dapm_routes[] = {
	/* Playback */
	{ "AOUT1L", NULL, "DAC1" },
	{ "AOUT1R", NULL, "DAC1" },

	{ "AOUT2L", NULL, "DAC2" },
	{ "AOUT2R", NULL, "DAC2" },

	{ "AOUT3L", NULL, "DAC3" },
	{ "AOUT3R", NULL, "DAC3" },

	{ "AOUT4L", NULL, "DAC4" },
	{ "AOUT4R", NULL, "DAC4" },

	/* Record */
	{ "AIN1L", NULL, "ADC1" },
	{ "AIN1R", NULL, "ADC1" },

	{ "AIN2L", NULL, "ADC2" },
	{ "AIN2R", NULL, "ADC2" },

	{ "AIN3L", NULL, "ADC3" },
	{ "AIN3R", NULL, "ADC3" },

	{ "AIN4L", NULL, "ADC4" },
	{ "AIN4R", NULL, "ADC4" },


};

struct cs4244_ratios {
	unsigned int mfreq;
	unsigned int min_mclk;
	unsigned int max_mclk;
	unsigned int ratio[3];
};

static const struct cs4244_ratios cs4244_ratios[] = {
	{ 0, 1029000, 12800000, {256, 128, 64} },
	{ 2, 1536000, 19200000, {384, 192, 96} },
	{ 4, 2048000, 25600000, {512, 256, 128} },
	{ 6, 3072000, 38400000, {768, 384, 192} },
	{ 8, 4096000, 51200000, {1024, 512, 256} },
};

static int cs4244_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4244_priv *cs4244 = snd_soc_codec_get_drvdata(codec);

	cs4244->sysclk = freq;

	return 0;
}

static int cs4244_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4244_priv *cs4244 = snd_soc_codec_get_drvdata(codec);
	u32 val;
	
	pr_info("cs4244_set_dai_fmt\n");

	/* Set DAI format */
	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		val = CS4244_INTF_DAC_SPF_LEFTJ;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = CS4244_INTF_DAC_SPF_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		val = CS4244_INTF_DAC_SPF_TDM;
		break;
	default:
		dev_err(codec->dev, "unsupported dai format\n");
		return -EINVAL;
	}

	regmap_update_bits(cs4244->regmap, CS4244_PORT_CTRL,
			   CS4244_INTF_DAC_SPF_MASK, val);

	/* Set master/slave audio interface */
	switch (format & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		cs4244->slave_mode = true;
		val = CS4244_INTF_DAC_SLAVE; 
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		cs4244->slave_mode = false;
		val = CS4244_INTF_DAC_MASTER;
		break;
	default:
		dev_err(codec->dev, "unsupported master/slave mode\n");
		return -EINVAL;
	}

	regmap_update_bits(cs4244->regmap, CS4244_PORT_CTRL,
		CS4244_INTF_DAC_MASTER_SLAVE_MASK, val);


	return 0;
}

static int cs4244_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs4244_priv *cs4244 = snd_soc_codec_get_drvdata(codec);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 rate = params_rate(params);

	pr_info("cs4244_hw_params\n");
	/*Hardcoded to x512*/
//	regmap_write(cs4244->regmap, CS4244_MCLK_SPD, 0x34 /* 0x30 */); 	

	return 0;
}

static int cs4244_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct cs4244_priv *cs4244 = snd_soc_codec_get_drvdata(codec);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	cs4244->rate[substream->stream] = 0;

	pr_info("cs4244_hw_free\n");

	return 0;
}

static int cs4244_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *cpu_dai)
{
	struct snd_soc_codec *codec = cpu_dai->codec;
	struct cs4244_priv *cs4244 = snd_soc_codec_get_drvdata(codec);
	int val1;

	pr_info("cs4244_trigger\n");

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

	
	cs4244->work_regmap.data = SNDRV_PCM_TRIGGER_START;
	break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:	
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH: 

	cs4244->work_regmap.data = SNDRV_PCM_TRIGGER_STOP;
	break;
	
	default:
		return -EINVAL;
	}

        queue_work(cs4244->wq, &cs4244->work_regmap.work);	

	return 0;
}

static int cs4244_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	pr_info("cs4244_trigger\n");

	return 0;
}
static int cs4244_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs4244_priv *cs4244 = snd_soc_codec_get_drvdata(codec);

	pr_info("cs4244_digital_mute\n");

	if(mute)
	{
		snd_soc_write(codec, CS4244_DACCTL3, 0xbf);
		snd_soc_write(codec, CS4244_DACCTL4, 0x1f);

	}
	return 0;
}

static const struct snd_soc_dai_ops cs4244_dai_ops = {
	.set_fmt	= cs4244_set_dai_fmt,
	.set_sysclk	= cs4244_set_dai_sysclk,
	.trigger 	= cs4244_trigger,
	.hw_free	= cs4244_hw_free,
	.digital_mute	= cs4244_digital_mute,
};

static struct snd_soc_dai_driver cs4244_dai = {
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = CS4244_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = CS4244_FORMATS,
	},
	.ops = &cs4244_dai_ops,
};



static int cs4244_fill_defaults(struct regmap *regmap,  struct reg_default * reg_vals)
{

        int i, ret, val, index;
        for (i = 0; i < ARRAY_SIZE(cs4244_reg); i++) {

                index = reg_vals[i].reg;
                val = reg_vals[i].def;
                ret = regmap_write(regmap, index, val);
        }

        return 0;

}

static bool cs4244_volatile_register(struct device *dev, unsigned int reg)
{
	switch(reg){
	case CS4244_INT1:
	case CS4244_INT2:
		return true;
	default:
		return false;
	}
}

static int read_chipid(struct cs4244_priv *cs4244)
{
	int ret, val, chipid;
	int val1,val2,val3; 
	ret = val = chipid = 0;
	ret = regmap_read(cs4244->regmap, CS4244_CHIPID1, &val1);
	ret |= regmap_read(cs4244->regmap, CS4244_CHIPID2, &val2);
	ret |= regmap_read(cs4244->regmap, CS4244_CHIPID3, &val3);
	if(ret < 0)
		return ret;
	
	chipid = (val1 >> CS4244_CHIPID_CHIP_ID_SHIFTA) & 0x03;
	chipid = chipid << 2;	
	chipid = chipid | ((val1 >> CS4244_CHIPID_CHIP_ID_SHIFTB) & 0x03);
	chipid = chipid << 2;
	
	chipid = chipid | ((val2 >> CS4244_CHIPID_CHIP_ID_SHIFTA) & 0x03);
	chipid = chipid << 2;	
	chipid = chipid | ((val2 >> CS4244_CHIPID_CHIP_ID_SHIFTB) & 0x03);
	chipid = chipid << 2;

	chipid = chipid | ((val3 >> CS4244_CHIPID_CHIP_ID_SHIFTA) & 0x03);
	chipid = chipid << 2;	
	chipid = chipid | ((val3 >> CS4244_CHIPID_CHIP_ID_SHIFTB) & 0x03);
	chipid = chipid << 2;

	return chipid;
}

static bool cs4244_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS4244_CHIPID1:
	case CS4244_CHIPID2:
	case CS4244_CHIPID3:
	case CS4244_RSVD:
	case CS4244_REVID:
	case CS4244_RSVD0A:
	case CS4244_RSVD0B:
	case CS4244_RSVD0C:
	case CS4244_RSVD0D:
	case CS4244_RSVD0E:
	case CS4244_RSVD11:
	case CS4244_RSVD1C:
	case CS4244_RSVD1D:
		return false;
	default:
		return true;
	}
}

const struct regmap_config cs4244_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = CS4244_NUMREGS,
	.reg_defaults = cs4244_reg,
	.num_reg_defaults = ARRAY_SIZE(cs4244_reg),
	.volatile_reg = cs4244_volatile_register,
	.writeable_reg = cs4244_writeable_register,
	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(cs4244_regmap_config);

static int cs4244_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static const struct snd_soc_codec_driver cs4244_driver = {
	.probe = cs4244_codec_probe,
	.idle_bias_off = true,

	.component_driver = { 
		.controls = cs4244_snd_controls,
		.num_controls = ARRAY_SIZE(cs4244_snd_controls),
		.dapm_widgets = cs4244_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(cs4244_dapm_widgets),
		.dapm_routes = cs4244_dapm_routes,
		.num_dapm_routes = ARRAY_SIZE(cs4244_dapm_routes),
	},
};
#if 0
const struct cs4244_driver_data cs4244_data = {
	.name = "cs4244",
	.num_adcs = 0,
};
EXPORT_SYMBOL_GPL(cs4244_data);
#endif

static const struct of_device_id cs4244_of_match[] = {
	{ .compatible = "cirrus,cs4244", .data = &cs4244_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cs4244_of_match);
EXPORT_SYMBOL_GPL(cs4244_of_match);

static void regmap_handler(struct work_struct *work)
{
    struct cs4244_priv *cs4244 = (struct cs4244_priv *)work;
    int cmd = cs4244->work_regmap.data;
    unsigned int status = 0;
    int ret, loop = 5;

	switch(cmd)
	{
	case SNDRV_PCM_TRIGGER_START:


	//Reset Codec
	gpiod_set_value_cansleep(cs4244->gpio_nreset, 1);
	mdelay(2);
	gpiod_set_value_cansleep(cs4244->gpio_nreset, 0);
	
	//Configure x512
        regmap_write(cs4244->regmap, CS4244_MCLK_SPD, 0x30 /* 0x30 */);

	/*Clear PDN DAC_Mute */

	regmap_write(cs4244->regmap,CS4244_DACCTL4,0x1f);
	regmap_write(cs4244->regmap,CS4244_DACCTL4,0x10);	
	regmap_write(cs4244->regmap,CS4244_DACCTL3, 0xb0);
	regmap_write(cs4244->regmap,CS4244_ADCCTL2, 0x00);
	//while(loop)
	{
	   ret = regmap_read(cs4244->regmap,0x21, &status);
	   if (ret < 0)
		pr_err("Failed to read status register: %d\n", ret);
	   if (status && 0xf0)
	   {
	      regmap_write(cs4244->regmap,CS4244_DACCTL4,0x1f);
	      regmap_write(cs4244->regmap,CS4244_DACCTL4,0x10);
	      regmap_write(cs4244->regmap,CS4244_ADCCTL2,0x0f);
	      regmap_write(cs4244->regmap,CS4244_ADCCTL2,0x00);
	   }
	   else{
	         break;
	    }
	   loop--;
	}
	break;
	case SNDRV_PCM_TRIGGER_STOP:
	break;

	default:

        regmap_update_bits(cs4244->regmap,CS4244_DACCTL3,0x0f, 0xbf);
        regmap_update_bits(cs4244->regmap,CS4244_DACCTL4,0x0f, 0x1f);
        regmap_update_bits(cs4244->regmap,CS4244_DACCTL4,0x0f, 0x10);
        regmap_update_bits(cs4244->regmap,CS4244_DACCTL4,0x0f, 0x1f);

        /*PDN DAC*/
        regmap_update_bits(cs4244->regmap,CS4244_ADCCTL2,0xff, 0xff);	

	}

}

int cs4244_probe(struct device *dev, struct regmap *regmap)
{
	const struct of_device_id *of_id = of_match_device(cs4244_of_match, dev);
	struct cs4244_priv *cs4244;
	int ret, val, i;
	cs4244 = devm_kzalloc(dev, sizeof(*cs4244), GFP_KERNEL);
	if (cs4244 == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, cs4244);

	if (of_id)
		cs4244->drvdata = of_id->data;

	if (!cs4244->drvdata) {
		dev_err(dev, "failed to find driver data\n");
		return -EINVAL;
	}

	cs4244->clk = devm_clk_get(dev, "mclk");
	if (IS_ERR(cs4244->clk)) {
		dev_err(dev, "failed to get the clock: %ld\n",
				PTR_ERR(cs4244->clk));
		return -EINVAL;
	}

	cs4244->sysclk = clk_get_rate(cs4244->clk);

	for (i = 0; i < ARRAY_SIZE(cs4244->supplies); i++)
		cs4244->supplies[i].supply = cs4244_supply_names[i];

	ret = devm_regulator_bulk_get(dev,
			ARRAY_SIZE(cs4244->supplies), cs4244->supplies);
	if (ret) {
		dev_err(dev, "failed to request supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(cs4244->supplies),
				    cs4244->supplies);
	if (ret) {
		dev_err(dev, "failed to enable supplies: %d\n", ret);
		return ret;
	}

	/* Make sure hardware reset done */
	msleep(5);

	cs4244->regmap = regmap;
	if (IS_ERR(cs4244->regmap)) {
		ret = PTR_ERR(cs4244->regmap);
		dev_err(dev, "failed to allocate regmap: %d\n", ret);
		goto err_enable;
	}
	/*
	 * We haven't marked the chip revision as volatile due to
	 * sharing a register with the right input volume; explicitly
	 * bypass the cache to read it.
	 */
	regcache_cache_bypass(cs4244->regmap, true);

	/* Validate the chip ID */
	ret = read_chipid(cs4244);
#if 0
	if (ret < 0) {
		dev_err(dev, "failed to get device ID, ret = %d", ret);
		goto err_enable;
	}
#endif
	/* Get Revision ID */
	ret = regmap_read(cs4244->regmap, CS4244_REVID, &val);
#if 0 	
	if (ret < 0) {
		dev_err(dev, "failed to get Revision ID, ret = %d", ret);
		goto err_enable;
	}
#endif
	dev_info(dev, "found device, revision %X\n", val );


	cs4244_dai.name = cs4244->drvdata->name;

	/* Each adc supports stereo input */
	cs4244_dai.capture.channels_max = cs4244->drvdata->num_adcs * 2;

	cs4244->gpio_nreset = devm_gpiod_get(dev, "reset",
				GPIOD_OUT_LOW);
        if (IS_ERR(cs4244->gpio_nreset))
               return PTR_ERR(cs4244->gpio_nreset);
	if(cs4244->gpio_nreset)
	gpiod_set_value_cansleep(cs4244->gpio_nreset, 1);
	mdelay(250);
	gpiod_set_value_cansleep(cs4244->gpio_nreset, 0);
	cs4244->wq = create_workqueue("regmap worqueue");
	INIT_WORK(&cs4244->work_regmap.work, regmap_handler);
	regcache_cache_bypass(cs4244->regmap, false);
	ret = snd_soc_register_codec(dev, &cs4244_driver, &cs4244_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register codec:%d\n", ret);
		goto err_enable;
	}

	regcache_cache_only(cs4244->regmap, false);
err_enable:
	regulator_bulk_disable(ARRAY_SIZE(cs4244->supplies),
			       cs4244->supplies);

	return ret;
}
EXPORT_SYMBOL_GPL(cs4244_probe);

#ifdef CONFIG_PM
static int cs4244_runtime_resume(struct device *dev)
{
	struct cs4244_priv *cs4244 = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(cs4244->clk);
	if (ret) {
		dev_err(dev, "failed to enable mclk: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(cs4244->supplies),
				    cs4244->supplies);
	if (ret) {
		dev_err(dev, "failed to enable supplies: %d\n", ret);
		goto err_clk;
	}

//	regcache_cache_only(cs4244->regmap, false);

	ret = regcache_sync(cs4244->regmap);
	if (ret) {
		dev_err(dev, "failed to sync regmap: %d\n", ret);
		goto err_bulk;
	}

	
	return 0;

err_bulk:
	regulator_bulk_disable(ARRAY_SIZE(cs4244->supplies),
			       cs4244->supplies);
err_clk:
	clk_disable_unprepare(cs4244->clk);

	return ret;
}

static int cs4244_runtime_suspend(struct device *dev)
{
	struct cs4244_priv *cs4244 = dev_get_drvdata(dev);

//	regcache_cache_only(cs4244->regmap, false);

	regulator_bulk_disable(ARRAY_SIZE(cs4244->supplies),
			       cs4244->supplies);

	clk_disable_unprepare(cs4244->clk);

	return 0;
}
#endif

const struct dev_pm_ops cs4244_pm = {
	SET_RUNTIME_PM_OPS(cs4244_runtime_suspend, cs4244_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(cs4244_pm);

MODULE_DESCRIPTION("Cirrus Logic CS4244 ALSA SoC Codec Driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
