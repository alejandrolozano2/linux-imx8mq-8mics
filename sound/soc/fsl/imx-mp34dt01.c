/*
 * ASoC driver for BBB + NXP TFA98xx family of devices
 *
 * Author:      Sebastien Jan <sjan@baylibre.com>
 * Copyright (C) 2015 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG
#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <asm/dma.h>
#include <asm-generic/types.h>
#include "fsl_sai.h"

#define DAI_NAME_SIZE	32

static u32 imx_mp34dt01_rates[] = { 32000, 48000 };
static struct snd_pcm_hw_constraint_list imx_mp34dt01_rate_constraints = {
	.count = ARRAY_SIZE(imx_mp34dt01_rates),
	.list = imx_mp34dt01_rates,
};

struct snd_soc_card_drvdata_imx_tfa {
	struct clk *mclk;
	struct snd_soc_dai_link *dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
	int pstreams;
	int cstreams;
};

static int imx_mp34dt01_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(soc_card);
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &imx_mp34dt01_rate_constraints);
	if (ret)
		return ret;

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

static void imx_mp34dt01_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

static int imx_mp34dt01_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card_drvdata_imx_tfa *data = snd_soc_card_get_drvdata(rtd->card);
	u32 channels = params_channels(params);
	u32 rate = params_rate(params);
	u32 bclk = rate * channels * 32;
	int ret = 0;

	/* set cpu DAI configuration SND_SOC_DAIFMT_I2S*/
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A//SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set cpu dai fmt\n");
		return ret;
	}

	pr_info("ALL mp34dt01 channels %d", channels);
	ret =  snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 8, params_width(params));
	
	ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_BIT, bclk, SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}
	return ret;
}

static int imx_mp34dt01_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->pstreams != 0 || drvdata->cstreams != 0)
		return 0;

	return 0;
}

static int imx_mp34dt01_trigger(struct snd_pcm_substream *stream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (stream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			drvdata->pstreams++;
		else
			drvdata->cstreams++;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (stream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (drvdata->pstreams > 0)
				drvdata->pstreams--;
			else
				pr_err("Error in playback streams count\n");
		} else {
			if (drvdata->cstreams > 0)
				drvdata->cstreams--;
			else
				pr_err("Error in capture streams count\n");
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return 0;
}

static struct snd_soc_ops imx_mp34dt01_ops = {
	.startup = imx_mp34dt01_startup,
	.shutdown = imx_mp34dt01_shutdown,
	.hw_params = imx_mp34dt01_hw_params,
	.hw_free = imx_mp34dt01_hw_free,
	.trigger = imx_mp34dt01_trigger,
};

static int imx_mp34dt01_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card_drvdata_imx_tfa *data = snd_soc_card_get_drvdata(rtd->card);;
	struct device *dev = rtd->card->dev;

	dev_dbg(rtd->card->dev, "%s,%d: dai_init\n", __FUNCTION__, __LINE__);

	return 0;
}

static void *tfa_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);

	if (!str)
		return str;
	memcpy(str, buf, strlen(buf));
	return str;
}

#if defined(CONFIG_OF)
/*
 * The structs are used as place holders. They will be completely
 * filled with data from dt node.
 */
	
static struct snd_soc_dai_link imx_dai_mp34dt01[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.ops            = &imx_mp34dt01_ops,
		.init           = imx_mp34dt01_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS |
			SND_SOC_DAIFMT_NB_NF,
	},
};

static const struct of_device_id imx_tfa98_dt_ids[] = {
	{
		.compatible = "nxp,imx-audio-mp34dt01",
		.data = &imx_dai_mp34dt01,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tfa98_dt_ids);

static const struct snd_soc_dapm_widget imx_mp34dt01_widgets[] = {
//	SND_SOC_DAPM_SPK("Speaker", NULL),
//	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_kcontrol_new imx_mp34dt01_controls[] = {
};

static struct snd_soc_card imx_mp34dt01_soc_card = {
	.owner = THIS_MODULE,
	.name = "MP34DT01",	/* default name if none defined in DT */
	.dai_link = imx_dai_mp34dt01,
	.num_links = ARRAY_SIZE(imx_dai_mp34dt01),
	.controls = imx_mp34dt01_controls,
	.num_controls = ARRAY_SIZE(imx_mp34dt01_controls),
	.dapm_widgets = imx_mp34dt01_widgets,
	.num_dapm_widgets = ARRAY_SIZE(imx_mp34dt01_widgets),
};

static int imx_mp34dt01_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev;
	struct snd_soc_dai_link_component *codecs;
	struct i2c_client *codec_dev;
	struct snd_soc_dai_link *dai;
	struct snd_soc_card_drvdata_imx_tfa *drvdata = NULL;
	struct clk *mclk;
	int ret = 0;
	int i, num_codecs;


	imx_mp34dt01_soc_card.dev = &pdev->dev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	ret = snd_soc_of_parse_card_name(&imx_mp34dt01_soc_card, "nxp,model");
	if (ret)
		goto fail;;
	num_codecs = of_count_phandle_with_args(np, "nxp,audio-codec", NULL);
	if (num_codecs < 1) {
		ret = -EINVAL;
		goto fail;
	}
	pr_info("Found %d codec(s)\n", num_codecs);

	codecs = devm_kzalloc(&pdev->dev,
			sizeof(struct snd_soc_dai_link_component) * num_codecs,
			GFP_KERNEL);

	if (!codecs) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < num_codecs; i++) {
		struct device_node *parent;
		int id;
		const u32 *property;
		int len;
		char name[18];

		codecs[i].of_node = of_parse_phandle(np, "nxp,audio-codec", i);
		snprintf(name, sizeof(name), "mp34dt01");
		codecs[i].dai_name = tfa_devm_kstrdup(&pdev->dev, name);
	}
	dai = &imx_dai_mp34dt01[0];
	dai->platform_of_node = cpu_np;
	dai->codecs = codecs;
	dai->cpu_dai_name = dev_name(&cpu_pdev->dev);
	dai->num_codecs = num_codecs;
	dai->cpu_of_node = of_parse_phandle(np, "ssi-controller", 0);
	if (!dai->cpu_of_node) {
		ret = -EINVAL;
		goto fail;
	}

	/* Only set the platform_of_node if the platform_name is not set */
	if (!dai->platform_name)
		dai->platform_of_node = dai->cpu_of_node;

	mclk = devm_clk_get(&pdev->dev, NULL);
	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		pr_info("getting clk defered\n");
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "clock not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		ret = -ENOMEM;
		goto fail;
	}

	if (mclk) {
		pr_info("Reference clock found: %s @ %ld\n",
			__clk_get_name(mclk),clk_get_rate(mclk));
		clk_prepare(mclk);
		clk_enable(mclk);
		drvdata->mclk = mclk;
	}
	drvdata->clk_frequency = clk_get_rate(drvdata->mclk);
	drvdata->dai = dai;

	platform_set_drvdata(pdev, &drvdata->card);
	snd_soc_card_set_drvdata(&imx_mp34dt01_soc_card, drvdata);

	ret = devm_snd_soc_register_card(&pdev->dev, &imx_mp34dt01_soc_card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codecs)
		of_node_put(codecs[0].of_node);

	return ret;
}

static int imx_mp34dt01_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
				snd_soc_card_get_drvdata(card);


	if (drvdata->mclk)
		clk_disable(drvdata->mclk);

	return 0;
}

static struct platform_driver imx_mp34dt01_driver = {
	.probe		= imx_mp34dt01_probe,
	.remove		= imx_mp34dt01_remove,
	.driver		= {
		.name	= "imx-mp34dt01",
		.owner	= THIS_MODULE,
		.pm	= &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(imx_tfa98_dt_ids),
	},
};
#endif

static int __init _mp34dt01_init(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt())
		return platform_driver_register(&imx_mp34dt01_driver);
#endif
	return 0;
}

static void __exit _mp34dt01_exit(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&imx_mp34dt01_driver);
		return;
	}
#endif
}

module_init(_mp34dt01_init);
module_exit(_mp34dt01_exit);

MODULE_AUTHOR("Alejandro Lozano");
MODULE_DESCRIPTION("i.MX7d MP34DT01 I2S MEMS driver");
MODULE_LICENSE("GPL");
