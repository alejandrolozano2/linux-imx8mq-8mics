/*
 * I2S MEMS microphone driver for MP34DT01
 *
 * - Non configurable.
 * - I2S interface 24 bit data
 *
 * Copyright (c) 2015 Axis Communications AB
 *
 * Licensed under GPL v2.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define MP34DT01_RATE_MIN 32000 /* 7190  Hz, from data sheet */
#define MP34DT01_RATE_MAX 64000 /* 52800 Hz, from data sheet */

#define MP34DT01_FORMATS ( SNDRV_PCM_FMTBIT_S32_BE | SNDRV_PCM_FMTBIT_S32  ) //SNDRV_PCM_FMTBIT_S32 SNDRV_PCM_FMTBIT_S32_LE

static struct snd_soc_dai_driver mp34dt01_dai = {
	.name = "mp34dt01",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rate_min = MP34DT01_RATE_MIN,
		.rate_max = MP34DT01_RATE_MAX,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = MP34DT01_FORMATS,
	},
};

static struct snd_soc_codec_driver mp34dt01_codec_driver = {
};

static int mp34dt01_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &mp34dt01_codec_driver,
			&mp34dt01_dai, 1);
}

static int mp34dt01_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mp34dt01_ids[] = {
	{ .compatible = "knowles,mp34dt01", },
	{ }
};
MODULE_DEVICE_TABLE(of, mp34dt01_ids);
#endif

static struct platform_driver mp34dt01_driver = {
	.driver = {
		.name = "mp34dt01",
		.of_match_table = of_match_ptr(mp34dt01_ids),
	},
	.probe = mp34dt01_probe,
	.remove = mp34dt01_remove,
};

module_platform_driver(mp34dt01_driver);

MODULE_DESCRIPTION("ASoC MP34DT01 driver");
MODULE_AUTHOR("Alejandro Lozano <alejandro.lozano@nxp.com>");
MODULE_LICENSE("GPL v2");
