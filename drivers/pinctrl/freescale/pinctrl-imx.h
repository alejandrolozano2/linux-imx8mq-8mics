/*
 * IMX pinmux core definitions
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 * Copyright 2017 NXP
 *
 * Author: Dong Aisheng <dong.aisheng@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DRIVERS_PINCTRL_IMX_H
#define __DRIVERS_PINCTRL_IMX_H

struct platform_device;

/**
 * struct imx_pin_group - describes a single i.MX pin
 * @pin: the pin_id of this pin
 * @mux_mode: the mux mode for this pin.
 * @input_reg: the select input register offset for this pin if any
 *	0 if no select input setting needed.
 * @input_val: the select input value for this pin.
 * @configs: the config for this pin.
 */
struct imx_pin_memmap {
	unsigned int mux_mode;
	u16 input_reg;
	unsigned int input_val;
	unsigned long config;
};

struct imx_pin_scu {
	unsigned long mux;
	unsigned long config;
};

struct imx_pin {
	unsigned int pin;
	union {
		struct imx_pin_memmap pin_memmap;
		struct imx_pin_scu pin_scu;
	} pin_conf;
};

/**
 * struct imx_pin_group - describes an IMX pin group
 * @name: the name of this specific pin group
 * @npins: the number of pins in this group array, i.e. the number of
 *	elements in .pins so we can iterate over that array
 * @pin_ids: array of pin_ids. pinctrl forces us to maintain such an array
 * @pins: array of pins
 */
struct imx_pin_group {
	const char *name;
	unsigned npins;
	unsigned int *pin_ids;
	struct imx_pin *pins;
};

/**
 * struct imx_pmx_func - describes IMX pinmux functions
 * @name: the name of this specific function
 * @groups: corresponding pin groups
 * @num_groups: the number of groups
 */
struct imx_pmx_func {
	const char *name;
	const char **groups;
	unsigned num_groups;
};

/**
 * struct imx_pin_reg - describe a pin reg map
 * @mux_reg: mux register offset
 * @conf_reg: config register offset
 */
struct imx_pin_reg {
	s16 mux_reg;
	s16 conf_reg;
};

struct imx_pinctrl_soc_info {
	struct device *dev;
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	struct imx_pin_reg *pin_regs;
	struct imx_pin_group *groups;
	unsigned int ngroups;
	unsigned int group_index;
	struct imx_pmx_func *functions;
	unsigned int nfunctions;
	unsigned int flags;
	const char *gpr_compatible;
	unsigned int mux_mask;
};

/**
 * @dev: a pointer back to containing device
 * @base: the offset to the controller in virtual memory
 */
struct imx_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *base;
	void __iomem *input_sel_base;
	const struct imx_pinctrl_soc_info *info;
};

#define SHARE_MUX_CONF_REG	0x1
#define ZERO_OFFSET_VALID	0x2
#define CONFIG_IBE_OBE		0x4
#define IMX8_ENABLE_MUX_CONFIG	(1 << 29)
#define IMX8_ENABLE_PAD_CONFIG	(1 << 30)
#define IMX8_USE_SCU		(1 << 31)

#define BM_IMX8_GP_ENABLE	(1 << 30)
#define BM_IMX8_IFMUX_ENABLE	(1 << 31)

#define NO_MUX		0x0
#define NO_PAD		0x0

#define IMX_PINCTRL_PIN(pin) PINCTRL_PIN(pin, #pin)

#define PAD_CTL_MASK(len)	((1 << len) - 1)
#define IMX_MUX_MASK	0x7
#define IOMUXC_CONFIG_SION	(0x1 << 4)

int imx_pinctrl_probe(struct platform_device *pdev,
			struct imx_pinctrl_soc_info *info);
int imx_pinctrl_suspend(struct device *dev);
int imx_pinctrl_resume(struct device *dev);

#ifdef CONFIG_PINCTRL_IMX_MEMMAP
int imx_pmx_set_one_pin_mem(struct imx_pinctrl *ipctl, struct imx_pin *pin);
int imx_pmx_backend_gpio_request_enable_mem(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned offset);
void imx_pmx_backend_gpio_disable_free_mem(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned offset);
int imx_pmx_backend_gpio_set_direction_mem(struct pinctrl_dev *pctldev,
	   struct pinctrl_gpio_range *range, unsigned offset, bool input);
int imx_pinconf_backend_get_mem(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *config);
int imx_pinconf_backend_set_mem(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *configs, unsigned num_configs);
int imx_pinctrl_parse_pin_mem(struct imx_pinctrl_soc_info *info,
	unsigned int *pin_id, struct imx_pin *pin, const __be32 **list_p);
#else
static inline int imx_pmx_set_one_pin_mem(struct imx_pinctrl *ipctl, struct imx_pin *pin)
{
	return 0;
}
static inline int imx_pmx_backend_gpio_request_enable_mem(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned offset)
{
	return 0;
}
static inline void imx_pmx_backend_gpio_disable_free_mem(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned offset)
{
	return;
}
static inline int imx_pmx_backend_gpio_set_direction_mem(struct pinctrl_dev *pctldev,
	   struct pinctrl_gpio_range *range, unsigned offset, bool input)
{
	return 0;
}
static inline int imx_pinconf_backend_get_mem(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *config)
{
	return 0;
}
static inline int imx_pinconf_backend_set_mem(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *configs, unsigned num_configs)
{
	return 0;
}
static inline int imx_pinctrl_parse_pin_mem(struct imx_pinctrl_soc_info *info,
	unsigned int *pin_id, struct imx_pin *pin, const __be32 **list_p)
{
	return 0;
}
#endif

#ifdef CONFIG_PINCTRL_IMX_SCU
int imx_pmx_set_one_pin_scu(struct imx_pinctrl *ipctl, struct imx_pin *pin);
int imx_pmx_backend_gpio_request_enable_scu(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned offset);
void imx_pmx_backend_gpio_disable_free_scu(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned offset);
int imx_pmx_backend_gpio_set_direction_scu(struct pinctrl_dev *pctldev,
	   struct pinctrl_gpio_range *range, unsigned offset, bool input);
int imx_pinconf_backend_get_scu(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *config);
int imx_pinconf_backend_set_scu(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *configs, unsigned num_configs);
int imx_pinctrl_parse_pin_scu(struct imx_pinctrl_soc_info *info,
	unsigned int *pin_id, struct imx_pin *pin, const __be32 **list_p);
#else
static inline int imx_pmx_set_one_pin_scu(struct imx_pinctrl *ipctl, struct imx_pin *pin)
{
	return 0;
}
static inline int imx_pmx_backend_gpio_request_enable_scu(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned offset)
{
	return 0;
}
static inline void imx_pmx_backend_gpio_disable_free_scu(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range, unsigned offset)
{
	return;
}
static inline int imx_pmx_backend_gpio_set_direction_scu(struct pinctrl_dev *pctldev,
	   struct pinctrl_gpio_range *range, unsigned offset, bool input)
{
	return 0;
}
static inline int imx_pinconf_backend_get_scu(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *config)
{
	return 0;
}
static inline int imx_pinconf_backend_set_scu(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *configs, unsigned num_configs)
{
	return 0;
}
static inline int imx_pinctrl_parse_pin_scu(struct imx_pinctrl_soc_info *info,
	unsigned int *pin_id, struct imx_pin *pin, const __be32 **list_p)
{
	return 0;
}
#endif
#endif /* __DRIVERS_PINCTRL_IMX_H */
