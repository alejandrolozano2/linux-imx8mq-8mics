/*
 * cs4244.h - Cirrus Logic CS42448/CS42888 Audio CODEC driver header file
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Author: Nicolin Chen <Guangyu.Chen@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef _CS4244_H
#define _CS4244_H

struct cs4244_driver_data {
	char name[32];
	int num_adcs;
};

extern const struct dev_pm_ops cs4244_pm;
extern const struct cs4244_driver_data cs4244_data;
extern const struct regmap_config cs4244_regmap_config;
extern int cs4244_probe(struct device *dev, struct regmap *regmap);

extern const struct cs4244_driver_data cs4244_data = {
        .name = "cs4244",
        .num_adcs = 2,
};

/* CS42888 register map */

#define CS4244_RSVD0A				0x0A	/* Chip ID dd*/
#define CS4244_RSVD0B				0x0B	/* Chip ID dd*/
#define CS4244_RSVD0C				0x0C	/* Chip ID dd*/
#define CS4244_RSVD0D				0x0D	/* Chip ID dd*/
#define CS4244_RSVD0E				0x0E	/* Chip ID dd*/
#define CS4244_RSVD11				0x11	/* Chip ID dd*/
#define CS4244_RSVD1C				0x1C	/* Chip ID dd*/
#define CS4244_RSVD1D				0x1D	/* Chip ID dd*/
#define CS4244_MCLK_SPD_MCLK_MASK		0xE0    /*MCLK RATE MASK*/
#define CS4244_INT1                             0x21    /* Chip ID dd*/
#define CS4244_INT2	                        0x22    /* Chip ID dd*/

#define CS4244_CHIPID1				0x01	/* Chip ID dd*/
#define CS4244_CHIPID2				0x02	/* Chip ID dd*/
#define CS4244_CHIPID3				0x03	/* Chip ID dd*/
#define CS4244_RSVD				0x04	/*  Reserved Field */
#define CS4244_REVID				0x05	/* Revision ID dd*/
#define CS4244_MCLK_SPD				0x06	/* Funtional Mode Clock & Speed dd*/
#define CS4244_SAMPLE_WIDTH			0x07	/* Functional Mode Sample Width  dd*/
#define CS4244_PORT_CTRL			0x08	/* Functional Mode SP Control dd*/
#define CS4244_DAC_SRC				0x09	/* DAC Data Source Formats dd*/
#define CS4244_ADCCTL1				0x0F	/* ADC VA HiPass Fltr and INV Control dd*/
#define CS4244_ADCCTL2				0x10	/* ADC Mute & PDN Control dd*/
#define CS4244_DACCTL1				0x12	/* DAC Noise Gate and De-empheasis Control dd*/
#define CS4244_DACCTL2				0x13	/* DAC Channel Invert dd*/
#define CS4244_DACCTL3				0x014	/* DAC Attenuation & Mute dd*/
#define CS4244_DACCTL4				0x015	/* DAC Power Down Control dd*/
#define CS4244_VOLMODE				0x016	/* Volume Mode Control - Delay dd*/
#define CS4244_VOLAOUT				0x17	/* Master Volume Control AOUT dd*/
#define CS4244_VOLAOUT1				0x18	/* Volume Control AOUT1 dd*/
#define CS4244_VOLAOUT2				0x19	/* Volume Control AOUT2 dd*/
#define CS4244_VOLAOUT3				0x1A	/* Volume Control AOUT3 dd*/
#define CS4244_VOLAOUT4				0x1B	/* Volume Control AOUT4 dd*/

#define CS4244_FIRSTREG				CS4244_CHIPID1
#define CS4244_LASTREG				CS4244_VOLAOUT4
#define CS4244_NUMREGS				0x22 //(CS4244_LASTREG - CS4244_FIRSTREG + 1)
#define CS4244_I2C_INCR				0x80

/* Chip I.D. (Address 01h - 03h) dd*/
#define CS4244_CHIPID_CHIP_ID_SHIFTA		5
#define CS4244_CHIPID_CHIP_ID_SHIFTB		1
#define CS4244_CHIPID_CHIP_ID_WIDTH		2


/* Chip Rev (Address 5h) dd*/
#define CS4244_CHIPID_REV_ID_MASK		0xFF

/* Functional Mode (Address 06h) dd*/
#define CS4244_FUNCMOD_MCLK_SHIFT		1
#define CS4244_FUNCMOD_MCLK_WIDTH		3
#define CS4244_FUNCMOD_MCLK_MASK		(((1 << CS4244_FUNCMOD_MCLK_WIDTH) - 1) << CS4244_FUNCMOD_MCLK_SHIFT)

#define CS4244_FM_MCLK_RATE_256			0
#define CS4244_FM_MCLK_RATE_384			1
#define CS4244_FM_MCLK_RATE_512			2

#define CS4244_FUNCMOD_SPEED_MODE_SHIFT		4
#define CS4244_FUNCMOD_SPEED_MODE_WIDTH		2
#define CS4244_FUNCMOD_SPEED_MODE_MASK		(((1 << CS4244_FUNCMOD_SPEED_MODE_WIDTH) - 1) << CS4244_FUNCMOD_SPEED_MODE_SHIFT)

#define CS4244_FM_SINGLE			0
#define CS4244_FM_DOUBLE			1
#define CS4244_FM_QUAD				2
#define CS4244_FM_AUTO				3


/* Sample Width Formats (Address 07h) dd */
#define CS4244_INTF_DAC_SW_WIDTH_SHIFT		6
#define CS4244_INTF_DAC_SW_WIDTH		2
#define CS4244_INTF_DAC_SW_WIDTH_MASK		(((1 << CS4244_INTF_DAC_SW_WIDTH) - 1) << CS4244_INTF_DAC_SW_WIDTH_SHIFT)

/* Serial Port Formats (Address 08h) dd */
#define CS4244_INTF_DAC_SPF_SHIFT		2
#define CS4244_INTF_DAC_SPF_WIDTH		2
#define CS4244_INTF_DAC_SPF_MASK		(((1 << CS4244_INTF_DAC_SPF_WIDTH) - 1) << CS4244_INTF_DAC_SPF_SHIFT)
#define CS4244_INTF_DAC_SPF_LEFTJ		(0 << CS4244_INTF_DAC_SPF_SHIFT)
#define CS4244_INTF_DAC_SPF_I2S			(1 << CS4244_INTF_DAC_SPF_SHIFT)
#define CS4244_INTF_DAC_SPF_TDM			(2 << CS4244_INTF_DAC_SPF_SHIFT)

#define CS4244_INTF_DAC_MASTER_SLAVE_MASK	0x01
#define CS4244_INTF_DAC_MASTER			1
#define CS4244_INTF_DAC_SLAVE			0

/* Serial Port Data Select (Address 09h) dd */
#define CS4244_INTF_DAC_SPDS_SHIFT		3
#define CS4244_INTF_DAC_SPDS_WIDTH		3

#define CS4244_SPDS_SLOT0			0 //Slots 1-4 of the TDM stream on SDIN1
#define CS4244_SPDS_SLOT1			1 //Slots 5-8 of the TDM stream on SDIN1
#define CS4244_SPDS_SLOT2			2 //Slots 9-12 of the TDM stream on SDIN1
#define CS4244_SPDS_SLOT3			3 //Slots 13-16 of the TDM stream on SDIN1
#define CS4244_SPDS_SLOT4			4 //Slots 1-4 of the TDM stream on SDIN2
#define CS4244_SPDS_SLOT5			5 //Slots 5-8 of the TDM stream on SDIN2
#define CS4244_SPDS_SLOT6			6 //Slots 9-12 of the TDM stream on SDIN2
#define CS4244_SPDS_SLOT7			7 //Slots 13-16 of the TDM stream on SDIN2


/* DAC Control Noise Gate (Address 12h) dd*/
#define CS4244_NGCTL_DAC_SHIFT			5
#define CS4244_NGCTL_DAC_MASK			(3 << CS4244_INVCTL_DAC4_SHIFT)

#define CS4244_DEMCTL_DAC_SHIFT			4
#define CS4244_DEMCTL_DAC_DEM_MASK		(1 << CS4244_DEMCTL_DAC_SHIFT)
#define CS4244_DEMCTL_DAC_DEM			(1 << CS4244_DEMCTL_DAC_SHIFT)


 
/* DAC INV Control (Address 13h) dd*/
#define CS4244_INVCTL_DAC4_SHIFT		3
#define CS4244_INVCTL_DAC4_MASK			(1 << CS4244_INVCTL_DAC4_SHIFT)
#define CS4244_INVCTL_DAC4			(1 << CS4244_INVCTL_DAC4_SHIFT)
#define CS4244_INVCTL_DAC3_SHIFT		2
#define CS4244_INVCTL_DAC3_MASK			(1 << CS4244_INVCTL_DAC3_SHIFT)
#define CS4244_INVCTL_DAC3			(1 << CS4244_INVCTL_DAC3_SHIFT)
#define CS4244_INVCTL_DAC2_SHIFT		1
#define CS4244_INVCTL_DAC2_MASK			(1 << CS4244_INVCTL_DAC2_SHIFT)
#define CS4244_INVCTL_DAC2			(1 << CS4244_INVCTL_DAC2_SHIFT)
#define CS4244_INVCTL_DAC1_SHIFT		0
#define CS4244_INVCTL_DAC1_MASK			(1 << CS4244_INVCTL_DAC1_SHIFT)
#define CS4244_INVCTL_DAC1			(1 << CS4244_INVCTL_DAC1_SHIFT)
	
	
/* DAC Mute Control (Address 14h) dd*/
#define CS4244_MUTECTL_DAC4_SHIFT		3
#define CS4244_MUTECTL_DAC4_MASK		(1 << CS4244_MUTE_DAC4_SHIFT)
#define CS4244_MUTECTL_DAC4_MUTE		(1 << CS4244_MUTE_DAC4_SHIFT)
#define CS4244_MUTECTL_DAC3_SHIFT		2
#define CS4244_MUTECTL_DAC3_MASK		(1 << CS4244_MUTE_DAC3_SHIFT)
#define CS4244_MUTECTL_DAC3_MUTE		(1 << CS4244_MUTE_DAC3_SHIFT)
#define CS4244_MUTECTL_DAC2_SHIFT		1
#define CS4244_MUTECTL_DAC2_MASK		(1 << CS4244_MUTE_DAC2_SHIFT)
#define CS4244_MUTECTL_DAC2_MUTE		(1 << CS4244_MUTE_DAC2_SHIFT)
#define CS4244_MUTECTL_DAC1_SHIFT		0			
#define CS4244_MUTECTL_DAC1_MASK		(1 << CS4244_MUTE_DAC1_SHIFT)
#define CS4244_MUTECTL_DAC1_MUTE		(1 << CS4244_MUTE_DAC1_SHIFT)
#define CS4244_MUTECTL_DAC_ALL_MUTE_MASK	0x0F
#define CS4244_MUTECTL_DAC_ALL_MUTE		0x0F


/* DAC Power Control (Address 15h) dd*/
#define CS4244_PWRCTL_DAC4_SHIFT		3
#define CS4244_PWRCTL_DAC4_MASK			(1 << CS4244_PWRCTL_DAC4_SHIFT)
#define CS4244_PWRCTL_DAC4_PDN			(1 << CS4244_PWRCTL_DAC4_SHIFT)
#define CS4244_PWRCTL_DAC3_SHIFT		2
#define CS4244_PWRCTL_DAC3_MASK			(1 << CS4244_PWRCTL_DAC3_SHIFT)
#define CS4244_PWRCTL_DAC3_PDN			(1 << CS4244_PWRCTL_DAC3_SHIFT)
#define CS4244_PWRCTL_DAC2_SHIFT		1
#define CS4244_PWRCTL_DAC2_MASK			(1 << CS4244_PWRCTL_DAC2_SHIFT)
#define CS4244_PWRCTL_DAC2_PDN			(1 << CS4244_PWRCTL_DAC2_SHIFT)
#define CS4244_PWRCTL_DAC1_SHIFT		0
#define CS4244_PWRCTL_DAC1_MASK			(1 << CS4244_PWRCTL_DAC1_SHIFT)
#define CS4244_PWRCTL_DAC1_PDN			(1 << CS4244_PWRCTL_DAC1_SHIFT)
#define CS4244_PWRCTL_DAC_PDN_ALL_MASK		0x0F
#define CS4244_PWRCTL_DAC_PDN_ALL		0x0F


#endif /* _CS4244_H */
