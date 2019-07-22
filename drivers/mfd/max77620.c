/*
 * Maxim MAX77620 MFD Driver
 *
 * Copyright (C) 2016 NVIDIA CORPORATION. All rights reserved.
 *
 * Author:
 *	Laxman Dewangan <ldewangan@nvidia.com>
 *	Chaitanya Bandi <bandik@nvidia.com>
 *	Mallikarjun Kasoju <mkasoju@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/****************** Teminology used in driver ********************
 * Here are some terminology used from datasheet for quick reference:
 * Flexible Power Sequence (FPS):
 * The Flexible Power Sequencer (FPS) allows each regulator to power up under
 * hardware or software control. Additionally, each regulator can power on
 * independently or among a group of other regulators with an adjustable
 * power-up and power-down delays (sequencing). GPIO1, GPIO2, and GPIO3 can
 * be programmed to be part of a sequence allowing external regulators to be
 * sequenced along with internal regulators. 32KHz clock can be programmed to
 * be part of a sequence.
 * There is 3 FPS confguration registers and all resources are configured to
 * any of these FPS or no FPS.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77620.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

struct max77620_desc {
	enum max77620_chip_id chip_id;
};

static const struct resource gpio_resources[] = {
	DEFINE_RES_IRQ(MAX77620_IRQ_TOP_GPIO),
};

static const struct resource power_resources[] = {
	DEFINE_RES_IRQ(MAX77620_IRQ_LBT_MBATLOW),
};

static const struct resource rtc_resources[] = {
	DEFINE_RES_IRQ(MAX77620_IRQ_TOP_RTC),
};

static const struct resource thermal_resources[] = {
	DEFINE_RES_IRQ(MAX77620_IRQ_LBT_TJALRM1),
	DEFINE_RES_IRQ(MAX77620_IRQ_LBT_TJALRM2),
};

static const struct regmap_irq max77620_top_irqs[] = {
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_GLBL, 0, MAX77620_IRQ_TOP_GLBL_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_SD, 0, MAX77620_IRQ_TOP_SD_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_LDO, 0, MAX77620_IRQ_TOP_LDO_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_GPIO, 0, MAX77620_IRQ_TOP_GPIO_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_RTC, 0, MAX77620_IRQ_TOP_RTC_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_32K, 0, MAX77620_IRQ_TOP_32K_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_TOP_ONOFF, 0, MAX77620_IRQ_TOP_ONOFF_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_LBT_MBATLOW, 1, MAX77620_IRQ_LBM_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_LBT_TJALRM1, 1, MAX77620_IRQ_TJALRM1_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_LBT_TJALRM2, 1, MAX77620_IRQ_TJALRM2_MASK),
};

static const struct mfd_cell max77620_children[] = {
	{ .name = "max77620-pinctrl", },
	{ .name = "max77620-clock", },
	{ .name = "max77620-pmic", },
	{ .name = "max77620-watchdog", },
	{
		.name = "max77620-gpio",
		.resources = gpio_resources,
		.num_resources = ARRAY_SIZE(gpio_resources),
	}, {
		.name = "max77620-rtc",
		.resources = rtc_resources,
		.num_resources = ARRAY_SIZE(rtc_resources),
	}, {
		.name = "max77620-power",
		.resources = power_resources,
		.num_resources = ARRAY_SIZE(power_resources),
	}, {
		.name = "max77620-thermal",
		.resources = thermal_resources,
		.num_resources = ARRAY_SIZE(thermal_resources),
	},
};

static const struct mfd_cell max20024_children[] = {
	{ .name = "max20024-pinctrl", },
	{ .name = "max77620-clock", },
	{ .name = "max20024-pmic", },
	{ .name = "max77620-watchdog", },
	{
		.name = "max77620-gpio",
		.resources = gpio_resources,
		.num_resources = ARRAY_SIZE(gpio_resources),
	}, {
		.name = "max77620-rtc",
		.resources = rtc_resources,
		.num_resources = ARRAY_SIZE(rtc_resources),
	}, {
		.name = "max20024-power",
		.resources = power_resources,
		.num_resources = ARRAY_SIZE(power_resources),
	},
};

static const struct mfd_cell max77663_children[] = {
	{ .name = "max77620-pinctrl", },
	{ .name = "max77620-clock", },
	{ .name = "max77663-pmic", },
	{ .name = "max77620-watchdog", },
	{
		.name = "max77620-gpio",
		.resources = gpio_resources,
		.num_resources = ARRAY_SIZE(gpio_resources),
	}, {
		.name = "max77620-rtc",
		.resources = rtc_resources,
		.num_resources = ARRAY_SIZE(rtc_resources),
	}, {
		.name = "max77663-power",
		.resources = power_resources,
		.num_resources = ARRAY_SIZE(power_resources),
	},
};

static const struct regmap_range max77620_readable_ranges[] = {
	regmap_reg_range(MAX77620_REG_CNFGGLBL1, MAX77620_REG_DVSSD4),
};

static const struct regmap_access_table max77620_readable_table = {
	.yes_ranges = max77620_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77620_readable_ranges),
};

static const struct regmap_range max20024_readable_ranges[] = {
	regmap_reg_range(MAX77620_REG_CNFGGLBL1, MAX77620_REG_DVSSD4),
	regmap_reg_range(MAX20024_REG_MAX_ADD, MAX20024_REG_MAX_ADD),
};

static const struct regmap_access_table max20024_readable_table = {
	.yes_ranges = max20024_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max20024_readable_ranges),
};

static const struct regmap_range max77620_writable_ranges[] = {
	regmap_reg_range(MAX77620_REG_CNFGGLBL1, MAX77620_REG_DVSSD4),
};

static const struct regmap_access_table max77620_writable_table = {
	.yes_ranges = max77620_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77620_writable_ranges),
};

static const struct regmap_range max77620_cacheable_ranges[] = {
	regmap_reg_range(MAX77620_REG_SD0_CFG, MAX77620_REG_LDO_CFG3),
	regmap_reg_range(MAX77620_REG_FPS_CFG0, MAX77620_REG_FPS_SD3),
};

static const struct regmap_access_table max77620_volatile_table = {
	.no_ranges = max77620_cacheable_ranges,
	.n_no_ranges = ARRAY_SIZE(max77620_cacheable_ranges),
};

static const struct regmap_config max77620_regmap_config = {
	.name = "power-slave",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77620_REG_DVSSD4 + 1,
	.cache_type = REGCACHE_RBTREE,
	.rd_table = &max77620_readable_table,
	.wr_table = &max77620_writable_table,
	.volatile_table = &max77620_volatile_table,
};

static const struct regmap_config max20024_regmap_config = {
	.name = "power-slave",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX20024_REG_MAX_ADD + 1,
	.cache_type = REGCACHE_RBTREE,
	.rd_table = &max20024_readable_table,
	.wr_table = &max77620_writable_table,
	.volatile_table = &max77620_volatile_table,
};

static const struct regmap_range max77663_readable_ranges[] = {
	regmap_reg_range(MAX77620_REG_CNFGGLBL1, MAX77620_REG_CID5),
};

static const struct regmap_access_table max77663_readable_table = {
	.yes_ranges = max77663_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77663_readable_ranges),
};

static const struct regmap_range max77663_writable_ranges[] = {
	regmap_reg_range(MAX77620_REG_CNFGGLBL1, MAX77620_REG_CID5),
};

static const struct regmap_access_table max77663_writable_table = {
	.yes_ranges = max77663_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77663_writable_ranges),
};

static const struct regmap_config max77663_regmap_config = {
	.name = "power-slave",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77620_REG_CID5 + 1,
	.cache_type = REGCACHE_RBTREE,
	.rd_table = &max77663_readable_table,
	.wr_table = &max77663_writable_table,
	.volatile_table = &max77620_volatile_table,
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debug routines (direct read)
// Code pulled from old driver

static inline int max77612_i2c_raw_read(struct i2c_client *client, u8 addr, void *dest, u32 bytes) {
	int ret;

	if (bytes > 1) {
		ret = i2c_smbus_read_i2c_block_data(client, addr, bytes, dest);
		if (ret < 0) return ret;
	} else {
		ret = i2c_smbus_read_byte_data(client, addr);
		if (ret < 0) return ret;

		*((u8 *)dest) = (u8)ret;
	}

	dev_dbg(&client->dev, "i2c_read: addr=0x%02x, dest=0x%02x, bytes=%u\n", addr, *((u8 *)dest), bytes);
	return 0;
}

static void max77612_dump_regs(struct i2c_client *client, u8 *addrs, int num_addrs) {
	int ret = 0;
	u8 val;
	int i;

	for (i = 0; i < num_addrs; i++) {
		printk("   0x%02x: ", addrs[i]);

		ret = max77612_i2c_raw_read(client, addrs[i], &val, 1);
		if (ret == 0) {
			printk("0x%02x\n", val);
		} else {
			printk("<read fail: %d>\n", ret);
		}
	}
}

static void max77612_dbg_dump(struct i2c_client *client) {
	/* Excluded interrupt status register to prevent register clear */
	u8 global_regs[] = { 0x00, 0x01, 0x02, 0x05, 0x0D, 0x0E, 0x13 };
	u8 sd_regs[] = {
		0x07, 0x0F, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
		0x1E, 0x1F, 0x20, 0x21, 0x22
	};
	u8 ldo_regs[] = {
		0x10, 0x11, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
		0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34,
		0x35
	};
	u8 gpio_regs[] = {
		0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
		0x40
	};
	u8 osc_32k_regs[] = { 0x03 };
	u8 bbc_regs[] = { 0x04 };
	u8 onoff_regs[] = { 0x12, 0x15, 0x41, 0x42 };
	u8 fps_regs[] = {
		0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
		0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
		0x57
	};
	u8 cid_regs[] = { 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D };

	printk("MAX77612 Registers\n");
	printk(" [Global]\n");
	max77612_dump_regs(client, global_regs, ARRAY_SIZE(global_regs));
	printk(" [Step-Down]\n");
	max77612_dump_regs(client, sd_regs, ARRAY_SIZE(sd_regs));
	printk(" [LDO]\n");
	max77612_dump_regs(client, ldo_regs, ARRAY_SIZE(ldo_regs));
	printk(" [GPIO]\n");
	max77612_dump_regs(client, gpio_regs, ARRAY_SIZE(gpio_regs));
	printk(" [32kHz Oscillator]\n");
	max77612_dump_regs(client, osc_32k_regs, ARRAY_SIZE(osc_32k_regs));
	printk(" [Backup Battery Charger]\n");
	max77612_dump_regs(client, bbc_regs, ARRAY_SIZE(bbc_regs));
	printk(" [On/OFF Controller]\n");
	max77612_dump_regs(client, onoff_regs, ARRAY_SIZE(onoff_regs));
	printk(" [Flexible Power Sequencer]\n");
	max77612_dump_regs(client, fps_regs, ARRAY_SIZE(fps_regs));
	printk(" [Chip Identification]\n");
	max77612_dump_regs(client, cid_regs, ARRAY_SIZE(cid_regs));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * MAX77620 and MAX20024 has the following steps of the interrupt handling
 * for TOP interrupts:
 * 1. When interrupt occurs from PMIC, mask the PMIC interrupt by setting GLBLM.
 * 2. Read IRQTOP and service the interrupt.
 * 3. Once all interrupts has been checked and serviced, the interrupt service
 *    routine un-masks the hardware interrupt line by clearing GLBLM.
 */
static int max77620_irq_global_mask(void *irq_drv_data)
{
	struct max77620_chip *chip = irq_drv_data;
	int ret;

	ret = regmap_update_bits(chip->rmap, MAX77620_REG_INTENLBT,
				 MAX77620_GLBLM_MASK, MAX77620_GLBLM_MASK);
	if (ret < 0)
		dev_err(chip->dev, "Failed to set GLBLM: %d\n", ret);

	return ret;
}

static int max77620_irq_global_unmask(void *irq_drv_data)
{
	struct max77620_chip *chip = irq_drv_data;
	int ret;

	ret = regmap_update_bits(chip->rmap, MAX77620_REG_INTENLBT,
				 MAX77620_GLBLM_MASK, 0);
	if (ret < 0)
		dev_err(chip->dev, "Failed to reset GLBLM: %d\n", ret);

	return ret;
}

static struct regmap_irq_chip max77620_top_irq_chip = {
	.name = "max77620-top",
	.irqs = max77620_top_irqs,
	.num_irqs = ARRAY_SIZE(max77620_top_irqs),
	.num_regs = 2,
	.status_base = MAX77620_REG_IRQTOP,
	.mask_base = MAX77620_REG_IRQTOPM,
	.handle_pre_irq = max77620_irq_global_mask,
	.handle_post_irq = max77620_irq_global_unmask,
};

/* max77620_get_fps_period_reg_value:  Get FPS bit field value from
 *				       requested periods.
 * MAX77620 supports the FPS period of 40, 80, 160, 320, 540, 1280, 2560
 * and 5120 microseconds. MAX20024 supports the FPS period of 20, 40, 80,
 * 160, 320, 540, 1280 and 2560 microseconds.
 * The FPS register has 3 bits field to set the FPS period as
 * bits		max77620		max20024
 * 000		40			20
 * 001		80			40
 * :::
*/
static int max77620_get_fps_period_reg_value(struct max77620_chip *chip,
					     int tperiod)
{
	int fps_min_period;
	int i;

	switch (chip->chip_id) {
	case MAX20024:
		fps_min_period = MAX20024_FPS_PERIOD_MIN_US;
		break;
	case MAX77663:
		fps_min_period = MAX20024_FPS_PERIOD_MIN_US;
		break;
	case MAX77620:
		fps_min_period = MAX77620_FPS_PERIOD_MIN_US;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < 7; i++) {
		if (fps_min_period >= tperiod)
			return i;
		fps_min_period *= 2;
	}

	return i;
}

/* max77620_config_fps: Configure FPS configuration registers
 *			based on platform specific information.
 */
static int max77620_config_fps(struct max77620_chip *chip,
			       struct device_node *fps_np)
{
	struct device *dev = chip->dev;
	unsigned int mask = 0, config = 0;
	u32 fps_max_period;
	u32 param_val;
	int tperiod, fps_id;
	int ret;
	char fps_name[10];

	switch (chip->chip_id) {
	case MAX20024:
		fps_max_period = MAX20024_FPS_PERIOD_MAX_US;
		break;
	case MAX77620:
		fps_max_period = MAX77620_FPS_PERIOD_MAX_US;
		break;
	case MAX77663:
		fps_max_period = MAX20024_FPS_PERIOD_MAX_US;
		break;
	default:
		return -EINVAL;
	}

	for (fps_id = 0; fps_id < MAX77620_FPS_COUNT; fps_id++) {
		sprintf(fps_name, "fps%d", fps_id);
		if (of_node_name_eq(fps_np, fps_name))
			break;
	}

	if (fps_id == MAX77620_FPS_COUNT) {
		dev_err(dev, "FPS node name %s is not valid\n", fps_np->name);
		return -EINVAL;
	}

	ret = of_property_read_u32(fps_np, "maxim,shutdown-fps-time-period-us",
				   &param_val);
	if (!ret) {
		mask |= MAX77620_FPS_TIME_PERIOD_MASK;
		chip->shutdown_fps_period[fps_id] = min(param_val,
							fps_max_period);
		tperiod = max77620_get_fps_period_reg_value(chip,
				chip->shutdown_fps_period[fps_id]);
		config |= tperiod << MAX77620_FPS_TIME_PERIOD_SHIFT;
	}

	ret = of_property_read_u32(fps_np, "maxim,suspend-fps-time-period-us",
				   &param_val);
	if (!ret)
		chip->suspend_fps_period[fps_id] = min(param_val,
						       fps_max_period);

	ret = of_property_read_u32(fps_np, "maxim,fps-event-source",
				   &param_val);
	if (!ret) {
		if (param_val > 2) {
			dev_err(dev, "FPS%d event-source invalid\n", fps_id);
			return -EINVAL;
		}
		mask |= MAX77620_FPS_EN_SRC_MASK;
		config |= param_val << MAX77620_FPS_EN_SRC_SHIFT;
		if (param_val == 2) {
			mask |= MAX77620_FPS_ENFPS_SW_MASK;
			config |= MAX77620_FPS_ENFPS_SW;
		}
	}

	if (!chip->sleep_enable && !chip->enable_global_lpm) {
		ret = of_property_read_u32(fps_np,
				"maxim,device-state-on-disabled-event",
				&param_val);
		if (!ret) {
			if (param_val == 0)
				chip->sleep_enable = true;
			else if (param_val == 1)
				chip->enable_global_lpm = true;
		}
	}

	ret = regmap_update_bits(chip->rmap, MAX77620_REG_FPS_CFG0 + fps_id,
				 mask, config);
	if (ret < 0) {
		dev_err(dev, "Failed to update FPS CFG: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77620_initialise_fps(struct max77620_chip *chip)
{
	struct device *dev = chip->dev;
	struct device_node *fps_np, *fps_child;
	u8 config;
	int fps_id;
	int ret;

	for (fps_id = 0; fps_id < MAX77620_FPS_COUNT; fps_id++) {
		chip->shutdown_fps_period[fps_id] = -1;
		chip->suspend_fps_period[fps_id] = -1;
	}

	fps_np = of_get_child_by_name(dev->of_node, "fps");
	if (!fps_np)
		goto skip_fps;

	for_each_child_of_node(fps_np, fps_child) {
		ret = max77620_config_fps(chip, fps_child);
		if (ret < 0)
			return ret;
	}

	config = chip->enable_global_lpm ? MAX77620_ONOFFCNFG2_SLP_LPM_MSK : 0;
	ret = regmap_update_bits(chip->rmap, MAX77620_REG_ONOFFCNFG2,
				 MAX77620_ONOFFCNFG2_SLP_LPM_MSK, config);
	if (ret < 0) {
		dev_err(dev, "Failed to update SLP_LPM: %d\n", ret);
		return ret;
	}

skip_fps:
	if (chip->chip_id == MAX77663)
		return 0;

	/* Enable wake on EN0 pin */
	ret = regmap_update_bits(chip->rmap, MAX77620_REG_ONOFFCNFG2,
				 MAX77620_ONOFFCNFG2_WK_EN0,
				 MAX77620_ONOFFCNFG2_WK_EN0);
	if (ret < 0) {
		dev_err(dev, "Failed to update WK_EN0: %d\n", ret);
		return ret;
	}

	/* For MAX20024, SLPEN will be POR reset if CLRSE is b11 */
	if ((chip->chip_id == MAX20024) && chip->sleep_enable) {
		config = MAX77620_ONOFFCNFG1_SLPEN | MAX20024_ONOFFCNFG1_CLRSE;
		ret = regmap_update_bits(chip->rmap, MAX77620_REG_ONOFFCNFG1,
					 config, config);
		if (ret < 0) {
			dev_err(dev, "Failed to update SLPEN: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int max77620_read_es_version(struct max77620_chip *chip)
{
	unsigned int val;
	u8 cid_val[6];
	int i;
	int ret;

	for (i = MAX77620_REG_CID0; i <= MAX77620_REG_CID5; i++) {
		ret = regmap_read(chip->rmap, i, &val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to read CID: %d\n", ret);
			return ret;
		}
		dev_dbg(chip->dev, "CID%d: 0x%02x\n",
			i - MAX77620_REG_CID0, val);
		cid_val[i - MAX77620_REG_CID0] = val;
	}

	/* CID4 is OTP Version  and CID5 is ES version */
	dev_info(chip->dev, "PMIC Version OTP:0x%02X and ES:0x%X\n",
		 cid_val[4], MAX77620_CID5_DIDM(cid_val[5]));

	return ret;
}

static int max77620_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	const struct regmap_config *rmap_config;
	struct max77620_chip *chip;
	const struct mfd_cell *mfd_cells;
	const struct max77620_desc *desc;
	int n_mfd_cells;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	desc = of_device_get_match_data(&client->dev);

	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	chip->irq_base = -1;
	chip->chip_irq = client->irq;
	chip->chip_id = desc->chip_id;

	switch (chip->chip_id) {
	case MAX77620:
		mfd_cells = max77620_children;
		n_mfd_cells = ARRAY_SIZE(max77620_children);
		rmap_config = &max77620_regmap_config;
		break;
	case MAX20024:
		mfd_cells = max20024_children;
		n_mfd_cells = ARRAY_SIZE(max20024_children);
		rmap_config = &max20024_regmap_config;
		break;
	case MAX77663:
		mfd_cells = max77663_children;
		n_mfd_cells = ARRAY_SIZE(max77663_children);
		rmap_config = &max77663_regmap_config;
		break;
	default:
		dev_err(chip->dev, "ChipID is invalid %d\n", chip->chip_id);
		return -EINVAL;
	}

	chip->rmap = devm_regmap_init_i2c(client, rmap_config);
	if (IS_ERR(chip->rmap)) {
		ret = PTR_ERR(chip->rmap);
		dev_err(chip->dev, "Failed to initialise regmap: %d\n", ret);
		return ret;
	}

	ret = max77620_read_es_version(chip);
	if (ret < 0)
		return ret;

	max77620_top_irq_chip.irq_drv_data = chip;
	ret = devm_regmap_add_irq_chip(chip->dev, chip->rmap, client->irq,
				       IRQF_ONESHOT | IRQF_SHARED,
				       chip->irq_base, &max77620_top_irq_chip,
				       &chip->top_irq_data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add regmap irq: %d\n", ret);
		return ret;
	}

	ret = max77620_initialise_fps(chip);
	if (ret < 0)
		return ret;

	ret =  devm_mfd_add_devices(chip->dev, PLATFORM_DEVID_NONE,
				    mfd_cells, n_mfd_cells, NULL, 0,
				    regmap_irq_get_domain(chip->top_irq_data));
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add MFD children: %d\n", ret);
		return ret;
	}

	//max77612_dbg_dump(client);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77620_set_fps_period(struct max77620_chip *chip,
				   int fps_id, int time_period)
{
	int period = max77620_get_fps_period_reg_value(chip, time_period);
	int ret;

	ret = regmap_update_bits(chip->rmap, MAX77620_REG_FPS_CFG0 + fps_id,
				 MAX77620_FPS_TIME_PERIOD_MASK,
				 period << MAX77620_FPS_TIME_PERIOD_SHIFT);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to update FPS period: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77620_i2c_suspend(struct device *dev)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int config;
	int fps;
	int ret;

	for (fps = 0; fps < MAX77620_FPS_COUNT; fps++) {
		if (chip->suspend_fps_period[fps] < 0)
			continue;

		ret = max77620_set_fps_period(chip, fps,
					      chip->suspend_fps_period[fps]);
		if (ret < 0)
			return ret;
	}

	/*
	 * For MAX20024: No need to configure SLPEN on suspend as
	 * it will be configured on Init.
	 */
	if (chip->chip_id == MAX20024)
		goto out;

	config = (chip->sleep_enable) ? MAX77620_ONOFFCNFG1_SLPEN : 0;
	ret = regmap_update_bits(chip->rmap, MAX77620_REG_ONOFFCNFG1,
				 MAX77620_ONOFFCNFG1_SLPEN,
				 config);
	if (ret < 0) {
		dev_err(dev, "Failed to configure sleep in suspend: %d\n", ret);
		return ret;
	}

	if (chip->chip_id == MAX77663)
		goto out;

	/* Disable WK_EN0 */
	ret = regmap_update_bits(chip->rmap, MAX77620_REG_ONOFFCNFG2,
				 MAX77620_ONOFFCNFG2_WK_EN0, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to configure WK_EN in suspend: %d\n", ret);
		return ret;
	}

out:
	disable_irq(client->irq);

	return 0;
}

static int max77620_i2c_resume(struct device *dev)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	int fps;

	for (fps = 0; fps < MAX77620_FPS_COUNT; fps++) {
		if (chip->shutdown_fps_period[fps] < 0)
			continue;

		ret = max77620_set_fps_period(chip, fps,
					      chip->shutdown_fps_period[fps]);
		if (ret < 0)
			return ret;
	}

	/*
	 * For MAX20024: No need to configure WKEN0 on resume as
	 * it is configured on Init.
	 */
	if (chip->chip_id == MAX20024 || chip->chip_id == MAX77663)
		goto out;

	/* Enable WK_EN0 */
	ret = regmap_update_bits(chip->rmap, MAX77620_REG_ONOFFCNFG2,
				 MAX77620_ONOFFCNFG2_WK_EN0,
				 MAX77620_ONOFFCNFG2_WK_EN0);
	if (ret < 0) {
		dev_err(dev, "Failed to configure WK_EN0 n resume: %d\n", ret);
		return ret;
	}

out:
	enable_irq(client->irq);

	return 0;
}
#endif

static const struct max77620_desc max77620_desc = {
	.chip_id = MAX77620,
};

static const struct max77620_desc max20024_desc = {
	.chip_id = MAX20024,
};

static const struct max77620_desc max77663_desc = {
	.chip_id = MAX77663,
};

static const struct of_device_id max77620_of_match[] = {
	{ .compatible = "maxim,max77620", .data = &max77620_desc },
	{ .compatible = "maxim,max20024", .data = &max20024_desc },
	{ .compatible = "maxim,max77663", .data = &max77663_desc },
	{ },
};

static const struct dev_pm_ops max77620_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77620_i2c_suspend, max77620_i2c_resume)
};

static struct i2c_driver max77620_driver = {
	.driver = {
		.name = "max77620",
		.pm = &max77620_pm_ops,
		.of_match_table = of_match_ptr(max77620_of_match),
	},
	.probe = max77620_probe,
};
builtin_i2c_driver(max77620_driver);
