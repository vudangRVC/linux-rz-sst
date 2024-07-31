// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/G3E System controller (SYS) driver
 *
 * Copyright (C) 2025 Renesas Electronics Corp.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>

#include "rz-sysc.h"

/* Register Offsets */
#define SYS_LSI_MODE		0x300
/*
 * BOOTPLLCA[1:0]
 *	    [0,0] => 1.1GHZ
 *	    [0,1] => 1.5GHZ
 *	    [1,0] => 1.6GHZ
 *	    [1,1] => 1.7GHZ
 */
#define SYS_LSI_MODE_STAT_BOOTPLLCA55	GENMASK(12, 11)
#define SYS_LSI_MODE_CA55_1_7GHZ	0x3

#define SYS_LSI_PRR			0x308
#define SYS_LSI_PRR_CA55_DIS		BIT(8)
#define SYS_LSI_PRR_NPU_DIS		BIT(1)
#define SYS_ADC_CFG_PWE_B		0x1600
#define SYS_ADC_MSTP_ADA_B		BIT(0)
#define SYS_MAX_REG			0x170C
#define SYS_START_REG			0x300

static void rzg3e_sys_print_id(struct device *dev,
				void __iomem *sysc_base,
				struct soc_device_attribute *soc_dev_attr)
{
	bool is_quad_core, npu_enabled;
	u32 prr_val, mode_val;

	prr_val = readl(sysc_base + SYS_LSI_PRR);
	mode_val = readl(sysc_base + SYS_LSI_MODE);

	/* Check CPU and NPU configuration */
	is_quad_core = !(prr_val & SYS_LSI_PRR_CA55_DIS);
	npu_enabled = !(prr_val & SYS_LSI_PRR_NPU_DIS);

	dev_info(dev, "Detected Renesas %s Core %s %s Rev %s%s\n",
		 is_quad_core ? "Quad" : "Dual", soc_dev_attr->family,
		 soc_dev_attr->soc_id, soc_dev_attr->revision,
		 npu_enabled ? " with Ethos-U55" : "");

	/* Check CA55 PLL configuration */
	if (FIELD_GET(SYS_LSI_MODE_STAT_BOOTPLLCA55, mode_val) != SYS_LSI_MODE_CA55_1_7GHZ)
		dev_warn(dev, "CA55 PLL is not set to 1.7GHz\n");
}

static const struct rz_sysc_signal_init_data rzg3e_sysc_signals_init_data[] __initconst = {
	{
		.name = "ADC_MSTP_ADA_B",
		.offset = SYS_ADC_CFG_PWE_B,
		.mask = SYS_ADC_MSTP_ADA_B,
		.refcnt_incr_val = 0
	}
};

static const struct rz_sysc_soc_id_init_data rzg3e_sys_soc_id_init_data __initconst = {
	.family = "RZ/G3E",
	.id = 0x8679447,
	.devid_offset = 0x304,
	.revision_mask = GENMASK(31, 28),
	.specific_id_mask = GENMASK(27, 0),
	.print_id = rzg3e_sys_print_id,
};

const struct rz_sysc_init_data rzg3e_sys_init_data = {
	.soc_id_init_data = &rzg3e_sys_soc_id_init_data,
	.signals_init_data = rzg3e_sysc_signals_init_data,
	.num_signals = ARRAY_SIZE(rzg3e_sysc_signals_init_data),
	.max_register = SYS_MAX_REG,
	.start_register = SYS_START_REG,
};
