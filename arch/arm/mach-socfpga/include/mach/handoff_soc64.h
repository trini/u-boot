/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2016-2024 Intel Corporation <www.intel.com>
 * Copyright (C) 2025 Altera Corporation <www.altera.com>
 *
 */

#ifndef _HANDOFF_SOC64_H_
#define _HANDOFF_SOC64_H_

/*
 * Offset for HW handoff from Quartus tools
 */
/* HPS handoff */
#define SOC64_HANDOFF_MAGIC_BOOT	0x424F4F54
#define SOC64_HANDOFF_MAGIC_MUX	0x504D5558
#define SOC64_HANDOFF_MAGIC_IOCTL	0x494F4354
#define SOC64_HANDOFF_MAGIC_FPGA	0x46504741
#define SOC64_HANDOFF_MAGIC_DELAY	0x444C4159
#define SOC64_HANDOFF_MAGIC_CLOCK	0x434C4B53
#define SOC64_HANDOFF_MAGIC_SDRAM	0x5344524d
#if IS_ENABLED(CONFIG_TARGET_SOCFPGA_AGILEX5)
#define SOC64_HANDOFF_MAGIC_PERI	0x50455249
#else
#define SOC64_HANDOFF_MAGIC_MISC	0x4D495343
#endif

#define SOC64_HANDOFF_OFFSET_LENGTH	0x4
#define SOC64_HANDOFF_OFFSET_DATA	0x10
#define SOC64_HANDOFF_SIZE		4096

#if IS_ENABLED(CONFIG_TARGET_SOCFPGA_STRATIX10) || \
	IS_ENABLED(CONFIG_TARGET_SOCFPGA_AGILEX)
#define SOC64_HANDOFF_BASE		0xFFE3F000
#define SOC64_HANDOFF_MISC		(SOC64_HANDOFF_BASE + 0x610)
#elif IS_ENABLED(CONFIG_TARGET_SOCFPGA_AGILEX5)
#define SOC64_HANDOFF_BASE		0x0007F000
#elif IS_ENABLED(CONFIG_TARGET_SOCFPGA_N5X)
#define SOC64_HANDOFF_BASE		0xFFE5F000
#define SOC64_HANDOFF_MISC		(SOC64_HANDOFF_BASE + 0x630)

/* DDR handoff */
#define SOC64_HANDOFF_DDR_BASE			0xFFE5C000
#define SOC64_HANDOFF_DDR_MAGIC			0x48524444
#define SOC64_HANDOFF_DDR_UMCTL2_MAGIC		0x4C54434D
#define SOC64_HANDOFF_DDR_UMCTL2_DDR4_TYPE	0x34524444
#define SOC64_HANDOFF_DDR_UMCTL2_LPDDR4_0_TYPE	0x3044504C
#define SOC64_HANDOFF_DDR_UMCTL2_LPDDR4_1_TYPE	0x3144504C
#define SOC64_HANDOFF_DDR_MEMRESET_BASE		(SOC64_HANDOFF_DDR_BASE + 0xC)
#define SOC64_HANDOFF_DDR_UMCTL2_SECTION	(SOC64_HANDOFF_DDR_BASE + 0x10)
#define SOC64_HANDOFF_DDR_PHY_MAGIC		0x43594850
#define SOC64_HANDOFF_DDR_PHY_INIT_ENGINE_MAGIC	0x45594850
#define SOC64_HANDOFF_DDR_PHY_BASE_OFFSET	0x8
#define SOC64_HANDOFF_DDR_UMCTL2_TYPE_OFFSET	0x8
#define SOC64_HANDOFF_DDR_UMCTL2_BASE_ADDR_OFFSET	0xC
#define SOC64_HANDOFF_DDR_TRAIN_IMEM_1D_SECTION	0xFFE50000
#define SOC64_HANDOFF_DDR_TRAIN_DMEM_1D_SECTION	0xFFE58000
#define SOC64_HANDOFF_DDR_TRAIN_IMEM_2D_SECTION	0xFFE44000
#define SOC64_HANDOFF_DDR_TRAIN_DMEM_2D_SECTION	0xFFE4C000
#define SOC64_HANDOFF_DDR_TRAIN_IMEM_LENGTH	SZ_32K
#define SOC64_HANDOFF_DDR_TRAIN_DMEM_LENGTH	SZ_16K
#endif

#define SOC64_HANDOFF_MUX		(SOC64_HANDOFF_BASE + 0x10)
#define SOC64_HANDOFF_IOCTL		(SOC64_HANDOFF_BASE + 0x1A0)
#define SOC64_HANDOFF_FPGA		(SOC64_HANDOFF_BASE + 0x330)
#define SOC64_HANDOFF_DELAY		(SOC64_HANDOFF_BASE + 0x3F0)
#define SOC64_HANDOFF_CLOCK		(SOC64_HANDOFF_BASE + 0x580)
#if IS_ENABLED(CONFIG_TARGET_SOCFPGA_AGILEX5)
#define SOC64_HANDOFF_PERI		(SOC64_HANDOFF_BASE + 0x620)
#define SOC64_HANDOFF_PERI_LEN		1
#define SOC64_HANDOFF_SDRAM		(SOC64_HANDOFF_BASE + 0x634)
#define SOC64_HANDOFF_SDRAM_LEN		5
#endif

#if IS_ENABLED(CONFIG_TARGET_SOCFPGA_STRATIX10)
#define SOC64_HANDOFF_CLOCK_OSC			(SOC64_HANDOFF_BASE + 0x608)
#define SOC64_HANDOFF_CLOCK_FPGA		(SOC64_HANDOFF_BASE + 0x60C)
#elif IS_ENABLED(CONFIG_TARGET_SOCFPGA_AGILEX5)
#define SOC64_HANDOFF_CLOCK_OSC			(SOC64_HANDOFF_BASE + 0x60c)
#define SOC64_HANDOFF_CLOCK_FPGA		(SOC64_HANDOFF_BASE + 0x610)
#else
#define SOC64_HANDOFF_CLOCK_OSC			(SOC64_HANDOFF_BASE + 0x5fc)
#define SOC64_HANDOFF_CLOCK_FPGA		(SOC64_HANDOFF_BASE + 0x600)
#endif

#define SOC64_HANDOFF_MUX_LEN			96
#define SOC64_HANDOFF_IOCTL_LEN			96
#if IS_ENABLED(CONFIG_TARGET_SOCFPGA_STRATIX10)
#define SOC64_HANDOFF_FPGA_LEN			42
#else
#define SOC64_HANDOFF_FPGA_LEN			40
#endif
#define SOC64_HANDOFF_DELAY_LEN			96

#ifndef __ASSEMBLY__
#include <asm/types.h>
int socfpga_get_handoff_size(void *handoff_address);
int socfpga_handoff_read(void *handoff_address, void *table, u32 table_len);
#endif
#endif /* _HANDOFF_SOC64_H_ */
