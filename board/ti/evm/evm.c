/*
 * (C) Copyright 2004-2011
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Manikandan Pillai <mani.pillai@ti.com>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include "evm.h"

#define OMAP3EVM_GPIO_ETH_RST_GEN1		64
#define OMAP3EVM_GPIO_ETH_RST_GEN2		7

DECLARE_GLOBAL_DATA_PTR;

static u32 omap3_evm_version;

u32 get_omap3_evm_rev(void)
{
	return omap3_evm_version;
}

static void omap3_evm_get_revision(void)
{
#if defined(CONFIG_CMD_NET)
	/*
	 * Board revision can be ascertained only by identifying
	 * the Ethernet chipset.
	 */
	unsigned int smsc_id;

	/* Ethernet PHY ID is stored at ID_REV register */
	smsc_id = readl(CONFIG_SMC911X_BASE + 0x50) & 0xFFFF0000;
	printf("Read back SMSC id 0x%x\n", smsc_id);

	switch (smsc_id) {
	/* SMSC9115 chipset */
	case 0x01150000:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_1;
		break;
	/* SMSC 9220 chipset */
	case 0x92200000:
	default:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_2;
       }
#else
#if defined(CONFIG_STATIC_BOARD_REV)
	/*
	 * Look for static defintion of the board revision
	 */
	omap3_evm_version = CONFIG_STATIC_BOARD_REV;
#else
	/*
	 * Fallback to the default above.
	 */
	omap3_evm_version = OMAP3EVM_BOARD_GEN_2;
#endif
#endif	/* CONFIG_CMD_NET */
}

#ifdef CONFIG_USB_OMAP3
/*
 * MUSB port on OMAP3EVM Rev >= E requires extvbus programming.
 */
u8 omap3_evm_need_extvbus(void)
{
	u8 retval = 0;

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		retval = 1;

	return retval;
}
#endif

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3EVM;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

/* The only way we can tell what kind of DDR we have is to see what NAND chip
 * we are using.  We look at the vendor of the NAND chip to see if we have a
 * Hynix part or not.  This is how we determine which set of DDR timings to
 * use.
 */
#define HYNIX4GiB_NAND_MFR	0xAD 

#define NAND_CMD_STATUS		0x70
#define NAND_CMD_READID		0x90
#define NAND_CMD_RESET		0xff

#define GPMC_NAND_COMMAND_0      (OMAP34XX_GPMC_BASE+0x7C)
#define GPMC_NAND_ADDRESS_0      (OMAP34XX_GPMC_BASE+0x80)
#define GPMC_NAND_DATA_0	 (OMAP34XX_GPMC_BASE+0x84)

#define WRITE_NAND_COMMAND(d, adr) \
	do {*(volatile u16 *)GPMC_NAND_COMMAND_0 = d;} while(0)
#define WRITE_NAND_ADDRESS(d, adr) \
	do {*(volatile u16 *)GPMC_NAND_ADDRESS_0 = d;} while(0)
#define READ_NAND(adr) \
	(*(volatile u16 *)GPMC_NAND_DATA_0)

/* nand_command: Send a flash command to the flash chip */
static void nand_command(unsigned char command)
{
 	WRITE_NAND_COMMAND(command, NAND_ADDR);

  	if (command == NAND_CMD_RESET) {
		unsigned char ret_val;
		nand_command(NAND_CMD_STATUS);
		do {
			ret_val = READ_NAND(NAND_ADDR);/* wait till ready */
  		} while ((ret_val & 0x40) != 0x40);
 	}
}

static int is_hynix_memory(void)
{
 	nand_command(NAND_CMD_RESET);
 	nand_command(NAND_CMD_READID);

	WRITE_NAND_ADDRESS(0x0, NAND_ADDR);

 	if (READ_NAND(NAND_ADDR) == HYNIX4GiB_NAND_MFR)
		return 1;

	return 0;
}

/* 
 * Routine: board_early_sdrc_init
 * Description: If we use SPL then there is no x-loader nor config header
 * so we have to setup the DDR timings outself on both banks.
 */
void board_early_sdrc_init(struct sdrc *sdrc_base, struct sdrc_actim *sdrc_actim_base0)
{
	/* We pick from hard coded values for the MCFG register as these
	 * come from x-loader which says they come from the vendor and
	 * these disagree with how the TRM says to calculate them.
	 */
	unsigned int val_mcfg, val_actim_ctrla, val_actim_ctrlb;
	struct sdrc_actim *sdrc_actim_base1 = (struct sdrc_actim *)SDRC_ACTIM_CTRL1_BASE;

	/* If we are OMAP36XX and Hynix we have one set of timings,
	 * otherwise it's the Micron timings.  Both share the same
	 * value for RFR_CTRL and MR.
	 */
	if (get_cpu_family() == CPU_OMAP36XX && is_hynix_memory()) {
		val_mcfg = 0x03588099;
		val_actim_ctrla = HYNIX_V_ACTIMA_200;
		val_actim_ctrlb = HYNIX_V_ACTIMB_200;
	} else {
		val_mcfg = 0x02584099;
		val_actim_ctrla = MICRON_V_ACTIMA_165;
		val_actim_ctrlb = MICRON_V_ACTIMB_165;
	}

	/* SDRC_MCFG0 register */
	writel(val_mcfg, &sdrc_base->cs[CS0].mcfg);

	/* SDRC_ACTIM_CTRLA0 register */
	writel(val_actim_ctrla, &sdrc_actim_base0->ctrla);
	/* SDRC_ACTIM_CTRLB0 register */
	writel(val_actim_ctrlb, &sdrc_actim_base0->ctrlb);

	/* Both parts use this value. */
	writel(MICRON_V_RFR_CTRL_165, &sdrc_base->cs[CS0].rfr_ctrl);

	/* SDRC_Manual command register */
	writel(CMD_NOP, &sdrc_base->cs[CS0].manual);
	/* supposed to be 100us per design spec for mddr/msdr */
	sdelay(5000);
	writel(CMD_PRECHARGE, &sdrc_base->cs[CS0].manual);
	writel(CMD_AUTOREFRESH, &sdrc_base->cs[CS0].manual);
	writel(CMD_AUTOREFRESH, &sdrc_base->cs[CS0].manual);

	/* SDRC MR0 register */
	writel(MICRON_V_MR, &sdrc_base->cs[CS0].mr);

	make_cs1_contiguous();

	writel(val_mcfg, &sdrc_base->cs[CS1].mcfg);
	writel(val_actim_ctrla, &sdrc_actim_base1->ctrla);
	writel(val_actim_ctrlb, &sdrc_actim_base1->ctrlb);
	writel(MICRON_V_RFR_CTRL_165, &sdrc_base->cs[CS1].rfr_ctrl);

	/* init sequence for mDDR/mSDR using manual commands */
	writel(CMD_NOP, &sdrc_base->cs[CS1].manual);
	/* supposed to be 100us per design spec for mddr/msdr */
	sdelay(5000);
	writel(CMD_PRECHARGE, &sdrc_base->cs[CS1].manual);
	writel(CMD_AUTOREFRESH, &sdrc_base->cs[CS1].manual);
	writel(CMD_AUTOREFRESH, &sdrc_base->cs[CS1].manual);
	writel(MICRON_V_MR, &sdrc_base->cs[CS1].mr);
}

/*
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 */
int misc_init_r(void)
{

#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

#if defined(CONFIG_CMD_NET)
	setup_net_chip();
#endif
	omap3_evm_get_revision();

#if defined(CONFIG_CMD_NET)
	reset_net_chip();
#endif
	dieid_num_r();

	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
	MUX_EVM();
}

#ifdef CONFIG_CMD_NET
/*
 * Routine: setup_net_chip
 * Description: Setting up the configuration GPMC registers specific to the
 *		Ethernet hardware.
 */
static void setup_net_chip(void)
{
	struct ctrl *ctrl_base = (struct ctrl *)OMAP34XX_CTRL_BASE;

	/* Configure GPMC registers */
	writel(NET_GPMC_CONFIG1, &gpmc_cfg->cs[5].config1);
	writel(NET_GPMC_CONFIG2, &gpmc_cfg->cs[5].config2);
	writel(NET_GPMC_CONFIG3, &gpmc_cfg->cs[5].config3);
	writel(NET_GPMC_CONFIG4, &gpmc_cfg->cs[5].config4);
	writel(NET_GPMC_CONFIG5, &gpmc_cfg->cs[5].config5);
	writel(NET_GPMC_CONFIG6, &gpmc_cfg->cs[5].config6);
	writel(NET_GPMC_CONFIG7, &gpmc_cfg->cs[5].config7);

	/* Enable off mode for NWE in PADCONF_GPMC_NWE register */
	writew(readw(&ctrl_base ->gpmc_nwe) | 0x0E00, &ctrl_base->gpmc_nwe);
	/* Enable off mode for NOE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_noe) | 0x0E00, &ctrl_base->gpmc_noe);
	/* Enable off mode for ALE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_nadv_ale) | 0x0E00,
		&ctrl_base->gpmc_nadv_ale);
}

/**
 * Reset the ethernet chip.
 */
static void reset_net_chip(void)
{
	int ret;
	int rst_gpio;

	if (get_omap3_evm_rev() == OMAP3EVM_BOARD_GEN_1) {
		rst_gpio = OMAP3EVM_GPIO_ETH_RST_GEN1;
	} else {
		rst_gpio = OMAP3EVM_GPIO_ETH_RST_GEN2;
	}

	ret = gpio_request(rst_gpio, "");
	if (ret < 0) {
		printf("Unable to get GPIO %d\n", rst_gpio);
		return ;
	}

	/* Configure as output */
	gpio_direction_output(rst_gpio, 0);

	/* Send a pulse on the GPIO pin */
	gpio_set_value(rst_gpio, 1);
	udelay(1);
	gpio_set_value(rst_gpio, 0);
	udelay(1);
	gpio_set_value(rst_gpio, 1);
}

int board_eth_init(bd_t *bis)
{
	int rc = 0;
#ifdef CONFIG_SMC911X
	rc = smc911x_initialize(0, CONFIG_SMC911X_BASE);
#endif
	return rc;
}
#endif /* CONFIG_CMD_NET */

#if defined(CONFIG_GENERIC_MMC) && !defined(CONFIG_SPL_BUILD)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0);
	return 0;
}
#endif
