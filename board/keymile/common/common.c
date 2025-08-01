// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2008
 * Heiko Schocher, DENX Software Engineering, hs@denx.de.
 *
 * (C) Copyright 2011
 * Holger Brunck, Keymile GmbH Hannover, holger.brunck@keymile.com
 */

#include <config.h>
#include <env.h>
#include <ioports.h>
#include <command.h>
#include <malloc.h>
#include <cli_hush.h>
#include <net.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/bug.h>
#include <bootcount.h>

#if defined(CONFIG_POST)
#include "post.h"
#endif
#include "common.h"
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Set Keymile specific environment variables
 * Currently only some memory layout variables are calculated here
 * ... ------------------------------------------------
 * ... |@rootfsaddr |@pnvramaddr |@varaddr |@reserved |@END_OF_RAM
 * ... |<------------------- pram ------------------->|
 * ... ------------------------------------------------
 * @END_OF_RAM: denotes the RAM size
 * @pnvramaddr: Startadress of pseudo non volatile RAM in hex
 * @pram      : preserved ram size in k
 * @varaddr   : startadress for /var mounted into RAM
 */
int set_km_env(void)
{
	unsigned int pnvramaddr;
	unsigned int pram;
	unsigned int varaddr;
	unsigned int kernelmem;
	unsigned long rootfssize = 0;
	char envval[16];
	char *p;

	pnvramaddr = CFG_SYS_SDRAM_BASE + gd->ram_size -
		CONFIG_KM_PHRAM - CONFIG_KM_PNVRAM;
	sprintf(envval, "0x%x", pnvramaddr);
	env_set("pnvramaddr", envval);

	/* try to read rootfssize (ram image) from environment */
	p = env_get("rootfssize");
	if (p)
		strict_strtoul(p, 16, &rootfssize);
	pram = (rootfssize + CONFIG_KM_PHRAM + CONFIG_KM_PNVRAM) / 0x400;
	env_set_ulong("pram", pram);

	varaddr = CFG_SYS_SDRAM_BASE + gd->ram_size - CONFIG_KM_PHRAM;
	env_set_hex("varaddr", varaddr);
	sprintf(envval, "0x%x", varaddr);
	env_set("varaddr", envval);

	kernelmem = gd->ram_size - 0x400 * pram;
	sprintf(envval, "0x%x", kernelmem);
	env_set("kernelmem", envval);

	return 0;
}

#if IS_ENABLED(CONFIG_PG_WCOM_UBOOT_UPDATE_SUPPORTED)
#if   ((!IS_ENABLED(CONFIG_PG_WCOM_UBOOT_BOOTPACKAGE) && \
	!IS_ENABLED(CONFIG_PG_WCOM_UBOOT_UPDATE)) ||     \
	(IS_ENABLED(CONFIG_PG_WCOM_UBOOT_BOOTPACKAGE) && \
	IS_ENABLED(CONFIG_PG_WCOM_UBOOT_UPDATE)))
#error "It has to be either bootpackage or update u-boot image!"
#endif
void check_for_uboot_update(void)
{
	void (*uboot_update_entry)(void) =
		(void (*)(void)) CONFIG_PG_WCOM_UBOOT_UPDATE_TEXT_BASE;
	char *isupdated = env_get("updateduboot");
	ulong bootcount = bootcount_load();
	ulong ebootcount = 0;

	if (IS_ENABLED(CONFIG_PG_WCOM_UBOOT_BOOTPACKAGE)) {
		/*
		 * When running in factory burned u-boot move to the updated
		 * u-boot version only if updateduboot envvar is set to 'yes'
		 * and bootcount limit is not exceeded.
		 * Board must be able to start in factory bootloader mode!
		 */
		if (isupdated && !strncmp(isupdated, "yes", 3) &&
		    bootcount <= CONFIG_BOOTCOUNT_BOOTLIMIT) {
			printf("Check update: update detected, ");
			printf("starting new image @%08x ...\n",
			       CONFIG_PG_WCOM_UBOOT_UPDATE_TEXT_BASE);
			ebootcount = early_bootcount_load();
			if (ebootcount <= CONFIG_BOOTCOUNT_BOOTLIMIT) {
				early_bootcount_store(++ebootcount);
				uboot_update_entry();
			} else {
				printf("Check update: warning: ");
				printf("early bootcount exceeded (%lu)\n",
				       ebootcount);
			}
		}
		printf("Check update: starting factory image @%08x ...\n",
		       CONFIG_TEXT_BASE);
	} else if (IS_ENABLED(CONFIG_PG_WCOM_UBOOT_UPDATE)) {
		/*
		 * When running in field updated u-boot, make sure that
		 * bootcount limit is never exceeded. Must never happen!
		 */
		WARN_ON(bootcount > CONFIG_BOOTCOUNT_BOOTLIMIT);
		printf("Check update: updated u-boot starting @%08x ...\n",
		       CONFIG_TEXT_BASE);
	}
}
#endif

static void i2c_write_start_seq(void)
{
	set_sda(1);
	udelay(DELAY_HALF_PERIOD);
	set_scl(1);
	udelay(DELAY_HALF_PERIOD);
	set_sda(0);
	udelay(DELAY_HALF_PERIOD);
	set_scl(0);
	udelay(DELAY_HALF_PERIOD);
}

/*
 * I2C is a synchronous protocol and resets of the processor in the middle
 * of an access can block the I2C Bus until a powerdown of the full unit is
 * done. This function toggles the SCL until the SCL and SCA line are
 * released, but max. 16 times, after this a I2C start-sequence is sent.
 * This I2C Deblocking mechanism was developed by Keymile in association
 * with Anatech and Atmel in 1998.
 */
int i2c_make_abort(void)
{
	int	scl_state = 0;
	int	sda_state = 0;
	int	i = 0;
	int	ret = 0;

	if (!get_sda()) {
		ret = -1;
		while (i < 16) {
			i++;
			set_scl(0);
			udelay(DELAY_ABORT_SEQ);
			set_scl(1);
			udelay(DELAY_ABORT_SEQ);
			scl_state = get_scl();
			sda_state = get_sda();
			if (scl_state && sda_state) {
				ret = 0;
				break;
			}
		}
	}
	if (ret == 0)
		for (i = 0; i < 5; i++)
			i2c_write_start_seq();

	/* respect stop setup time */
	udelay(DELAY_ABORT_SEQ);
	set_scl(1);
	udelay(DELAY_ABORT_SEQ);
	set_sda(1);
	get_sda();

	return ret;
}

#if defined(CONFIG_KM_COMMON_ETH_INIT)
int board_eth_init(struct bd_info *bis)
{
	if (ethernet_present())
		return cpu_eth_init(bis);

	return -1;
}
#endif

/*
 * do_setboardid command
 * read out the board id and the hw key from the intventory EEPROM and set
 * this values as environment variables.
 */
static int do_setboardid(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	unsigned char buf[32];
	char *p;

	p = get_local_var("IVM_BoardId");
	if (!p) {
		printf("can't get the IVM_Boardid\n");
		return 1;
	}
	strcpy((char *)buf, p);
	env_set("boardid", (char *)buf);
	printf("set boardid=%s\n", buf);

	p = get_local_var("IVM_HWKey");
	if (!p) {
		printf("can't get the IVM_HWKey\n");
		return 1;
	}
	strcpy((char *)buf, p);
	env_set("hwkey", (char *)buf);
	printf("set hwkey=%s\n", buf);
	printf("Execute manually saveenv for persistent storage.\n");

	return 0;
}

U_BOOT_CMD(km_setboardid, 1, 0, do_setboardid, "setboardid",
	   "read out bid and hwkey from IVM and set in environment");

/*
 * command km_checkbidhwk
 *	if "boardid" and "hwkey" are not already set in the environment, do:
 *		if a "boardIdListHex" exists in the environment:
 *			- read ivm data for boardid and hwkey
 *			- compare each entry of the boardIdListHex with the
 *				IVM data:
 *			if match:
 *				set environment variables boardid, boardId,
 *				hwkey, hwKey to	the found values
 *				both (boardid and boardId) are set because
 *				they might be used differently in the
 *				application and in the init scripts (?)
 *	return 0 in case of match, 1 if not match or error
 */
static int do_checkboardidhwk(struct cmd_tbl *cmdtp, int flag, int argc,
			      char *const argv[])
{
	unsigned long ivmbid = 0, ivmhwkey = 0;
	unsigned long envbid = 0, envhwkey = 0;
	char *p;
	int verbose = argc > 1 && *argv[1] == 'v';
	int rc = 0;

	/*
	 * first read out the real inventory values, these values are
	 * already stored in the local hush variables
	 */
	p = get_local_var("IVM_BoardId");
	if (!p) {
		printf("can't get the IVM_Boardid\n");
		return 1;
	}
	rc = strict_strtoul(p, 16, &ivmbid);

	p = get_local_var("IVM_HWKey");
	if (!p) {
		printf("can't get the IVM_HWKey\n");
		return 1;
	}
	rc = strict_strtoul(p, 16, &ivmhwkey);

	if (!ivmbid || !ivmhwkey) {
		printf("Error: IVM_BoardId and/or IVM_HWKey not set!\n");
		return rc;
	}

	/* now try to read values from environment if available */
	p = env_get("boardid");
	if (p)
		rc = strict_strtoul(p, 16, &envbid);
	p = env_get("hwkey");
	if (p)
		rc = strict_strtoul(p, 16, &envhwkey);
	if (rc != 0) {
		printf("strict_strtoul returns error: %d", rc);
		return rc;
	}

	if (!envbid || !envhwkey) {
		/*
		 * BoardId/HWkey not available in the environment, so try the
		 * environment variable for BoardId/HWkey list
		 */
		char *bidhwklist = env_get("boardIdListHex");

		if (bidhwklist) {
			int found = 0;
			char *rest = bidhwklist;
			char *endp;

			if (verbose) {
				printf("IVM_BoardId: %ld, IVM_HWKey=%ld\n",
				       ivmbid, ivmhwkey);
				printf("boardIdHwKeyList: %s\n", bidhwklist);
			}
			while (!found) {
				/* loop over each bid/hwkey pair in the list */
				unsigned long bid   = 0;
				unsigned long hwkey = 0;

				while (*rest && !isxdigit(*rest))
					rest++;
				/*
				 * use simple_strtoul because we need &end and
				 * we know we got non numeric char at the end
				 */
				bid = hextoul(rest, &endp);
				/* BoardId and HWkey are separated with a "_" */
				if (*endp == '_') {
					rest  = endp + 1;
					/*
					 * use simple_strtoul because we need
					 * &end
					 */
					hwkey = hextoul(rest, &endp);
					rest  = endp;
					while (*rest && !isxdigit(*rest))
						rest++;
				}
				if (!bid || !hwkey) {
					/* end of list */
					break;
				}
				if (verbose) {
					printf("trying bid=0x%lX, hwkey=%ld\n",
					       bid, hwkey);
				}
				/*
				 * Compare the values of the found entry in the
				 * list with the valid values which are stored
				 * in the inventory eeprom. If they are equal
				 * set the values in environment variables.
				 */
				if (bid == ivmbid && hwkey == ivmhwkey) {
					found = 1;
					envbid   = bid;
					envhwkey = hwkey;
					env_set_hex("boardid", bid);
					env_set_hex("hwkey", hwkey);
				}
			} /* end while( ! found ) */
		}
	}

	/* compare now the values */
	if (ivmbid == envbid && ivmhwkey == envhwkey) {
		printf("boardid=0x%3lX, hwkey=%ld\n", envbid, envhwkey);
		rc = 0; /* match */
	} else {
		printf("Error: env boardid=0x%3lX, hwkey=%ld\n", envbid,
		       envhwkey);
		printf("       IVM bId=0x%3lX, hwKey=%ld\n", ivmbid, ivmhwkey);
		rc = 1; /* don't match */
	}
	return rc;
}

U_BOOT_CMD(km_checkbidhwk, 2, 0, do_checkboardidhwk,
	   "check boardid and hwkey",
	   "[v]\n  - check environment parameter \"boardIdListHex\" against stored boardid and hwkey from the IVM\n    v: verbose output"
);

/*
 * command km_checktestboot
 *  if the testpin of the board is asserted, return 1
 *  *	else return 0
 */
static int do_checktestboot(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	int testpin = 0;
	char *s = NULL;
	int testboot = 0;
	int verbose = argc > 1 && *argv[1] == 'v';

#if defined(CONFIG_POST)
	testpin = post_hotkeys_pressed();
#endif

	s = env_get("test_bank");
	/* when test_bank is not set, act as if testpin is not asserted */
	testboot = (testpin != 0) && (s);
	if (verbose) {
		printf("testpin   = %d\n", testpin);
		/* cppcheck-suppress nullPointer */
		printf("test_bank = %s\n", s ? s : "not set");
		printf("boot test app : %s\n", (testboot) ? "yes" : "no");
	}
	/* return 0 means: testboot, therefore we need the inversion */
	return !testboot;
}

U_BOOT_CMD(km_checktestboot, 2, 0, do_checktestboot,
	   "check if testpin is asserted",
	   "[v]\n  v - verbose output"
);
