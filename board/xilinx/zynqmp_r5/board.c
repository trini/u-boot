// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2018 Xilinx, Inc. (Michal Simek)
 */

#include <fdtdec.h>
#include <init.h>
#include <linux/errno.h>

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int dram_init(void)
{
	if (fdtdec_setup_mem_size_base() != 0)
		return -EINVAL;

	return 0;
}
