# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2016 Toradex, Inc.

# Common for all Toradex modules
ifeq ($(CONFIG_XPL_BUILD),y)
# Necessary to create built-in.a
obj- := __dummy__.a
else
obj-$(CONFIG_TDX_CFG_BLOCK) += tdx-cfg-block.o
obj-y += tdx-common.o
obj-y += tdx-eeprom.o
endif
