.. SPDX-License-Identifier: GPL-2.0+
.. Copyright (C) 2018, 2023 Thomas Fitzsimmons <fitzsim@fitzsim.org>

BCM7445 and BCM7260
===================

This document describes how to use U-Boot on the Broadcom 7445 and
Broadcom 7260 SoC, as a third stage bootloader loaded by Broadcom's
BOLT bootloader.

BOLT loads U-Boot as a generic ELF binary.  Some U-Boot features such
as networking are not implemented but other important features are,
including:

* ext4 file system traversal
* support for loading FIT images
* advanced scripting
* support for FIT-provided DTBs instead of relying on the BOLT-provided DTB

A customized version of this port has been used in production.  The
same approach may work on other BCM7xxx boards, with some
configuration adjustments and memory layout experimentation.

Configure
---------

BCM7445
^^^^^^^

.. code-block:: console

	$ make bcm7445_defconfig

BCM7260
^^^^^^^

.. code-block:: console

	$ make bcm7260_defconfig

Build
-----

.. code-block:: console

	$ make
	$ ${CROSS_COMPILE}strip u-boot

Run
---

To tell U-Boot which serial port to use for its console, set the
``stdout-path`` property in the ``/chosen`` node of the BOLT-generated
device tree.  For example:

::

	BOLT> dt add prop chosen stdout-path s serial0:115200n8

Flash the ``u-boot`` binary into board storage, then invoke it from
BOLT.  For example:

::

	BOLT> boot -bsu -elf flash0.u-boot1

This port assumes that I-cache and D-cache are already enabled when
U-Boot is entered.

Flattened Image Tree Support
----------------------------

What follows is an example FIT image source file.  Build it with:

.. code-block:: console

	$ mkimage -f image.its image.itb

Booting the resulting ``image.itb`` was tested on BOLT v1.20, with the
following kernels:

* https://github.com/Broadcom/stblinux-3.14
* https://github.com/Broadcom/stblinux-4.1
* https://github.com/Broadcom/stblinux-4.9

and with a generic ARMv7 root file system.

**image.its**

::

	/dts-v1/;
	/ {
		description = "BCM7445 FIT";
		images {
			kernel@1 {
				description = "Linux kernel";
				/*
				 * This kernel image output format can be
				 * generated with:
				 *
				 * make vmlinux
				 * ${CROSS_COMPILE}objcopy -O binary -S vmlinux vmlinux.bin
				 * gzip -9 vmlinux.bin
				 *
				 * For stblinux-3.14, the specific Broadcom
				 * board type should be configured in the
				 * kernel, for example CONFIG_BCM7445D0=y.
				 */
				data = /incbin/("<vmlinux.bin.gz>");
				type = "kernel";
				arch = "arm";
				os = "linux";
				compression = "gzip";
				load = <0x8000>;
				entry = <0x8000>;
				hash@1 {
					algo = "sha256";
				};
			};
			ramdisk@1 {
				description = "Initramfs root file system";
				data = /incbin/("<initramfs.cpio.gz>");
				type = "ramdisk";
				arch = "arm";
				os = "linux";
				compression = "gzip";
				/*
				 * Set the environment variable initrd_high to
				 * 0xffffffff, and set "load" and "entry" here
				 * to 0x0 to keep initramfs in-place and to
				 * accommodate stblinux bmem/CMA reservations.
				 */
				load = <0x0>;
				entry = <0x0>;
				hash@1 {
					algo = "sha256";
				};
			};
			fdt@1 {
				description = "Device tree dumped from BOLT";
				/*
				 * This DTB should be similar to the
				 * BOLT-generated device tree, after BOLT has
				 * done its runtime modifications to it.  For
				 * example, it can be dumped from within
				 * U-Boot (at ${fdtcontroladdr}), after BOLT
				 * has loaded U-Boot.  The result can be added
				 * to the Linux source tree as a .dts file.
				 *
				 * To support modifications to the device tree
				 * in-place in U-Boot, set the config variable
				 * CONFIG_SYS_DTC_PAD_BYTES as needed.
				 *
				 * This will leave some padding in the DTB and
				 * thus reserve room for node additions.
				 *
				 * Also, set the environment variable fdt_high
				 * to 0xffffffff to keep the DTB in-place and
				 * to accommodate stblinux bmem/CMA
				 * reservations.
				 */
				data = /incbin/("<bolt-<version>.dtb");
				type = "flat_dt";
				arch = "arm";
				compression = "none";
				hash@1 {
					algo = "sha256";
				};
			};
		};
		configurations {
			default = "conf@bcm7445";
			conf@bcm7445 {
				description = "BCM7445 configuration";
				kernel = "kernel@1";
				ramdisk = "ramdisk@1";
				fdt = "fdt@1";
			};
		};
	};
