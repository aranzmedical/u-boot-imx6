/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the SolidRun mx6 based boards
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __SILHOUETTESTAR2_CONFIG_H
#define __SILHOUETTESTAR2_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#include "mx6_common.h"

#undef CONFIG_LDO_BYPASS_CHECK

// #define CONFIG_IMX_THERMAL

#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)
#define CONFIG_MXC_UART

#define CONFIG_PREBOOT \
	"setenv stdin  serial; " \
	"setenv stdout serial; " \
	"setenv stderr serial; "

/* Command definition */

#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV	"ttymxc0"
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC2 */

#ifdef CONFIG_SPL_BOOT_DEVICE_MMC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC3_BASE_ADDR
#else
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR
#endif

#ifndef CONFIG_SPL_BUILD
#define CONFIG_EXTRA_ENV_SETTINGS \
  "som_rev=undefined\0" \
	"has_emmc=undefined\0" \
	"fdtfile=undefined\0" \
	"fdt_addr_r=0x18000000\0" \
	"fdt_addr=0x18000000\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0"  \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"ramdisk_addr_r=0x13000000\0" \
	"ramdiskaddr=0x13000000\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_high=0xffffffff\0" \
	"ip_dyn=yes\0" \
	"console=" CONSOLE_DEV ",115200\0" \
	"bootm_size=0x10000000\0" \
	"finduuid=part uuid mmc 0:1 uuid\0" \
  "fdt_addr=0x18000000\0" \
  "loadftdfilefromboot=load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /boot/${fdt_file};\0" \
  "loadftdfile=load mmc ${mmcdev}:1 ${fdt_addr} ${fdt_file};\0" \
  "loadzImagefilefromboot=load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/zImage;\0" \
  "loadzImagefile=load mmc ${mmcdev}:1 ${loadaddr} zImage;\0" \
  "boot_aranz=" \
    "setenv bootargs console=${console},${baudrate} quiet coherent_pool=180M root=${mmcroot} rootwait ro rootfstype=ext4 nohlt; " \
    "if test ${board_rev} = MX6DL; then " \
      "setenv fdt_prefix imx6dl; " \
    "else " \
      "setenv fdt_prefix imx6q; " \
    "fi; " \
    "if test ${som_rev} = V15; then " \
			"setenv fdtsuffix -som-v15; fi; " \
		"if test ${has_emmc} = yes; then " \
			"setenv emmcsuffix -emmc; fi; " \
    "setenv fdt_file ${fdt_prefix}-silhouettestar{emmcsuffix}${fdtsuffix}.dtb;" \
    "if run loadftdfilefromboot; then " \
      "echo Loaded Devicetree from mmc${mmcdev}:${mmcpart} /boot/${fdt_file};" \
    "else " \
      "if run loadftdfile; then " \
        "echo Loaded Devicetree from Boot Partition.;" \
      "else " \
        "echo WARNING: Could not determine dtb to use;" \
      "fi; " \
    "fi; " \
    "if run loadzImagefilefromboot; then " \
      "echo Loaded Linux Kernel from mmc${mmcdev}:${mmcpart} /boot/zImage; " \
    "else " \
      "if run loadzImagefile; then " \
        "echo Loaded Linux Kernel from Boot Partition.; " \
      "else " \
        "echo WARNING: Could not determine Linux Kernel to load;" \
      "fi; " \
    "fi; " \
    "bootz ${loadaddr} - ${fdt_addr};\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	BOOTENV

#ifdef CONFIG_SPL_BOOT_DEVICE_MMC
#define BOOT_TARGET_DEVICES(func) func(MMC, mmc, 1) func(MMC, mmc, 0)
#else
#define BOOT_TARGET_DEVICES(func) func(MMC, mmc, 0) func(MMC, mmc, 1)
#endif

#include <config_distro_bootcmd.h>

#else
#define CONFIG_EXTRA_ENV_SETTINGS
#endif /* CONFIG_SPL_BUILD */

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define CONFIG_SYS_SDRAM_BASE          MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)
#endif                         /* __SILHOUETTESTAR2_CONFIG_H */
