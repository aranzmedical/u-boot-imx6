/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Author: Chris Schuster <chris.schuster@aranzmedical.com>
 *
 * Based on SPL code from Solidrun tree, which is:
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 * 
 * Based on SPL code from Solidrun tree, which is:
 * Author: Tungyi Lin <tungyilin1127@gmail.com>
 *
 * Derived from EDM_CF_IMX6 code by TechNexion,Inc
 * Ported to SolidRun microSOM by Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <init.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <env.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/crm_regs.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  ((PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE) | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define ETH_PHY_RESET	IMX_GPIO_NR(4, 15)
#define USB_H1_VBUS	IMX_GPIO_NR(1, 0)

#define MEM_STRIDE 0x4000000
static u32 get_ram_size_stride_test(u32 *base, u32 maxsize)
{
        volatile u32 *addr;
        u32          save[64];
        u32          cnt;
        u32          size;
        int          i = 0;

        /* First save the data */
        for (cnt = 0; cnt < maxsize; cnt += MEM_STRIDE) {
                addr = (volatile u32 *)((u32)base + cnt);       /* pointer arith! */
                sync ();
                save[i++] = *addr;
                sync ();
        }

        /* First write a signature */
        * (volatile u32 *)base = 0x12345678;
        for (size = MEM_STRIDE; size < maxsize; size += MEM_STRIDE) {
                * (volatile u32 *)((u32)base + size) = size;
                sync ();
                if (* (volatile u32 *)((u32)base) == size) {	/* We reached the overlapping address */
                        break;
                }
        }

        /* Restore the data */
        for (cnt = (maxsize - MEM_STRIDE); i > 0; cnt -= MEM_STRIDE) {
                addr = (volatile u32 *)((u32)base + cnt);       /* pointer arith! */
                sync ();
                *addr = save[i--];
                sync ();
        }

        return (size);
}

int dram_init(void)
{
	u32 max_size = imx_ddr_size();

	gd->ram_size = get_ram_size_stride_test((u32 *) CONFIG_SYS_SDRAM_BASE,
						(u32)max_size);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_RST__SD3_RESET       | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const som_rev_detect[] = {
	/* These pins are for sensing if it is a CuBox-i or a HummingBoard */
	IOMUX_PADS(PAD_CSI0_DAT14__GPIO6_IO00  | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT18__GPIO6_IO04  | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const boot_sense[] = {
	/* These pins are for sensing if it is a CuBox-i or a HummingBoard */
	IOMUX_PADS(PAD_EIM_DA1__GPIO3_IO01  | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__GPIO3_IO27  | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

#define USDHC2_CD_GPIO  IMX_GPIO_NR(1, 4)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = (mmc_get_op_cond(mmc) < 0) ? 0 : 1; /* eMMC/uSDHC3 has no CD GPIO */
		break;
	}

	return ret;
}

static struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    SOM MicroSD
	 * mmc1                    Carrier board MicroSD
	 */
	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			SETUP_IOMUX_PADS(usdhc2_pads);
			usdhc_cfg[index].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			usdhc_cfg[index].max_bus_width = 4;
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc3_pads);
			usdhc_cfg[index].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[index].max_bus_width = 8;
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (ret)
			return ret;
	}

	return 0;
#else
        struct src *psrc = (struct src *)SRC_BASE_ADDR;
        unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD2
	 * 0x2                  SD3
	 */

        switch (reg & 0x3) {
        case 0x1:
                SETUP_IOMUX_PADS(usdhc2_pads);
                usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
                usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		usdhc_cfg[0].max_bus_width = 4;
                gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
                break;
        case 0x2:
                SETUP_IOMUX_PADS(usdhc3_pads);
                usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
                usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		usdhc_cfg[0].max_bus_width = 8;
                gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
                break;
        }

        return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif

}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

// int board_eth_init(bd_t *bis)
// {
//   return 0;
// }

int board_early_init_f(void)
{
	setup_iomux_uart();
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	return 0;
}

static bool is_rev_15_som(void)
{
	int val1, val2;
	SETUP_IOMUX_PADS(som_rev_detect);

	val1 = gpio_get_value(IMX_GPIO_NR(6, 0));
	val2 = gpio_get_value(IMX_GPIO_NR(6, 4));

	if (val1 == 1 && val2 == 0)
		return true;

	return false;
}

static bool has_emmc(void)
{
	struct mmc *mmc;
	mmc = find_mmc_device(2);
	if (!mmc)
		return 0;
	return (mmc_get_op_cond(mmc) < 0) ? 0 : 1;
}

int checkboard(void)
{
	puts("Board: SilhouetteStar2");

	if (is_rev_15_som())
		puts(" (som rev 1.5)\n");
	else
		puts("\n");

	return 0;
}

int board_late_init(void)
{
  char const *aranzboot;
  char const *aranzmmcroot;

  env_set("board_name", "SILHOUETTESTAR2");

	if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else
		env_set("board_rev", "MX6DL");

   /* Detect Boot pins */
  SETUP_IOMUX_PADS(boot_sense);

  gpio_direction_input(IMX_GPIO_NR(3, 1));
  gpio_direction_input(IMX_GPIO_NR(3, 27));

  int aranzboot1 = gpio_get_value(IMX_GPIO_NR(3, 1));
  int aranzboot2 = gpio_get_value(IMX_GPIO_NR(3, 27));

  env_set("mmcdev","0");

  if (aranzboot1 == 1 && aranzboot2 == 0)
  {
		puts("Boot: OS1\n");
    aranzboot = "OS1";
    aranzmmcroot = "/dev/mmcblk0p2\0";
    env_set("mmcpart","2");
  }
  else if (aranzboot1 == 0 && aranzboot2 == 1)
  {
		puts("Boot: OS2\n");
    aranzboot = "OS2";
    aranzmmcroot = "/dev/mmcblk0p3\0";
    env_set("mmcpart","3");
  }
  else
	{
		puts("Boot: RESCUE\n");
    aranzboot = "RESCUE";
    aranzmmcroot = "/dev/mmcblk0p5\0";
    env_set("mmcpart","5");
	}

  env_set("aranzboot",aranzboot);
  env_set("aranzmmcroot",aranzmmcroot);
  env_set("mmcroot",aranzmmcroot);

  if (is_rev_15_som())
	env_set("som_rev", "V15");

  if (has_emmc())
    env_set("has_emmc", "yes");

//#endif

	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
static const struct mx6dq_iomux_ddr_regs mx6q_ddr_ioregs = {
	.dram_sdclk_0 =  0x00020030,
	.dram_sdclk_1 =  0x00020030,
	.dram_cas =  0x00020030,
	.dram_ras =  0x00020030,
	.dram_reset =  0x000c0030,
	.dram_sdcke0 =  0x00003000,
	.dram_sdcke1 =  0x00003000,
	.dram_sdba2 =  0x00000000,
	.dram_sdodt0 =  0x00003030,
	.dram_sdodt1 =  0x00003030,
	.dram_sdqs0 =  0x00000030,
	.dram_sdqs1 =  0x00000030,
	.dram_sdqs2 =  0x00000030,
	.dram_sdqs3 =  0x00000030,
	.dram_sdqs4 =  0x00000030,
	.dram_sdqs5 =  0x00000030,
	.dram_sdqs6 =  0x00000030,
	.dram_sdqs7 =  0x00000030,
	.dram_dqm0 =  0x00020030,
	.dram_dqm1 =  0x00020030,
	.dram_dqm2 =  0x00020030,
	.dram_dqm3 =  0x00020030,
	.dram_dqm4 =  0x00020030,
	.dram_dqm5 =  0x00020030,
	.dram_dqm6 =  0x00020030,
	.dram_dqm7 =  0x00020030,
};

static const struct mx6sdl_iomux_ddr_regs mx6dl_ddr_ioregs = {
	.dram_sdclk_0 = 0x00000028,
	.dram_sdclk_1 = 0x00000028,
	.dram_cas =	0x00000028,
	.dram_ras =	0x00000028,
	.dram_reset =	0x000c0028,
	.dram_sdcke0 =	0x00003000,
	.dram_sdcke1 =	0x00003000,
	.dram_sdba2 =	0x00000000,
	.dram_sdodt0 =	0x00003030,
	.dram_sdodt1 =	0x00003030,
	.dram_sdqs0 =	0x00000028,
	.dram_sdqs1 =	0x00000028,
	.dram_sdqs2 =	0x00000028,
	.dram_sdqs3 =	0x00000028,
	.dram_sdqs4 =	0x00000028,
	.dram_sdqs5 =	0x00000028,
	.dram_sdqs6 =	0x00000028,
	.dram_sdqs7 =	0x00000028,
	.dram_dqm0 =	0x00000028,
	.dram_dqm1 =	0x00000028,
	.dram_dqm2 =	0x00000028,
	.dram_dqm3 =	0x00000028,
	.dram_dqm4 =	0x00000028,
	.dram_dqm5 =	0x00000028,
	.dram_dqm6 =	0x00000028,
	.dram_dqm7 =	0x00000028,
};

static const struct mx6dq_iomux_grp_regs mx6q_grp_ioregs = {
	.grp_ddr_type =  0x000C0000,
	.grp_ddrmode_ctl =  0x00020000,
	.grp_ddrpke =  0x00000000,
	.grp_addds =  0x00000030,
	.grp_ctlds =  0x00000030,
	.grp_ddrmode =  0x00020000,
	.grp_b0ds =  0x00000030,
	.grp_b1ds =  0x00000030,
	.grp_b2ds =  0x00000030,
	.grp_b3ds =  0x00000030,
	.grp_b4ds =  0x00000030,
	.grp_b5ds =  0x00000030,
	.grp_b6ds =  0x00000030,
	.grp_b7ds =  0x00000030,
};

static const struct mx6sdl_iomux_grp_regs mx6sdl_grp_ioregs = {
	.grp_ddr_type = 0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_addds = 0x00000028,
	.grp_ctlds = 0x00000028,
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000028,
	.grp_b1ds = 0x00000028,
	.grp_b2ds = 0x00000028,
	.grp_b3ds = 0x00000028,
	.grp_b4ds = 0x00000028,
	.grp_b5ds = 0x00000028,
	.grp_b6ds = 0x00000028,
	.grp_b7ds = 0x00000028,
};

/* microSOM with Dual processor and 1GB memory */
static const struct mx6_mmdc_calibration mx6q_1g_mmcd_calib = {
	.p0_mpwldectrl0 =  0x00000000,
	.p0_mpwldectrl1 =  0x00000000,
	.p1_mpwldectrl0 =  0x00000000,
	.p1_mpwldectrl1 =  0x00000000,
	.p0_mpdgctrl0 =    0x0314031c,
	.p0_mpdgctrl1 =    0x023e0304,
	.p1_mpdgctrl0 =    0x03240330,
	.p1_mpdgctrl1 =    0x03180260,
	.p0_mprddlctl =    0x3630323c,
	.p1_mprddlctl =    0x3436283a,
	.p0_mpwrdlctl =    0x36344038,
	.p1_mpwrdlctl =    0x422a423c,
};

/* microSOM with Quad processor and 2GB memory */
static const struct mx6_mmdc_calibration mx6q_2g_mmcd_calib = {
	.p0_mpwldectrl0 =  0x00000000,
	.p0_mpwldectrl1 =  0x00000000,
	.p1_mpwldectrl0 =  0x00000000,
	.p1_mpwldectrl1 =  0x00000000,
	.p0_mpdgctrl0 =    0x0314031c,
	.p0_mpdgctrl1 =    0x023e0304,
	.p1_mpdgctrl0 =    0x03240330,
	.p1_mpdgctrl1 =    0x03180260,
	.p0_mprddlctl =    0x3630323c,
	.p1_mprddlctl =    0x3436283a,
	.p0_mpwrdlctl =    0x36344038,
	.p1_mpwrdlctl =    0x422a423c,
};

/* microSOM with Solo processor and 512MB memory */
static const struct mx6_mmdc_calibration mx6dl_512m_mmcd_calib = {
	.p0_mpwldectrl0 = 0x0045004D,
	.p0_mpwldectrl1 = 0x003A0047,
	.p0_mpdgctrl0 =   0x023C0224,
	.p0_mpdgctrl1 =   0x02000220,
	.p0_mprddlctl =   0x44444846,
	.p0_mpwrdlctl =   0x32343032,
};

/* microSOM with Dual lite processor and 1GB memory */
static const struct mx6_mmdc_calibration mx6dl_1g_mmcd_calib = {
	.p0_mpwldectrl0 =  0x0045004D,
	.p0_mpwldectrl1 =  0x003A0047,
	.p1_mpwldectrl0 =  0x001F001F,
	.p1_mpwldectrl1 =  0x00210035,
	.p0_mpdgctrl0 =    0x023C0224,
	.p0_mpdgctrl1 =    0x02000220,
	.p1_mpdgctrl0 =    0x02200220,
	.p1_mpdgctrl1 =    0x02040208,
	.p0_mprddlctl =    0x44444846,
	.p1_mprddlctl =    0x4042463C,
	.p0_mpwrdlctl =    0x32343032,
	.p1_mpwrdlctl =    0x36363430,
};

static struct mx6_ddr3_cfg mem_ddr_2g = {
	.mem_speed = 1600,
	.density   = 2,
	.width     = 16,
	.banks     = 8,
	.rowaddr   = 14,
	.coladdr   = 10,
	.pagesz    = 2,
	.trcd      = 1375,
	.trcmin    = 4875,
	.trasmin   = 3500,
};

static struct mx6_ddr3_cfg mem_ddr_4g = {
	.mem_speed = 1600,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 16,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void spl_dram_init(int width)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus: 0=16, 1=32, 2=64 */
		.dsize = width / 32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32,	/* 32Gb per CS */
		.ncs = 1,		/* single chip select */
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
		.refsel = 1,	/* Refresh cycles at 32KHz */
		.refr = 7,	/* 8 refresh commands per refresh cycle */
	};

	if (is_mx6dq())
		mx6dq_dram_iocfg(width, &mx6q_ddr_ioregs, &mx6q_grp_ioregs);
	else
		mx6sdl_dram_iocfg(width, &mx6dl_ddr_ioregs, &mx6sdl_grp_ioregs);

	if (is_cpu_type(MXC_CPU_MX6D))
		mx6_dram_cfg(&sysinfo, &mx6q_1g_mmcd_calib, &mem_ddr_2g);
	else if (is_cpu_type(MXC_CPU_MX6Q))
		mx6_dram_cfg(&sysinfo, &mx6q_2g_mmcd_calib, &mem_ddr_4g);
	else if (is_cpu_type(MXC_CPU_MX6DL))
		mx6_dram_cfg(&sysinfo, &mx6dl_1g_mmcd_calib, &mem_ddr_2g);
	else if (is_cpu_type(MXC_CPU_MX6SOLO))
		mx6_dram_cfg(&sysinfo, &mx6dl_512m_mmcd_calib, &mem_ddr_2g);
}

// void board_init_f(ulong dummy)
// {
// 	/* setup AIPS and disable watchdog */
// 	arch_cpu_init();

// 	ccgr_init();
// 	gpr_init();

// 	/* iomux and setup of i2c */
// 	board_early_init_f();

// 	/* setup GP timer */
// 	timer_init();

// 	/* UART clocks enabled and gd valid - init serial console */
// 	preloader_console_init();

// 	/* DDR initialization */
// 	if (is_cpu_type(MXC_CPU_MX6SOLO))
// 		spl_dram_init(32);
// 	else
// 		spl_dram_init(64);

// 	/* Clear the BSS. */
// 	memset(__bss_start, 0, __bss_end - __bss_start);

// 	/* load/boot image from boot device */
// 	board_init_r(NULL, 0);
// }
#endif
