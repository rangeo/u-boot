/*
 * Technologic TS-7690 Single-board Computer
 *
 * (C) Copyright 2016 Technologic Systems
 * Based on work by:
 * Stuart Longland of VRT Systems <stuartl@vrt.com.au>
 *
 * Based on mx28evk.c:
 * Freescale MX28EVK board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>
#include <spi.h>
#include <fpga.h>
#include <lattice.h>
#include <i2c.h>

#define TS7690_SDBOOT_JP	MX28_PAD_LCD_WR_RWN__GPIO_1_25
#define TS7690_UBOOT_JP		MX28_PAD_LCD_RESET__GPIO_3_30

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	mac[0] = 0x00;
	mac[1] = 0xD0;
	mac[2] = 0x69;
}

static void enable_fpga_clk(void) 
{
	// Clear PWM clk gate
	writel(0x20000000, 0x80040088); // HW_CLKCTRL_XTAL_SET

	// Take PWM out of reset
	writel(0x80000000, 0x80064008); // HW_PWM_CTRL_CLR
	writel(0x40000000, 0x80064008); // HW_PWM_CTRL_CLR

	// Set PWM active and period
	writel(0x10000, 0x80064050); // HW_PWM_ACTIVE2
	writel(0xb0001, 0x80064060); // HW_PWM_PERIOD2

	// Enable PWM output
	writel(0x4, 0x80064004); // HW_PWM_CTRL_SET
	
}


#if defined(CONFIG_FPGA)

static void ts7690_jtag_init(void)
{
	gpio_direction_output(CONFIG_FPGA_TDI, 1);
	gpio_direction_output(CONFIG_FPGA_TCK, 1);
	gpio_direction_output(CONFIG_FPGA_TMS, 1);
	gpio_direction_input(CONFIG_FPGA_TDO);
	return;
}

static void ts7690_fpga_jtag_set_tdi(int value)
{
	gpio_set_value(CONFIG_FPGA_TDI, value);
}

static void ts7690_fpga_jtag_set_tms(int value)
{
	gpio_set_value(CONFIG_FPGA_TMS, value);
}

static void ts7690_fpga_jtag_set_tck(int value)
{
	gpio_set_value(CONFIG_FPGA_TCK, value);
}

static int ts7690_fpga_jtag_get_tdo(void)
{
	return gpio_get_value(CONFIG_FPGA_TDO);
}

lattice_board_specific_func ts7690_fpga_fns = {
	ts7690_jtag_init,
	ts7690_fpga_jtag_set_tdi,
	ts7690_fpga_jtag_set_tms,
	ts7690_fpga_jtag_set_tck,
	ts7690_fpga_jtag_get_tdo
};

Lattice_desc ts7690_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	589012,
	(void *) &ts7690_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

int ts7690_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts7690_fpga);

	return 0;
}

#endif // CONFIG_FPGA

int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP1 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK1, 96000, 0);
	/* SSP2 clock at 160MHz */
	mxs_set_sspclk(MXC_SSPCLK2, 160000, 0);

	/* XXX: shouldnt be needed? */
	//mxs_iomux_setup_pad(MX28_PAD_SSP0_DETECT__GPIO_2_9);

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int misc_init_r(void)
{
	int sdboot = 0;

	setenv("model", "7690");

	gpio_direction_input(TS7690_SDBOOT_JP);
	sdboot = gpio_get_value(TS7690_SDBOOT_JP);

	if(sdboot) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");
	
	gpio_direction_input(TS7690_UBOOT_JP);
	sdboot = gpio_get_value(TS7690_UBOOT_JP);

	if(sdboot) setenv("jpuboot", "off");
	else setenv("jpuboot", "on");

#if defined(CONFIG_FPGA)
	ts7690_fpga_init();
#endif
	enable_fpga_clk();

	return 0;
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

static int ts7690_mmc_cd(int id) {
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	int ret;

	/* SD card */
	ret = mxsmmc_initialize(bis, 0, NULL, ts7690_mmc_cd);
	if(ret != 0) {
		printf("SD controller initialized with %d\n", ret);
	}

	/* eMMC */
	ret = mxsmmc_initialize(bis, 1, NULL, ts7690_mmc_cd);
	if(ret != 0) {
		printf("eMMC controller initialized with %d\n", ret);
	}

	return 0;
}

#ifdef	CONFIG_CMD_NET

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;
	uchar enetaddr[6];
	uint8_t val = 0x2;

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	/* Put PHY in reset */
	i2c_write(0x28, 0x4e, 2, &val, 1);
	val = 0x0;

	/* MX28EVK uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
	       &clkctrl_regs->hw_clkctrl_enet);

	/* Take PHY out of reset */
	udelay(150);
	i2c_write(0x28, 0x4e, 2, &val, 1);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

	eth_parse_enetaddr(getenv("ethaddr"), enetaddr);
        if (!enetaddr[3] && !enetaddr[4] && !enetaddr[5]) {
                printf("No MAC address set in fuses.  Using random mac address.\n");
                eth_random_addr(enetaddr);
                random_mac = 1;
                if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
                        printf("Failed to set ethernet address\n");
                }
        }

	return ret;
}

#endif


static int set_mx28_spi(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int mode;
	unsigned char val = 0xa5;

	if(argc != 2) {
		printf("Requires a single argument\n");
		return 1;
	}

	mode = simple_strtoul(argv[1], NULL, 16);
	switch(mode) {
	  case 0:
		val = 0;
		break;
	  case 1:
		val = 0x3;
		break;
	  case 2:
		val = 0x1;
		break;
	  case 3:
		val = 0x4;
		break;
	  default:
		printf("Argument must be 0, 1, 2, or 3\n");
		return 1;
		break;
	}

	i2c_write(0x28, 0x2a, 2, &val, 1);
	return 0;
}

U_BOOT_CMD(mx28_prod, 2, 0, set_mx28_spi, 
	"Production command to set boot SPI settings",
	"[Mode]\n"
	"  Where mode is:\n"
	"    0 - En. SPI CS#, 9468 switch selected\n"
	"    1 - En. SPI CS#, force on-board SPI\n"
	"    2 - En. SPI CS#, force off-board SPI\n"
	"    3 - Dis. SPI CS# output (En. use of UART 2 & 3)\n");
