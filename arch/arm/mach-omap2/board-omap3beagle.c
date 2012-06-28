/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/backlight.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/usb/android_composite.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>
#include <linux/i2c.h>
#include <linux/i2c/liquidware_pcap.h>
#include <linux/i2c/tsc2007.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/pwm.h>

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"

#define NAND_BLOCK_SIZE		SZ_128K

#ifdef CONFIG_USB_ANDROID

#define GOOGLE_VENDOR_ID		0x18d1
#define GOOGLE_PRODUCT_ID		0x9018
#define GOOGLE_ADB_PRODUCT_ID		0x9015

static char *usb_functions_ums[] = {
    "usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
   "usb_mass_storage",
   "adb",
};

static char *usb_functions_all[] = {
    "usb_mass_storage",
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
        .product_id     = 0x4e11,
        .num_functions  = ARRAY_SIZE(usb_functions_ums),
        .functions      = usb_functions_ums,
	},
	{
       .product_id     = 0x4e12,
       .num_functions  = ARRAY_SIZE(usb_functions_ums_adb),
       .functions      = usb_functions_ums_adb,
    },
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns          = 1,
       .vendor         = "Liquidware",
       .product        = "Liquidware Android",
       .release        = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
       .name   = "usb_mass_storage",
       .id     = -1,
       .dev    = {
               .platform_data = &mass_storage_pdata,
        },
 };

static struct android_usb_platform_data android_usb_pdata = {
    .vendor_id      = 0x18d1,
    .product_id     = 0x4e12,
	.version	= 0x0100,
	.product_name	= "Liquidware Android",
	.manufacturer_name	= "Liquidware",
	.serial_number	= "20100720",
    .num_products = ARRAY_SIZE(usb_products),
    .products = usb_products,
    .num_functions = ARRAY_SIZE(usb_functions_all),
    .functions = usb_functions_all,
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static void omap3evm_android_gadget_init(void)
{
    platform_device_register(&usb_mass_storage_device);
	platform_device_register(&androidusb_device);
}
#endif
/*
 * OMAP3 Beagle revision
 * Run time detection of Beagle revision is done by reading GPIO.
 * GPIO ID -
 *	AXBX	= GPIO173, GPIO172, GPIO171: 1 1 1
 *	C1_3	= GPIO173, GPIO172, GPIO171: 1 1 0
 *	C4	= GPIO173, GPIO172, GPIO171: 1 0 1
 *	XM	= GPIO173, GPIO172, GPIO171: 0 0 0
 */
enum {
	OMAP3BEAGLE_BOARD_UNKN = 0,
	OMAP3BEAGLE_BOARD_AXBX,
	OMAP3BEAGLE_BOARD_C1_3,
	OMAP3BEAGLE_BOARD_C4,
	OMAP3BEAGLE_BOARD_XM,
	OMAP3BEAGLE_BOARD_XMC,
};

static u8 omap3_beagle_version;

static u8 omap3_beagle_get_rev(void)
{
	return omap3_beagle_version;
}

static void __init omap3_beagle_init_rev(void)
{
	int ret;
	u16 beagle_rev = 0;

	omap_mux_init_gpio(171, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(172, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(173, OMAP_PIN_INPUT_PULLUP);

	ret = gpio_request(171, "rev_id_0");
	if (ret < 0)
		goto fail0;

	ret = gpio_request(172, "rev_id_1");
	if (ret < 0)
		goto fail1;

	ret = gpio_request(173, "rev_id_2");
	if (ret < 0)
		goto fail2;

	gpio_direction_input(171);
	gpio_direction_input(172);
	gpio_direction_input(173);

	beagle_rev = gpio_get_value(171) | (gpio_get_value(172) << 1)
			| (gpio_get_value(173) << 2);

	switch (beagle_rev) {
	case 7:
		printk(KERN_INFO "OMAP3 Beagle Rev: Ax/Bx\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_AXBX;
		break;
	case 6:
		printk(KERN_INFO "OMAP3 Beagle Rev: C1/C2/C3\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C1_3;
		break;
	case 5:
		printk(KERN_INFO "OMAP3 Beagle Rev: C4\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C4;
		break;
	case 2:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM C\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XMC;
		break;
	case 0:
		printk(KERN_INFO "OMAP3 Beagle Rev: xM\n");
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XM;
		break;
	default:
		printk(KERN_INFO "OMAP3 Beagle Rev: unknown %hd\n", beagle_rev);
		omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
	}

	return;

fail2:
	gpio_free(172);
fail1:
	gpio_free(171);
fail0:
	printk(KERN_ERR "Unable to get revision detection GPIO pins\n");
	omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;

	return;
}

static struct mtd_partition omap3beagle_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

/* DSS */

static int beagle_enable_dvi(struct omap_dss_device *dssdev)
{
#define ENABLE_VAUX1_DEDICATED	0x04
#define ENABLE_VAUX1_DEV_GRP	0x20

    /* Enable VAUX1 regulator */
    twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEDICATED,
			TWL4030_VAUX1_DEDICATED);
    twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEV_GRP,
			TWL4030_VAUX1_DEV_GRP);

	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void beagle_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device beagle_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "liquidware_beagletouch",
	.phy.dpi.data_lines = 24,
	.reset_gpio = -EINVAL,
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
};

static struct omap_dss_device beagle_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *beagle_dss_devices[] = {
	&beagle_dvi_device,
	&beagle_tv_device,
};

static struct omap_dss_board_info beagle_dss_data = {
	.num_devices = ARRAY_SIZE(beagle_dss_devices),
	.devices = beagle_dss_devices,
	.default_device = &beagle_dvi_device,
};

static struct platform_device beagle_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &beagle_dss_data,
	},
};

static struct regulator_consumer_supply beagle_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");

static struct regulator_consumer_supply beagle_vdvi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

/* Pins for the BeagleTouch */
#define CS_PIN			139
#define MISO_PIN		146
#define MOSI_PIN		144
#define CLK_PIN			138
//#define RESET_PIN		137 //oled
#define RESET_PIN		129 //openpad
#define PANEL_PWR_PIN	143

//#define OMAP3_BEAGLETOUCH_TS_INT 	136 //BeagleTouch
#define OMAP3_BEAGLETOUCH_TS_INT 	15  //OpenPad
#define OMAP3_BEAGLETOUCH_TS_RESET	158 //OpenPad

static void __init beagle_display_init(void)
{
	int r;

	r = gpio_request(beagle_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}

	gpio_direction_output(beagle_dvi_device.reset_gpio, 0);

        /* BeagleTouch GPIO */
	r |= omap_mux_init_gpio(CS_PIN, OMAP_PIN_OUTPUT);
	r |= omap_mux_init_gpio(MISO_PIN, OMAP_PIN_INPUT_PULLUP);
	r |= omap_mux_init_gpio(MOSI_PIN, OMAP_PIN_OUTPUT);
	r |= omap_mux_init_gpio(CLK_PIN, OMAP_PIN_OUTPUT);
	r |= omap_mux_init_gpio(RESET_PIN, OMAP_PIN_OUTPUT);
	r |= omap_mux_init_gpio(PANEL_PWR_PIN, OMAP_PIN_OUTPUT);

	/* OpenPad switches */
	r |= omap_mux_init_gpio(152, OMAP_PIN_INPUT_PULLUP);
	r |= omap_mux_init_gpio(153, OMAP_PIN_INPUT_PULLUP);
	r |= omap_mux_init_gpio(154, OMAP_PIN_INPUT_PULLUP);
	r |= omap_mux_init_gpio(155, OMAP_PIN_INPUT_PULLUP);

	if (r < 0) {
		printk(KERN_ERR "Unable to init display GPIO\n");
		return;
	}

	/* OpenPad Backlight PWM */
	r = omap_mux_init_signal("gpmc_ncs4.gpt9_pwm_evt", OMAP_PIN_OUTPUT);
	if (r < 0) {
		printk(KERN_ERR "Unable to init display backlight GPIO\n");
		return;
	}

	/* BeagleTouch Backlight PWM */
	r = omap_mux_init_signal("uart2_cts.gpt9_pwm_evt", OMAP_PIN_OUTPUT);
	if (r < 0) {
		printk(KERN_ERR "Unable to init display backlight GPIO\n");
		return;
	}
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= 29,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply beagle_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply beagle_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct regulator_consumer_supply beagle_vaux3_supply = {
	.supply         = "cam_1v8",
};

static struct regulator_consumer_supply beagle_vaux4_supply = {
	.supply         = "cam_2v8",
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC) {
		mmc[0].gpio_wp = -EINVAL;
	} else if ((omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_C1_3) ||
		(omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_C4)) {
		omap_mux_init_gpio(23, OMAP_PIN_INPUT);
		mmc[0].gpio_wp = 23;
	} else {
		omap_mux_init_gpio(29, OMAP_PIN_INPUT);
	}
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	beagle_vmmc1_supply.dev = mmc[0].dev;
	beagle_vsim_supply.dev = mmc[0].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */
	if (omap3_beagle_get_rev() != OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC) {
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);
	}

	/*
	 * TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, XM active
	 * high / others active low)
	 */
	gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
	gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM)
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	else
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);

	/* DVI reset GPIO is different between beagle revisions */
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM ||
		omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC)
		beagle_dvi_device.reset_gpio = 129;
	else
		beagle_dvi_device.reset_gpio = 170;

	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM) {
		/* Power on camera interface */
		gpio_request(gpio + 2, "CAM_EN");
		gpio_direction_output(gpio + 2, 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	} else {
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	}
	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	/*
	 * gpio + 1 on Xm controls the TFP410's enable line (active low)
	 * gpio + 2 control varies depending on the board rev as follows:
	 * P7/P8 revisions(prototype): Camera EN
	 * A2+ revisions (production): LDO (supplies DVI, serial, led blocks)
	 */
	if (omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XM || omap3_beagle_get_rev() == OMAP3BEAGLE_BOARD_XMC) {
		gpio_request(gpio + 1, "nDVI_PWR_EN");
		gpio_direction_output(gpio + 1, 0);
		gpio_request(gpio + 2, "DVI_LDO_EN");
		gpio_direction_output(gpio + 2, 1);
	}

	return 0;
}

static int liquidware_pcap_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_BEAGLETOUCH_TS_INT);
}

static void liquidware_pcap_perform_reset(void)
{
	gpio_direction_output(OMAP3_BEAGLETOUCH_TS_RESET, 1);
	mdelay(10);
	gpio_direction_output(OMAP3_BEAGLETOUCH_TS_RESET, 0);

}

static int omap3beagletouch_liquidware_pcap_init(struct liquidware_pcap_platform_data *pdata)
{
	int gpio;
	int ret = 0;

	gpio = pdata->pen_gpio;
	ret |= omap_mux_init_gpio(gpio, OMAP_PIN_INPUT);
	ret |= gpio_request(gpio, "liquidware_pcap ts_int");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for "
				"liquidware_pcap\n", gpio);
		return ret;
	}
	gpio_direction_input(gpio);

	gpio = pdata->reset_gpio;
	ret |= omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	ret |= gpio_request(gpio, "liquidware_pcap reset");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for "
				"liquidware_pcap\n", gpio);
		return ret;
	}
	gpio_direction_output(gpio, 0);

	return ret;
}

static struct liquidware_pcap_platform_data liquidware_pcap_info = {
	.model			= 2007,
	.x_plate_ohms	= 180,
	.pen_gpio		= OMAP3_BEAGLETOUCH_TS_INT,
	.reset_gpio		= OMAP3_BEAGLETOUCH_TS_RESET,
	.get_pendown_state	= liquidware_pcap_get_pendown_state,
	.perform_reset		= liquidware_pcap_perform_reset,
	.init_platform_hw	= omap3beagletouch_liquidware_pcap_init,
};

static struct tsc2007_platform_data tsc2007_info = {
	.model				= 2007,
	.x_plate_ohms		= 180,
    .get_pendown_state  = NULL,
};

static struct i2c_board_info __initdata i2c3_clients[] = {
	{
		I2C_BOARD_INFO("liquidware_pcap", 0x38),
		.irq	= OMAP_GPIO_IRQ(OMAP3_BEAGLETOUCH_TS_INT),
		.platform_data	= &liquidware_pcap_info,
	},
#if 0
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.platform_data	= &tsc2007_info,
	},
#endif
};

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data beagle_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data beagle_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdvi_supply,
};

/* VAUX3 for CAM_1V8 */
static struct regulator_init_data beagle_vaux3 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &beagle_vaux3_supply,
};

 /* VAUX4 for CAM_2V8 */
static struct regulator_init_data beagle_vaux4 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &beagle_vaux4_supply,
};

static struct twl4030_usb_data beagle_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data beagle_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data beagle_codec_data = {
	.audio_mclk = 26000000,
	.audio = &beagle_audio_data,
};

static struct twl4030_platform_data beagle_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &beagle_usb_data,
	.gpio		= &beagle_gpio_data,
	.codec		= &beagle_codec_data,
	.vmmc1		= &beagle_vmmc1,
	.vsim		= &beagle_vsim,
	.vdac		= &beagle_vdac,
	.vpll2		= &beagle_vpll2,
	.vaux3		= &beagle_vaux3,
	.vaux4		= &beagle_vaux4,
};

static struct i2c_board_info __initdata beagle_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &beagle_twldata,
	},
};

static struct i2c_board_info __initdata beagle_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("eeprom", 0x50),
       },
};

static struct i2c_board_info __initdata beagle_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("itaniumpack_battery", 0x0B),
	},
};

static int __init omap3_beagle_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, beagle_i2c_boardinfo,
			ARRAY_SIZE(beagle_i2c_boardinfo));

	/* Bus 2 is used for Camera/Sensor interface */
	omap_register_i2c_bus(2, 50, beagle_i2c2_boardinfo, ARRAY_SIZE(beagle_i2c2_boardinfo));

	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100,  i2c3_clients, ARRAY_SIZE(i2c3_clients));

	return 0;
}

static struct omap2_pwm_platform_config pwm_config = {
	.timer_id           = 9,   // GPT9_PWM_EVT
	.polarity           = 0    // Active-high
};

static struct platform_device pwm_device = {
	.name               = "omap-pwm",
	.id                 = 0,
	.dev                = {
		.platform_data  = &pwm_config
	}
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "openpad::glow-t",
		.gpio			= 34,
	},
	{
		.name			= "openpad::glow0",
		.gpio			= 50,
	},
        {
		.name			= "openpad::glow1",
		.gpio			= 51,
	},
	{
		.name			= "openpad::glow2",
		.gpio			= 49,
	},
        {
		.name			= "openpad::glow3",
		.gpio			= 47,
	},
	{
		.name			= "openpad::glow4",
		.gpio			= 45,
	},
        {
		.name			= "openpad::glow5",
		.gpio			= 46,
	},
	{
		.name			= "openpad::glow6",
		.gpio			= 48,
	},
        {
		.name			= "openpad::glow7",
		.gpio			= 44,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= 125,
		.gpio			= 4,
		.desc			= "POWER",
		.wakeup			= 1,
	},
	{
		.code			= 102,
		.gpio			= 152,
		.desc			= "HOME",
		.wakeup			= 1,
		.active_low 	= 1,
	},
	{
		.code			= 109,
		.gpio			= 153,
		.desc			= "VOL_DOWN",
		.wakeup			= 1,
		.active_low 	= 1,
	},
	{
		.code			= 104,
		.gpio			= 154,
		.desc			= "VOL_UP",
		.wakeup			= 1,
		.active_low 	= 1,
	},
	{
		.code			= 1,
		.gpio			= 155,
		.desc			= "BACK",
		.wakeup			= 1,
		.active_low 	= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static void __init omap3_beagle_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
	omap_init_irq();
	gpmc_init();
#ifdef CONFIG_OMAP_32K_TIMER
	if (omap3_beagle_version == OMAP3BEAGLE_BOARD_AXBX)
		omap2_gp_clockevent_set_gptimer(12);
	else
		omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct platform_device *omap3_beagle_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&beagle_dss_device,
        &pwm_device,
};

static void __init omap3beagle_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		board_nand_init(omap3beagle_nand_partitions,
			ARRAY_SIZE(omap3beagle_nand_partitions),
			nandcs, NAND_BUSWIDTH_16);
	}
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {

	/* LCD PWM */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), //OpenPad
	OMAP3_MUX(UART2_CTS, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT), //BeagleTouch

	/* Touchscreen */
	OMAP3_MUX(ETK_D1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), //OpenPad Touch INT, GPIO 15
	OMAP3_MUX(MCBSP1_DX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), //OpenPad Touch RESET

#if 0
	/* Liquidware BeagleTouch */
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP), //BeagleTouch Touch INT

	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),       //GPIO 139
	OMAP3_MUX(UART2_TX,    OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), //GPIO 146
	OMAP3_MUX(UART2_CTS,   OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),       //GPIO 144
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),       //GPIO 138
	OMAP3_MUX(SDMMC2_DAT5, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),       //GPIO 137
	OMAP3_MUX(SDMMC1_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),       //GPIO 129
	OMAP3_MUX(MCBSP3_FSX,  OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),       //GPIO 143
#endif

	/* Liquidware OpenPad Glow LEDs */
	OMAP3_MUX(GPMC_A1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D15, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D14, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D13, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D12, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D11, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D10, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D9, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_D8, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* Liquidware OpenPad Keys */
	OMAP3_MUX(SYS_BOOT2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),			//GPIO 4
	OMAP3_MUX(MCBSP4_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),			//GPIO 152
	OMAP3_MUX(MCBSP4_DR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),			//GPIO 153
	OMAP3_MUX(MCBSP4_DX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),			//GPIO 154
	OMAP3_MUX(MCBSP4_FSX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),			//GPIO 155

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void __init omap3_beagle_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_init_rev();
	omap3_beagle_i2c_init();
	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_serial_init();

	omap_mux_init_gpio(170, OMAP_PIN_INPUT);
	gpio_request(170, "DVI_nPD");
	/* REVISIT leave DVI powered down until it's needed ... */
	gpio_direction_output(170, true);

	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);
	omap3beagle_flash_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	beagle_display_init();
#ifdef CONFIG_USB_ANDROID
	omap3evm_android_gadget_init();
#endif
}

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap_timer,
MACHINE_END
