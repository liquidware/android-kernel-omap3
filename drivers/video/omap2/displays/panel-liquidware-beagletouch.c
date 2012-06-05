/*
 * Liquidware beagletouch
 *
 * Copyright (C) 2011 Liquidware
 * Author: Chris Ladden <chris.ladden@liquidware.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <plat/display.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

/* Pins for the BeagleTouch OLED */
#define CS_PIN          139
#define MOSI_PIN        144
#define CLK_PIN         138
//#define RESET_PIN		137 //disabled, testing OpenPad below
#define RESET_PIN       129
#define PANEL_PWR_PIN   143

static struct omap_video_timings liquidware_beagletouch_timings_openpad = {
	/* Frame Rate = 1/((vbp + vfp + 272)*(hbp + hfp + 480)/pixel_clock) */
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 32000,
	.hfp		= 41,
	.hsw		= 6,
	.hbp		= 211,
	.vfp		= 12,
	.vsw		= 6,
	.vbp		= 31,
};

static struct omap_video_timings liquidware_beagletouch_timings = {
	/* Frame Rate = 1/((vbp + vfp + 272)*(hbp + hfp + 480)/pixel_clock) */
	.x_res		= 480,
	.y_res		= 272,
	.pixel_clock	= 12600,
	.hfp		= 2,
	.hsw		= 41,
	.hbp		= 2,
	.vfp		= 2,
	.vsw		= 10,
	.vbp		= 2,
};

static struct pwm_device	*pwm = 0;

static void liquidware_beagletouch_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	unsigned int max 		= cdev->max_brightness;
	unsigned int period_ns 	= 30000;
	unsigned int duty_ns 	= value * period_ns / max;

	printk(KERN_INFO "\nLCD Brightness duty_ns=%d, period_ns=%d\n", duty_ns, period_ns);

	pwm_config(pwm, duty_ns, period_ns);
	pwm_enable(pwm);
}

static struct led_classdev liquidware_beagletouch_led_ops = {
    .name = "lcd-backlight",
	.brightness_set = liquidware_beagletouch_led_set,
    .brightness = 159,
    .max_brightness = 255,
};

static int liquidware_beagletouch_power_on(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	/* Wait some time for the screen to settle */
	mdelay(100);

	/* Turn on the backlight */
	led_classdev_resume(&liquidware_beagletouch_led_ops);

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void liquidware_beagletouch_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	/* Turn off the backlight */
	led_classdev_suspend(&liquidware_beagletouch_led_ops);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
}

static int liquidware_beagletouch_probe(struct omap_dss_device *dssdev)
{
	int r = 0;

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = liquidware_beagletouch_timings_openpad;

	/* PWM */
	pwm = pwm_request(0, "lcd-backlight");
	if (IS_ERR(pwm)) {
		dev_err(&dssdev->dev, "unable to request PWM %d\n",
				0);
	}

	/* LED interface */
	r = led_classdev_register(&dssdev->dev,
							  &liquidware_beagletouch_led_ops);

	return r;
}

static void liquidware_beagletouch_remove(struct omap_dss_device *dssdev)
{
}

static int liquidware_beagletouch_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = liquidware_beagletouch_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void liquidware_beagletouch_disable(struct omap_dss_device *dssdev)
{
	liquidware_beagletouch_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int liquidware_beagletouch_suspend(struct omap_dss_device *dssdev)
{
	liquidware_beagletouch_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int liquidware_beagletouch_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = liquidware_beagletouch_power_on(dssdev);
	if (r)
		return r;
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void liquidware_beagletouch_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void liquidware_beagletouch_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int liquidware_beagletouch_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver beagletouch_lcd_driver = {
	.probe		= liquidware_beagletouch_probe,
	.remove		= liquidware_beagletouch_remove,

	.enable		= liquidware_beagletouch_enable,
	.disable	= liquidware_beagletouch_disable,
	.suspend	= liquidware_beagletouch_suspend,
	.resume		= liquidware_beagletouch_resume,

	.set_timings	= liquidware_beagletouch_set_timings,
	.get_timings	= liquidware_beagletouch_get_timings,
	.check_timings	= liquidware_beagletouch_check_timings,

	.driver         = {
		.name   = "liquidware_beagletouch",
		.owner  = THIS_MODULE,
	},
};

static int __init liquidware_beagletouch_drv_init(void)
{
	int r = 0;

	printk(KERN_INFO "beagletouch: init panel\n");

	r = gpio_request(RESET_PIN, "beagletouch_reset");
	if (r < 0) {
		printk(KERN_INFO "beagletouch: Error: unable to request reset gpio\n");
	} else {
		printk(KERN_INFO "beagletouch: Got reset gpio\n");
	}

	r = gpio_request(CS_PIN, "beagletouch_CS");
	if (r < 0) {
		printk(KERN_INFO "beagletouch: Error: unable to request reset gpio\n");
	} else {
		printk(KERN_INFO "beagletouch: Got cs gpio\n");
	}

	/* Set the enable pin, for testing */
	gpio_direction_output(RESET_PIN, 0);
	gpio_direction_output(RESET_PIN, 1);
	gpio_direction_output(RESET_PIN, 0);
	gpio_direction_output(RESET_PIN, 1);
	gpio_direction_output(CS_PIN, 1);

	printk(KERN_INFO "beagletouch: done\n");
	return omap_dss_register_driver(&beagletouch_lcd_driver);
}

static void __exit liquidware_beagletouch_drv_exit(void)
{
	omap_dss_unregister_driver(&beagletouch_lcd_driver);
}

module_init(liquidware_beagletouch_drv_init);
module_exit(liquidware_beagletouch_drv_exit);
MODULE_LICENSE("GPL");
