/*
 * OLED panel driver for CMEL OLED43
 *
 * Author: Christopher Ladden <christopher.ladden@gmail.com>
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
#include <linux/leds.h>
#include <linux/fb.h>

#include <plat/display.h>

#define CS_PIN          139
#define MOSI_PIN        144
#define CLK_PIN         138
#define RESET_PIN       137
#define PANEL_PWR_PIN   143

#define digitalWrite(PIN,VALUE) (gpio_direction_output(PIN,VALUE))

#define CS_LOW digitalWrite(CS_PIN, 0)
#define CS_HIGH digitalWrite(CS_PIN, 1)
#define MOSI_LOW digitalWrite(MOSI_PIN, 0)
#define MOSI_HIGH digitalWrite(MOSI_PIN, 1)
#define CLK_LOW digitalWrite(CLK_PIN, 0)
#define CLK_HIGH digitalWrite(CLK_PIN, 1)
#define RESET_LOW digitalWrite(RESET_PIN, 0)
#define RESET_HIGH digitalWrite(RESET_PIN, 1)
#define PANEL_PWR_LOW digitalWrite(PANEL_PWR_PIN, 0)
#define PANEL_PWR_HIGH digitalWrite(PANEL_PWR_PIN, 1)

/*
 *   Video Timings
 *
 *   Frame Rate = 1/((vbp + vfp + 272)*(hbp + hfp + 480)/pixel_clock)
 *   Where hfp must be > 30,
 *         hbp must be = 102,
 *         vfp must be > 10,
 *         vbp must be = 20,
 */
static struct omap_video_timings oled43_timings = {
	.x_res = 480,
	.y_res = 272,

	.pixel_clock	= 12342,

	.hsw		= 30,
	.hfp		= 31,
	.hbp		= 102,

	.vsw		= 3,
	.vfp		= 11,
	.vbp		= 20,
};

/* Initialize the software SPI interface */
static void oled43_spi_init(void) {
    CS_HIGH;
    MOSI_HIGH;
    CLK_HIGH;
}

/* Write an 8-bit byte onto the SPI bus */
static void oled43_spi_write8(uint8_t data) {
uint8_t ii;
uint8_t bit;

    for (ii=0; ii < 8; ii++) {
        bit = (data >> (7-ii)) & 1;
        if (bit) {
            MOSI_HIGH;
        } else {
            MOSI_LOW;
        }

        CLK_LOW;		//clock the data in
        CLK_HIGH;
    }
}

/* Write a value to the OLED panel */
static void oled43_writeReg(uint8_t index, uint8_t val) {
    CS_LOW;
    oled43_spi_write8((index << 1));
    oled43_spi_write8(val);
    CS_HIGH;
}

static int oled43_hardware_init(void) {
uint8_t brightness;
double percent;

	/* Panel init sequence from the panel datasheet */
	PANEL_PWR_LOW;						// just to be sure, hold the oled power supply off
	RESET_LOW;							// panel in reset
	oled43_spi_init();					// init spi interface
	RESET_HIGH;							// panel out of reset

    oled43_writeReg(0x04, 0x23); //DISPLAY_MODE2
    oled43_writeReg(0x05, 0x82); //DISPLAY_MODE3

    oled43_writeReg(0x07, 0x0F); //DRIVER_CAPABILITY
    oled43_writeReg(0x34, 0x18);
    oled43_writeReg(0x35, 0x28);
    oled43_writeReg(0x36, 0x16);
    oled43_writeReg(0x37, 0x01);

    oled43_writeReg(0x03, 35);   //VGAM1_LEVEL

    /* Set the brightness
     *  0x20 - 200 nits
     *  0x1E - 175 nits
     *  0x1C - 150 nits
     *  0x17 - 100 nits
     *  0x14 -  70 nits
     *  0x11 -  50 nits */
    brightness = 0x14;
    percent = ((double)brightness)/
               (0x20) * 100.0;

    printk(KERN_INFO "cmel_oled43_panel: Setting brightness to %d percent\n", (int)percent);

    oled43_writeReg(0x3A, brightness);

    /* Display ON */
    oled43_writeReg(0x06, 0x03); //POWER_CTRL1

    printk(KERN_INFO "cmel_oled43_panel: HIGH\n");
    PANEL_PWR_HIGH;

    return 0;
}

static void oled43_panel_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	int level;

    level = value / 8; //scaled for the controller
    if (value <= 20) {
        /* brightness threshold reached, turning off */
        printk(KERN_INFO "cmel_oled43_panel: brightness set at or below threshold, turning OFF\n");
        level = 0;
    }
    oled43_writeReg(0x3A, level);
}

static struct led_classdev cmel_oled43_panel_led_ops = {
    .name = "lcd-backlight",
	.brightness_set = oled43_panel_led_set,
    .brightness = 159,
    .max_brightness = 255,
};

static int oled43_panel_power_on(struct omap_dss_device *dssdev)
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

	oled43_hardware_init();

        return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void oled43_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

        /* Turn off the power supply */
	PANEL_PWR_LOW;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
}

static int oled43_panel_probe(struct omap_dss_device *dssdev)
{
    struct led_classdev	*cdev;

	dssdev->panel.config |= OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = oled43_timings;

	cdev = led_classdev_register((struct device *)&dssdev->dev,
			&cmel_oled43_panel_led_ops);

	return 0;
}

static void oled43_panel_remove(struct omap_dss_device *dssdev)
{
}

static int oled43_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	pr_info("cmel_oled43_panel: panel_enable begin\n");
	r = oled43_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	pr_info("cmel_oled43_panel: panel_enable end\n");

	return 0;
}

static void oled43_panel_disable(struct omap_dss_device *dssdev)
{
	oled43_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int oled43_panel_suspend(struct omap_dss_device *dssdev)
{
	oled43_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	pr_info("cmel_oled43_panel: panel_suspend\n");
	return 0;
}

static int oled43_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;
	pr_info("cmel_oled43_panel: panel_resume\n");
	r = oled43_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void oled43_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void oled43_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int oled43_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver oled43_driver = {
	.probe		= oled43_panel_probe,
	.remove		= oled43_panel_remove,

	.enable		= oled43_panel_enable,
	.disable	= oled43_panel_disable,
	.suspend	= oled43_panel_suspend,
	.resume		= oled43_panel_resume,

    .set_timings	= oled43_panel_set_timings,
	.get_timings	= oled43_panel_get_timings,
	.check_timings	= oled43_panel_check_timings,

	.driver         = {
		.name   = "cmel_oled43_panel",
		.owner  = THIS_MODULE,
	},
};

static void print_pins(void) {
    printk(KERN_INFO "cmel_oled43_panel: "
           "PANEL_PWR_PIN=%d,"
           "RESET_PIN=%d,"
           "CLK_PIN=%d,"
           "MOSI_PIN=%d,"
           "CS_PIN=%d\n",
           gpio_get_value(PANEL_PWR_PIN),
           gpio_get_value(RESET_PIN),
           gpio_get_value(CLK_PIN),
           gpio_get_value(MOSI_PIN),
           gpio_get_value(CS_PIN));
}

static void test_pins(void) {
    int i;
    int pins[5] = {
        PANEL_PWR_PIN,
        RESET_PIN,
        CLK_PIN,
        MOSI_PIN,
        CS_PIN,
    };

    for (i=0; i<5;i++) {
        printk(KERN_INFO "cmel_oled43_panel: pin=%d\n", pins[i]);
        digitalWrite(pins[i], 1);
        mdelay(500);
        print_pins();
        digitalWrite(pins[i], 0);
        mdelay(500);
        print_pins();
        digitalWrite(pins[i], 1);
        mdelay(500);
        print_pins();
    }
}

static int __init oled43_panel_drv_init(void)
{
	int ret = 0;
    int res = 0;


	printk(KERN_INFO "cmel_oled43_panel: init panel\n");

	/* Request GPIO pins used for the panel */
	printk(KERN_INFO "cmel_oled43_panel: requesting GPIOs\n");
	res |= gpio_request(CS_PIN, "OLED43_CS_PIN");
	res |= gpio_request(MOSI_PIN, "OLED43_MOSI_PIN");
	res |= gpio_request(CLK_PIN, "OLED43_CLK_PIN");
	res |= gpio_request(RESET_PIN, "OLED43_RESET_PIN");
	res |= gpio_request(PANEL_PWR_PIN, "OLED43_PANEL_PWR_PIN");
    	if (!res)
            printk(KERN_INFO "cmel_oled43_panel: got all GPIOs\n");
   	else
       	    printk(KERN_INFO "cmel_oled43_panel: error requesting GPIO\n");

	/* Hold the power supply off until enabled */
	PANEL_PWR_LOW;

//    test_pins();

	ret = omap_dss_register_driver(&oled43_driver);
	if (ret != 0)
		pr_err("cmel_oled43: Unable to register panel driver: %d\n", ret);

	printk(KERN_INFO "cmel_oled43_panel: done\n");
	return ret;
}

static void __exit oled43_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&oled43_driver);
}

module_init(oled43_panel_drv_init);
module_exit(oled43_panel_drv_exit);
MODULE_LICENSE("GPL");
