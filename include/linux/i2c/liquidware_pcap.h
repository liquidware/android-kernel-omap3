#ifndef __LINUX_I2C_LIQUIDWARE_PCAP_H
#define __LINUX_I2C_LIQUIDWARE_PCAP_H

/* linux/i2c/liquidware_pcap.h */

struct liquidware_pcap_platform_data {
	u16	model;				/* 2007. */
	u16	x_plate_ohms;
    u16 pen_gpio;
    u16 reset_gpio;

	int	(*get_pendown_state)(void);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   interrupt source */
    void	(*perform_reset)(void);
	int	(*init_platform_hw)(struct liquidware_pcap_platform_data *);
	void	(*exit_platform_hw)(void);
};

#endif
