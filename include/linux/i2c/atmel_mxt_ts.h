/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

#define MXT1188_I2C_SLAVEADDRESS  (0x4a)
#define MXT768_I2C_SLAVEADDRESS  (0x4c)

#define MXT_DRIVER_NAME "atmel_mxt_ts"
#define MXT_DEVICE_224_NAME "atmel_mxt_224"
#define MXT_DEVICE_768_NAME "atmel_mxt_768"
#define MXT_DEVICE_1188_NAME "atmel_mxt_1188"

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const u8 *config;
	size_t config_length;

	unsigned int x_line;
	unsigned int y_line;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int blen;
	unsigned int threshold;
	unsigned char orient;
	unsigned long irqflags;
	unsigned long config_crc;
	unsigned int reset_gpio;
	u8(*read_chg) (void);
	int (*request_resources)(struct device *dev);
	int (*release_resources)(struct device *dev);
	int (*power_on)(struct device *dev);
	int (*power_off)(struct device *dev);
        unsigned int reset_on_resume;
        unsigned int use_fw_gestures;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
