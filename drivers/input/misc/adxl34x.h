/*
 * ADXL345/346 Three-Axis Digital Accelerometers (I2C/SPI Interface)
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright (C) 2009 Michael Hennerich, Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef _ADXL34X_H_
#define _ADXL34X_H_

struct device;
struct adxl34x;
typedef int (adxl34x_read_t) (struct device *, unsigned char);
typedef int (adxl34x_read_block_t) (struct device *, unsigned char, int, unsigned char *);
typedef int (adxl34x_write_t) (struct device *, unsigned char, unsigned char);

void adxl34x_disable(struct adxl34x *ac);
void adxl34x_enable(struct adxl34x *ac);
int adxl34x_probe(struct adxl34x **pac, struct device *dev, u16 bus_type,
	int irq, int fifo_delay_default, adxl34x_read_t read,
	adxl34x_read_block_t read_block, adxl34x_write_t write);
int adxl34x_remove(struct adxl34x *ac);

#endif
