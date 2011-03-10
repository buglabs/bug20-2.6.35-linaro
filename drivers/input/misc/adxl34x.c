/*
 * ADXL345/346 Three-Axis Digital Accelerometers
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright (C) 2009 Michael Hennerich, Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include <linux/input/adxl34x.h>

#include "adxl34x.h"

/* ADXL345/6 Register Map */
#define DEVID		0x00	/* R   Device ID */
#define THRESH_TAP	0x1D	/* R/W Tap threshold */
#define OFSX		0x1E	/* R/W X-axis offset */
#define OFSY		0x1F	/* R/W Y-axis offset */
#define OFSZ		0x20	/* R/W Z-axis offset */
#define DUR		0x21	/* R/W Tap duration */
#define LATENT		0x22	/* R/W Tap latency */
#define WINDOW		0x23	/* R/W Tap window */
#define THRESH_ACT	0x24	/* R/W Activity threshold */
#define THRESH_INACT	0x25	/* R/W Inactivity threshold */
#define TIME_INACT	0x26	/* R/W Inactivity time */
#define ACT_INACT_CTL	0x27	/* R/W Axis enable control for activity and */
				/* inactivity detection */
#define THRESH_FF	0x28	/* R/W Free-fall threshold */
#define TIME_FF		0x29	/* R/W Free-fall time */
#define TAP_AXES	0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE		0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL	0x2D	/* R/W Power saving features control */
#define INT_ENABLE	0x2E	/* R/W Interrupt enable control */
#define INT_MAP		0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE	0x30	/* R   Source of interrupts */
#define DATA_FORMAT	0x31	/* R/W Data format control */
#define DATAX0		0x32	/* R   X-Axis Data 0 */
#define DATAX1		0x33	/* R   X-Axis Data 1 */
#define DATAY0		0x34	/* R   Y-Axis Data 0 */
#define DATAY1		0x35	/* R   Y-Axis Data 1 */
#define DATAZ0		0x36	/* R   Z-Axis Data 0 */
#define DATAZ1		0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL	0x38	/* R/W FIFO control */
#define FIFO_STATUS	0x39	/* R   FIFO status */
#define TAP_SIGN	0x3A	/* R   Sign and source for tap/double tap */
/* Orientation ADXL346 only */
#define ORIENT_CONF	0x3B	/* R/W Orientation configuration */
#define ORIENT		0x3C	/* R   Orientation status */

/* DEVIDs */
#define ID_ADXL345	0xE5
#define ID_ADXL346	0xE6

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define DATA_READY	(1 << 7)
#define SINGLE_TAP	(1 << 6)
#define DOUBLE_TAP	(1 << 5)
#define ACTIVITY	(1 << 4)
#define INACTIVITY	(1 << 3)
#define FREE_FALL	(1 << 2)
#define WATERMARK	(1 << 1)
#define OVERRUN		(1 << 0)

/* ACT_INACT_CONTROL Bits */
#define ACT_ACDC	(1 << 7)
#define ACT_X_EN	(1 << 6)
#define ACT_Y_EN	(1 << 5)
#define ACT_Z_EN	(1 << 4)
#define INACT_ACDC	(1 << 3)
#define INACT_X_EN	(1 << 2)
#define INACT_Y_EN	(1 << 1)
#define INACT_Z_EN	(1 << 0)

/* TAP_AXES Bits */
#define SUPPRESS	(1 << 3)
#define TAP_X_EN	(1 << 2)
#define TAP_Y_EN	(1 << 1)
#define TAP_Z_EN	(1 << 0)

/* ACT_TAP_STATUS Bits */
#define ACT_X_SRC	(1 << 6)
#define ACT_Y_SRC	(1 << 5)
#define ACT_Z_SRC	(1 << 4)
#define ASLEEP		(1 << 3)
#define TAP_X_SRC	(1 << 2)
#define TAP_Y_SRC	(1 << 1)
#define TAP_Z_SRC	(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER	(1 << 4)
#define RATE(x)		((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_LINK	(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP	(1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST	(1 << 7)
#define SPI		(1 << 6)
#define INT_INVERT	(1 << 5)
#define FULL_RES	(1 << 3)
#define JUSTIFY		(1 << 2)
#define RANGE(x)	((x) & 0x3)
#define RANGE_PM_2g	0
#define RANGE_PM_4g	1
#define RANGE_PM_8g	2
#define RANGE_PM_16g	3

/*
 * Maximum value our axis may get in full res mode for the input device
 * (signed 13 bits)
 */
#define ADXL_FULLRES_MAX_VAL 4096

/*
 * Maximum value our axis may get in fixed res mode for the input device
 * (signed 10 bits)
 */
#define ADXL_FIXEDRES_MAX_VAL 512

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	(((x) & 0x3) << 6)
#define FIFO_BYPASS	0
#define FIFO_FIFO	1
#define FIFO_STREAM	2
#define FIFO_TRIGGER	3
#define TRIGGER		(1 << 5)
#define SAMPLES(x)	((x) & 0x1F)

/* FIFO_STATUS Bits */
#define FIFO_TRIG	(1 << 7)
#define ENTRIES(x)	((x) & 0x3F)

/* TAP_SIGN Bits ADXL346 only */
#define XSIGN		(1 << 6)
#define YSIGN		(1 << 5)
#define ZSIGN		(1 << 4)
#define XTAP		(1 << 3)
#define YTAP		(1 << 2)
#define ZTAP		(1 << 1)

/* ORIENT_CONF ADXL346 only */
#define ORIENT_DEADZONE(x)	(((x) & 0x7) << 4)
#define ORIENT_DIVISOR(x)	((x) & 0x7)

/* ORIENT ADXL346 only */
#define ADXL346_2D_VALID		(1 << 6)
#define ADXL346_2D_ORIENT(x)		(((x) & 0x3) >> 4)
#define ADXL346_3D_VALID		(1 << 3)
#define ADXL346_3D_ORIENT(x)		((x) & 0x7)
#define ADXL346_2D_PORTRAIT_POS		0	/* +X */
#define ADXL346_2D_PORTRAIT_NEG		1	/* -X */
#define ADXL346_2D_LANDSCAPE_POS	2	/* +Y */
#define ADXL346_2D_LANDSCAPE_NEG	3	/* -Y */

#define ADXL346_3D_FRONT		3	/* +X */
#define ADXL346_3D_BACK			4	/* -X */
#define ADXL346_3D_RIGHT		2	/* +Y */
#define ADXL346_3D_LEFT			5	/* -Y */
#define ADXL346_3D_TOP			1	/* +Z */
#define ADXL346_3D_BOTTOM		6	/* -Z */

#undef ADXL_DEBUG

#define AC_READ(ac, reg)	((ac)->read((ac)->dev, reg))
#define AC_WRITE(ac, reg, val)	((ac)->write((ac)->dev, reg, val))

struct axis_triple {
	int x;
	int y;
	int z;
};

struct adxl34x {
	struct device *dev;
	int irq;
	struct input_dev *input;
	struct work_struct work;
	struct mutex mutex;	/* reentrant protection for struct */
	struct adxl34x_platform_data pdata;
	struct axis_triple swcal;
	struct axis_triple hwcal;
	struct axis_triple saved;
	char phys[32];
	unsigned disabled:1;	/* P: mutex */
	unsigned opened:1;	/* P: mutex */
	unsigned fifo_delay:1;
	unsigned model;
	unsigned int_mask;

	adxl34x_read_t *read;
	adxl34x_read_block_t *read_block;
	adxl34x_write_t *write;
};

static const struct adxl34x_platform_data adxl34x_default_init = {
	.tap_threshold = 35,
	.tap_duration = 3,
	.tap_latency = 20,
	.tap_window = 20,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 6,
	.inactivity_threshold = 4,
	.inactivity_time = 3,
	.free_fall_threshold = 8,
	.free_fall_time = 0x20,
	.data_rate = 8,
	.data_range = ADXL_FULL_RES,

	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,	/* EV_REL */
	.ev_code_y = ABS_Y,	/* EV_REL */
	.ev_code_z = ABS_Z,	/* EV_REL */

	.ev_code_tap_x = BTN_TOUCH,	/* EV_KEY */
	.ev_code_tap_y = BTN_TOUCH,	/* EV_KEY */
	.ev_code_tap_z = BTN_TOUCH,	/* EV_KEY */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = FIFO_STREAM,
	.watermark = 0,
};

static void adxl34x_get_triple(struct adxl34x *ac, struct axis_triple *axis)
{
	short buf[3];

	ac->read_block(ac->dev, DATAX0, DATAZ1 - DATAX0 + 1,
		       (unsigned char *)buf);

	mutex_lock(&ac->mutex);
	ac->saved.x = (s16) le16_to_cpu(buf[0]);
	axis->x = ac->saved.x;

	ac->saved.y = (s16) le16_to_cpu(buf[1]);
	axis->y = ac->saved.y;

	ac->saved.z = (s16) le16_to_cpu(buf[2]);
	axis->z = ac->saved.z;
	mutex_unlock(&ac->mutex);
}

static void adxl34x_service_ev_fifo(struct adxl34x *ac)
{
	struct adxl34x_platform_data *pdata = &ac->pdata;
	struct axis_triple axis;

	adxl34x_get_triple(ac, &axis);

	input_event(ac->input, pdata->ev_type, pdata->ev_code_x,
		    axis.x - ac->swcal.x);
	input_event(ac->input, pdata->ev_type, pdata->ev_code_y,
		    axis.y - ac->swcal.y);
	input_event(ac->input, pdata->ev_type, pdata->ev_code_z,
		    axis.z - ac->swcal.z);
}

static void adxl34x_report_key_single(struct input_dev *input, int key)
{
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
}

static void adxl34x_report_key_double(struct input_dev *input, int key)
{
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
	input_sync(input);
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
}

static void adxl34x_work(struct work_struct *work)
{
	struct adxl34x *ac = container_of(work, struct adxl34x, work);
	struct adxl34x_platform_data *pdata = &ac->pdata;
	int int_stat, tap_stat, samples;

	/*
	 * ACT_TAP_STATUS should be read before clearing the interrupt
	 * Avoid reading ACT_TAP_STATUS in case TAP detection is disabled
	 */

	if (pdata->tap_axis_control & (TAP_X_EN | TAP_Y_EN | TAP_Z_EN))
		tap_stat = AC_READ(ac, ACT_TAP_STATUS);
	else
		tap_stat = 0;

	int_stat = AC_READ(ac, INT_SOURCE);

	if (int_stat & FREE_FALL)
		adxl34x_report_key_single(ac->input, pdata->ev_code_ff);

	if (int_stat & OVERRUN)
		dev_dbg(ac->dev, "OVERRUN\n");

	if (int_stat & SINGLE_TAP) {
		if (tap_stat & TAP_X_SRC)
			adxl34x_report_key_single(ac->input,
						  pdata->ev_code_tap_x);
		if (tap_stat & TAP_Y_SRC)
			adxl34x_report_key_single(ac->input,
						  pdata->ev_code_tap_y);
		if (tap_stat & TAP_Z_SRC)
			adxl34x_report_key_single(ac->input,
						  pdata->ev_code_tap_z);
	}

	if (int_stat & DOUBLE_TAP) {
		if (tap_stat & TAP_X_SRC)
			adxl34x_report_key_double(ac->input,
						  pdata->ev_code_tap_x);
		if (tap_stat & TAP_Y_SRC)
			adxl34x_report_key_double(ac->input,
						  pdata->ev_code_tap_y);
		if (tap_stat & TAP_Z_SRC)
			adxl34x_report_key_double(ac->input,
						  pdata->ev_code_tap_z);
	}

	if (pdata->ev_code_act_inactivity) {
		if (int_stat & ACTIVITY)
			input_report_key(ac->input,
					 pdata->ev_code_act_inactivity, 1);
		if (int_stat & INACTIVITY)
			input_report_key(ac->input,
					 pdata->ev_code_act_inactivity, 0);
	}

	if (int_stat & (DATA_READY | WATERMARK)) {

		if (pdata->fifo_mode)
			samples = ENTRIES(AC_READ(ac, FIFO_STATUS)) + 1;
		else
			samples = 1;

		for (; samples > 0; samples--) {
			adxl34x_service_ev_fifo(ac);
			/*
			 * To ensure that the FIFO has
			 * completely popped, there must be at least 5 us between
			 * the end of reading the data registers, signified by the
			 * transition to register 0x38 from 0x37 or the CS pin
			 * going high, and the start of new reads of the FIFO or
			 * reading the FIFO_STATUS register. For SPI operation at
			 * 1.5 MHz or lower, the register addressing portion of the
			 * transmission is sufficient delay to ensure the FIFO has
			 * completely popped. It is necessary for SPI operation
			 * greater than 1.5 MHz to de-assert the CS pin to ensure a
			 * total of 5 us, which is at most 3.4 us at 5 MHz
			 * operation.
			 */
			if (ac->fifo_delay && (samples > 1))
				udelay(3);
		}
	}

	input_sync(ac->input);
	enable_irq(ac->irq);
}

static irqreturn_t adxl34x_irq(int irq, void *handle)
{
	struct adxl34x *ac = handle;

	disable_irq_nosync(irq);
	schedule_work(&ac->work);

	return IRQ_HANDLED;
}

void adxl34x_disable(struct adxl34x *ac)
{
	mutex_lock(&ac->mutex);
	if (!ac->disabled && ac->opened) {
		ac->disabled = 1;
		cancel_work_sync(&ac->work);
		/*
		 * A '0' places the ADXL34x into standby mode
		 * with minimum power consumption.
		 */
		AC_WRITE(ac, POWER_CTL, 0);
	}
	mutex_unlock(&ac->mutex);
}
EXPORT_SYMBOL_GPL(adxl34x_disable);

void adxl34x_enable(struct adxl34x *ac)
{
	mutex_lock(&ac->mutex);
	if (ac->disabled && ac->opened) {
		AC_WRITE(ac, POWER_CTL, ac->pdata.power_mode | PCTL_MEASURE);
		ac->disabled = 0;
	}
	mutex_unlock(&ac->mutex);
}
EXPORT_SYMBOL_GPL(adxl34x_enable);

static ssize_t adxl34x_disable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ac->disabled);
}

static ssize_t adxl34x_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		adxl34x_disable(ac);
	else
		adxl34x_enable(ac);

	return count;
}

static DEVICE_ATTR(disable, 0664, adxl34x_disable_show, adxl34x_disable_store);

static ssize_t adxl34x_calibrate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);
	count = sprintf(buf, "%d,%d,%d\n", ac->hwcal.x * 4 + ac->swcal.x,
			ac->hwcal.y * 4 + ac->swcal.y,
			ac->hwcal.z * 4 + ac->swcal.z);
	mutex_unlock(&ac->mutex);

	return count;
}

static ssize_t adxl34x_calibrate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	/*
	 * Hardware offset calibration has a resolution of 15.6 mg/LSB.
	 * We use HW calibration and handle the remaining bits in SW. (4mg/LSB)
	 */

	mutex_lock(&ac->mutex);
	ac->hwcal.x -= (ac->saved.x / 4);
	ac->swcal.x = ac->saved.x % 4;

	ac->hwcal.y -= (ac->saved.y / 4);
	ac->swcal.y = ac->saved.y % 4;

	ac->hwcal.z -= (ac->saved.z / 4);
	ac->swcal.z = ac->saved.z % 4;

	AC_WRITE(ac, OFSX, (s8) ac->hwcal.x);
	AC_WRITE(ac, OFSY, (s8) ac->hwcal.y);
	AC_WRITE(ac, OFSZ, (s8) ac->hwcal.z);
	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(calibrate, 0664, adxl34x_calibrate_show,
		   adxl34x_calibrate_store);

static ssize_t adxl34x_rate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);
	count = sprintf(buf, "%u\n", RATE(ac->pdata.data_rate));
	mutex_unlock(&ac->mutex);

	return count;
}

static ssize_t adxl34x_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	mutex_lock(&ac->mutex);
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	ac->pdata.data_rate = RATE(val);

	AC_WRITE(ac, BW_RATE, ac->pdata.data_rate |
		 (ac->pdata.low_power_mode ? LOW_POWER : 0));
	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(rate, 0664, adxl34x_rate_show, adxl34x_rate_store);

static ssize_t adxl34x_autosleep_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);
	count = sprintf(buf, "%u\n", ac->pdata.power_mode &
		(PCTL_AUTO_SLEEP | PCTL_LINK) ? 1 : 0);
	mutex_unlock(&ac->mutex);

	return count;
}

static ssize_t adxl34x_autosleep_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	mutex_lock(&ac->mutex);
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		ac->pdata.power_mode |= (PCTL_AUTO_SLEEP | PCTL_LINK);
	else
		ac->pdata.power_mode &= ~(PCTL_AUTO_SLEEP | PCTL_LINK);

	if (!ac->disabled && ac->opened)
		AC_WRITE(ac, POWER_CTL, ac->pdata.power_mode | PCTL_MEASURE);

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(autosleep, 0664, adxl34x_autosleep_show,
		   adxl34x_autosleep_store);

static ssize_t adxl34x_position_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);

	count = sprintf(buf, "(%d, %d, %d)\n",
			ac->saved.x, ac->saved.y, ac->saved.z);

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(position, 0444, adxl34x_position_show, NULL);

#ifdef ADXL_DEBUG
static ssize_t adxl34x_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	/*
	 * This allows basic ADXL register write access for debug purposes.
	 */
	mutex_lock(&ac->mutex);
	error = strict_strtoul(buf, 16, &val);
	if (error)
		return error;

	AC_WRITE(ac, val >> 8, val & 0xFF);
	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(write, 0664, NULL, adxl34x_write_store);
#endif

static struct attribute *adxl34x_attributes[] = {
	&dev_attr_disable.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_rate.attr,
	&dev_attr_autosleep.attr,
	&dev_attr_position.attr,
#ifdef ADXL_DEBUG
	&dev_attr_write.attr,
#endif
	NULL
};

static const struct attribute_group adxl34x_attr_group = {
	.attrs = adxl34x_attributes,
};

static int adxl34x_input_open(struct input_dev *input)
{
	struct adxl34x *ac = input_get_drvdata(input);

	mutex_lock(&ac->mutex);
	ac->opened = 1;
	mutex_unlock(&ac->mutex);

	adxl34x_enable(ac);

	return 0;
}

static void adxl34x_input_close(struct input_dev *input)
{
	struct adxl34x *ac = input_get_drvdata(input);

	adxl34x_disable(ac);

	mutex_lock(&ac->mutex);
	ac->opened = 0;
	mutex_unlock(&ac->mutex);
}

int adxl34x_probe(struct adxl34x **pac, struct device *dev, u16 bus_type,
	int irq, int fifo_delay_default, adxl34x_read_t read,
	adxl34x_read_block_t read_block, adxl34x_write_t write)
{
	struct adxl34x *ac;
	struct input_dev *input_dev;
	struct adxl34x_platform_data *pdata;
	int err, range;
	unsigned char revid;

	if (!irq) {
		dev_err(dev, "no IRQ?\n");
		return -ENODEV;
	}

	*pac = ac = kzalloc(sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return -ENOMEM;
	ac->fifo_delay = fifo_delay_default;

	pdata = dev->platform_data;
	if (!pdata) {
		dev_dbg(dev,
			"No platfrom data: Using default initialization\n");
		pdata = (struct adxl34x_platform_data *)&adxl34x_default_init;
	}
	memcpy(&ac->pdata, pdata, sizeof(*pdata));
	pdata = &ac->pdata;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	ac->input = input_dev;
	ac->disabled = 1;
	ac->dev = dev;
	ac->irq = irq;
	ac->write = write;
	ac->read = read;
	ac->read_block = read_block;

	INIT_WORK(&ac->work, adxl34x_work);
	mutex_init(&ac->mutex);

	input_dev->name = "ADXL34x accelerometer";
	revid = ac->read(dev, DEVID);

	switch (revid) {
	case ID_ADXL345:
		ac->model = 345;
		break;
	case ID_ADXL346:
		ac->model = 346;
		break;
	default:
		dev_err(dev, "Failed to probe %s\n", input_dev->name);
		err = -ENODEV;
		goto err_free_mem;
	}

	snprintf(ac->phys, sizeof(ac->phys), "%s/input0", dev_name(dev));

	input_dev->phys = ac->phys;
	input_dev->dev.parent = dev;
	input_dev->id.product = ac->model;
	input_dev->id.bustype = bus_type;
	input_dev->open = adxl34x_input_open;
	input_dev->close = adxl34x_input_close;

	input_set_drvdata(input_dev, ac);

	__set_bit(ac->pdata.ev_type, input_dev->evbit);

	if (ac->pdata.ev_type == EV_REL) {
		__set_bit(REL_X, input_dev->relbit);
		__set_bit(REL_Y, input_dev->relbit);
		__set_bit(REL_Z, input_dev->relbit);
	} else {
		/* EV_ABS */
		__set_bit(ABS_X, input_dev->absbit);
		__set_bit(ABS_Y, input_dev->absbit);
		__set_bit(ABS_Z, input_dev->absbit);

		if (pdata->data_range & FULL_RES)
			range = ADXL_FULLRES_MAX_VAL;	/* Signed 13-bit */
		else
			range = ADXL_FIXEDRES_MAX_VAL;	/* Signed 10-bit */

		input_set_abs_params(input_dev, ABS_X, -range, range, 3, 3);
		input_set_abs_params(input_dev, ABS_Y, -range, range, 3, 3);
		input_set_abs_params(input_dev, ABS_Z, -range, range, 3, 3);
	}

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(pdata->ev_code_tap_x, input_dev->keybit);
	__set_bit(pdata->ev_code_tap_y, input_dev->keybit);
	__set_bit(pdata->ev_code_tap_z, input_dev->keybit);

	if (pdata->ev_code_ff) {
		ac->int_mask = FREE_FALL;
		__set_bit(pdata->ev_code_ff, input_dev->keybit);
	}

	if (pdata->ev_code_act_inactivity)
		__set_bit(pdata->ev_code_act_inactivity, input_dev->keybit);

	ac->int_mask |= ACTIVITY | INACTIVITY;

	if (pdata->watermark) {
		ac->int_mask |= WATERMARK;
		if (!FIFO_MODE(pdata->fifo_mode))
			pdata->fifo_mode |= FIFO_STREAM;
	} else {
		ac->int_mask |= DATA_READY;
	}

	if (pdata->tap_axis_control & (TAP_X_EN | TAP_Y_EN | TAP_Z_EN))
		ac->int_mask |= SINGLE_TAP | DOUBLE_TAP;

	if (FIFO_MODE(pdata->fifo_mode) == FIFO_BYPASS)
		ac->fifo_delay = 0;

	ac->write(dev, POWER_CTL, 0);

	err = request_irq(ac->irq, adxl34x_irq,
			  IRQF_TRIGGER_HIGH, dev_name(dev), ac);
	if (err) {
		dev_err(dev, "irq %d busy?\n", ac->irq);
		goto err_free_mem;
	}

	err = sysfs_create_group(&dev->kobj, &adxl34x_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	AC_WRITE(ac, THRESH_TAP, pdata->tap_threshold);
	AC_WRITE(ac, OFSX, pdata->x_axis_offset);
	ac->hwcal.x = pdata->x_axis_offset;
	AC_WRITE(ac, OFSY, pdata->y_axis_offset);
	ac->hwcal.y = pdata->y_axis_offset;
	AC_WRITE(ac, OFSZ, pdata->z_axis_offset);
	ac->hwcal.z = pdata->z_axis_offset;
	AC_WRITE(ac, THRESH_TAP, pdata->tap_threshold);
	AC_WRITE(ac, DUR, pdata->tap_duration);
	AC_WRITE(ac, LATENT, pdata->tap_latency);
	AC_WRITE(ac, WINDOW, pdata->tap_window);
	AC_WRITE(ac, THRESH_ACT, pdata->activity_threshold);
	AC_WRITE(ac, THRESH_INACT, pdata->inactivity_threshold);
	AC_WRITE(ac, TIME_INACT, pdata->inactivity_time);
	AC_WRITE(ac, THRESH_FF, pdata->free_fall_threshold);
	AC_WRITE(ac, TIME_FF, pdata->free_fall_time);
	AC_WRITE(ac, TAP_AXES, pdata->tap_axis_control);
	AC_WRITE(ac, ACT_INACT_CTL, pdata->act_axis_control);
	AC_WRITE(ac, BW_RATE, RATE(ac->pdata.data_rate) |
		 (pdata->low_power_mode ? LOW_POWER : 0));
	AC_WRITE(ac, DATA_FORMAT, pdata->data_range);
	AC_WRITE(ac, FIFO_CTL, FIFO_MODE(pdata->fifo_mode) |
			SAMPLES(pdata->watermark));

	if (pdata->use_int2)
		/* Map all INTs to INT2 */
		AC_WRITE(ac, INT_MAP, ac->int_mask | OVERRUN);
	else
		/* Map all INTs to INT1 */
		AC_WRITE(ac, INT_MAP, 0);

	AC_WRITE(ac, INT_ENABLE, ac->int_mask | OVERRUN);

	pdata->power_mode &= (PCTL_AUTO_SLEEP | PCTL_LINK);

	dev_info(dev, "ADXL%d accelerometer, irq %d\n",
		 ac->model, ac->irq);

	return 0;

 err_remove_attr:
	sysfs_remove_group(&dev->kobj, &adxl34x_attr_group);
 err_free_irq:
	free_irq(ac->irq, ac);
 err_free_mem:
	input_free_device(input_dev);

	return err;
}
EXPORT_SYMBOL_GPL(adxl34x_probe);

int adxl34x_remove(struct adxl34x *ac)
{
	adxl34x_disable(ac);
	sysfs_remove_group(&ac->dev->kobj, &adxl34x_attr_group);
	free_irq(ac->irq, ac);
	input_unregister_device(ac->input);
	dev_dbg(ac->dev, "unregistered accelerometer\n");
	kfree(ac);

	return 0;
}
EXPORT_SYMBOL_GPL(adxl34x_remove);

/* Stub functions so we can load/unload the module */
static __init int ad714x_init(void)
{
	return 0;
}
module_init(ad714x_init);

static __exit void ad714x_exit(void)
{
}
module_exit(ad714x_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADXL345/346 Three-Axis Digital Accelerometer Driver");
MODULE_LICENSE("GPL");
