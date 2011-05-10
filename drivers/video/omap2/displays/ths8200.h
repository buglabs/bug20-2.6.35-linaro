//#ifndef __LINUX_TFP
//#define __LINUX_TFP

/*
 * ths8200_enable - pulls ths8200 out of reset
 */

int ths8200_enable(struct i2c_client *client);

/*
 * ths8200_disable - places ths8200 into reset
 */
int ths8200_disable(struct i2c_client *client);

/* reset the ths8200 chip via a reset line that is connected
 * to a GPIO
 */
int ths8200_reset(int gpio, struct i2c_client *client);


/* powers off the reset line to leave the chip in standby, ready
 * to get powered back up and resume working */
int ths8200_standby(int gpio);

/*
 * ths8200_init - enables ths8200, sets i2c registers; configs ths
 * to default resolution
 */
int ths8200_init(struct i2c_client *client);

/*
 * frees any resources held by ths8200
 */
void ths8200_cleanup(struct i2c_client *client);

//#endif /* __LINUX_TFP */
