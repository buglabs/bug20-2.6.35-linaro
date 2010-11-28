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

/*
 * ths8200_init - enables ths8200, sets i2c registers; configs ths to default resolution
 */

int ths8200_init(struct i2c_client *client);


//#endif /* __LINUX_TFP */
