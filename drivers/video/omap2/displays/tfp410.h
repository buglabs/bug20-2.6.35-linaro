//#ifndef __LINUX_TFP
//#define __LINUX_TFP

/*
 * tfp410_enable - pulls tfp410 out of reset
 */

int tfp410_enable(struct i2c_client *client);

/*
 * tfp410_disable - places tfp410 into reset
 */

int tfp410_disable(struct i2c_client *client);

/*
 * tfp410_init - enables tfp410, sets i2c registers; configs tfp to default resolution
 */

int tfp410_init(struct i2c_client *client);


//#endif /* __LINUX_TFP */
