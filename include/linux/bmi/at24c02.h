#include <linux/semaphore.h>
#include <linux/i2c.h>

/*--------------------------------
 *
 *   AT24C02 I2C Eeprom Device
 *
 *--------------------------------
 */


struct at24c02 {

	unsigned char addr;
	struct i2c_adapter *adap;
};

void at24c02_init (struct at24c02 *dev, u8 addr, struct i2c_adapter *adap);  

int at24c02_read_byte  ( struct at24c02 *dev, u8 offset, u8 *data);
int at24c02_write_byte ( struct at24c02 *dev, u8 offset, u8 data); 
int at24c02_read ( struct at24c02 *dev, u8 offset, u8 *data, int size);
int at24c02_write_page ( struct at24c02 *dev, u8 offset, u8 *data, int size); 



