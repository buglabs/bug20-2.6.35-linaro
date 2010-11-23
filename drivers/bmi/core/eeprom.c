#include <linux/types.h>
#include <linux/bmi/bmi-eeprom.h>


static inline __u8 bmi_eeprom_checksum (struct bmi_eeprom_data *raw)
{
	int i;
	__u8 sum = 0;
	__u8 *buf = (__u8*)raw;

	for (i = 0; i < (sizeof (struct bmi_eeprom_data) - 1); i++) {
		sum ^= *buf++;
	}
	return sum;
}


int bmi_eeprom_checksum_validate (struct bmi_eeprom_data *raw)
{
	int ret = 0;
	u8 calcsum;

	calcsum = bmi_eeprom_checksum (raw);

	if (calcsum != raw->checksum) {
		//Rework: add conditional debug messages here
		ret = -1;
	}
	return ret;
}


