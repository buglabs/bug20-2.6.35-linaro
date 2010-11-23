#ifndef BMI_EEPROM_H
#define BMI_EEPROM_H

#include <linux/types.h>

union _bmi_vendor {
  __u16 vendor;
  __u8	vendor_msb;
  __u8	vendor_lsb;
};

union _bmi_product {
  __u16 product;
  __u8	product_msb;
  __u8	product_lsb;
};

union _bmi_revision {
  __u16 revision;
  __u8	revision_msb;
  __u8	revision_lsb;
};

struct bmi_eeprom_data
{
  __u8 format;			/* byte 0x00 */
  __u8 vendor_msb;		/* byte 0x01 */
  __u8 vendor_lsb;		/* byte 0x02 */
  __u8 product_msb;		/* byte 0x03 */
  __u8 product_lsb;		/* byte 0x04 */
  __u8 revision_msb;		/* byte 0x05 */
  __u8 revision_lsb;		/* byte 0x06 */
/*   __u16 vendor; */
/*   __u16 product; */
/*   __u16 revision; */
  __u8 bus_usage;			/* byte 0x07 */ 
  __u8 gpio_usage;		/* byte 0x08 */ 
  __u8 power_use;		       	/* byte 0x09 */ 
  __u8 power_charging;		/* byte 0x0A */ 
  __u8 memory_size_msb;		/* byte 0x0B */ 
  __u8 memory_size_lsb;		/* byte 0x0C */ 
  __u8 serial_num_loc;		/* byte 0x0D */ 
  __u8 serial_num_year;		/* byte 0x0E */ 
  __u8 serial_num_week;		/* byte 0x0F */ 
  __u8 serial_num_seq_msb;	/* byte 0x10 */ 
  __u8 serial_num_seq_mid;	/* byte 0x11 */ 
  __u8 serial_num_seq_lsb;	/* byte 0x12 */ 
  __s8 description[108];		/* byte 0x13-0x7E */
  __u8 checksum;			/* byte 0x7F */ 
};


struct bmi_eeprom_id
{
	__u16 vendor;
	__u16 product;
	__u16 revision;
};


enum {
	BMI_EPSTATE_UNKNOWN = 0,
	BMI_EPSTATE_I2C_READ_ERROR,
	BMI_EPSTATE_CHECKSUM_ERROR,
	BMI_EPSTATE_VALID,
};


//__u8 bmi_eeprom_checksum ( struct bmi_eeprom_data *raw );
int bmi_eeprom_checksum_validate ( struct bmi_eeprom_data *raw );
//extern void bmi_eeprom_get_id (struct bmi_eeprom_data *raw, struct bmi_eeprom_id *epid);
//extern int bmi_eeprom_checksum_validate ( struct bmi_eeprom_data *raw );


#endif /* BMI_EEPROM_H */
