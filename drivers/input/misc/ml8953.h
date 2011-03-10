
// I2C ACC register addresses - OKI
#define ACC_PAGESEL			0x1E	// device ready status
// page 0
#define ACC_DVRST			0x01	// device reset
	#define ACC_DVRST_RST		0x3C	// device reset
	#define ACC_DVRST_EN		0xC3	// device enable
#define ACC_PDWN			0x02	// osc power down
	#define ACC_PWDN_RST		0x01	// device reset
	#define ACC_PWDN_EN		0x00	// device enable
#define ACC_CTRL0			0x03	// control 0
	#define ACC_CTRL0_CTSTR		0x40	// control 0 - temp sensor
	#define ACC_CTRL0_CGSTRNC	0x08	// control 0 - 3-axis/no tilt
	#define ACC_CTRL0_CGSTRC	0x04	// control 0 - 3-axis/tilt
	#define ACC_CTRL0_CGAUTO	0x01	// control 0 - auto
#define ACC_MODE0			0x05	// control 0
	#define ACC_MODE0_PDOFF		0x80	// mode 0 - disable auto power down
	#define ACC_MODE0_RVOFF		0x40	// mode 0 - disable temp compensation
	#define ACC_MODE0_TMPOFF	0x20	// mode 0 - disable temp measurement
	#define ACC_MODE0_AGCON		0x10	// mode 0 - enable auto mode pitch and roll
	#define ACC_MODE0_MAUTO		0x04	// mode 0 - enable auto termination
	#define ACC_MODE0_GDET00	0x00	// mode 0 - g detection threshold - see ML8953 data sheet
	#define ACC_MODE0_GDET01	0x01	// mode 0 - g detection threshold - see ML8953 data sheet
	#define ACC_MODE0_GDET10	0x02	// mode 0 - g detection threshold - see ML8953 data sheet
#define ACC_MODE1			0x06	// mode 1
	#define ACC_MODE1_MOFF		0x20	// mode 1 - disable 3-axis continuous mode
	#define ACC_MODE1_ZAXIS		0x03	// mode 1 - Z axis
	#define ACC_MODE1_YAXIS		0x02	// mode 1 - Y axis
	#define ACC_MODE1_XAXIS		0x01	// mode 1 - X axis
	#define ACC_MODE1_RAXIS		0x00	// mode 1 - Reference axis
#define ACC_INTRQ			0x07	// interrupt request (1 = request)
#define ACC_INTMSK			0x08	// interrupt mask (1 = masked)
	#define ACC_INT_TREQ		0x20	// interrupt - temperature
	#define ACC_INT_GREQ		0x08	// interrupt - acceleration/no tilt
	#define ACC_INT_GCREQ		0x04	// interrupt - acceleration/tilt
	#define ACC_INT_GAREQ		0x01	// interrupt - automatic
#define ACC_TMDL			0x09	// timer LSB = (1/6.2 MHz) x 2048 x TMD
#define ACC_TMDH			0x0A	// timer MSB
#define ACC_CFG				0x0C	// configuration
	#define ACC_CFG_REGMD		0x80	// address auto-increment
	#define ACC_CFG_SPI3M_3		0x40	// spi mode = 3-wire
	#define ACC_CFG_SPI3M_4		0x00	// spi mode = 4-wire
	#define ACC_CFG_SDOCFG_T	0x10	// sdo mode = totem-pole
	#define ACC_CFG_SDOCFG_OC	0x00	// sdo mode = open-drain
	#define ACC_CFG_INT1EN_G	0x08	// interrupt 1 mode = g only
	#define ACC_CFG_INT1EN_ALL	0x00	// interrupt 1 mode = all
	#define ACC_CFG_INTLVL		0x04	// interrupt level mode
	#define ACC_CFG_INT1CFG_T	0x02	// interrupt 1 mode = totem-pole
	#define ACC_CFG_INT1CFG_OC	0x00	// interrupt 1 mode = open-drain
	#define ACC_CFG_INT0CFG_T	0x01	// interrupt 0 mode = totem-pole
	#define ACC_CFG_INT0CFG_OC	0x00	// interrupt 0 mode = open-drain
#define ACC_INTOTM			0x0D	// interrupt output conditions
#define ACC_GAAVE			0x0E	// Data averaging - automatic mode
#define ACC_GNAVE			0x0F	// Data averaging - normal mode
#define ACC_GDTCT0L			0x11	// threshold 0 LSB
#define ACC_GDTCT0H			0x12	// threshold 0 MSB
#define ACC_GDTCT1L			0x13	// threshold 1 LSB
#define ACC_GDTCT1H			0x14	// threshold 1 MSB
#define ACC_CPURDY			0x15	// device ready status (ready = 0x01)
	// page 1
#define ACC_STATUS			0x01	// measurment status
	#define ACC_STATUS_ASTS		0x02	// acceleration measurement - automatic modes
	#define ACC_STATUS_STS		0x01	// acceleration measurement - non-automatic modes
#define ACC_GAXL			0x02	// g vector
#define ACC_GAXH			0x03	// g vector
#define ACC_GAYL			0x04	// g vector
#define ACC_GAYH			0x05	// g vector
#define ACC_GAZL			0x06	// g vector
#define ACC_GAZH			0x07	// g vector
#define ACC_GASVL			0x08	// g vector
#define ACC_GASVH			0x09	// g vector
#define ACC_GNXL                        0x0A    // g vector
#define ACC_GNXH                        0x0B    // g vector
#define ACC_GNYL                        0x0C    // g vector
#define ACC_GNYH                        0x0D    // g vector    
#define ACC_GNZL                        0x0E    // g vector
#define ACC_GNZH                        0x0F    // g vector
#define ACC_GNSVL			0x10	// g vector
#define ACC_GNSVH			0x11	// g vector
#define ACC_PITCHL			0x12	// pitch
#define ACC_PITCHH			0x13	// pitch
#define ACC_ROLLL			0x14	// roll
#define ACC_ROLLH			0x15	// roll 
#define ACC_TEMPL			0x19	// temperature
#define ACC_TEMPH			0x1A	// temperature


struct ml8953 {
	struct i2c_client *client;
	struct mutex mutex;
	struct work_struct work;
	struct input_dev *input;
	char phys[32];
	u8 sample[6];
	short saved[3];
	u8 open;
	u8 disabled;
};

