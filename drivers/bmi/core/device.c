

// bmi_device accessors
static inline int bmi_device_get_status_irq (struct bmi_device *bdev)
{
	return (bdev->slot->status_irq);
}

static inline int bmi_device_get_present_irq (struct bmi_device *bdev)
{
	return (bdev->slot->present_irq);
}

static inline struct i2c_adapter* bmi_device_get_i2c_adapter (struct bmi_device *bdev)
{
	return (&bdev->slot->adap);
}

static inline int bmi_device_get_slot (struct bmi_device *bdev)
{
	return (bdev->slot->slotnum);
}

int bmi_device_present (struct bmi_device *bdev);
struct bmi_device *bmi_device_get(struct bmi_device *dev);
void bmi_device_put(struct bmi_device *dev);

int  bmi_device_read_inventory_eeprom ( struct bmi_device *bdev );
int  bmi_device_init ( struct bmi_device *bdev, struct bmi_info *info, struct bus_type *bmi_bus_type);
void bmi_device_cleanup( struct bmi_device* bdev);
int  bmi_device_i2c_setup( struct bmi_device *bdev);
void bmi_device_i2c_cleanup( struct bmi_device* bdev);
int  bmi_device_spi_setup( struct bmi_device *bdev, u32 speed, u8 mode, u8 bits_per_word);
void bmi_device_spi_cleanup( struct bmi_device* bdev);
struct spi_device *bmi_device_get_spi( struct bmi_device *bdev);
