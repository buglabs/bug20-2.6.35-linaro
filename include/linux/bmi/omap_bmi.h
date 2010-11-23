/* platform data for OMAP BMI slots */

struct omap_bmi_platform_data {
  u8 gpios[4];
  char spi_cs;
  char i2c_bus_no;
};
