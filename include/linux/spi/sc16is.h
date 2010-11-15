#include <linux/spi/spi.h>

struct sc16is {
  struct spi_device *spi_dev;
  struct mutex lock;
  unsigned irq_pin;
};

int sc16is_write_reg(struct sc16is *sc16is, unsigned char channel, unsigned char address, unsigned char data);

int sc16is_read_reg(struct sc16is *sc16is, unsigned char channel, unsigned char address, unsigned char *data);

void sc16is_dump_regs(struct sc16is *sc16is);

/* platform data for the SC16IS driver */


struct sc16is_uart_platform_data {
  unsigned irq_pin;
};

struct sc16is_gpio_platform_data {
  /* number of the first GPIO */
  unsigned	gpio_base;

  /* initial polarity inversion setting */
  uint16_t	invert;

  void		*context;	/* param to setup/teardown */
  
  int		(*setup)(struct spi_device *spi_dev,
			 unsigned gpio, unsigned ngpio,
			 void *context);
  int		(*teardown)(struct spi_device *spi_dev,
			    unsigned gpio, unsigned ngpio,
			    void *context);
  char		**names;
};

struct sc16is_platform_data {
  struct sc16is_gpio_platform_data *gpios;
  struct sc16is_uart_platform_data *uarts;
};

