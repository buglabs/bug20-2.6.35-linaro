/**
 * sc16is.h - NXP SC16IS754 Dual UART and GPIO driver (Multifunction device).
 *
 * Copyright (C) 2010 Bug Labs Inc.
 *      Matt Isaacs <izzy@buglabs.net>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

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
  const char		*const *names;
};

struct sc16is_platform_data {
  struct sc16is_gpio_platform_data *gpios;
  struct sc16is_uart_platform_data *uarts;
};

