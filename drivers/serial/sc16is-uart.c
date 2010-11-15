#include <linux/init.h>
#include <linux/module.h>

//#define DEBUG 1

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/spi/sc16is.h>


#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
#define BOTH_EMPTY 	(UART_LSR_TEMT | UART_LSR_THRE)

struct sc16is_regs {
  unsigned char		ier;
  unsigned char		fcr;
  unsigned char		iir;
  unsigned char		lcr;
  unsigned char		mcr;
  unsigned char		lsr;
  unsigned char		msr;
  unsigned char		efr;
};

struct sc16is_port {
  struct uart_port      port;
  struct sc16is         *sc16is;
  spinlock_t            c_lock;
  struct timer_list     timer;
  
  unsigned int          quot;
  unsigned int          saved_quot;
  int                   poll_time;

  unsigned char		lsr_saved_flags;
  unsigned char		msr_saved_flags;

  
  int                   tx_empty;
  

  struct sc16is_regs    regs;
  struct sc16is_regs    saved_regs;

  unsigned char		mcr_mask;	/* mask of user bits */
  unsigned char		mcr_force;	/* mask of forced bits */

  struct workqueue_struct *workqueue;
  struct work_struct work;
  /* set to 1 to make the workhandler exit as soon as possible */
  int  force_end_work;

};

struct sc16is_port sc16is_ports[2];

static void serial_sc16is_stop_tx(struct uart_port *port);

static unsigned int serial_in(struct sc16is_port *p, int offset)
{
  u8 data = 0;
  int res = 0;
    
  res = sc16is_read_reg(p->sc16is, p->port.line, offset, &data);
  if (res < 0)
    return res;

  return data;  
}

static void serial_out(struct sc16is_port *p, int offset, int value)
{
  int res = 0;

  res = sc16is_write_reg(p->sc16is, p->port.line, offset, value);  
}

static unsigned int get_msr(struct sc16is_port *s)
{
	unsigned int status = serial_in(s, UART_MSR);

	status |= s->msr_saved_flags;
	s->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && s->regs.ier & UART_IER_MSI &&
	    s->port.state != NULL) {
		if (status & UART_MSR_TERI)
			s->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			s->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&s->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&s->port, status & UART_MSR_CTS);

		wake_up_interruptible(&s->port.state->port.delta_msr_wait);
	}

	return status;
}

/*
 *	Wait for transmitter & holding register to empty
 */
static void wait_for_xmitr(struct sc16is_port *s, int bits)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(s, UART_LSR);
		
		s->regs.lsr = status;
		s->lsr_saved_flags |= status & LSR_SAVE_FLAGS;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & bits) != bits);

	/* Wait up to 1s for flow control if necessary */
	if (s->port.flags & UPF_CONS_FLOW) {
		unsigned int tmout;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(s, UART_MSR);
			s->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			s->regs.msr = msr;
			if (msr & UART_MSR_CTS)
				break;
			udelay(1);
		}
	}
}

static void sc16is_clear_fifos(struct sc16is_port *p)
{
  dev_dbg(&p->sc16is->spi_dev->dev, "%s\n", __func__);
  serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
  serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
	     UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
  serial_out(p, UART_FCR, 0);
}

static void sc16is_set_clock(struct sc16is_port *s)
{
  unsigned int dll, dlh;

  dev_dbg(&s->sc16is->spi_dev->dev, "%s 0x%x\n", __func__, s->quot);
  s->regs.efr = serial_in(s, UART_EFR);
  s->regs.lcr = serial_in(s, UART_LCR);
  dev_dbg(&s->sc16is->spi_dev->dev, "EFR 0x%x LCR 0x%x\n", s->regs.efr, s->regs.lcr);

  serial_out(s, UART_LCR, 0xBF);
  serial_out(s, UART_EFR, s->regs.efr);

  serial_out(s, UART_LCR, s->regs.lcr | UART_LCR_DLAB);/* set DLAB */

  dll = serial_in(s, UART_DLL);
  dlh = serial_in(s, UART_DLM);
  dev_dbg(&s->sc16is->spi_dev->dev, "Clock Divisor latches 0x%x 0x%x\n", dll, dlh);
  serial_out(s, UART_DLL, s->quot & 0xff);
  serial_out(s, UART_DLM, s->quot >> 8 & 0xff);

  serial_out(s, UART_LCR, s->regs.lcr);
}

static void receive_chars(struct sc16is_port *s, unsigned char *status)
{
  struct tty_struct *tty = s->port.state->port.tty;
  unsigned char ch, lsr = *status;
  int max_count = 256;
  char flag;


  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
  do {
    if (likely(lsr & UART_LSR_DR))
      ch = serial_in(s, UART_RX);
    else

      ch = 0;

    flag = TTY_NORMAL;
    s->port.icount.rx++;
    
    lsr |= s->lsr_saved_flags;
    s->lsr_saved_flags = 0;

    if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
      if (lsr & UART_LSR_BI) {
	lsr &= ~(UART_LSR_FE | UART_LSR_PE);
	s->port.icount.brk++;

	if (uart_handle_break(&s->port))
	  goto ignore_char;
      } else if (lsr & UART_LSR_PE)
	s->port.icount.parity++;
      else if (lsr & UART_LSR_FE)
	s->port.icount.frame++;
      if (lsr & UART_LSR_OE)
	s->port.icount.overrun++;
      
      /*
       * Mask off conditions which should be ignored.
       */
      lsr &= s->port.read_status_mask;
      
      if (lsr & UART_LSR_BI) {
	flag = TTY_BREAK;
      } else if (lsr & UART_LSR_PE)
	flag = TTY_PARITY;
      else if (lsr & UART_LSR_FE)
	flag = TTY_FRAME;
    }
    if (uart_handle_sysrq_char(&s->port, ch))
      goto ignore_char;
    
    uart_insert_char(&s->port, lsr, UART_LSR_OE, ch, flag);
    
ignore_char:
    lsr = serial_in(s, UART_LSR);
  } while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
  tty_flip_buffer_push(tty);
  *status = lsr;
}

static void transmit_chars(struct sc16is_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;
	int count;
	

	dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
	if (s->port.x_char) {
		serial_out(s, UART_TX, s->port.x_char);
		s->port.icount.tx++;
		s->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&s->port) ||uart_circ_empty(xmit)) {
	  //stop transmitting
		s->regs.ier &= ~UART_IER_THRI;
		serial_out(s, UART_IER, s->regs.ier);
		return;
	}

	count = 64;         //fifo_size
	do {
		serial_out(s, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		s->port.icount.tx++;
		if (uart_circ_empty(xmit))
		  break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);


	if (uart_circ_empty(xmit)){
	  s->regs.ier &= ~UART_IER_THRI;
	  serial_out(s, UART_IER, s->regs.ier);	
	}
}

static void serial_sc16is_handle_port(struct sc16is_port *s)
{	
	s->regs.lsr = serial_in(s, UART_LSR);
	dev_dbg(&s->sc16is->spi_dev->dev, "%s 0x%x\n", __func__, s->regs.lsr);

	if (s->regs.lsr & (UART_LSR_DR | UART_LSR_BI))
		receive_chars(s, &s->regs.lsr);
	get_msr(s);
	if (s->regs.lsr & UART_LSR_THRE)
		transmit_chars(s);
}

static void sc16is_dowork(struct sc16is_port *s)
{
	if (!s->force_end_work && !work_pending(&s->work) &&
	    !freezing(current))
	  queue_work(s->workqueue, &s->work);
}

static void serial_sc16is_timeout(unsigned long data)
{
  struct sc16is_port *s = (struct sc16is_port *)data;

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
  if (s->port.state) {
    sc16is_dowork(s);
    mod_timer(&s->timer, jiffies + s->poll_time);
  }
}

static void serial_sc16is_work(struct work_struct *w)
{
  struct sc16is_port *s = container_of(w, struct sc16is_port, work);

  struct circ_buf *xmit = &s->port.state->xmit;

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  if (s->quot != s->saved_quot)
    sc16is_set_clock(s);
  if (s->regs.lcr != s->saved_regs.lcr)
    serial_out(s, UART_LCR, s->regs.lcr);
  if (s->regs.mcr != s->saved_regs.mcr)
    serial_out(s, UART_MCR, s->regs.mcr);
  if (s->regs.ier != s->saved_regs.ier)
    serial_out(s, UART_IER, s->regs.ier);
  if (s->regs.fcr != s->saved_regs.fcr)
    serial_out(s, UART_FCR, s->regs.fcr);
  s->saved_regs.lcr = s->regs.lcr;
  s->saved_regs.mcr = s->regs.mcr;
  s->saved_regs.ier = s->regs.ier;
  s->saved_regs.fcr = s->regs.fcr;
  s->saved_quot = s->quot;
  do {
    serial_sc16is_handle_port(s);

  } while (!s->force_end_work &&
	   !freezing(current) &&
	   !uart_circ_empty(xmit) &&
	   !uart_tx_stopped(&s->port));  

}


static irqreturn_t sc16is_irq(int irqno, void *dev_id)
{
	struct sc16is_port *s = dev_id;

	dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

	sc16is_dowork(s);

	return IRQ_HANDLED;
}

static unsigned int serial_sc16is_tx_empty(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  sc16is_dowork(s);

  s->lsr_saved_flags |= s->regs.lsr & LSR_SAVE_FLAGS;

  return (s->regs.lsr & BOTH_EMPTY) == BOTH_EMPTY ? TIOCSER_TEMT : 0;
}

static void serial_sc16is_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  unsigned char mcr = 0;

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  if (mctrl & TIOCM_RTS)
    mcr |= UART_MCR_RTS;
  if (mctrl & TIOCM_DTR)
    mcr |= UART_MCR_DTR;
  if (mctrl & TIOCM_OUT1)
    mcr |= UART_MCR_OUT1;
  if (mctrl & TIOCM_LOOP)
    mcr |= UART_MCR_LOOP;
  
  //  s->regs.mcr |= (mcr & s->mcr_mask) | s->mcr_force | UART_MCR_LOOP;
  s->regs.mcr |= (mcr & s->mcr_mask) | s->mcr_force;
  
  sc16is_dowork(s);
}

static unsigned int serial_sc16is_get_mctrl(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
  unsigned int status;
  unsigned int ret;
  
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  sc16is_dowork(s);
  status = s->regs.msr;
  ret = 0;
  if (status & UART_MSR_DCD)
    ret |= TIOCM_CAR;
  if (status & UART_MSR_RI)
    ret |= TIOCM_RNG;
  if (status & UART_MSR_DSR)
    ret |= TIOCM_DSR;
  if (status & UART_MSR_CTS)
    ret |= TIOCM_CTS;
  return ret;
}

static void serial_sc16is_stop_tx(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
 
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
  
  if (s->regs.ier & UART_IER_THRI) {
    s->regs.ier &= ~UART_IER_THRI;
  }

  sc16is_dowork(s);
}

static void serial_sc16is_start_tx(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
 
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  if (!(s->regs.ier & UART_IER_THRI)) {
    s->regs.ier |= UART_IER_THRI;
  }

  sc16is_dowork(s);
}

static void serial_sc16is_stop_rx(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
 
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  s->regs.ier &= ~UART_IER_RLSI;
  s->port.read_status_mask &= ~UART_LSR_DR;

  sc16is_dowork(s);
}

static void serial_sc16is_enable_ms(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
 
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  s->regs.ier |= UART_IER_MSI;
  sc16is_dowork(s);
}

static void serial_sc16is_break_ctl(struct uart_port *port, int break_state)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  if (break_state == -1)
    s->regs.lcr |= UART_LCR_SBC;
  else
    s->regs.lcr &= ~UART_LCR_SBC;

  sc16is_dowork(s);
}

static int serial_sc16is_startup(struct uart_port *port)
{

  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
  char q[12];
  
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  s->quot = 0;
  s->saved_quot = 0;
  memset(&s->regs, 0, sizeof(struct sc16is_regs));
  memset(&s->saved_regs, 0, sizeof(struct sc16is_regs));
  sc16is_clear_fifos(s);
  
  s->force_end_work = 0;
  sprintf(q, "sc16is-%d", s->port.line);
  s->workqueue = create_freezeable_workqueue(q);
  if (!s->workqueue) {
    dev_warn(&s->sc16is->spi_dev->dev, "cannot create workqueue\n");
    return -EBUSY;
  }
  INIT_WORK(&s->work, serial_sc16is_work);
  
  if (request_irq(s->port.irq, sc16is_irq,
		    IRQF_TRIGGER_FALLING | IRQF_SHARED, "sc16is", s) < 0){
    dev_warn(&s->sc16is->spi_dev->dev, "cannot allocate irq %d\n", s->port.irq);
    s->port.irq = 0;
    destroy_workqueue(s->workqueue);
    s->workqueue = NULL;
    return -EBUSY;
  }

  (void) serial_in(s, UART_LSR);
  (void) serial_in(s, UART_RX);
  (void) serial_in(s, UART_IIR);
  (void) serial_in(s, UART_MSR);

  serial_out(s, UART_LCR, UART_LCR_WLEN8);
  
  serial_in(s, UART_LSR);
  serial_in(s, UART_RX);
  serial_in(s, UART_IIR);
  serial_in(s, UART_MSR);

  s->regs.ier = UART_IER_RLSI | UART_IER_RDI;
  serial_out(s, UART_IER, s->regs.ier);
  
  return 0;
}

static void serial_sc16is_shutdown(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  s->regs.ier = 0;
  serial_out(s, UART_IER, 0);

  s->force_end_work = 1;

  if (s->poll_time > 0)
    del_timer_sync(&s->timer);
  
  if (s->workqueue) {
    flush_workqueue(s->workqueue);
    destroy_workqueue(s->workqueue);
    s->workqueue = NULL;
  }
  if (s->port.irq)
    free_irq(s->port.irq, s);

  serial_out(s, UART_LCR, serial_in(s, UART_LCR) & ~UART_LCR_SBC);
  sc16is_clear_fifos(s);
  (void) serial_in(s, UART_RX);
}

static void
serial_sc16is_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
 


  unsigned char cval;
  unsigned int baud;
  

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  switch (termios->c_cflag & CSIZE) {
  case CS5:
    cval = UART_LCR_WLEN5;
    break;
  case CS6:
    cval = UART_LCR_WLEN6;
    break;
  case CS7:
    cval = UART_LCR_WLEN7;
    break;
  default:
  case CS8:
    cval = UART_LCR_WLEN8;
    break;
  }
  
  if (termios->c_cflag & CSTOPB)
    cval |= UART_LCR_STOP;
  if (termios->c_cflag & PARENB)
    cval |= UART_LCR_PARITY;
  if (!(termios->c_cflag & PARODD))
    cval |= UART_LCR_EPAR;
  
  baud = uart_get_baud_rate(port, termios, old,
			    port->uartclk / 16 / 0xffff,
			    port->uartclk / 16);
  s->quot = uart_get_divisor(port, baud);

  s->regs.fcr = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10;
  s->regs.mcr &= ~UART_MCR_AFE;
  if (termios->c_cflag & CRTSCTS)
    s->regs.mcr |= UART_MCR_AFE;
	     

  /*
   * Update the per-port timeout.
   */
  uart_update_timeout(port, termios->c_cflag, baud);
  
  s->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
  if (termios->c_iflag & INPCK)
    s->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
  if (termios->c_iflag & (BRKINT | PARMRK))
    s->port.read_status_mask |= UART_LSR_BI;
  
  /*
   * Characters to ignore
   */
  s->port.ignore_status_mask = 0;
  if (termios->c_iflag & IGNPAR)
    s->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
  if (termios->c_iflag & IGNBRK) {
    s->port.ignore_status_mask |= UART_LSR_BI;
    /*
     * If we're ignoring parity and break indicators,
     * ignore overruns too (for real raw support).
     */
    if (termios->c_iflag & IGNPAR)
      s->port.ignore_status_mask |= UART_LSR_OE;
  }
  
  /*
   * ignore all characters if CREAD is not set
   */
  if ((termios->c_cflag & CREAD) == 0)
    s->port.ignore_status_mask |= UART_LSR_DR;

  s->regs.ier &= ~UART_IER_MSI;
  if (UART_ENABLE_MS(&s->port, termios->c_cflag))
    s->regs.ier |= UART_IER_MSI;

  if (termios->c_cflag & CRTSCTS)
    s->regs.efr |= UART_EFR_CTS;
  
  if (tty_termios_baud_rate(termios))
    tty_termios_encode_baud_rate(termios, baud, baud);
  sc16is_dowork(s);
}

static void
serial_sc16is_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{

}

static void serial_sc16is_release_port(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
 
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
}

static int serial_sc16is_request_port(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
  return 0;
}

static void serial_sc16is_config_port(struct uart_port *port, int flags)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  if (flags & UART_CONFIG_TYPE)
    s->port.type = PORT_16450;
}

static int serial_sc16is_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
  int ret = -EINVAL;

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);

  if (ser->type == PORT_16450)
    ret = 0;
  return ret;

}

static const char * serial_sc16is_type(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);

  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
  return s->port.type == PORT_16450 ? "SC16IS7XX" : NULL;
}

static int serial_sc16is_get_poll_char(struct uart_port *port)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
  unsigned char lsr = serial_in(s, UART_LSR);
  
  dev_dbg(&s->sc16is->spi_dev->dev, "%s\n", __func__);
  while (!(lsr & UART_LSR_DR))
    lsr = serial_in(s, UART_LSR);
  
  return serial_in(s, UART_RX);
}

static void serial_sc16is_put_poll_char(struct uart_port *port,
			 unsigned char c)
{
  struct sc16is_port *s = container_of(port, struct sc16is_port, port);
  unsigned int ier;
  

  /*
   *	First save the IER then disable the interrupts
   */
  ier = serial_in(s, UART_IER);
  serial_out(s, UART_IER, 0);

  wait_for_xmitr(s, BOTH_EMPTY);
  /*
   *	Send the character out.
   *	If a LF, also do CR...
   */
  serial_out(s, UART_TX, c);
  if (c == 10) {
    wait_for_xmitr(s, BOTH_EMPTY);
    serial_out(s, UART_TX, 13);
  }

  /*
   *	Finally, wait for transmitter to become empty
   *	and restore the IER
   */
  wait_for_xmitr(s, BOTH_EMPTY);
  serial_out(s, UART_IER, ier);
}

static struct uart_ops sc16is_uart_ops = {
  .tx_empty	= serial_sc16is_tx_empty,
  .set_mctrl	= serial_sc16is_set_mctrl,
  .get_mctrl	= serial_sc16is_get_mctrl,
  .stop_tx	= serial_sc16is_stop_tx,
  .start_tx	= serial_sc16is_start_tx,
  .stop_rx	= serial_sc16is_stop_rx,
  .enable_ms	= serial_sc16is_enable_ms,
  .break_ctl	= serial_sc16is_break_ctl,
  .startup	= serial_sc16is_startup,
  .shutdown	= serial_sc16is_shutdown,
  .set_termios	= serial_sc16is_set_termios,
  .pm		= serial_sc16is_pm,
  .type		= serial_sc16is_type,
  .release_port	= serial_sc16is_release_port,
  .request_port	= serial_sc16is_request_port,
  .config_port	= serial_sc16is_config_port,
  .verify_port	= serial_sc16is_verify_port,
#ifdef CONFIG_CONSOLE_POLL
  .poll_get_char = serial_sc16is_get_poll_char,
  .poll_put_char = serial_sc16is_put_poll_char,
#endif
};

static struct uart_driver sc16is_uart_driver = {
  .owner            = THIS_MODULE,
  .driver_name      = "ttysc16is",
  .dev_name         = "ttySC",
  .major            = 204,
  .minor            = 209,
  .nr               = 2,
};



static int __devinit sc16is_uart_probe(struct platform_device *pdev)
{
  struct sc16is_uart_platform_data *pdata = pdev->dev.platform_data;
  struct sc16is *sc16is;
  int ret =0;

  printk(KERN_INFO "sc16is uart probed..\n");
  sc16is = dev_get_drvdata(pdev->dev.parent);
  
  ret = gpio_request(pdata->irq_pin, "spi-uart");
  if (ret)
      printk(KERN_INFO "sc16is uart gpio request failed..\n");
  gpio_direction_input(pdata->irq_pin);
  
  sc16is_ports[0].sc16is = sc16is;
  sc16is_ports[0].port.irq = gpio_to_irq(pdata->irq_pin);
  sc16is_ports[0].port.dev = &pdev->dev;
  sc16is_ports[0].port.line = 0;
  sc16is_ports[0].port.ops = &sc16is_uart_ops;
  sc16is_ports[0].port.fifosize = 64;
  sc16is_ports[0].port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
  sc16is_ports[0].port.uartclk = 18432000;
  sc16is_ports[0].port.type = PORT_16450;
  sc16is_ports[0].timer.function = serial_sc16is_timeout;
  sc16is_ports[0].timer.data = (unsigned long)(&sc16is_ports[0]);

  sc16is_ports[1].sc16is = sc16is;
  sc16is_ports[1].port.irq = gpio_to_irq(pdata->irq_pin);
  sc16is_ports[1].port.dev = &pdev->dev;
  sc16is_ports[1].port.line = 1;
  sc16is_ports[1].port.ops = &sc16is_uart_ops;
  sc16is_ports[1].port.fifosize = 64;
  sc16is_ports[1].port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
  sc16is_ports[1].port.uartclk = 18432000;
  sc16is_ports[1].port.type = PORT_16450;
  sc16is_ports[1].timer.function = serial_sc16is_timeout;
  sc16is_ports[1].timer.data = (unsigned long)(&sc16is_ports[1]);
  ret = uart_add_one_port(&sc16is_uart_driver, &sc16is_ports[0].port);
  if (ret < 0)
    dev_warn(&sc16is_ports[0].sc16is->spi_dev->dev, 
	     "uart_add_one_port failed for line %d with error %d\n",
	     1, ret);
  ret = uart_add_one_port(&sc16is_uart_driver, &sc16is_ports[1].port);
  if (ret < 0)
    dev_warn(&sc16is_ports[1].sc16is->spi_dev->dev, 
	     "uart_add_one_port failed for line %d with error %d\n",
	     1, ret);
  return ret;
}

static int __devinit sc16is_uart_remove(struct platform_device *pdev)
{
  return 0;
}


struct platform_driver sc16is_uart_plat_driver = {
  .driver.name  = "sc16is-uart",
  .driver.owner = THIS_MODULE,
  .probe        = sc16is_uart_probe,
  .remove       = sc16is_uart_remove,
};

static int __init sc16is_uart_init(void)
{
  int ret;
  ret = uart_register_driver(&sc16is_uart_driver);
  if (ret) {
    printk(KERN_ERR "Couldn't register sc16is uart driver\n");
    return ret;
  }

  return platform_driver_register(&sc16is_uart_plat_driver);
}
subsys_initcall(sc16is_uart_init);

static void __exit sc16is_uart_exit(void)
{
  platform_driver_unregister(&sc16is_uart_plat_driver);
}
module_exit(sc16is_uart_exit);

MODULE_AUTHOR("Matt Isaacs - Buglabs, Inc.");
MODULE_DESCRIPTION("UART interface for SC16IS7X");
MODULE_LICENSE("GPL");
