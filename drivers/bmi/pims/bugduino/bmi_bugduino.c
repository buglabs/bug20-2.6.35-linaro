#include <linux/bmi/bmi_ioctl.h>
#include <linux/bmi/bmi_bugduino.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/bmi-ids.h>
#include <linux/buffer_head.h>

#include <asm/segment.h>
#include <asm/uaccess.h>

static int bugduino_i2c_write( struct i2c_client *i2c_passed_client, 
	struct bmi_device *bmi_device,
	uint8_t address, uint8_t offset, uint8_t data );

static int bugduino_i2c_read( struct i2c_client *i2c_passed_client, 
	struct bmi_device *bmi_device,
	uint8_t address, uint8_t offset, uint8_t *data );

static int io_expander_write_byte( struct i2c_client* i2c_client,
	uint8_t address, uint8_t data );
static int io_expander_read_byte( struct i2c_client* i2c_client,
	uint8_t address, uint8_t* data );
static irqreturn_t duino_irq_handler( int interrupt, void* dummy );
int bugduino_probe( struct bmi_device* bmi_device );
void bugduino_remove( struct bmi_device* bmi_device );
int bugduino_ioctl( struct inode* inode, struct file* file,
		 unsigned int command, unsigned long arg );
int bugduino_open( struct inode* inode,
		struct file* file );
int bugduino_close( struct inode* inode,
		 struct file* file );

static int __init bugduino_init( void );
static void __exit duino_cleanup( void );

static struct bmi_bugduino {
	struct bmi_device* bmi_device; 
	struct cdev char_device;
	struct device* class_device;
	int open_flag;
	char interrupt_name[BUGDUINO_INTERRUPT_NAME_SIZE];
	struct i2c_client* io_expander;
	struct spi_device* spi_device;
	struct spi_board_info spi_info;
	char spi_buffer_read[BUGDUINO_SPI_BUFFER_SIZE];
	char spi_buffer_write[BUGDUINO_SPI_BUFFER_SIZE];
};

/* Global Variables */
static struct bmi_bugduino bmi_bugduino_collection[4];
static int major;

static struct bmi_device_id duino_table[] = {
	//TRICKY: this can have a BUG_LABS or A SEED_STUDIO vendor as a valid id, just match device.
	{
	.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	.vendor   = BMI_VENDOR_BUG_LABS, 
	//.match_flags = BMI_DEVICE_ID_MATCH_PRODUCT,
	//.vendor = BMI_VENDOR_BUG_LABS,
	.product = BMI_PRODUCT_BUGDUINO,
	.revision = BMI_ANY,
	},
	{ 0,},
};

static struct bmi_driver duino_driver = {
		.name = "bmi_bugduino",
		.id_table = duino_table,
		.probe = bugduino_probe,
		.remove = bugduino_remove,
};

MODULE_DEVICE_TABLE(bmi, duino_table);

static struct file_operations duino_fops = {
		.owner = THIS_MODULE,
		.ioctl = bugduino_ioctl,
		.open = bugduino_open,
		.release = bugduino_close,
		/*
		   .read = duino_read,
		   .write = duino_write,
		 */
};

static struct i2c_board_info io_expander_info = {
		I2C_BOARD_INFO( "BUGDUINO_IO_EXPANDER", BUGDUINO_IOX_I2C_ADDRESS ),
};

static int bugduino_i2c_write( struct i2c_client *i2c_passed_client, 
				struct bmi_device *bmi_device,
				uint8_t address, uint8_t offset, uint8_t data )
{

	int rc = 0;
	uint8_t message[BUGDUINO_I2C_MESSAGE_SEND_SIZE];
	struct i2c_board_info i2c_info = {
			I2C_BOARD_INFO( "I2C_TMP_DEVICE", address ), };
	struct i2c_client *i2c_client;

	if( i2c_passed_client != NULL ){
		i2c_client = i2c_passed_client;
	} else {
		i2c_client = i2c_new_device( bmi_device -> slot -> adap, &i2c_info );
	}

	if( i2c_client == NULL ){
		printk( KERN_ERR "bmi_bugduino.c: could not register"
			" temporary i2c device" );
		return -ENODEV;
	}

	/* i2c send the data location */
	message[0] = offset;
	message[1] = data;
	rc = i2c_master_send( i2c_client, message, BUGDUINO_I2C_MESSAGE_SEND_SIZE );
	if( rc < 0 ){
		printk( KERN_ERR "bmi_bugduino.c: i2c_write() failed. Errno = %i\n", rc );
	}
	
	if( i2c_passed_client == NULL ){
		i2c_unregister_device( i2c_client );
	}  

	return rc;
}

static int bugduino_i2c_read( struct i2c_client *i2c_passed_client, 
	struct bmi_device *bmi_device,
	uint8_t address, uint8_t offset, uint8_t *data ){
	int rc;
	struct i2c_board_info i2c_info = {
		I2C_BOARD_INFO( "I2C_TMP_DEVICE", address ),
	};
	struct i2c_client *i2c_client;

	if( i2c_passed_client != NULL ){
		i2c_client = i2c_passed_client;
	} else {
		i2c_client = i2c_new_device( bmi_device -> slot -> adap, &i2c_info );
	}

	rc = i2c_master_send( i2c_client, &offset, 1 );
	if( rc == 1 ){
		rc = i2c_master_recv( i2c_client, data, 1 );
	} else if( rc < 0 ){
		printk( KERN_ERR "bmi_bugduino.c: i2c_read() failed. Errno = %i\n", rc );
	}
	
	if( i2c_passed_client == NULL ){
		i2c_unregister_device( i2c_client );
	}  

	return rc;
}

static int io_expander_write_byte( struct i2c_client* i2c_client,
	uint8_t address, uint8_t data ){
	int rc = 0;
	uint8_t message[BUGDUINO_IOX_MESSAGE_SIZE];
	
	message[BUGDUINO_IOX_MESSAGE_ADDRESS] = address;
	message[BUGDUINO_IOX_MESSAGE_DATA] = data;

	rc = i2c_master_send( i2c_client, message, sizeof( message ) );
	if( rc < 0 ){
		printk( KERN_ERR "bmi_bugduino.c: io_expander_write_byte()"
		" failed. Errno = %i\n", rc );
	}
	return rc;
}

static int io_expander_read_byte( struct i2c_client* i2c_client,
uint8_t address, uint8_t* data ){
	int rc;
	
	rc = i2c_master_send( i2c_client, &address, 1 );
	if( rc == 1 ){
		rc = i2c_master_recv( i2c_client, data, 1 );
	} else if( rc < 0 ){
		printk( KERN_ERR "bmi_bugduino.c: io_expander_read_byte() "
			"failed. Errno = %i\n", rc );
	}
	return rc;
}

/* why doesn't this function work? */
static irqreturn_t duino_irq_handler( int interrupt, void* dummy ){
	//printk( KERN_INFO "Interrupt detected.\n" );
	return IRQ_HANDLED;
}

int bugduino_probe( struct bmi_device* bmi_device ){
	int slot;
	int rc;
	int interrupt;
	struct cdev* char_device;
	dev_t char_device_id;

	struct bmi_bugduino* bmi_bugduino;
	struct class* class_device;
	struct i2c_adapter* i2c_adapter;

	slot = bmi_device->slot->slotnum;
	i2c_adapter = bmi_device->slot->adap;

	printk( KERN_INFO "bmi_bugduino.c: BUGduino Module "
		"Detected in slot %i\n", slot );
	
	bmi_bugduino = &(bmi_bugduino_collection[slot]);
	
	bmi_bugduino -> bmi_device = 0;
	bmi_bugduino -> open_flag = 0;
	
	/* Create a minor device */
	char_device = &( bmi_bugduino->char_device );
	cdev_init( char_device, &duino_fops );
	char_device_id = MKDEV( major, slot );
	rc = cdev_add( char_device, char_device_id, 1 );
	if( rc != 0 ){
		printk( KERN_ERR "cdev add failed with error %d",rc);
		return rc;
	}

	/* Create a class device */
	class_device = bmi_get_class();
	bmi_bugduino->class_device = device_create( class_device,
		NULL, MKDEV( major, slot ), NULL, "bmi_bugduino_slot%i", slot );

	if( IS_ERR( bmi_bugduino -> class_device ) ){
		printk( KERN_ERR "bmi_bugduino.c: Could not create class "
			"device for bmi_bugduino_slot%i. Errno = %ld.\n", slot,
		    PTR_ERR( bmi_bugduino -> class_device ) );
		bmi_bugduino -> class_device = NULL;
		cdev_del( &( bmi_bugduino -> char_device ) );
		return -ENODEV;
	}
	else {
		printk(KERN_ERR "bmi device is named %s", 
			bmi_bugduino->class_device->init_name);	
	}
	/* Bind the driver and the bmi_device */
	bmi_bugduino -> bmi_device = bmi_device;

	/* Find the IO expander */
	bmi_bugduino -> io_expander = i2c_new_device( bmi_device -> slot -> adap,
		&io_expander_info );
	if( bmi_bugduino -> io_expander == NULL ){
		printk( KERN_ERR "Could not create IO expander device...\n" );
		printk( KERN_ERR "Some bugduino features may not be available\n");
	}
	
	/* Configure SPI */
	strcpy( bmi_bugduino -> spi_info.modalias, "spidev" );
	bmi_bugduino -> spi_info.max_speed_hz = BUGDUINO_SPI_SPEED;
	bmi_bugduino -> spi_info.bus_num = bmi_device -> slot -> spi_bus_num;
	bmi_bugduino -> spi_info.chip_select = bmi_device -> slot -> spi_cs;
	bmi_bugduino -> spi_info.mode = BUGDUINO_SPI_MODE;
	bmi_bugduino -> spi_device = spi_new_device( 
		spi_busnum_to_master( bmi_bugduino -> spi_info.bus_num ),
		&( bmi_bugduino -> spi_info ) );
	if( !( bmi_bugduino -> spi_device ) ){
		printk( KERN_WARNING "bmi_bugduino.c: spi_new_device failed...\n" );
	}

	/* yeah, I don't know what it does, either */
	bmi_device_set_drvdata( bmi_device, bmi_bugduino );

	/* Configure IO expander */
	/* Set 0-3 as inputs, 4-7 to output 1 */
	rc = io_expander_write_byte( bmi_bugduino -> io_expander,
		BUGDUINO_IOX_REG_OUTPUT, 0xFF );
	if( rc < 0 ){
		printk( KERN_WARNING "bmi_bugduino.c: could not configure"
			"IO expander...\n" );

		/* Don't worry - I'm in the kernel. */
		goto error;
	}

	rc = io_expander_write_byte( bmi_bugduino -> io_expander,
			BUGDUINO_IOX_REG_CONTROL, 0x0F );
	if( rc < 0 ){
		printk( KERN_WARNING "bmi_bugduino.c: could not "
		"configure IO expander...\n" );

		/* Don't worry - I'm in the kernel. */
		goto error;
	}

	/* Request PIM interrupt */
	interrupt = bmi_device -> slot -> status_irq;
	sprintf( bmi_bugduino -> interrupt_name, "bmi_bugduino%i", slot );
	rc = request_irq( interrupt,
		&duino_irq_handler, 0, bmi_bugduino -> interrupt_name, bmi_bugduino );
	if( rc != 0 ){
		printk( KERN_WARNING "bmi_bugduino.c: could not configure interrupt\n" );
		/* Don't worry - I'm in the kernel. */
		goto error;
	}

	/* Initialize GPIO1 (that's the reset pin) */
	bmi_slot_gpio_direction_out( slot, BUGDUINO_GPIO_RESET_PIN, BUGDUINO_LOW );

	/* Get the appropriate UART path and print it */
	{
	  char* uart_path;
	  
	  switch( slot ){
	  case 0:
		uart_path = UART_SLOT_0;
	    break;
	  case 2:
	    uart_path = UART_SLOT_2;
	    break;
	  case 3:
	    uart_path = UART_SLOT_3;
	    break;
	  default:
	    printk( KERN_WARNING "bmi_bugduino.c: you plugged the "
			"BUGduino in to a slot that does not exist. You monster!...\n" );
	    
	    /* Don't worry - I'm in the kernel */
	    goto error;
	  }
	  
	  printk( KERN_INFO "bmi_bugduino.c: using slot %i, "
		"corresponding to UART %s", slot, uart_path );
	}

	/* REMOVE */
	//i2c_unregister_device( bmi_bugduino -> io_expander );

	return 0; 
	
 error:
	bmi_bugduino -> class_device = NULL;
	cdev_del( &( bmi_bugduino -> char_device ) );
	device_destroy( class_device, MKDEV( major, slot ) );
	bmi_bugduino -> bmi_device = 0;
	i2c_unregister_device( bmi_bugduino -> io_expander );
	spi_unregister_device( bmi_bugduino -> spi_device );
	return -ENODEV;
}

void bugduino_remove( struct bmi_device* bmi_device ){
	int i;
	int slot;
	struct bmi_bugduino* bmi_bugduino;
	struct class* bmi_class;

	slot = bmi_device -> slot -> slotnum;
	printk( KERN_ERR "bmi_bugduino.c: Module removed from slot %i.\n", slot );

	bmi_bugduino = &( bmi_bugduino_collection[slot] );
	
	/* ADD */
	i2c_unregister_device( bmi_bugduino -> io_expander );
	spi_unregister_device( bmi_bugduino -> spi_device );

	free_irq( bmi_device -> slot -> status_irq, bmi_bugduino );
	
	/* reset the GPIO pins */
	for( i = 0; i < 4; ++i ){
	  bmi_slot_gpio_direction_in( slot, i );
	}

	bmi_class = bmi_get_class();
	device_destroy( bmi_class, MKDEV( major, slot ) );
	
	bmi_bugduino -> class_device = 0;

	cdev_del( &(bmi_bugduino -> char_device ) );

	/* I have no idea what this does, but it was done in
	   the vonHippel drivers */
	bmi_device_set_drvdata( bmi_device, 0 );
	bmi_bugduino -> bmi_device = 0;
	
	return;
}

int bugduino_ioctl( struct inode* inode, struct file* file,
		 unsigned int command, unsigned long arg )
{
	struct i2c_adapter *i2c_adapter;
	int rc = 0;

	struct bmi_bugduino *duino;
	int slot;
	
	duino = (struct bmi_bugduino *)(file -> private_data );

	/* error if the BUGduino isn't present */
	if( duino -> bmi_device == 0 ){
	  return -ENODEV;
	}

	slot = duino -> bmi_device -> slot -> slotnum;
	i2c_adapter = duino -> bmi_device -> slot -> adap;

	switch( command ){
	case BMI_BUGDUINO_RESET:
	  {
		printk( KERN_ERR "bmi_bugduino.c: doing bugduino reset.\n" );
	    bmi_slot_gpio_set_value( slot, BUGDUINO_GPIO_RESET_PIN, 
			( ( arg == 0x00 ) ? ( BUGDUINO_RESET_OFF ) : ( BUGDUINO_RESET_ON ) ) );
	  }
	  break;
	case BMI_BUGDUINO_IOX_CTRL:
	  {      
	    rc = io_expander_write_byte( duino -> io_expander, 
				BUGDUINO_IOX_REG_CONTROL, (uint8_t)arg );
	  }
	  break;
	case BMI_BUGDUINO_IOX_WRITE:
	  {
	    rc = io_expander_write_byte( duino -> io_expander, 
				BUGDUINO_IOX_REG_OUTPUT, (uint8_t)arg );
	  }
	  break;
	case BMI_BUGDUINO_IOX_READ:
	  {
	    rc = io_expander_read_byte( duino -> io_expander, 
				BUGDUINO_IOX_REG_INPUT, (uint8_t *)arg );
	  }
	  break;
	case BMI_BUGDUINO_I2C_WRITE:
	  {
	    struct bugduino_i2c_package package;

	    rc = copy_from_user( &package, (struct duinio_i2c_package __user *)arg,
			 sizeof( struct bugduino_i2c_package ) );
	    if( rc != 0x00 ){
	printk( KERN_ERR "bmi_bugduino.c: Probmem with i2c package.\n" );
	    	return rc;
	    }
	    
	    rc = bugduino_i2c_write( NULL, duino -> bmi_device, 
			( (package.address) >> 1 ), package.offset, package.data );
	    
	  }
	  break;
	case BMI_BUGDUINO_I2C_READ:
	  {
	    struct bugduino_i2c_package package;

	    rc = copy_from_user( &package, (struct duinio_i2c_package __user *)arg, 
			sizeof( struct bugduino_i2c_package ) );
	    if( rc != 0 ){
	printk( KERN_ERR "bmi_bugduino.c: Probmem with i2c package.\n" );
	    	return rc;
	    }
	    
	    rc = bugduino_i2c_read( NULL,
		     duino -> bmi_device,
		     ( (package.address) >> 1 ), 
		     package.offset,
		     &( package.data ) );

	    rc = copy_to_user( (struct bugduinio_i2c_package __user *)arg, 
			&package, sizeof( struct bugduino_i2c_package ) );
	    if( rc != 0x00 ){
	printk( KERN_ERR "bmi_bugduino.c: Probmem with i2c package.\n" );
	    	return rc;
	    }

	  }
	  break;
	case BMI_BUGDUINO_SPI_XFER:
	  {
	    /* BUGduino should manually control CSN pin of the slave device */
	    int rc;
	    unsigned char tx_data;
	    unsigned char rx_data;

	    rc = copy_from_user( &tx_data, (unsigned char __user *)arg, sizeof( tx_data ) );
	    if( rc != 0 ){
	printk( KERN_ERR "bmi_bugduino.c: Problem copying SPI data from userspace.\n" );
	return rc;
	    }

	    rc = spi_write_then_read( duino -> spi_device,
				&tx_data,
				sizeof( tx_data ),
				&rx_data,
				sizeof( rx_data ) );

	    rc = copy_to_user( (unsigned char __user *)arg, &rx_data, sizeof( rx_data ) );
	    if( rc != 0 ){
	printk( KERN_ERR "bmi_bugduino.c: Problem copying SPI data to userspace.\n" );
	return rc;
	    }

	  }
	  break;
	default:
	  return -ENOTTY;
	  break;
	}

	return rc;
}

int bugduino_open( struct inode* inode,
		struct file* file ){
	struct bmi_bugduino *duino;

	duino = container_of( inode -> i_cdev,
			struct bmi_bugduino,
			char_device );

	if( duino -> open_flag == 1 ){
	  return -EBUSY;
	}

	duino -> open_flag = 1;

	file -> private_data = duino;

	return 0;
}
		
int bugduino_close( struct inode* inode,
		 struct file* file ){
	struct bmi_bugduino *duino;

	duino = (struct bmi_bugduino *)( file -> private_data );
	duino -> open_flag = 0;

	return 0;
}

/*
static int duino_file_open( const char* path,
			    int flags,
			    int rights ){
	int fd;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs( get_ds() );
	fd = sys_open( path, flags, rights );
	set_fs( oldfs );
	
	return fd;
}

static void duino_file_close( int fd ){
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs( get_ds() );
	sys_close( fd );
	set_fs( oldfs );
	
	return;
}

static ssize_t duino_file_read( int fd,
				char* buffer,
				size_t size ){
	mm_segment_t oldfs;
	ssize_t rc;

	oldfs = get_fs();
	set_fs( get_ds() );
	rc = sys_read( fd, buffer, size );
	set_fs( oldfs );

	return rc;
}

static ssize_t duino_file_write( int fd,
			     char* buffer,
			     size_t size ){
	mm_segment_t oldfs;
	ssize_t rc;

	oldfs = get_fs();
	set_fs( get_ds() );
	rc = sys_write( fd, buffer, size );
	set_fs( oldfs );

	return rc;
}
*/
/*
ssize_t duino_read( struct file* filp,
		    char __user* data,
		    size_t count,
		    loff_t* offset ){
	return 0; 
}

ssize_t duino_write( struct file* filp,
		     const char __user* data,
		     size_t count,
		     loff_t* offset ){
	return 0;
}

int file_sync( struct file* file ){
	file_fsync( file, file -> f_dentry, 0 );
	
	return 0;
}
*/
static int __init bugduino_init( void ){
	dev_t dev_id;
	int rc;

	rc = alloc_chrdev_region( &dev_id, 0, 4, "BMI Duino driver" );

	if( rc != 0 ){
	  return -ENODEV;
	}

	major = MAJOR( dev_id );
	rc = bmi_register_driver( &duino_driver );
	if( rc != 0 ){
	  unregister_chrdev_region( dev_id, 4 );
	  return -ENODEV;
	}

	printk( KERN_INFO "bmi_bugduino.c: BMI bugduino Driver v%s.\n", 
		BUGDUINO_VERSION );

	return 0;
}

static void __exit duino_cleanup( void ){
	dev_t device_id;
	bmi_unregister_driver( &duino_driver );
	device_id = MKDEV( major, 0 );
	unregister_chrdev_region( device_id, 4 );
	return;
}

module_init( bugduino_init );
module_exit( duino_cleanup );

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Buglabs Inc." );
MODULE_DESCRIPTION( "BMI bugdevice driver" );
MODULE_SUPPORTED_DEVICE( "bmi_bugduino_mX" );
