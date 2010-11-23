/*
 *      bmi_sensor.c
 *
 *      BMI sensor device driver
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *      Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>

#include <linux/bmi.h>
#include <linux/bmi-ids.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_sensor.h>

#define BMISENSOR_VERSION       "2.0"

#define work_to_sensor(w) (struct bmi_sensor*) container_of(w, struct bmi_sensor, work_item)
#define dev_to_bmi_device(d) (struct bmi_device*) container_of(d, struct bmi_device, dev)

/*
 *      Global variables
 */

// TODO Why do I usually get "IOX not found" if I probe it
// (even after it's been in the slot for a while)
#undef PROBE_IOX
#if defined(PROBE_IOX)
static struct i2c_board_info iox_info = {
    .type = "SENSOR_IOX",
};
static const unsigned short iox_addresses[] = { BMI_IOX_I2C_ADDRESS, I2C_CLIENT_END };
#else
static struct i2c_board_info iox_info = {
    I2C_BOARD_INFO("SENSOR_IOX", BMI_IOX_I2C_ADDRESS)
};
#endif

static struct i2c_board_info adc_info = {
    .type = "SENSOR_ADC",
};
static const unsigned short adc_addresses[] = { BMI_ADC_I2C_ADDRESS, I2C_CLIENT_END };

static struct i2c_board_info pl_info = {
    .type = "SENSOR_PL",
};
static const unsigned short pl_addresses[] = { BMI_PL_I2C_ADDRESS, I2C_CLIENT_END };

static struct i2c_board_info temp_info = {
    .type = "SENSOR_TEMP", 
};
static const unsigned short temp_addresses[] = { BMI_TEMP_I2C_ADDRESS, I2C_CLIENT_END };

static struct i2c_board_info acc_info = {
    .type = "SENSOR_ACC",
};
static const unsigned short acc_addresses[] = { BMI_ACC_I2C_ADDRESS, I2C_CLIENT_END };

static struct i2c_board_info dcomp_info = {
    .type = "SENSOR_DCOMP",
};
static const unsigned short dcomp_addresses[] = { BMI_DCOMP_I2C_ADDRESS, I2C_CLIENT_END };

#define ADC_PRESENT (sensor->adc_i2c_client != NULL)
#define ACC_PRESENT (sensor->acc_i2c_client != NULL)
#define PL_PRESENT (sensor->pl_i2c_client != NULL)
// presume DL is there if AL is as they're the same part, right? TODO
#define DL_PRESENT PL_PRESENT
#define TEMP_PRESENT (sensor->temp_i2c_client != NULL)

// we need to do this by board revision number
#define MOTION_PRESENT (true)

#define DCOMP_PRESENT (sensor->dcomp_i2c_client != NULL)

// presume all analog sensors are present if the ADC is present
// if we ever need to care we'll do it based on the board
// revision number
#define ACOMP_PRESENT ADC_PRESENT
#define ALIGHT_PRESENT ADC_PRESENT
#define APROX_PRESENT ADC_PRESENT
#define HUMIDITY_PRESENT ADC_PRESENT
#define SOUND_PRESENT ADC_PRESENT

// private device structure
struct bmi_sensor
{
    struct semaphore                sem;
    struct bmi_device               *bdev;
    struct cdev                     cdev;
    struct device                   *class_dev;

#undef IRQ_STUFF_IMPLEMENTED
#if defined(IRQ_STUFF_IMPLEMENTED)
    char                            int_name[20];
#endif

    struct workqueue_struct         *workqueue;
    struct work_struct              work_item;
    char                            workqueue_name[20];

    wait_queue_head_t               pl_wait_queue;
    unsigned char                   pl_int_en;
    unsigned char                   pl_int_fl;

    wait_queue_head_t               temp_wait_queue;
    unsigned char                   temp_int_en;
    unsigned char                   temp_int_fl;

    wait_queue_head_t               mot_wait_queue;
    unsigned char                   mot_int_en;
    unsigned char                   mot_int_fl;
    unsigned int                    mot_state;

    wait_queue_head_t               acc_wait1_queue;
    unsigned char                   acc_int1_en;
    unsigned char                   acc_int1_fl;
    wait_queue_head_t               acc_wait2_queue;
    unsigned char                   acc_int2_en;
    unsigned char                   acc_int2_fl;

    wait_queue_head_t               dcomp_wait_queue;
    unsigned char                   dcomp_int_en;
    unsigned char                   dcomp_int_fl;

    unsigned int                    aprox_duration;
    struct timer_list               aprox_timer;
    wait_queue_head_t               aprox_wait_queue;
    unsigned char                   aprox_int_en;
    unsigned char                   aprox_int_fl;

    wait_queue_head_t               dlight_wait_queue;
    unsigned char                   dlight_int_en;
    unsigned char                   dlight_int_fl;

    struct i2c_client *             iox_i2c_client;
    struct i2c_client *             adc_i2c_client;
    struct i2c_client *             pl_i2c_client;
    struct i2c_client *             dlight_i2c_client;
    struct i2c_client *             temp_i2c_client;
    struct i2c_client *             acc_i2c_client;
    struct i2c_client *             dcomp_i2c_client;
};

static struct bmi_sensor bmi_sensor[4]; // per slot device structure
static int major; // control device major

/*
 *      BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_sensor_tbl[] =
{
    {
        .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT,
        .vendor   = BMI_VENDOR_BUG_LABS,
        .product  = BMI_PRODUCT_SENSOR,
        .revision = BMI_ANY,
    },
    { 0, },           /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_sensor_tbl);

int     bmi_sensor_probe(struct bmi_device *bdev);
void    bmi_sensor_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_sensor_driver =
{
    .name = "bmi_sensor",
    .id_table = bmi_sensor_tbl,
    .probe   = bmi_sensor_probe,
    .remove  = bmi_sensor_remove,
};

/*
 *      I2C set up
 */


#define get_mot_det_state(slot) ((bmi_slot_gpio_get_all( (slot) ) & (2 ^SENSOR_GPIO_MOT_DET)) ? 1 : 0)

// IOX
// write byte to I2C IO expander
static int WriteByte_IOX(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write to IOX without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char msg[2];

        msg[0] = offset;
        msg[1] = data;
        ret = i2c_master_send(client, msg, sizeof(msg));

        if (ret < 0)
            printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// read byte from I2C IO expander
static int ReadByte_IOX(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to read from IOX without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_send(client, &offset, 1);

        if (ret == 1)
            ret = i2c_master_recv(client, data, 1);

        if (ret < 0)
            printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// ADC
// write byte to ADC
static int WriteByte_ADC(struct i2c_client *client, unsigned char data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write to ADC without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_send(client, &data, sizeof(data));

        if (ret < 0)
            printk (KERN_ERR "WriteByte_ADC() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// read data from ADC
static int ReadByte_ADC(struct i2c_client *client, unsigned char *data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to read from ADC without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_recv(client, data, 2);

        if (ret < 0)
            printk (KERN_ERR "ReadByte_ADC() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// Proximity/Light and Digital Light (same I2c address and format)
// write byte to I2C PL
static int WriteByte_PL(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write to PL without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char msg[2];

        msg[0] = offset;
        msg[1] = data;
        ret = i2c_master_send(client, msg, sizeof(msg));

        if (ret < 0)
            printk (KERN_ERR "WriteByte_PL() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// read byte from I2C PL
static int ReadByte_PL(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to read from PL without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_send(client, &offset, 1);
        if (ret == 1)
            ret = i2c_master_recv(client, data, 1);
        if (ret < 0)
            printk (KERN_ERR "ReadByte_PL() - i2c_transfer() failed...%d\n",ret);
        return ret;
    }
}

// write byte to I2C PL SYNC
static int WriteByte_PL_SYNC(struct i2c_client *client)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write PL SYNC without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char offset = SENSOR_PL_EXT_SYNC;

        ret = i2c_master_send(client, &offset, 1);
        if (ret < 0)
            printk (KERN_ERR "WriteByte_PL_SYNC() - i2c_transfer() failed...%d\n",ret);
        return ret;
    }
}

// write byte to I2C DL Interrupt Clear
static int WriteByte_DL_IC(struct i2c_client *client)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write DL IC without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char offset = SENSOR_DL_INT_CLR;

        ret = i2c_master_send(client, &offset, 1);
        if (ret < 0)
            printk (KERN_ERR "WriteByte_DL_IC() - i2c_transfer() failed...%d\n",ret);
        return ret;
    }
}

// Temperature
// write byte to Temperature sensor
static int WriteByte_TEMP(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write to TEMP without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char msg[2];

        msg[0] = offset;
        msg[1] = data;
        ret = i2c_master_send(client, msg, sizeof(msg));

        if (ret < 0)
            printk (KERN_ERR "WriteByte_TEMP() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// read byte from Temperature sensor
static int ReadByte_TEMP(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to read from TEMP without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_send(client, &offset, 1);
        if (ret == 1)
            ret = i2c_master_recv(client, data, 1);
        if (ret < 0)
            printk (KERN_ERR "ReadByte_TEMP() - i2c_transfer() failed...%d\n",ret);
        return ret;
    }
}

// Accelerometer
// write byte to I2C Accelerometer
static int WriteByte_ACC(struct i2c_client *client, struct sensor_acc_rw *acc_rw)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write to ACC without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char msg[2];

        msg[0] = acc_rw->address;
        msg[1] = acc_rw->data[0];

        ret = i2c_master_send(client, msg, sizeof(msg));
        if (ret < 0)
            printk (KERN_ERR "WriteByte_ACC() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// read byte(s) from Acceleromter
static int ReadByte_ACC(struct i2c_client *client, struct sensor_acc_rw *acc_rw)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to read from ACC without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_send(client, &acc_rw->address, 1);
        if (ret == 1)
            ret = i2c_master_recv(client, acc_rw->data, acc_rw->count);
        if (ret < 0)
            printk (KERN_ERR "ReadByte_ACC() - i2c_transfer() failed...%d\n",ret);
        return ret;
    }
}

// digital compass
// write byte to digital compass
static int WriteByte_DCOMP(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to write to DCOMP without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;
        unsigned char msg[2];

        msg[0] = offset;
        msg[1] = data;
        ret = i2c_master_send(client, msg, sizeof(msg));

        if (ret < 0)
            printk (KERN_ERR "WriteByte_DCOMP() - i2c_transfer() failed...%d\n",ret);

        return ret;
    }
}

// read byte from digital compass
static int ReadByte_DCOMP(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    if (client == NULL)
    {
        printk(KERN_ERR "Tried to read from DCOMP without i2c client\n");
        return -ENODEV;
    }
    else
    {
        int ret = 0;

        ret = i2c_master_send(client, &offset, 1);
        if (ret == 1)
            ret = i2c_master_recv(client, data, 1);
        if (ret < 0)
            printk (KERN_ERR "ReadByte_DCOMP() - i2c_transfer() failed...%d\n",ret);
        return ret;
    }
}

/*
 *      control device operations
 */

int cntl_open(struct inode *inode, struct file *file)
{
    struct bmi_sensor *sensor = container_of(inode->i_cdev, struct bmi_sensor, cdev);
    file->private_data = sensor;
    return 0;

}

int cntl_release(struct inode *inode, struct file *file)
{
    return 0;
}

// analog proximity timer function
void aptimer(unsigned long arg)
{
    struct bmi_sensor *sensor = (struct bmi_sensor *) arg;
    int ret;

    del_timer (&sensor->aprox_timer);

    // wake sleepers
    ret = down_interruptible(&sensor->sem);
    sensor->aprox_int_en = 0;
    sensor->aprox_int_fl = 1;
    up(&sensor->sem);
    wake_up_all(&sensor->aprox_wait_queue);
}

int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
               unsigned long arg)
{
    unsigned char iox_data;
    struct bmi_sensor *sensor = (struct bmi_sensor *)(file->private_data);
    int slot = sensor->bdev->slot->slotnum;
    int ret = 0;

    // error if sensor not present
    if(sensor->bdev == 0)
        return -ENODEV;

    // ioctl's
    switch(cmd) {
    case BMI_SENSOR_RLEDOFF:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_RED_LED, 1);
        break;

    case BMI_SENSOR_RLEDON:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_RED_LED, 0);
        break;

    case BMI_SENSOR_GLEDOFF:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_GREEN_LED, 1);
        break;

    case BMI_SENSOR_GLEDON:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_GREEN_LED, 0);
        break;

    case BMI_SENSOR_GETSTAT:
    {
        int read_data;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox_data) < 0)
            return -ENODEV;
        read_data = iox_data;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;
        read_data |= (iox_data << 8) | (bmi_slot_gpio_get_all(slot) << 16);

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ADCWR:
    {
        if (!ADC_PRESENT)
            return -ENODEV;

        if (WriteByte_ADC(sensor->adc_i2c_client, (unsigned char) (arg & 0xFF)) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_ADCRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if (!ADC_PRESENT)
            return -ENODEV;
        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_HUMRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if (!HUMIDITY_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_HUMIDITY | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ACOMPRST:
    {
        if(!ACOMP_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        iox_data &= ~(0x1 << SENSOR_IOX_COMP_RS_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;

        mdelay(5);

        iox_data |= (0x1 << SENSOR_IOX_COMP_RS_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_ACOMPXRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(!ACOMP_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_X | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ACOMPYRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(!ACOMP_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Y | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ACOMPZRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(!ACOMP_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Z | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_PLWR:
    {
        struct sensor_pl_rw *pl = NULL;
        unsigned char pl_data;

        if(!PL_PRESENT)
            return -ENODEV;

        if ((pl = kmalloc(sizeof(struct sensor_pl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(pl, (struct sensor_pl_rw *) arg, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        pl_data = pl->cmd1;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->cmd2;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD2, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        kfree(pl);
    }
    break;

    case BMI_SENSOR_PLRD:
    {
        struct sensor_pl_rw *pl = NULL;
        unsigned char pl_data;

        if(!PL_PRESENT)
            return -ENODEV;

        if ((pl = kmalloc(sizeof(struct sensor_pl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->cmd1 = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dm = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dl = pl_data;

        if(copy_to_user((struct sensor_pl_rw *) arg, pl, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        kfree(pl);
    }
    break;

    case BMI_SENSOR_PL_SYNC:
    {
        if(!PL_PRESENT)
            return -ENODEV;

        if(WriteByte_PL_SYNC(sensor->pl_i2c_client) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_PL_IWAIT:
    {
        struct sensor_pl_rw *pl = NULL;
        unsigned char pl_data;

        if(!PL_PRESENT)
            return -ENODEV;

        if ((pl = kmalloc(sizeof(struct sensor_pl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(pl, (struct sensor_pl_rw *) arg, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        pl_data = pl->cmd1;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->cmd2;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD2, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        ret = down_interruptible(&sensor->sem);
        sensor->pl_int_en = 1;
        sensor->pl_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->pl_wait_queue, (sensor->pl_int_fl == 1));
        if(ret)
            return ret;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->cmd1 = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dm = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dl = pl_data;

        if(copy_to_user((struct sensor_pl_rw *) arg, pl, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        kfree(pl);
    }
    break;

    case BMI_SENSOR_SNDARD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(!SOUND_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_AVG | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_SNDPRD:
    case BMI_SENSOR_SNDIRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(!SOUND_PRESENT)
            return -ENODEV;

        // read peak
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        // clear peak
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;

        iox_data &= ~(0x1 << SENSOR_IOX_S_PK_CLR_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        mdelay(1);

        iox_data |= (0x1 << SENSOR_IOX_S_PK_CLR_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        if(cmd == BMI_SENSOR_SNDPRD) {
            // return data
            if(put_user(read_data, (int __user *) arg))
                return -EFAULT;
        } else {

            // read peak
            if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK | SENSOR_ADC_PD_OFF) < 0)
                return -ENODEV;

            mdelay(1);

            if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
                return -ENODEV;
            read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

            // return data
            if(put_user(read_data, (int __user *) arg))
                return -EFAULT;
        }

    }
    break;

    case BMI_SENSOR_TEMPWR:
    {
        struct sensor_temp_rw *temp = NULL;
        unsigned char temp_addr;
        unsigned char temp_data;

        if(!TEMP_PRESENT)
            return -ENODEV;

        if ((temp = kmalloc(sizeof(struct sensor_temp_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(temp, (struct sensor_temp_rw *) arg, sizeof(struct sensor_temp_rw))) {
            kfree(temp);
            return -EFAULT;
        }

        temp_addr = temp->address;
        temp_data = temp->d1;
        if(WriteByte_TEMP(sensor->temp_i2c_client, temp_addr, temp_data) < 0) {
            kfree(temp);
            return -ENODEV;
        }

        kfree(temp);
    }
    break;

    case BMI_SENSOR_TEMPRD:
    {
        struct sensor_temp_rw *temp = NULL;
        unsigned char temp_addr;
        unsigned char temp_data;

        if(!TEMP_PRESENT)
            return -ENODEV;

        if ((temp = kmalloc(sizeof(struct sensor_temp_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(temp, (struct sensor_temp_rw *) arg, sizeof(struct sensor_temp_rw))) {
            kfree(temp);
            return -EFAULT;
        }

        temp_addr = temp->address;
        if(ReadByte_TEMP(sensor->temp_i2c_client, temp_addr, &temp_data) < 0) {
            kfree(temp);
            return -ENODEV;
        }

        temp->d1 = temp_data;
        if(copy_to_user((struct sensor_temp_rw *) arg, temp, sizeof(struct sensor_temp_rw))) {
            kfree(temp);
            return -EFAULT;
        }

        kfree(temp);
    }
    break;

    case BMI_SENSOR_TEMPRD_SL:
    {
        unsigned int read_data;
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(!TEMP_PRESENT)
            return -ENODEV;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_MSB, &temp_datam) < 0) {
            return -ENODEV;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_LSB, &temp_datal) < 0) {
            return -ENODEV;
        }

        read_data = (temp_datam << 8) | temp_datal;
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_TEMPRD_SR:
    {
        unsigned int read_data;
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(!TEMP_PRESENT)
            return -ENODEV;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_MSB, &temp_datam) < 0) {
            return -ENODEV;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_LSB, &temp_datal) < 0) {
            return -ENODEV;
        }

        read_data = (temp_datam << 8) | temp_datal;
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_TEMPRD_UR:
    {
        unsigned int read_data;
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(!TEMP_PRESENT)
            return -ENODEV;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_MSB, &temp_datam) < 0) {
            return -ENODEV;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_LSB, &temp_datal) < 0) {
            return -ENODEV;
        }

        read_data = (temp_datam << 8) | temp_datal;
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_TEMP_IWAIT:
    {
        if(!TEMP_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->temp_int_en = 1;
        sensor->temp_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->temp_wait_queue, (sensor->temp_int_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_MOTRD:
    {
        unsigned int read_data;

        if(!MOTION_PRESENT)
            return -ENODEV;

        read_data = get_mot_det_state(slot);
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_MOT_IWAIT:
    {
        unsigned int read_data;

        if(!MOTION_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->mot_int_en = 1;
        sensor->mot_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->mot_wait_queue, (sensor->mot_int_fl == 1));
        if(ret)
            return ret;

        read_data = get_mot_det_state(slot);
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_ACCWR:
    {
        struct sensor_acc_rw *acc = NULL;

        if (!ACC_PRESENT)
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(acc, (struct sensor_acc_rw *) arg, sizeof(struct sensor_acc_rw))) {
            kfree(acc);
            return -EFAULT;
        }

        if(WriteByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            kfree(acc);
            return -ENODEV;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCRD:
    {
        struct sensor_acc_rw *acc = NULL;

        if(!ACC_PRESENT)
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(acc, (struct sensor_acc_rw *) arg, sizeof(struct sensor_acc_rw))) {
            kfree(acc);
            return -EFAULT;
        }

        if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            kfree(acc);
            return -ENODEV;
        }

        if(copy_to_user((struct sensor_acc_rw *) arg, acc, sizeof(struct sensor_acc_rw))) {
            kfree(acc);
            return -EFAULT;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCXRD:
    {
        struct sensor_acc_rw *acc = NULL;
        unsigned int read_data;

        if(!ACC_PRESENT)
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        acc->address = SENSOR_ACC_DX0;
        acc->count = 2;

        if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            printk(KERN_ERR "ReadByte_ACC failed for ACCXRD\n");
            kfree(acc);
            return -ENODEV;
        }

        read_data = (acc->data[1] << 8) | acc->data[0];
        if(put_user(read_data, (int __user *) arg)) {
            printk(KERN_ERR "XRD failed to put_user\n");
            kfree(acc);
            return -EFAULT;
        }
        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCYRD:
    {
        struct sensor_acc_rw *acc = NULL;
        unsigned int read_data;

        if(!ACC_PRESENT)
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        acc->address = SENSOR_ACC_DY0;
        acc->count = 2;

        if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            printk(KERN_ERR "ReadByte_ACC failed for ACCYRD\n");
            kfree(acc);
            return -ENODEV;
        }

        read_data = (acc->data[1] << 8) | acc->data[0];
        if(put_user(read_data, (int __user *) arg)) {
            printk(KERN_ERR "YRD failed to put_user\n");
            kfree(acc);
            return -EFAULT;
        }
        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCZRD:
    {
        struct sensor_acc_rw *acc = NULL;
        unsigned int read_data;

        if(!ACC_PRESENT)
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        acc->address = SENSOR_ACC_DZ0;
        acc->count = 2;

        if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            printk(KERN_ERR "ReadByte_ACC failed for ACCZRD\n");
            kfree(acc);
            return -ENODEV;
        }

        read_data = (acc->data[1] << 8) | acc->data[0];
        if(put_user(read_data, (int __user *) arg)) {
            printk(KERN_ERR "ZRD failed to put_user\n");
            kfree(acc);
            return -EFAULT;
        }
        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACC_I1WAIT:
    {
        if(!ACC_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->acc_int1_en = 1;
        sensor->acc_int1_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->acc_wait1_queue, (sensor->acc_int1_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_ACC_I2WAIT:
    {
        if(!ACC_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->acc_int2_en = 1;
        sensor->acc_int2_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->acc_wait2_queue, (sensor->acc_int2_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_MOT_IE:
    {
        if(!MOTION_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_MOT_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_MOT_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_HUM_PWR_EN:
    {
        if (!HUMIDITY_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_HUM_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_HUM_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_DCOM_RST:
    {
        if (!DCOMP_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_COMP_RS_N);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_COMP_RS_N);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;

    }
    break;

    case BMI_SENSOR_DCWR:
    {
        struct sensor_rw *dc = NULL;
        unsigned char dc_addr;
        unsigned char dc_data;

        if(!DCOMP_PRESENT)
            return -ENODEV;

        if ((dc = kmalloc(sizeof(struct sensor_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dc, (struct sensor_rw *) arg, sizeof(struct sensor_rw))) {
            kfree(dc);
            return -EFAULT;
        }

        dc_addr = dc->address;
        dc_data = dc->data;
        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, dc_addr, dc_data) < 0) {
            kfree(dc);
            return -ENODEV;
        }

        kfree(dc);
    }
    break;

    case BMI_SENSOR_DCRD:
    {
        struct sensor_rw *dc = NULL;
        unsigned char dc_addr;
        unsigned char dc_data;

        if(!DCOMP_PRESENT)
            return -ENODEV;

        if ((dc = kmalloc(sizeof(struct sensor_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dc, (struct sensor_rw *) arg, sizeof(struct sensor_rw))) {
            kfree(dc);
            return -EFAULT;
        }

        dc_addr = dc->address;
        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, dc_addr, &dc_data) < 0) {
            kfree(dc);
            return -ENODEV;
        }

        dc->data = dc_data;
        if(copy_to_user((struct sensor_rw *) arg, dc, sizeof(struct sensor_rw))) {
            kfree(dc);
            return -EFAULT;
        }

        kfree(dc);
    }
    break;

    case BMI_SENSOR_DC_IWAIT:
    {
        if(!DCOMP_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->dcomp_int_en = 1;
        sensor->dcomp_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->dcomp_wait_queue, (sensor->dcomp_int_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_APROX_DUR:
    {
        if(!APROX_PRESENT)
            return -ENODEV;

        if(arg < 2)
            sensor->aprox_duration = HZ/50;
        else if(arg > 100)
            sensor->aprox_duration = HZ/10;
        else
            sensor->aprox_duration = (HZ/100) * arg;
    }
    break;

    case BMI_SENSOR_APROXRD:
    {
        unsigned char aprox_data[2];
        int read_data;

        if(!APROX_PRESENT)
            return -ENODEV;

        // start burst to LED
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;
        iox_data |= (0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        // set up timer
        sensor->aprox_timer.expires = jiffies + sensor->aprox_duration;
        add_timer (&sensor->aprox_timer);

        // wait for timer
        ret = down_interruptible(&sensor->sem);
        sensor->aprox_int_en = 1;
        sensor->aprox_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->aprox_wait_queue, (sensor->aprox_int_fl == 1));
        if(ret)
            return ret;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;

        // digital output
        read_data = (iox_data & (0x1 << SENSOR_IOX_PROX_OUT)) << 14;

        // read ADC - analog output
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_APROXIMITY | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, aprox_data) < 0)
            return -ENODEV;
        read_data |= (aprox_data[0] << 8) | aprox_data[1];

        // stop burst to LED
        iox_data &= ~(0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ALIGHTRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(!ALIGHT_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_LIGHT | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_DLIGHTWR:
    {
        struct sensor_dl_rw *dl = NULL;
        unsigned char dl_data;

        if(!DL_PRESENT)
            return -ENODEV;

        if ((dl = kmalloc(sizeof(struct sensor_dl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dl, (struct sensor_dl_rw *) arg, sizeof(struct sensor_dl_rw))) {
            kfree(dl);
            return -EFAULT;
        }

        dl_data = dl->cmd;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CMD, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->control;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_thi;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_THI, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_tlo;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_TLO, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        kfree(dl);
    }
    break;

    case BMI_SENSOR_DLIGHTRD:
    {
        unsigned char dl_data;
        unsigned int read_data;

        if(!DL_PRESENT)
            return -ENODEV;

        // read sensor data
        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data = ((unsigned int) (dl_data)) << 8;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_LSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data |= dl_data;

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_DLIGHT_IC:
    {
        if(!DL_PRESENT)
            return -ENODEV;

        if(WriteByte_DL_IC(sensor->pl_i2c_client) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_DLIGHT_IWAIT:
    {
        struct sensor_dl_rw *dl = NULL;
        unsigned char dl_data;
        unsigned int read_data;

        if(!DL_PRESENT)
            return -ENODEV;

        // write all register
        if ((dl = kmalloc(sizeof(struct sensor_dl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dl, (struct sensor_dl_rw *) arg, sizeof(struct sensor_dl_rw))) {
            kfree(dl);
            return -EFAULT;
        }

        dl_data = dl->cmd;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CMD, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->control;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_thi;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_THI, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_tlo;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_TLO, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        // enable interrupt
        ret = down_interruptible(&sensor->sem);
        sensor->dlight_int_en = 1;
        sensor->dlight_int_fl = 0;
        up(&sensor->sem);
        // wait on interrupt
        ret = wait_event_interruptible(sensor->dlight_wait_queue, (sensor->dlight_int_fl == 1));
        if(ret)
            return ret;

        // read sensor data
        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data = ((unsigned int) (dl_data)) << 8;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_LSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data |= dl_data;
        dl->sensor_data = read_data;

        if(copy_to_user((struct sensor_dl_rw *) arg, dl, sizeof(struct sensor_dl_rw))) {
            kfree(dl);
            return -EFAULT;
        }

        kfree(dl);
    }
    break;

    case BMI_SENSOR_MIC_EN:
    {
        if(!SOUND_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_MIC_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_MIC_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_PRESENCE:
    {
        struct sensor_presence *p = NULL;

        if ((p = kmalloc(sizeof(*p), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        p->adc = ADC_PRESENT ? 1 : 0;
        p->acc = ACC_PRESENT ? 1 : 0;
        p->pl = PL_PRESENT ? 1 : 0;
        p->dl = DL_PRESENT ? 1 : 0;
        p->temp = TEMP_PRESENT ? 1 : 0;
        p->motion = MOTION_PRESENT ? 1 : 0;
        p->dcomp = DCOMP_PRESENT ? 1 : 0;
        p->acomp = ACOMP_PRESENT ? 1 : 0;
        p->alight = ALIGHT_PRESENT ? 1 : 0;
        p->aprox = APROX_PRESENT ? 1 : 0;
        p->humidity = HUMIDITY_PRESENT ? 1 : 0;
        p->sound = SOUND_PRESENT ? 1 : 0;

        if (copy_to_user((struct sensor_presence *) arg, p, sizeof(*p))) {
            kfree(p);
            return -EFAULT;
        }
        kfree(p);
    }
    break;

    default:
        return -ENOTTY;
    }

    return 0;
} // ioctl

// control file operations
struct file_operations cntl_fops = {
    .owner = THIS_MODULE,
    .ioctl = cntl_ioctl,
    .open = cntl_open,
    .release = cntl_release,
};

/*
 *      PIM functions
 */

// interrupt handler
static void sensor_work_handler(struct work_struct * work)
{
    struct bmi_sensor *sensor = work_to_sensor(work);
    int slot = sensor->bdev->slot->slotnum;
    unsigned char iox0;
    unsigned char iox1;
    unsigned char i2c_dummy;
    struct sensor_acc_rw acc_rw;

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: sensor_work_handler - IOX0 error\n");
        return;
    }

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox1) < 0) {
        printk(KERN_ERR "bmi_sensor.c: sensor_work_handler - IOX1 error\n");
        return;
    }

    if(PL_PRESENT && sensor->pl_int_en) {
        if((iox1 & (0x1 << SENSOR_IOX_PL_INT)) == 0) {
            sensor->pl_int_en = 0;
            sensor->pl_int_fl = 1;
            // clear interrupts
            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, &i2c_dummy) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read error\n");
            }
            wake_up_all(&sensor->pl_wait_queue);
        }
    }

    if(TEMP_PRESENT && sensor->temp_int_en) {
        if((iox1 & (0x1 << SENSOR_IOX_TEMP_INT)) == 0) {
            sensor->temp_int_en = 0;
            sensor->temp_int_fl = 1;
            // disable interrupts
            if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
                printk(KERN_ERR "bmi_sensor.c: TEMP write error\n");
            }
            wake_up_all(&sensor->temp_wait_queue);
        }
    }

    if(MOTION_PRESENT && sensor->mot_int_en) {
        if(sensor->mot_state != get_mot_det_state(slot)) {
            sensor->mot_state = get_mot_det_state(slot);
            sensor->mot_int_en = 0;
            sensor->mot_int_fl = 1;
            wake_up_all(&sensor->mot_wait_queue);
        }
    }

    if(ACC_PRESENT) {
        if(sensor->acc_int1_en) {
            if((iox0 & (0x1 << SENSOR_IOX_ACC_INT1)) == 0) {
                sensor->acc_int1_en = 0;
                sensor->acc_int1_fl = 1;
                wake_up_all(&sensor->acc_wait1_queue);
            }
        }

        if(sensor->acc_int2_en) {
            if((iox0 & (0x1 << SENSOR_IOX_ACC_INT2)) == 0) {
                sensor->acc_int2_en = 0;
                sensor->acc_int2_fl = 1;
                wake_up_all(&sensor->acc_wait2_queue);
            }
        }

        // clear interrupts
        acc_rw.address = SENSOR_ACC_IS;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC_IS read error\n");
        }

        acc_rw.address = SENSOR_ACC_DX0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DX0) error\n");
        }

        acc_rw.address = SENSOR_ACC_DY0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DY0) error\n");
        }

        acc_rw.address = SENSOR_ACC_DZ0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DZ0) error\n");
        }
    }

    if(DCOMP_PRESENT &&  sensor->dcomp_int_en) {
        if((iox1 & (0x1 << SENSOR_IOX_DCOMP_INT)) != 0) {
            sensor->dcomp_int_en = 0;
            sensor->dcomp_int_fl = 1;
            // clear interrupts
            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_TMPS, &i2c_dummy) < 0) {
                printk(KERN_ERR "bmi_sensor.c: TMPS error\n");
            }

            wake_up_all(&sensor->dcomp_wait_queue);
        }
    }

    if(DL_PRESENT && sensor->dlight_int_en) {
        if((iox1 & (0x1 << SENSOR_IOX_PL_INT)) == 0) {
            sensor->dlight_int_en = 0;
            sensor->dlight_int_fl = 1;
            // clear interrupts
            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, &i2c_dummy) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DL read error\n");
            }
            i2c_dummy &= ~(SENSOR_DL_CONT_INT);
            if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, i2c_dummy) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DL write error\n");
            }
            if(WriteByte_DL_IC(sensor->pl_i2c_client) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DL interrupt clear error\n");
            }
            wake_up_all(&sensor->pl_wait_queue);
        }
    }
}

/*
 *      BMI functions
 */

/*-------------------------------------
 *
 *      bmi device sysfs attributes
 *
 *-------------------------------------
 */

static ssize_t show_humidity(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_HUMIDITY) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(humidity, S_IRUGO, show_humidity, NULL);

static ssize_t show_acompass(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];
    unsigned int compass_x;
    unsigned int compass_y;
    unsigned int compass_z;

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    compass_x = (adc_data[0] << 8) | adc_data[1];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Y) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    compass_y = (adc_data[0] << 8) | adc_data[1];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Z) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    compass_z = (adc_data[0] << 8) | adc_data[1];

    return sprintf(buf, "X=0x%x\nY=0x%x\nZ=0x%x\n",
                   compass_x, compass_y, compass_z);
}
static DEVICE_ATTR(acompass, S_IRUGO, show_acompass, NULL);

static ssize_t show_dcompass(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char compass_i;
    unsigned char compass_t;
    unsigned char compass_x;
    unsigned char compass_y;
    unsigned char compass_z;

    if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_SENSOR) < 0) {
        printk(KERN_ERR "bmi_sensor.c: DCOMP MS1 write error\n");
    }

    mdelay(20);

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_ST, &compass_i) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    if((compass_i & SENSOR_DCOMP_ST_INT) == 0) {
        printk(KERN_ERR "bmi_sensor.c: DCOMP interrupt error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_TMPS, &compass_t) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TMPS error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1X, &compass_x) < 0) {
        printk(KERN_ERR "bmi_sensor.c: H1X error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Y, &compass_y) < 0) {
        printk(KERN_ERR "bmi_sensor.c: H1Y error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Z, &compass_z) < 0) {
        printk(KERN_ERR "bmi_sensor.c: H1Z error\n");
    }

    return sprintf(buf, "T=0x%x\nX=0x%x\nY=0x%x\nZ=0x%x\n",
                   compass_t, compass_x, compass_y, compass_z);
}
static DEVICE_ATTR(dcompass, S_IRUGO, show_dcompass, NULL);

static ssize_t show_als(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char pl_data[2];

    if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_ALS_1X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL write error\n");
    }

    mdelay(20);

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    return sprintf(buf, "0x%x\n", (pl_data[0] << 8) | pl_data[1]);
}
static DEVICE_ATTR(als, S_IRUGO, show_als, NULL);

static ssize_t show_ir(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char pl_data[2];

    if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_IR_1X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL write error\n");
    }

    mdelay(20);

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    return sprintf(buf, "0x%x\n", (pl_data[0] << 8) | pl_data[1]);
}
static DEVICE_ATTR(ir, S_IRUGO, show_ir, NULL);

static ssize_t show_proximity(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char pl_data[2];

    if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_PROX_1X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL write error\n");
    }

    mdelay(20);

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    return sprintf(buf, "0x%x\n", (pl_data[0] << 8) | pl_data[1]);
}
static DEVICE_ATTR(proximity, S_IRUGO, show_proximity, NULL);

static ssize_t show_snda(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_AVG) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(sound_avg, S_IRUGO, show_snda, NULL);

static ssize_t show_sndp(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(sound_peak, S_IRUGO, show_sndp, NULL);

static ssize_t show_temp_l(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char temp_datam;
    unsigned char temp_datal;

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    mdelay(400);

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_MSB, &temp_datam) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL MSB) error\n");
    }

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_LSB, &temp_datal) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL LSB) error\n");
    }

    return sprintf(buf, "0x%x\n", (temp_datam << 8) | temp_datal);
}
static DEVICE_ATTR(temp_local, S_IRUGO, show_temp_l, NULL);

static ssize_t show_temp_sr(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char temp_datam;
    unsigned char temp_datal;

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    mdelay(400);

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_MSB, &temp_datam) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (REM MSB) error\n");
    }

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_LSB, &temp_datal) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (REM LSB) error\n");
    }

    return sprintf(buf, "0x%x\n", (temp_datam << 8) | temp_datal);
}
static DEVICE_ATTR(temp_sremote, S_IRUGO, show_temp_sr, NULL);

static ssize_t show_temp_ur(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char temp_datam;
    unsigned char temp_datal;

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    mdelay(400);

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_MSB, &temp_datam) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM MSB) error\n");
    }

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_LSB, &temp_datal) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM LSB) error\n");
    }

    return sprintf(buf, "0x%x\n", (temp_datam << 8) | temp_datal);
}
static DEVICE_ATTR(temp_uremote, S_IRUGO, show_temp_ur, NULL);

static ssize_t show_motion(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    return sprintf(buf, "0x%x\n", get_mot_det_state(bdev->slot->slotnum));
}
static DEVICE_ATTR(motion, S_IRUGO, show_motion, NULL);

static ssize_t show_accel(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];

    struct sensor_acc_rw acc_rw;
    int x;
    int y;
    int z;

    acc_rw.address = SENSOR_ACC_DX0;
    acc_rw.count = 2;
    if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ACC read (DX0) error\n");
    }
    x = (acc_rw.data[1] << 8) | acc_rw.data[0];

    acc_rw.address = SENSOR_ACC_DY0;
    acc_rw.count = 2;
    if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ACC read (DY0) error\n");
    }
    y = (acc_rw.data[1] << 8) | acc_rw.data[0];

    acc_rw.address = SENSOR_ACC_DZ0;
    acc_rw.count = 2;
    if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ACC read (DZ0) error\n");
    }
    z = (acc_rw.data[1] << 8) | acc_rw.data[0];

    return sprintf(buf, "X=0x%x\nY=0x%x\nZ=0x%x\n", x, y, z);
}
static DEVICE_ATTR(accel, S_IRUGO, show_accel, NULL);

static ssize_t show_aprox(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned int read_data;
    unsigned char iox_data;
    unsigned char aprox_data;
    int ret;

    // start burst to LED
    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX read error\n");
    iox_data |= (0x1 << SENSOR_IOX_PROX_RST_N);
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX write error\n");

    // set up timer
    sensor->aprox_timer.expires = jiffies + sensor->aprox_duration;
    add_timer (&sensor->aprox_timer);

    // wait for timer
    ret = down_interruptible(&sensor->sem);
    sensor->aprox_int_en = 1;
    sensor->aprox_int_fl = 0;
    up(&sensor->sem);
    wait_event_interruptible(sensor->aprox_wait_queue, (sensor->aprox_int_fl == 1));

    // stop burst to LED
    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX read error\n");
    iox_data &= ~(0x1 << SENSOR_IOX_PROX_RST_N);
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX write error\n");

    // digital output
    read_data = (iox_data & (0x1 << SENSOR_IOX_PROX_OUT)) << 14;

    // read ADC - analog output
    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_APROXIMITY | SENSOR_ADC_PD_OFF) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX write error\n");

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, &aprox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX read error\n");
    read_data |= aprox_data;

    return sprintf(buf, "Analog Proxiimity = 0x%x\n", read_data);
}
static DEVICE_ATTR(aprox, S_IRUGO, show_aprox, NULL);

static ssize_t show_alight(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_LIGHT) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(alight, S_IRUGO, show_alight, NULL);

static ssize_t show_dlight(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char dl_data[2];

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: DL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_LSB, &dl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: DL read error\n");
    }

    return sprintf(buf, "0x%x\n", (dl_data[0] << 8) | dl_data[1]);
}
static DEVICE_ATTR(dlight, S_IRUGO, show_dlight, NULL);


static void cleanup_i2c_devices(struct bmi_sensor *sensor)
{
    if (sensor->iox_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->iox_i2c_client);
        sensor->iox_i2c_client = NULL;
    }

    if (sensor->adc_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->adc_i2c_client);
        sensor->adc_i2c_client = NULL;
    }

    if (sensor->pl_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->pl_i2c_client);
        sensor->pl_i2c_client = NULL;
    }

    if (sensor->temp_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->temp_i2c_client);
        sensor->temp_i2c_client = NULL;
    }

    if (sensor->acc_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->acc_i2c_client);
        sensor->acc_i2c_client = NULL;
    }

    if (sensor->dcomp_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->dcomp_i2c_client);
        sensor->dcomp_i2c_client = NULL;
    }
}

// probe - insert PIM
int bmi_sensor_probe(struct bmi_device *bdev)
{
    int err = 0;
    int slot = bdev->slot->slotnum;
    struct bmi_sensor *sensor = &bmi_sensor[slot];
    struct cdev *cdev;
    struct class *bmi_class;
    dev_t dev_id;
    unsigned char iox_data;

    sensor->bdev = 0;

    // Create 1 minor device
    cdev = &sensor->cdev;
    cdev_init(cdev, &cntl_fops);

    dev_id = MKDEV(major, slot);
    err = cdev_add(cdev, dev_id, 1);
    if(err) {
        printk(KERN_ERR "bmi_sensor cdev_add failed with %d\n", err);
        return err;
    }

    // Create class device
    bmi_class = bmi_get_class();
    sensor->class_dev = device_create(bmi_class, NULL,
                                      MKDEV(major, slot), NULL,
                                      "bmi_sensor_cntl_m%i", slot+1);

    if(IS_ERR(sensor->class_dev)) {
        printk(KERN_ERR "Unable to create "
               "class_device for bmi_sensor_cntl_m%i; errno = %ld\n",
               slot+1, PTR_ERR(sensor->class_dev));
        goto error;
    }

    // bind driver and bmi_device
    sensor->bdev = bdev;

#if defined(PROBE_IOX)
    sensor->iox_i2c_client = i2c_new_probed_device(bdev->slot->adap, &iox_info, iox_addresses);
#else
    sensor->iox_i2c_client = i2c_new_device(bdev->slot->adap, &iox_info);
#endif
    if (sensor->iox_i2c_client != NULL)
    {
#if defined(PROBE_IOX)
        printk(KERN_INFO "Found IOX\n");
#else
        printk(KERN_INFO "Presuming IOX is present\n");
#endif
    }
    else
    {
        printk(KERN_ERR "IOX not found\n");
        // we're screwed without the IOX
        goto error;
    }

    bmi_device_set_drvdata(bdev, sensor);

    printk(KERN_INFO "bmi_sensor.c: probe slot %d\n", slot);

    // configure IOX
    // USB/HUM/MOT_INT off, COMPASS RST High
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, 0x80) < 0) {
        printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_OUTPUT0_REG to 0x80\n", slot);
        goto error;
    }

    // Speaker Peak Clear/analog proximity enable high, all other outputs low
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, 0x0A) < 0) {
        printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d settng IOX_OUTPUT1_REG to 0x0A\n", slot);
        goto error;
    }

    // IOX[5,2:0]=IN, IOX[7:6,4:3]=OUT
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_CONTROL0_REG, 0x27) < 0) {
        printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_CONTROL0_REG to 0x27\n", slot);
        goto error;
    }

    // IOX[7,5:4,2]=IN, IOX[6,3,1:0]=OUT
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_CONTROL1_REG, 0xb4) < 0) {
        printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_CONTROL1_REG to 0xb4\n", slot);
        goto error;
    }

    // Initialize GPIOs (turn LED's on)
    printk(KERN_INFO "bmi_sensor: Configure LEDs and turn them on");
    bmi_slot_gpio_direction_out(slot, SENSOR_GPIO_RED_LED, 0);
    bmi_slot_gpio_direction_out(slot, SENSOR_GPIO_GREEN_LED, 0);
    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_PDOUT);
    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_MOT_DET);

    mdelay(200);

    // turn LED's off
    printk(KERN_INFO "bmi_sensor: Turn LEDs back off");
    bmi_slot_gpio_set_value(slot, SENSOR_GPIO_RED_LED, 1);
    bmi_slot_gpio_set_value(slot, SENSOR_GPIO_GREEN_LED, 1);

    // bmi_sensor initialization
    init_MUTEX(&sensor->sem);

    init_waitqueue_head(&sensor->pl_wait_queue);
    sensor->pl_int_en = 0;
    sensor->pl_int_fl = 0;

    init_waitqueue_head(&sensor->temp_wait_queue);
    sensor->temp_int_en = 0;
    sensor->temp_int_fl = 0;

    init_waitqueue_head(&sensor->mot_wait_queue);
    sensor->mot_int_en = 0;
    sensor->mot_int_fl = 0;
    sensor->mot_state = get_mot_det_state(slot);

    init_waitqueue_head(&sensor->acc_wait1_queue);
    sensor->acc_int1_en = 0;
    sensor->acc_int1_fl = 0;

    init_waitqueue_head(&sensor->acc_wait2_queue);
    sensor->acc_int2_en = 0;
    sensor->acc_int2_fl = 0;

    init_waitqueue_head(&sensor->dcomp_wait_queue);
    sensor->dcomp_int_en = 0;
    sensor->dcomp_int_fl = 0;

    sensor->aprox_duration = 200;
    init_timer(&sensor->aprox_timer);
    sensor->aprox_timer.data = (unsigned long) &bmi_sensor[slot];
    sensor->aprox_timer.function = aptimer;
    init_waitqueue_head(&sensor->aprox_wait_queue);
    sensor->aprox_int_en = 0;
    sensor->aprox_int_fl = 0;

    sprintf(sensor->workqueue_name, "sensor_m%d", slot + 1);
    sensor->workqueue = create_singlethread_workqueue(sensor->workqueue_name);
    if (!sensor->workqueue) {
        printk(KERN_ERR "bmi_sensor.c: Can't create_singlethread_workqueue() in slot %d\n",
               slot);
        goto error;
    }
    INIT_WORK(&sensor->work_item, sensor_work_handler);

    sensor->adc_i2c_client = i2c_new_probed_device(bdev->slot->adap, &adc_info, adc_addresses);
    if (sensor->adc_i2c_client == NULL)
    {
        printk(KERN_INFO "ADC not found\n");
    }
    else
    {
        unsigned char adc_data[2];

        printk(KERN_INFO "Found ADC\n");

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_CH0) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto error;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            goto error;
        }

        if((adc_data[0] & 0xF0) != 0x0) {
            printk(KERN_ERR "bmi_sensor.c: ADC compare error\n");
            goto error;
        }
    }

    if (HUMIDITY_PRESENT) {
        unsigned char adc_data[2];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_HUMIDITY) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Humidity) error\n");
            goto error;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Humidity) error\n");
            goto error;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Humidity = %d\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_humidity)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (humidity) failed.\n",
                    slot);
            goto sysfs_err1;
        }
    }

    if (ACOMP_PRESENT) {
        unsigned char adc_data[2];
        unsigned int compass_x;
        unsigned int compass_y;
        unsigned int compass_z;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_X) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto sysfs_err1;

        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            device_remove_file(&sensor->bdev->dev, &dev_attr_humidity);
            goto sysfs_err1;
        }
        compass_x = (adc_data[0] << 8) | adc_data[1];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Y) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto sysfs_err1;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            goto sysfs_err1;
        }
        compass_y = (adc_data[0] << 8) | adc_data[1];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Z) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto sysfs_err1;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            goto sysfs_err1;
        }
        compass_z = (adc_data[0] << 8) | adc_data[1];

        printk(KERN_INFO "bmi_sensor.c: initial COMPASS (X,Y,Z) = %d,%d,%d\n",
               compass_x, compass_y, compass_z);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_acompass)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (acompass) failed.\n",
                    slot);
            goto sysfs_err1;
        }
    }

    // try and initialize digital compass
    {
        unsigned char hxga;
        unsigned char hyga;
        unsigned char hzga;
        unsigned char compass_i;
        unsigned char compass_t;
        unsigned char compass_x;
        unsigned char compass_y;
        unsigned char compass_z;

        sensor->dcomp_i2c_client = i2c_new_probed_device(bdev->slot->adap, &dcomp_info, dcomp_addresses);
        if (sensor->dcomp_i2c_client == NULL)
        {
            printk(KERN_INFO "DCOMP not found\n");
        }
        else
        {
            printk(KERN_INFO "Found DCOMP\n");

            if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_EEPROM) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_MS1 write error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_EE_EHXGA, &hxga) < 0) {
                printk(KERN_ERR "bmi_sensor.c: EE_EHXGA read error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_EE_EHYGA, &hyga) < 0) {
                printk(KERN_ERR "bmi_sensor.c: EE_EHYGA read error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_EE_EHZGA, &hzga) < 0) {
                printk(KERN_ERR "bmi_sensor.c: EE_EHZGA read error\n");
                goto sysfs_err2;
            }

            if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_PD) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_MS1 write error\n");
                goto sysfs_err2;
            }

            if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HXGA, hxga) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_HXGA write error\n");
                goto sysfs_err2;
            }

            if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HYGA, hyga) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_HYGA write error\n");
                goto sysfs_err2;
            }

            if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HZGA, hzga) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_HZGA write error\n");
                goto sysfs_err2;
            }

            if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_SENSOR) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_MS1 write error\n");
                goto sysfs_err2;

            }

            mdelay(20);

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_ST, &compass_i) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_ST read error\n");
                goto sysfs_err2;
            }

            if((compass_i & SENSOR_DCOMP_ST_INT) == 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP interrupt error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_TMPS, &compass_t) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_TMPS error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1X, &compass_x) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_H1X error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Y, &compass_y) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_H1Y error\n");
                goto sysfs_err2;
            }

            if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Z, &compass_z) < 0) {
                printk(KERN_ERR "bmi_sensor.c: DCOMP_H1Z error\n");
                goto sysfs_err2;
            }

            printk(KERN_INFO "bmi_sensor.c: initial COMPASS (T,X,Y,Z) = 0x%x,0x%x,0x%x,0x%x\n",
                   compass_t, compass_x, compass_y, compass_z);

            if(device_create_file(&sensor->bdev->dev, &dev_attr_dcompass)) {
                printk (KERN_ERR
                        "bmi_sensor.c (%d): attr (dcompass) failed.\n",
                        slot);
                goto sysfs_err2;
            }
        }
    }

    // try and initialized proximity / light sensor
    {
        unsigned char pl_data[2];

        sensor->pl_i2c_client = i2c_new_probed_device(bdev->slot->adap, &pl_info, pl_addresses);
        if (sensor->pl_i2c_client == NULL)
        {
            printk(KERN_INFO "PL not found\n");
        }
        else
        {
            printk(KERN_INFO "PL found\n");

            if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_ALS_1X) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL write (ALS) error\n");
                goto sysfs_err3;
            }

            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read (ALS) error\n");
                goto sysfs_err3;
            }

            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[1]) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read (ALS) error\n");
                goto sysfs_err3;
            }
            printk(KERN_INFO "bmi_sensor.c: initial PL ALS = %d\n",
                   (pl_data[0] << 8) | pl_data[1]);

            if(device_create_file(&sensor->bdev->dev, &dev_attr_als)) {
                printk (KERN_ERR
                        "bmi_sensor.c (%d): attr (ALS) failed.\n",
                        slot);
                goto sysfs_err3;
            }

            if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_IR_1X) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL write (IR) error\n");
                goto sysfs_err4;
            }

            mdelay(20);

            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read (IR) error\n");
                goto sysfs_err4;
            }

            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[1]) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read (IR) error\n");
                goto sysfs_err4;
            }
            printk(KERN_INFO "bmi_sensor.c: initial IR = %d\n",
                   (pl_data[0] << 8) | pl_data[1]);

            if(device_create_file(&sensor->bdev->dev, &dev_attr_ir)) {
                printk (KERN_ERR
                        "bmi_sensor.c (%d): attr (IR) failed.\n",
                        slot);
                goto sysfs_err4;
            }

            if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_PROX_1X) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL write (Proximity) error\n");
                goto sysfs_err5;
            }

            mdelay(20);

            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read (Proximity) error\n");
                goto sysfs_err5;
            }

            if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[1]) < 0) {
                printk(KERN_ERR "bmi_sensor.c: PL read (Proximity) error\n");
                goto sysfs_err5;
            }
            printk(KERN_INFO "bmi_sensor.c: initial Proximity = %d\n",
                   (pl_data[0] << 8) | pl_data[1]);

            if(device_create_file(&sensor->bdev->dev, &dev_attr_proximity)) {
                printk (KERN_ERR
                        "bmi_sensor.c (%d): attr (Proximity) failed.\n",
                        slot);
                goto sysfs_err5;
            }
        }
    }

    if (SOUND_PRESENT)
    {
        unsigned char adc_data[2];
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_AVG) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Sound Average) error\n");
            goto sysfs_err6;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Sound Average) error\n");
            goto sysfs_err6;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Sound Average = %d\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_sound_avg)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Sound Average) failed.\n",
                    slot);
            goto sysfs_err6;
        }

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Sound Peak) error\n");
            goto sysfs_err7;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Sound Peak) error\n");
            goto sysfs_err7;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Sound Peak = %d\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_sound_peak)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Sound Peak) failed.\n",
                    slot);
            goto sysfs_err7;
        }
    }

    // try and initialize temperature sensor
    sensor->temp_i2c_client = i2c_new_probed_device(bdev->slot->adap, &temp_info, temp_addresses);
    if (sensor->temp_i2c_client == NULL)
    {
        printk(KERN_INFO "TEMP not found\n");
    }
    else
    {
        unsigned char temp_datam;
        unsigned char temp_datal;
        printk(KERN_INFO "TEMP found\n");

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_MAN_ID, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (Manufacturer ID) error\n");
            goto sysfs_err8;
        }

        if(temp_datam != SENSOR_TEMP_MAN_ID_DATA) {
            printk(KERN_ERR "bmi_sensor.c: TEMP MAN ID error (read=0x%x, expected=0x%x\n",
                   temp_datam, SENSOR_TEMP_MAN_ID_DATA);
            goto sysfs_err8;
        }

        printk(KERN_INFO "bmi_sensor.c: TEMP Manufacturer ID = 0x%x\n", temp_datam);

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REV_ID, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (Revision ID) error\n");
            goto sysfs_err8;
        }

        if(temp_datam != SENSOR_TEMP_REV_ID_DATA) {
            printk(KERN_ERR "bmi_sensor.c: TEMP REV ID error (read=0x%x, expected=0x%x\n",
                   temp_datam, SENSOR_TEMP_REV_ID_DATA);
            goto sysfs_err8;
        }

        printk(KERN_INFO "bmi_sensor.c: TEMP Revision ID = 0x%x\n", temp_datam);

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF2, 0x0) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF2) error\n");
            goto sysfs_err8;
        }

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
            goto sysfs_err8;
        }

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONV_WR, SENSOR_TEMP_CONV_P364) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
            goto sysfs_err8;
        }

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
            goto sysfs_err8;
        }

        mdelay(400);

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_MSB, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL MSB) error\n");
            goto sysfs_err8;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_LSB, &temp_datal) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL LSB) error\n");
            goto sysfs_err8;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Local temperature = 0x%x\n",
               (temp_datam << 8) | temp_datal);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_temp_local)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (TEMP local) failed.\n",
                    slot);
            goto sysfs_err8;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_MSB, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (REM MSB) error\n");
            goto sysfs_err9;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_LSB, &temp_datal) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (REM LSB) error\n");
            goto sysfs_err9;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Remote temperature = 0x%x\n",
               (temp_datam << 8) | temp_datal);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_temp_sremote)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (TEMP sremote) failed.\n",
                    slot);
            goto sysfs_err9;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_MSB, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM MSB) error\n");
            goto sysfs_err10;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_LSB, &temp_datal) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM LSB) error\n");
            goto sysfs_err10;
        }

        if(device_create_file(&sensor->bdev->dev, &dev_attr_temp_uremote)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (TEMP uremote) failed.\n",
                    slot);
            goto sysfs_err10;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Remote temperature (unsigned) = 0x%x\n",
               (temp_datam << 8) | temp_datal);
    }

    if(MOTION_PRESENT) {
        if(device_create_file(&sensor->bdev->dev, &dev_attr_motion)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (motion) failed.\n",
                    slot);
            goto sysfs_err11;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Motion state = 0x%x\n",
               get_mot_det_state(slot));
    }

    // try and initialize accelerometer
    sensor->acc_i2c_client = i2c_new_probed_device(bdev->slot->adap, &acc_info, acc_addresses);
    if (sensor->acc_i2c_client == NULL)
    {
        printk(KERN_INFO "ACC not found\n");
    }
    else
    {
        struct sensor_acc_rw acc_rw;

        printk(KERN_INFO "ACC found\n");

        acc_rw.address = SENSOR_ACC_ID;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (ID) error\n");
            goto sysfs_err12;
        }

        if(acc_rw.data[0] != SENSOR_ACC_ID_DATA) {
            printk(KERN_ERR "bmi_sensor.c: ACC ID error (read=0x%x, expected=0x%x)\n",
                   acc_rw.data[0], SENSOR_ACC_ID_DATA);
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_ACC_RATE;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_ACC_RC_3200_1600;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC write (RATE) error\n");
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_ACC_POWER;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_ACC_P_NORM;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC write (RATE) error\n");
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_ACC_DF;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_ACC_DF_LENGTH;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC write (DF) error\n");
            goto sysfs_err12;
        }

        mdelay(20);

        acc_rw.address = SENSOR_ACC_DX0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DX0) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC X state = 0x%x\n",
               (acc_rw.data[1] << 8) | acc_rw.data[0]);

        acc_rw.address = SENSOR_ACC_DY0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DY0) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC Y state = 0x%x\n",
               (acc_rw.data[1] << 8) | acc_rw.data[0]);

        acc_rw.address = SENSOR_ACC_DZ0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DZ0) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC Z state = 0x%x\n",
               (acc_rw.data[1] << 8) | acc_rw.data[0]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_accel)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (accel) failed.\n",
                    slot);
            goto sysfs_err12;
        }
    }

    if (APROX_PRESENT) {
        unsigned char aprox_data;
        unsigned int read_data;
        int ret;

        // enable sensor
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d\n", slot);
            goto sysfs_err13;
        }
        iox_data &= ~(0x1 << SENSOR_IOX_PROX_EN_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d\n", slot);
            goto sysfs_err13;
        }

        // start burst to LED
        iox_data |= (0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            goto sysfs_err13;

        // set up timer
        sensor->aprox_timer.expires = jiffies + sensor->aprox_duration;
        add_timer (&sensor->aprox_timer);

        // wait for timer
        ret = down_interruptible(&sensor->sem);
        sensor->aprox_int_en = 1;
        sensor->aprox_int_fl = 0;
        up(&sensor->sem);
        wait_event_interruptible(sensor->aprox_wait_queue, (sensor->aprox_int_fl == 1));

        // stop burst to LED
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            goto sysfs_err13;
        iox_data &= ~(0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            goto sysfs_err13;

        // digital output
        read_data = (iox_data & (0x1 << SENSOR_IOX_PROX_OUT)) << 14;

        // read ADC - analog output
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_APROXIMITY | SENSOR_ADC_PD_OFF) < 0)
            goto sysfs_err13;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, &aprox_data) < 0)
            goto sysfs_err13;
        read_data |= aprox_data;

        printk(KERN_INFO "bmi_sensor.c: initial analog proximity = 0x%x\n", read_data);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_aprox)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (aprox) failed.\n",
                    slot);
            goto sysfs_err13;
        }
    }

    if (ALIGHT_PRESENT) {
        unsigned char adc_data[2];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_LIGHT) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Analog Light) error\n");
            goto sysfs_err14;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Humidity) error\n");
            goto sysfs_err14;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Analog Light = 0x%x\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_alight)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (alight) failed.\n",
                    slot);
            goto sysfs_err14;
        }
    }

    if(DL_PRESENT) {
        unsigned char dl_data[2];

        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CMD, SENSOR_DL_CMD_ADC_EN) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL write (Digital Light) error\n");
            goto sysfs_err15;
        }

        mdelay(20);

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Digital Light) error\n");
            goto sysfs_err15;
        }

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data[1]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Digital Light) error\n");
            goto sysfs_err15;
        }
        printk(KERN_INFO "bmi_sensor.c: initial Digital Light = %d\n",
               (dl_data[0] << 8) | dl_data[1]);

        // clear interrupts
        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, &dl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DL read error\n");
        }
        dl_data[0] &= ~(SENSOR_DL_CONT_INT);
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, dl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DL write error\n");
        }
        if(WriteByte_DL_IC(sensor->pl_i2c_client) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DL interrupt clear error\n");
        }

        if(device_create_file(&sensor->bdev->dev, &dev_attr_dlight)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Digital Light) failed.\n",
                    slot);
            goto sysfs_err15;
        }
    }

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox_data) < 0) { // clear IOX interrupts
        printk (KERN_ERR
                "bmi_sensor.c(%d): IOX0 interrupt clear fail.\n",
                slot);
        goto sysfs_err16;
    }

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0) { // clear IOX interrupts
        printk (KERN_ERR
                "bmi_sensor.c(%d): IOX1 interrupt clear fail.\n",
                slot);
        goto sysfs_err16;
    }

#if defined(IRQ_STUFF_IMPLEMENTED)
    // request PIM interrupt
    {
        int irq = bdev->slot->status_irq;
        sprintf(sensor->int_name, "bmi_sensor%d", slot);
        if(request_irq(irq, &module_irq_handler, 0, sensor->int_name, sensor)) {
            printk(KERN_ERR "bmi_sensor.c: Can't allocate irq %d or find Sensor in slot %d\n",
                   irq, slot);
            goto sysfs_err16;
        }
    }
#endif

    return 0;

sysfs_err16:
    if(DL_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_dlight);
sysfs_err15:
    if(ALIGHT_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_alight);
sysfs_err14:
    if(APROX_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_aprox);
sysfs_err13:
    if(ACC_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_accel);
sysfs_err12:
    if(MOTION_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_motion);
sysfs_err11:
    if(TEMP_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_uremote);
sysfs_err10:
    if(TEMP_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_sremote);
sysfs_err9:
    if(TEMP_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_local);
sysfs_err8:
    if(SOUND_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_peak);
sysfs_err7:
    if(SOUND_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_avg);
sysfs_err6:
    if(PL_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_proximity);
sysfs_err5:
    if(PL_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_ir);
sysfs_err4:
    if(PL_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_als);
sysfs_err3:
    if(DCOMP_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_dcompass);
sysfs_err2:
    if(ACOMP_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_acompass);
sysfs_err1:
    if(HUMIDITY_PRESENT)
        device_remove_file(&sensor->bdev->dev, &dev_attr_humidity);
error:
    cleanup_i2c_devices(sensor);

    sensor->class_dev = NULL;
    cdev_del(&sensor->cdev);
    device_destroy(bmi_class, MKDEV(major, slot));
    bmi_device_set_drvdata(bdev, 0);
    sensor->bdev = 0;
    printk(KERN_ERR "bmi_sensor.c: probe slot %d FAILED\n", slot);
    return -ENODEV;

}

// remove PIM
void bmi_sensor_remove(struct bmi_device *bdev)
{
    int slot = bdev->slot->slotnum;
    struct bmi_sensor *sensor = &bmi_sensor[slot];
    struct class *bmi_class;

#if defined(IRQ_STUFF_IMPLEMENTED)
    {
        int irq = bdev->slot->status_irq;
        free_irq(irq, sensor);
    }
#endif

    printk(KERN_ERR "destroy_workqueue: sensor->workqueue is %p\n", sensor->workqueue);
    destroy_workqueue(sensor->workqueue);
    printk(KERN_ERR "back from destroy_workqueue\n");

    if(HUMIDITY_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_humidity);
    }
    if(ACOMP_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_acompass);
    }
    if(DCOMP_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_dcompass);
    }
    if(PL_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_als);
        device_remove_file(&sensor->bdev->dev, &dev_attr_ir);
        device_remove_file(&sensor->bdev->dev, &dev_attr_proximity);
    }
    if(SOUND_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_avg);
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_peak);
    }
    if(TEMP_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_local);
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_sremote);
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_uremote);
    }
    if(MOTION_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_motion);
    }
    if(ACC_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_accel);
    }
    if(APROX_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_aprox);
        del_timer(&sensor->aprox_timer);
    }
    if(ALIGHT_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_alight);
    }
    if(DL_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_dlight);
    }

    cleanup_i2c_devices(sensor);

    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_RED_LED);
    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_GREEN_LED);

    bmi_class = bmi_get_class();
    device_destroy(bmi_class, MKDEV(major, slot));

    sensor->class_dev = 0;
    cdev_del(&sensor->cdev);

    // de-attach driver-specific struct from bmi_device structure
    bmi_device_set_drvdata(bdev, 0);
    sensor->bdev = 0;
}

/*
 *      module routines
 */

static int __init bmi_sensor_init(void)
{
    dev_t   dev_id;
    int     retval;

    printk(KERN_ERR "bmi_sensor.c: init");
    // alloc char driver with 4 minor numbers
    retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI SENSOR Driver");
    if(retval) {
        return -ENODEV;
    }

    major = MAJOR(dev_id);
    retval = bmi_register_driver(&bmi_sensor_driver);
    if(retval) {
        unregister_chrdev_region(dev_id, 4);
        return -ENODEV;
    }

    printk(KERN_INFO "bmi_sensor.c: BMI_SENSOR Driver v%s \n", BMISENSOR_VERSION);
    return 0;
}

static void __exit bmi_sensor_cleanup(void)
{
    dev_t dev_id;

    bmi_unregister_driver(&bmi_sensor_driver);

    dev_id = MKDEV(major, 0);
    unregister_chrdev_region(dev_id, 4);
    return;
}

module_init(bmi_sensor_init);
module_exit(bmi_sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bug Labs");
MODULE_DESCRIPTION("BMI Sensor device driver");
MODULE_SUPPORTED_DEVICE("bmi_sensor_cntl_mX");

