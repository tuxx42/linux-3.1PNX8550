/* ------------------------------------------------------------------------- */
/* i2c-algo-IP0105.c i2c driver algorithms for PNX8550 Fast I2C ports        */
/* ------------------------------------------------------------------------- */

/* This module is the work of Willem van Beek,
 * evidently inspired by Simon G. Vogl.
 *
 * Restriction :
--------------------------------------------------------------------------------
Based on a driver of:
Willem van Beek <willem.van.beek@philips.com>
 */



/*
-------------------------------------------------------------------------------
Standard include files:
-------------------------------------------------------------------------------
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

/*
-------------------------------------------------------------------------------
Project include files:
-------------------------------------------------------------------------------
*/
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-pnx8550/int.h>
#include <linux/platform_device.h>
#include "i2c-algo-IP0105.h"

#ifndef FALSE
#define FALSE 0
#define TRUE  1
#endif

#define errids_ok 0
#define errids_i2cHardwareError		0x01
#define errids_i2cDeviceError		0x02
#define errids_i2cWriteError		0x03

#define evMasterReady               0x0001
#define evSlaveRxIntermediateNtf    0x0002
#define evSlaveReceiverReady        0x0004
#define evSlaveReadRequest          0x0008
#define evSlaveTransmitterReady     0x0010
#define evSlaveStopCondition        0x0020

#ifndef BOOL
#define BOOL       int
#endif

/* --- adapters                                        */
#define NR_I2C_DEVICES 2 /* Two Fast I2C ports */

#define I2C_HW_IP0105               0x00 /* IP0105 Controller on the Viper */
#define FAST_I2C_PORT_3             0x00 /* IP0105 Controller on the Viper */
#define FAST_I2C_PORT_4             0x01 /* IP0105 Controller on the Viper */
#define IP0105_UNIT0                0xBBE69000
#define IP0105_UNIT1                0xBBE4C000

#define MODULE_CLOCK                27000 /* I2C module clock speed in KHz */

#define I2CADDRESS( address )       ((address) & 0x00FE) /* only 7 bits I2C address implemented */
#define TIMER_OFF                   0
#define TIMEOUT				200
static __inline void staout(void __iomem *offset, int p);

typedef enum {
	I2C_IP0105_MODE_IDLE,
	I2C_IP0105_MODE_MASTER_TRANSMITTER,
	I2C_IP0105_MODE_MASTER_RECEIVER,
	I2C_IP0105_MODE_SLAVE_TRANSMITTER,
	I2C_IP0105_MODE_SLAVE_RECEIVER,
} i2c_mode;

struct i2c_algo_data {
	int			mst_txbuflen;
	int			mst_txbufindex;
	unsigned char *		mst_txbuf;
	unsigned char *		mst_rxbuf;
	int			mst_rxbuflen;
	int			mst_rxbufindex;
	int			mst_address;
	unsigned long		mst_status;
};

struct i2c_ip0105_struct {
	wait_queue_head_t	i2c_wait_master;
	resource_size_t		mem_size;
	int			irq;
	struct i2c_adapter	ip0105_adap;
	void __iomem		*base;
	struct resource		*res;
	struct i2c_algo_data	algo_data;
	i2c_mode		mode;
};

static __inline void staout(void __iomem *offset, int p)
{
	int val = read_IP0105_I2C_CONTROL(offset);
	if (p)
		val |= IP0105_STA;
	else
		val &= ~IP0105_STA;

	write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void stoout(void __iomem *offset, int p)
{
	write_IP0105_I2C_STOP(offset, p);
}

static __inline void aaout(void __iomem *offset, int p)
{
	int val = read_IP0105_I2C_CONTROL(offset);
	if (p)
		val |= IP0105_AA;
	else
		val &= ~IP0105_AA;

	write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void setspeed100(void __iomem * offset)
{
	int val = read_IP0105_I2C_CONTROL(offset);

	val &= 0x00F0;
	val |= 0x00F4;

	write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void disable_i2c_controller(void __iomem *offset)
{
	int val = read_IP0105_I2C_CONTROL(offset);

	val &= ~IP0105_EN;

	write_IP0105_I2C_CONTROL(offset, val);
}
static __inline void disable_i2c_interrupt(void __iomem *offset)
{
	write_IP0105_I2C_INT_EN(offset, 0);
}

static __inline void enable_i2c_interrupt(void __iomem *offset)
{
	write_IP0105_I2C_INT_EN(offset, IP0105_INTBIT);
}

static __inline void enable_i2c_controller(void __iomem *offset)
{
	int val = read_IP0105_I2C_CONTROL(offset);

	val |= IP0105_EN;

	write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void clear_i2c_interrupt(void __iomem *offset)
{
	write_IP0105_I2C_INT_CLR(offset, 1);
}

static __inline void wait_ip0105_int(struct i2c_ip0105_struct *i2c_ip0105, int timeout)
{
	int t = timeout;

	while(!(read_IP0105_I2C_INT_STATUS(i2c_ip0105->base) & IP0105_INTBIT) && t-- > 0);
}

static __inline void wait_ip0105_sto_or_int(struct i2c_ip0105_struct *i2c_ip0105, int timeout)
{
	int t = timeout;
	while(!(read_IP0105_I2C_INT_STATUS(i2c_ip0105->base) & IP0105_INTBIT) \
			&&(read_IP0105_I2C_CONTROL(i2c_ip0105->base) & IP0105_STO) \
			&&(t-->0));
}

static void ip0105_reset(struct i2c_adapter *i2c_adap)
{
	struct i2c_ip0105_struct *i2c_ip0105;

	i2c_ip0105 = i2c_get_adapdata(i2c_adap);

	printk("device reset\n");
	disable_i2c_interrupt(i2c_ip0105->base);
	aaout(i2c_ip0105->base, 0);
	staout(i2c_ip0105->base, 0);

	wait_ip0105_int(i2c_ip0105, TIMEOUT);
	if(read_IP0105_I2C_INT_STATUS(i2c_ip0105->base) & IP0105_INTBIT) {
		unsigned int i2c_state;

		i2c_state = read_IP0105_I2C_STATUS(i2c_ip0105->base);
		if((i2c_state == 0x08) || (i2c_state == 0x10)) {
			write_IP0105_I2C_DAT(i2c_ip0105, 0xEE);
			clear_i2c_interrupt(i2c_ip0105->base);
		} else if((i2c_state == 0x40) || (i2c_state == 0x50)) {
			aaout(i2c_ip0105->base, 0);
			clear_i2c_interrupt(i2c_ip0105->base);
		}
		wait_ip0105_int(i2c_ip0105, TIMEOUT);
		stoout(i2c_ip0105->base, 1);
		clear_i2c_interrupt(i2c_ip0105->base);

		wait_ip0105_int(i2c_ip0105, TIMEOUT);
		wait_ip0105_sto_or_int(i2c_ip0105, TIMEOUT);
	} else {
		stoout(i2c_ip0105->base, 1);
		clear_i2c_interrupt(i2c_ip0105->base);

		wait_ip0105_sto_or_int(i2c_ip0105, TIMEOUT);
	}

	if(read_IP0105_I2C_CONTROL(i2c_ip0105->base) & IP0105_STO) {
//		assert(false);
	}

	disable_i2c_controller(i2c_ip0105->base);
	disable_i2c_interrupt(i2c_ip0105->base);
	aaout(i2c_ip0105->base, 0);
	staout(i2c_ip0105->base, 0);
	clear_i2c_interrupt(i2c_ip0105->base);

	enable_i2c_controller(i2c_ip0105->base);
	enable_i2c_interrupt(i2c_ip0105->base);
	//if(i2c_ip0105->slv_enabled == TRUE) { aaout(i2c_ip0105->base, 1); }
}

static __inline void i2c_interrupt_master_transmitter(struct i2c_ip0105_struct *i2c_ip0105, int i2c_state)
{
	int mst_addr;
	struct i2c_algo_data *algo_data = &i2c_ip0105->algo_data;

	switch (i2c_state) {
		case 0x10:
			printk("should not be reached: line: %d\n", __LINE__);
			if ((algo_data->mst_txbufindex < algo_data->mst_txbuflen) ||
				(algo_data->mst_rxbufindex == algo_data->mst_rxbuflen)) {
				mst_addr = (algo_data->mst_address & 0xFE);
				write_IP0105_I2C_DAT(i2c_ip0105->base, mst_addr);
			}
			break;
		case 0x18:
		case 0x28:
			printk("%s %d\n", __func__, __LINE__);
			if (algo_data->mst_txbufindex < algo_data->mst_txbuflen) {
				write_IP0105_I2C_DAT(i2c_ip0105->base, *(algo_data->mst_txbuf+algo_data->mst_txbufindex));
				algo_data->mst_txbufindex++;
				/* disable repeast of START condition */
				staout(i2c_ip0105->base, 0);
			}
			clear_i2c_interrupt(i2c_ip0105->base);
			break;
		case 0x20:
			printk("%s %d\n", __func__, __LINE__);
		case 0x30:
			printk("%s %d\n", __func__, __LINE__);
			staout(i2c_ip0105->base, 0);
			stoout(i2c_ip0105->base, 1);
			//i2c_ip0105->mst_status = 
			i2c_ip0105->mode = I2C_IP0105_MODE_IDLE;
			clear_i2c_interrupt(i2c_ip0105->base);
			printk("wakeup\n");
			wake_up_interruptible(&(i2c_ip0105->i2c_wait_master));
			break;
		case 0x38:
			printk("%s %d\n", __func__, __LINE__);
			break;
		case 0x68:
			printk("%s %d\n", __func__, __LINE__);
			break;
		case 0xB0:
			printk("%s %d\n", __func__, __LINE__);
			break;
		case 0x70:
			printk("%s %d\n", __func__, __LINE__);
			break;
		case 0x78:
			printk("%s %d\n", __func__, __LINE__);
			break;
		default:
			printk("%s %d\n", __func__, __LINE__);
			break;
	}
}
static __inline void i2c_interrupt_idle(struct i2c_ip0105_struct *i2c_ip0105, int i2c_state)
{
	int mst_addr;
	struct i2c_algo_data *algo_data = &i2c_ip0105->algo_data;

	switch(i2c_state) {
		case 0x08:
			printk("0x08\n");

			printk("%s %d\n", __func__, __LINE__);
			if ((algo_data->mst_txbufindex < algo_data->mst_txbuflen) ||
				(algo_data->mst_rxbufindex == algo_data->mst_rxbuflen)) {
				printk("%s %d\n", __func__, __LINE__);
				mst_addr = (algo_data->mst_address & 0xFE);
				/* Read bit cleared */
				printk("START TRANSMITTED, Read bit cleared addr = %x\n", mst_addr);
				write_IP0105_I2C_DAT(i2c_ip0105->base, mst_addr);
				i2c_ip0105->mode = I2C_IP0105_MODE_MASTER_TRANSMITTER;
			} else {
				printk("%s %d\n", __func__, __LINE__);
				mst_addr = ((algo_data->mst_address & 0xFE) | 0x01);
				/* Read bit set */
				write_IP0105_I2C_DAT(i2c_ip0105->base, mst_addr);
				printk("START TRANSMITTED, Read bit set(%x)\n", mst_addr);
				i2c_ip0105->mode = I2C_IP0105_MODE_MASTER_RECEIVER;
			}
			/* Clear STA flag */
			staout(i2c_ip0105->base, 0);
			clear_i2c_interrupt(i2c_ip0105->base);
			break;
		case 0x60:
			printk("0x60\n");
			break;
		case 0xA8:
			printk("0xA8\n");
			break;
		case 0x70:
			printk("0x70\n");
			break;
		case 0x78:
			printk("0x78\n");
			break;
		default:
			printk("hardware error\n");
			break;
	}
}
static irqreturn_t i2c_ip0105_isr(int irq, void *dev_id)
{
	int i2c_state;
	struct i2c_ip0105_struct *i2c_ip0105 = (struct i2c_ip0105_struct *)dev_id;

	i2c_state = read_IP0105_I2C_STATUS(i2c_ip0105->base);
	switch(i2c_ip0105->mode) {
		case I2C_IP0105_MODE_IDLE:
			printk("%s %d\n", __func__, __LINE__);
			i2c_interrupt_idle(i2c_ip0105, i2c_state);
			printk("%s %d\n", __func__, __LINE__);
			break;
		case I2C_IP0105_MODE_MASTER_TRANSMITTER:
			printk("%s %d\n", __func__, __LINE__);
			i2c_interrupt_master_transmitter(i2c_ip0105, i2c_state);
			printk("I2C_IP0105_MODE_MASTER_TRANSMITTER\n");
			break;
		case I2C_IP0105_MODE_MASTER_RECEIVER:
			printk("I2C_IP0105_MODE_MASTER_RECEIVER\n");
			break;
		case I2C_IP0105_MODE_SLAVE_TRANSMITTER:
			printk("I2C_IP0105_MODE_SLAVE_TRANSMITTER\n");
			break;
		case I2C_IP0105_MODE_SLAVE_RECEIVER:
			printk("I2C_IP0105_SLAVE_RECEIVER\n");
			break;
	}

	return IRQ_HANDLED;
}



static unsigned int i2c_write_read(struct i2c_adapter * i2c_adap, int address,
		void *msgwram, int lenw, void *msgr, int lenr) {
	int retval = 0;
	struct i2c_algo_data *i2c_algo;
	struct i2c_ip0105_struct *i2c_ip0105;

	i2c_algo = i2c_adap->algo_data;
	i2c_ip0105 = i2c_get_adapdata(i2c_adap);

	i2c_algo->mst_status		= errids_ok;
	i2c_algo->mst_address		= address;
	i2c_algo->mst_txbuflen		= lenw;
	i2c_algo->mst_txbufindex	= 0;
	i2c_algo->mst_rxbuf		= msgr;
	i2c_algo->mst_rxbufindex	= 0;

	if (msgwram)
		i2c_algo->mst_txbuf = msgwram;
	else
		i2c_algo->mst_txbuflen = 0;

	if (msgr)
		i2c_algo->mst_rxbuflen = lenr;
	else
		i2c_algo->mst_rxbuflen = 0;

	staout(i2c_ip0105->base, 1);

	// XXX why 5HZ??
	printk("go to sleep\n");
	retval = interruptible_sleep_on_timeout(&(i2c_ip0105->i2c_wait_master), (50*HZ));
	if (!retval) {
		// XXX output
		//dev_dbg(&i2c_adap->dev, "i2c status 0x%x, int stat %x\n");
		return errids_i2cHardwareError;
	}


	return i2c_algo->mst_status;
}

static int i2c_ip0105_xfer(struct i2c_adapter *adapter,
		struct i2c_msg *msgs, int num)
{
	struct i2c_msg *pmsg;
	void * rd_buf = NULL;
	void * wr_buf = NULL;
	int i, wr_len = 0, rd_len = 0;
	unsigned char addr = msgs[0].addr;
	int ret=0;


	printk("i2c_ip0105_xfer(): num = %d\n", num);
	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		if (i == 0) {
			if (pmsg->flags & I2C_M_TEN) {
				printk("received a 10bit address: %x\n", addr);
				/* addr = 10 bit addr not supported */
			} else {
				printk("received a 7bit address: %x\n", addr);
				/* addr = 7 bit addr */
				addr &= 0x7f;
				addr <<= 1;
			}
		}
		if (pmsg->flags & I2C_M_RD) {
			/* read bytes into buffer*/
			rd_buf = pmsg->buf;
			rd_len = pmsg->len;
		} else {
			/* write bytes from buffer */
			wr_buf = pmsg->buf;
			wr_len = pmsg->len;
		}
	}
	if (num != 0) {
		switch(ret = i2c_write_read(adapter, addr, wr_buf, wr_len, rd_buf, rd_len)) {
			case errids_ok:
				break;
			case errids_i2cHardwareError:
				num = -1;
				printk("%s() %d\n", __func__, __LINE__);
				ip0105_reset(adapter);
				printk("%s() %d\n", __func__, __LINE__);
				break;
			case errids_i2cDeviceError:
				num = -1;
				printk("%s() %d\n", __func__, __LINE__);
				ip0105_reset(adapter);
				printk("%s() %d\n", __func__, __LINE__);
				break;
			case errids_i2cWriteError:
				num = -1;
				printk("%s() %d\n", __func__, __LINE__);
				ip0105_reset(adapter);
				printk("%s() %d\n", __func__, __LINE__);
				break;
			default:
				num = -1;
				printk("%s() %d\n", __func__, __LINE__);
				ip0105_reset(adapter);
				printk("%s() %d\n", __func__, __LINE__);
				break;
		}
	}

	//printk("i2c_ip0105_xfer num %d\n", num);

	return num;
}

static u32 i2c_ip0105_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR |
		I2C_FUNC_PROTOCOL_MANGLING;
}

static struct i2c_algorithm i2c_ip0105_algo = {
	.master_xfer	= i2c_ip0105_xfer,
	.functionality	= i2c_ip0105_func,
};

static int __devinit i2c_ip0105_probe(struct platform_device *pdev)
{
	int ret;
	struct i2c_ip0105_struct *i2c_ip0105;
	struct resource *res;

	i2c_ip0105 = kzalloc(sizeof(struct i2c_ip0105_struct), GFP_KERNEL); 
	if (!i2c_ip0105) {
		dev_err(&pdev->dev, "can't allocate interface\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get device resources\n");
		return -ENOENT;
	}

	i2c_ip0105->mem_size = resource_size(res);

	if (!request_mem_region(res->start, i2c_ip0105->mem_size, pdev->name))
		return -EBUSY;

	i2c_ip0105->base = ioremap(res->start, i2c_ip0105->mem_size);
	if (!i2c_ip0105->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -EIO;
	}

	init_waitqueue_head(&(i2c_ip0105->i2c_wait_master));

	i2c_ip0105->irq = platform_get_irq(pdev, 0);
	if (i2c_ip0105->irq < 0) {
		dev_err(&pdev->dev, "can't get irq number\n");
		// XXX cleanup memory mapping
		return -ENOENT;
	}

	disable_i2c_controller(i2c_ip0105->base);
	setspeed100(i2c_ip0105->base);

	aaout(i2c_ip0105->base, 0);	/* Slave mode disabled */
	stoout(i2c_ip0105->base, 0);	/* Do not generate stop condition */
	staout(i2c_ip0105->base, 0);	/* Do not generate Start condition */

	ret = request_irq(i2c_ip0105->irq, i2c_ip0105_isr, 0, pdev->name, (void *)i2c_ip0105);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't claim irq %d\n", i2c_ip0105->irq);
		// XXX memory cleanup
		return ret;
	}

	clear_i2c_interrupt(i2c_ip0105->base);
	disable_irq(i2c_ip0105->irq);	/* disable i2c interrupt in Interrupt controller */
	enable_irq(i2c_ip0105->irq);	/* enable i2c interrupt in Interrupt controller */
	//printk("irq: %d\n", i2c_ip0105->irq);

	enable_i2c_controller(i2c_ip0105->base);	/* Enable I2C Controller */
	enable_i2c_interrupt(i2c_ip0105->base);		/* Enable both the SI and DMA interrupts */

	strlcpy(i2c_ip0105->ip0105_adap.name, pdev->name, sizeof(i2c_ip0105->ip0105_adap.name));
	i2c_ip0105->mode			= I2C_IP0105_MODE_IDLE;
	i2c_ip0105->ip0105_adap.owner		= THIS_MODULE;
	i2c_ip0105->ip0105_adap.algo		= &i2c_ip0105_algo;
	i2c_ip0105->ip0105_adap.dev.parent	= &pdev->dev;
	i2c_ip0105->ip0105_adap.nr		= pdev->id;
	i2c_ip0105->res				= res;
	/* XXX neccessary ??? */
	i2c_ip0105->ip0105_adap.algo_data	= &i2c_ip0105->algo_data;
	/*i2c_ip0105->irq			= i2c_ip0105->irq;*/
	/*i2c_ip0105->base			= base;*/

	/* store structure in pdev and i2c_adapter */
	platform_set_drvdata(pdev, i2c_ip0105);
	i2c_set_adapdata(&i2c_ip0105->ip0105_adap, i2c_ip0105);

	ret = i2c_add_numbered_adapter(&i2c_ip0105->ip0105_adap);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "i2c-algo-IP0105: Unable to register with I2C\n");
		free_irq(i2c_ip0105->irq, i2c_ip0105);
		return -ENODEV;
	}

	printk(KERN_INFO "i2c-algo-IP0105: registered device at 0x%08lx\n", (unsigned long)i2c_ip0105->base);

	return 0;
}

static int __devexit i2c_ip0105_remove(struct platform_device *pdev)
{
	int res;
	struct i2c_ip0105_struct *i2c_ip0105;

	printk("%s() %d\n", __func__, __LINE__);

	i2c_ip0105 = platform_get_drvdata(pdev);

	iounmap(i2c_ip0105->base);
	release_mem_region(i2c_ip0105->res->start, i2c_ip0105->mem_size);

	disable_i2c_interrupt(i2c_ip0105->base);
	disable_i2c_controller(i2c_ip0105->base);

	disable_irq(i2c_ip0105->irq);
	free_irq(i2c_ip0105->irq, i2c_ip0105);
	enable_irq(i2c_ip0105->irq);

	res = i2c_del_adapter(&i2c_ip0105->ip0105_adap);
	if (res < 0)
		return res;

	return 0;
}

static struct platform_driver i2c_ip0105_driver = {
	.driver = {
		.name = "IP0105",
		.owner = THIS_MODULE,
		.bus = &platform_bus_type,
	},
	.probe = i2c_ip0105_probe,
	.remove = __devexit_p(i2c_ip0105_remove),
};

int __init i2c_algo_ip0105_init (void)
{
	//printk("%s() %d\n", __func__, __LINE__);
	return platform_driver_register(&i2c_ip0105_driver);
}

void __exit i2c_algo_ip0105_exit(void)
{
	printk("%s() %d\n", __func__, __LINE__);
	platform_driver_unregister(&i2c_ip0105_driver);
}

MODULE_AUTHOR("Joel Thomas <jt@emlix.com>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for IP0105");
MODULE_LICENSE("GPL");

subsys_initcall(i2c_algo_ip0105_init);
module_exit(i2c_algo_ip0105_exit);
