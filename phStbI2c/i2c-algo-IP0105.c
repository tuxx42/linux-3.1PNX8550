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
#include <linux/spinlock.h>

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

typedef enum {
	IP0105_I2C_ACTION_INVALID,
	IP0105_I2C_ACTION_CONTINUE,
	IP0105_I2C_ACTION_SEND_START,
	IP0105_I2C_ACTION_SEND_RESTART,
	IP0105_I2C_ACTION_SEND_ADDR,
	IP0105_I2C_ACTION_SEND_DATA,
	IP0105_I2C_ACTION_RCV_DATA,
	IP0105_I2C_ACTION_RCV_DATA_STOP,
	IP0105_I2C_ACTION_SEND_STOP,
} i2c_action;

typedef enum {
	IP0105_I2C_STATE_INVALID,     
	IP0105_I2C_STATE_IDLE,        
	IP0105_I2C_STATE_WAITING_FOR_START_COND,                                                         
	IP0105_I2C_STATE_WAITING_FOR_RESTART,
	IP0105_I2C_STATE_WAITING_FOR_ADDR_ACK,
	IP0105_I2C_STATE_WAITING_FOR_SLAVE_ACK,
	IP0105_I2C_STATE_WAITING_FOR_SLAVE_DATA,
} i2c_state;

struct i2c_ip0105_struct {
	wait_queue_head_t	i2c_wait_master;
	resource_size_t		mem_size;
	int			irq;
	struct i2c_adapter	ip0105_adap;
	void __iomem		*base;
	struct resource		*res;
	struct i2c_msg		*msg;
	i2c_mode		mode;
	i2c_action		action;
	i2c_state		state;
	u32			addr;
	u32			bytes_left;
	u32			byte_posn;
	u32			send_stop;
	u32			block;
	u32			aborting;
	spinlock_t		lock;
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

static void ip0105_i2c_do_action(struct i2c_ip0105_struct *i2c_ip0105)
{
	printk("ip0105_i2c_do_action(): ");
	switch(i2c_ip0105->action) {
	case IP0105_I2C_ACTION_SEND_RESTART:
		printk("IP0105_I2C_ACTION_SEND_RESTART\n");
		staout(i2c_ip0105->base, 1);
		i2c_ip0105->block = 0;
		clear_i2c_interrupt(i2c_ip0105->base);
		wake_up_interruptible(&i2c_ip0105->i2c_wait_master);
		break;
	case IP0105_I2C_ACTION_SEND_START:
		printk("IP0105_I2C_ACTION_SEND_START\n");
		/* next event will be 0x08 */
		staout(i2c_ip0105->base, 1);
		clear_i2c_interrupt(i2c_ip0105->base);
		break;
	case IP0105_I2C_ACTION_SEND_ADDR:
		printk("IP0105_I2C_ACTION_SEND_ADDR writing address 0x%x\n", i2c_ip0105->addr);

		/* write address */
		write_IP0105_I2C_DAT(i2c_ip0105->base, i2c_ip0105->addr);
		/* next will be 0x28 */

		/* clear start flag */
		staout(i2c_ip0105->base, 0);
		clear_i2c_interrupt(i2c_ip0105->base);
		break;
	case IP0105_I2C_ACTION_RCV_DATA:
		i2c_ip0105->msg->buf[i2c_ip0105->byte_posn++] = read_IP0105_I2C_DAT(i2c_ip0105->base);
		printk("IP0105_I2C_ACTION_RCV_DATA reading byte msgbuf[%d] = %d (%d)\n", i2c_ip0105->byte_posn, i2c_ip0105->msg->buf[i2c_ip0105->byte_posn], i2c_ip0105->bytes_left);

		if ((i2c_ip0105->bytes_left == 1) || i2c_ip0105->aborting) {
			printk("return no ack\n");
			aaout(i2c_ip0105->base, 0);
			/* dont return ACK after last byte reception */ 
		} else {
			printk("return ack\n");
			aaout(i2c_ip0105->base, 1);
			/* return ACK after byte reception */
		}

		clear_i2c_interrupt(i2c_ip0105->base);

		break;
	case IP0105_I2C_ACTION_SEND_DATA:
		printk("IP0105_I2C_ACTION_SEND_DATA data byte %x\n",
				i2c_ip0105->msg->buf[i2c_ip0105->byte_posn]);
		write_IP0105_I2C_DAT(i2c_ip0105->base,
				i2c_ip0105->msg->buf[i2c_ip0105->byte_posn++]);
		clear_i2c_interrupt(i2c_ip0105->base);
		break;
	case IP0105_I2C_ACTION_RCV_DATA_STOP:
		printk("IP0105_I2C_ACTION_RCV_DATA_STOP\n");
		/* START flag down, STOP up */
		stoout(i2c_ip0105->base, 0);
		staout(i2c_ip0105->base, 1);
		i2c_ip0105->block = 0;
		clear_i2c_interrupt(i2c_ip0105->base);
		wake_up_interruptible(&i2c_ip0105->i2c_wait_master);
		break;
	default:
		printk("default\n");
		clear_i2c_interrupt(i2c_ip0105->base);
		printk("ip0105_i2c_do_action(): reached default\n");
	case IP0105_I2C_ACTION_SEND_STOP:
		printk("IP0105_I2C_ACTION_SEND_STOP\n");
		staout(i2c_ip0105->base, 0);
		stoout(i2c_ip0105->base, 1);
		i2c_ip0105->block = 0;
		clear_i2c_interrupt(i2c_ip0105->base);
		wake_up_interruptible(&i2c_ip0105->i2c_wait_master);
		break;
	}
}

static irqreturn_t i2c_ip0105_isr(int irq, void *dev_id)
{
	unsigned long flags;
	int i2c_state;
	struct i2c_ip0105_struct *i2c_ip0105 = (struct i2c_ip0105_struct *)dev_id;

	spin_lock_irqsave(&(i2c_ip0105->lock), flags);

	i2c_state = read_IP0105_I2C_STATUS(i2c_ip0105->base);
	switch(i2c_state) {
		/* start condition interrupt */
		case 0x08: /* master start */
		case 0x10: /* repeat start */
			printk("isr(): %sstart interrupt 0x%x\n",i2c_state==0x08?"":"repeated ",i2c_state);
			i2c_ip0105->action = IP0105_I2C_ACTION_SEND_ADDR;
			i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_ADDR_ACK;
			break;
		case 0x58:
			printk("isr(): master read data no ack\n");
			i2c_ip0105->action = IP0105_I2C_ACTION_RCV_DATA_STOP;
			i2c_ip0105->state = IP0105_I2C_STATE_IDLE;
			break;
		case 0x18:
		case 0x28: /* ACK received */
			printk("isr(): data_byte transmitted. ACK received (0x%x)\n", i2c_state);
			if((i2c_ip0105->bytes_left == 0) ||
					(i2c_ip0105->aborting &&
					 i2c_ip0105->byte_posn != 0)) {
				if(i2c_ip0105->send_stop) {
					printk("isr(): no bytes left... send_stop\n");
					i2c_ip0105->action = IP0105_I2C_ACTION_SEND_STOP;
					i2c_ip0105->state = IP0105_I2C_STATE_IDLE;
				} else {
					printk("isr(): no bytes left... !send_stop\n");
					i2c_ip0105->action = IP0105_I2C_ACTION_SEND_RESTART;
					i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_RESTART;
				}
			} else {
				printk("isr(): bytes_left: send more data\n");
				staout(i2c_ip0105->base, 0);
				i2c_ip0105->action = IP0105_I2C_ACTION_SEND_DATA;
				i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_SLAVE_ACK;
				i2c_ip0105->bytes_left--;
			}
			clear_i2c_interrupt(i2c_ip0105->base);
			break;
		case 0x40:
			printk("isr(): master read address ack\n");
			if(i2c_ip0105->byte_posn < (i2c_ip0105->msg->len - 1)) {
				aaout(i2c_ip0105->base, 1);
			} else {
				aaout(i2c_ip0105->base, 0);
			}
			staout(i2c_ip0105->base, 0);
			clear_i2c_interrupt(i2c_ip0105->base);
			break;
		case 0x50: /* master read data ack */
			printk("isr(): receiving ack for data byte\n");
			i2c_ip0105->action = IP0105_I2C_ACTION_RCV_DATA;
			i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_SLAVE_DATA;
			i2c_ip0105->bytes_left--;
			break;
		case 0x20:
		case 0x30:
		case 0x48:
			printk("isr(): no device on the other end\n");
			i2c_ip0105->action = IP0105_I2C_ACTION_RCV_DATA_STOP;
			i2c_ip0105->state = IP0105_I2C_STATE_IDLE;
			clear_i2c_interrupt(i2c_ip0105->base);
			break;
		default:
			clear_i2c_interrupt(i2c_ip0105->base);
			printk("isr(): other condition reached: 0x%x\n", i2c_state);
			break;
	}

	ip0105_i2c_do_action(i2c_ip0105);
	spin_unlock_irqrestore(&(i2c_ip0105->lock), flags);

	return IRQ_HANDLED;
}

static void ip0105_i2c_wait_for_completion(struct i2c_ip0105_struct *i2c_ip0105)
{
	long		time_left;
	unsigned long	flags;
	char		abort = 0;

	printk("ip0105_i2c_wait_for_completion(): started\n");

	time_left = wait_event_interruptible_timeout(i2c_ip0105->i2c_wait_master,
			!i2c_ip0105->block, i2c_ip0105->ip0105_adap.timeout);

	spin_lock_irqsave(&(i2c_ip0105->lock), flags);

	if (time_left <= 0)
		abort = 1;

	if (abort && i2c_ip0105->block) {
		i2c_ip0105->aborting = 1;
		spin_unlock_irqrestore(&(i2c_ip0105->lock), flags);

		time_left = wait_event_timeout(i2c_ip0105->i2c_wait_master,
			!i2c_ip0105->block, i2c_ip0105->ip0105_adap.timeout);

		if ((time_left <= 0) && i2c_ip0105->block) {
			i2c_ip0105->state = IP0105_I2C_STATE_IDLE;
			dev_err(&i2c_ip0105->ip0105_adap.dev,
					"ip0105: I2C bus locked, block: %d, "
					"time_left: %d\n", i2c_ip0105->block,
					(int)time_left);
			// XXX dont just printf
			printk("reset hardware\n");
		}
	} else
		spin_unlock_irqrestore(&(i2c_ip0105->lock), flags);

	printk("ip0105_i2c_wait_for_completion(): completed\n");
}

static int ip0105_i2c_execute_msg(struct i2c_ip0105_struct *i2c_ip0105,
		struct i2c_msg *msg, int is_first, int is_last)
{
	unsigned long flags;
	static int var = 0;
	int dir = 0;

	spin_lock_irqsave(&(i2c_ip0105->lock), flags);

	i2c_ip0105->msg = msg;
	i2c_ip0105->byte_posn = 0;
	i2c_ip0105->bytes_left = msg->len;
	i2c_ip0105->aborting = 0;

	if(msg->flags & I2C_M_RD) {
		dir = 1;
	}

	i2c_ip0105->addr = ((u32)msg->addr & 0x7f) << 1 | dir;

	printk("ip0105_i2c_execute_msg() is_first: %d, is_last: %d\n", is_first, is_last);
	if(msg->flags & I2C_M_NOSTART) {
		if(i2c_ip0105->msg->flags & I2C_M_RD) {
			/* no action to do, wait for slave to send a byte */
			i2c_ip0105->action = IP0105_I2C_ACTION_CONTINUE;
			i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_SLAVE_DATA;
		} else {
			i2c_ip0105->action = IP0105_I2C_ACTION_SEND_DATA;
			i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_SLAVE_ACK;
			i2c_ip0105->bytes_left--;
		}
		printk("reached I2C_M_NOSTART\n");
	} else {
		if(is_first) {
			i2c_ip0105->action = IP0105_I2C_ACTION_SEND_START;
			i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_START_COND;
		} else {
			i2c_ip0105->action = IP0105_I2C_ACTION_SEND_ADDR;
			i2c_ip0105->state = IP0105_I2C_STATE_WAITING_FOR_ADDR_ACK;
		}
	}

	i2c_ip0105->send_stop = is_last;
	i2c_ip0105->block = 1;
	ip0105_i2c_do_action(i2c_ip0105);

	spin_unlock_irqrestore(&(i2c_ip0105->lock), flags);

	ip0105_i2c_wait_for_completion(i2c_ip0105);
	
	return 1;
}

static int i2c_ip0105_xfer(struct i2c_adapter *adapter,
		struct i2c_msg *msgs, int num)
{
	int i, ret;
	struct i2c_ip0105_struct *i2c_ip0105;

	i2c_ip0105 = i2c_get_adapdata(adapter);

	for (i = 0; i < num; i++) {
		printk("i2c_ip0105_xfer(): i = %d/%d\n", i, num);
		ret = ip0105_i2c_execute_msg(i2c_ip0105, &msgs[i], i == 0, i+1 == num);
		if (ret < 0)
			return ret;
	}

	return num;
}

static u32 i2c_ip0105_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL | //I2C_FUNC_10BIT_ADDR |
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
	spin_lock_init(&(i2c_ip0105->lock));

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
	i2c_ip0105->ip0105_adap.timeout		= msecs_to_jiffies(500);
	i2c_ip0105->res				= res;
	/* XXX neccessary ??? */
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
