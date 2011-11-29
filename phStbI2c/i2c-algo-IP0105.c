/* ------------------------------------------------------------------------- */
/* i2c-algo-IP0105.c i2c driver algorithms for PNX8550 Fast I2C ports        */
/* ------------------------------------------------------------------------- */

/* This module is the work of Willem van Beek,
 * evidently inspired by Simon G. Vogl.
 *
 * Restriction :
 * Slave Transmitter functionality not working (yet)
 * No time-out (or watchdog) on Slave functionality
 *
 *
Rev Date        Author        Comments
--------------------------------------------------------------------------------
001             M Neill       Initial
....
006 20051122    raiyat        Linux 2.6.14.2 changes
007 20060908    laird         Linux 2.6.17.7 changes
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
#include "i2c-algo-IP0105.h"
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-pnx8550/int.h>
#include <linux/platform_device.h>

#ifndef FALSE
#define FALSE 0
#define TRUE  1
#endif

#define errids_Ok 0
#define errids_IsI2cHardwareError 0x01
#define errids_IsI2cDeviceError   0x02
#define errids_IsI2cWriteError    0x03

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

#if 0
typedef enum
{
    Idle = 0u,
    MasterTransmitter,
    MasterReceiver,
    SlaveTransmitter,
    SlaveReceiver
} I2cMode;

struct I2cBusObject
{
    unsigned char  * mst_txbuf; /* Master mode variables */
    int     mst_txbuflen;
    int     mst_txbufindex;
    unsigned char  * mst_rxbuf;
    int     mst_rxbuflen;
    int     mst_rxbufindex;
    int     mst_address;
    wait_queue_head_t iic_wait_master;
        unsigned long int_pin;
    unsigned int mst_status;
//    int     mst_timer;
//    int     mst_timeout;
//    BOOL    mst_timeout_valid;

    unsigned char    * slv_buf; /* Slave mode variables */
    int     slv_bufsize;
    int     slv_buflen;
    int     slv_bufindex;
    BOOL    slv_enabled;
//    int     slv_timer;
//    int     slv_timeout;
//    BOOL    slv_timeout_valid;

    int     offset; /* I2C HW controller address offset, required by HAL */
    I2cMode mode;
    int     isr_event;
//    BOOL    bus_blocked;
    struct  fasync_struct ** slv_usr_notify;
};

/* declaration of static functions */

/* Local functions for IP0105 */

static __inline void STAOUT(struct I2cBusObject * a, int p)
{
        int val = read_IP0105_I2C_CONTROL(a);
        if (p) { val |= IP0105_STA; } else { val &= ~IP0105_STA; }
        write_IP0105_I2C_CONTROL(a, val);
}
static __inline void STOOUT(struct I2cBusObject * a, int p)
{
        write_IP0105_I2C_STOP(a, p);
}
static __inline void AAOUT(struct I2cBusObject * a, int p)
{
        int val = read_IP0105_I2C_CONTROL(a);
        if (p) { val |= IP0105_AA; } else { val &= ~IP0105_AA; }
        write_IP0105_I2C_CONTROL(a, val);
}
static __inline void ENABLE_I2C_CONTROLLER(struct I2cBusObject * a)
{
        int val = read_IP0105_I2C_CONTROL(a);

        val |= IP0105_EN;
        write_IP0105_I2C_CONTROL(a, val);
}
static __inline void CLEAR_I2C_INTERRUPT(struct I2cBusObject * a)
{
        write_IP0105_I2C_INT_CLR(a, 1);
}
static __inline void ENABLE_I2C_INTERRUPT(struct I2cBusObject * a)
{
        write_IP0105_I2C_INT_EN(a, IP0105_INTBIT);
}
static __inline void DISABLE_I2C_INTERRUPT(struct I2cBusObject * a)
{
        write_IP0105_I2C_INT_EN(a, 0);
}


/* ----- global defines ----------------------------------------------- */
#define DEB(x) if (i2c_debug>=1) x
#define DEB2(x) if (i2c_debug>=2) x
#define DEB3(x) if (i2c_debug>=3) x /* print several statistical values*/
#define ASSERT(x) if (!(x)) dev_dbg(&i2c_adap->dev, "ASSERTION FAILED at line %d in file %s\n", __LINE__, __FILE__);
/* Types and defines: */

#define TIMEOUT     200 /* Should be wait period of >100us i.e 1 byte @100KHz */


static __inline void SETSPEED100(struct I2cBusObject * a)
{
        int val = read_IP0105_I2C_CONTROL(a);

        val &= 0x00F0;
        val |= 0x00F4;
        write_IP0105_I2C_CONTROL(a, val);
}


static __inline void SETSPEED25(struct I2cBusObject * a)
{
        int val = read_IP0105_I2C_CONTROL(a);

        val |= 0x00F7;
        write_IP0105_I2C_CONTROL(a, val);
}

static __inline void SETSPEED400(struct I2cBusObject * a)
{
        int val = read_IP0105_I2C_CONTROL(a);

        val |= 0x00F0;
        write_IP0105_I2C_CONTROL(a, val);
}


/* Local Types */

static unsigned long IP0105_Controller[2] = {  IP0105_UNIT0, IP0105_UNIT1 };
static unsigned long IP0105_INTPIN[2] = { PNX8550_INT_I2C3, PNX8550_INT_I2C4 };

static struct i2c_adapter * slave_device = NULL;

static void do_slave_tasklet(unsigned long);
DECLARE_TASKLET(IP0105_slave_tasklet, do_slave_tasklet, 0);

static struct I2cBusObject IP0105_i2cbus[NR_I2C_DEVICES];

static int i2c_debug = 0;


/* This function is called from Primary ISR */
static void recover_from_hardware_error(struct i2c_adapter * i2c_adap)
{
        struct I2cBusObject * busptr = (struct I2cBusObject *)i2c_adap->algo_data;

        dev_dbg(&i2c_adap->dev, "Recover from HW Error\n");
        DISABLE_I2C_CONTROLLER( busptr );
        STAOUT( busptr, 0 ); /* Don't generate START condition */
        STOOUT( busptr, 1 ); /* Recover from error condition */

        if( ( busptr->mode == MasterReceiver ) || ( busptr->mode == MasterTransmitter ) )
        {
                disable_irq(busptr->int_pin);
                busptr->mst_status = errids_IsI2cHardwareError;
                busptr->isr_event |= evMasterReady;
                enable_irq(busptr->int_pin);  //isv_SwExtSet( INTERRUPT_ID( device ) );
                wake_up_interruptible(&(busptr->iic_wait_master));
        }
        if ( busptr->slv_enabled )
        {
                AAOUT( busptr, 1 ); /* ACK bit will be returned after slave address reception */
        }
        else
        {
                AAOUT( busptr, 0 ); /* NOT ACK bit will be returned after slave address reception */
        }
        busptr->mode = Idle;
        CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
        ENABLE_I2C_CONTROLLER( busptr ); /* Enable I2C controller */
}

static __inline void iic_interrupt_idle(struct i2c_adapter * i2c_adap, struct I2cBusObject * busptr, int i2c_state)
{
        switch (i2c_state)
        {
        case 0x08: /* Master Transmitter or Master Receiver: A START condition has been transmitted */
                if( ( busptr->mst_txbufindex < busptr->mst_txbuflen ) || ( busptr->mst_rxbufindex == busptr->mst_rxbuflen ) ) /* if bytes to transmit or no bytes to receive  ? */
                {
                        int myaddr = (busptr->mst_address & 0xFE);
                        dev_dbg(&i2c_adap->dev, "START TRANSMITTED, Read bit cleared addr = %x\n", myaddr);
                        write_IP0105_I2C_DAT(busptr, myaddr); /* Read bit cleared */
                        busptr->mode = MasterTransmitter;
                }
                else
                {
                        int myaddr = ((busptr->mst_address & 0xFE) | 0x01);
                        dev_dbg(&i2c_adap->dev, "START TRANSMITTED, Read bit set(%x)\n", myaddr);
                        write_IP0105_I2C_DAT(busptr, myaddr); /* Read bit set */
                        busptr->mode = MasterReceiver;
                }
                STAOUT( busptr, 0 );/* Clear STA flag */
                CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
                break;
        case 0x60: /* Own SLA+W has been received; ACK has been returned */
                busptr->slv_bufindex = 1; /* reset buffer pointer !!! */
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned after next byte reception */
                busptr->mode = SlaveReceiver;

//                Todo : Start timeout timer in tasklet
//                tasklet_schedule(&IP0105_slave_tasklet);
                CLEAR_I2C_INTERRUPT(busptr); /* Don't reset i2c interrupt */
                break;
        case 0xA8: /* Own SLA+R has been received; ACK has been returned */
                busptr->mode = SlaveTransmitter;
                busptr->isr_event |= evSlaveReadRequest;
                slave_device = i2c_adap;
                tasklet_schedule(&IP0105_slave_tasklet);
                /* Don't reset i2c interrupt */
                break;
        case 0x70: /* General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        case 0x78: /* Arbitration lost in SLA+R/~W as master; General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        default:
            recover_from_hardware_error( i2c_adap );
            break;
        }
}

static __inline void iic_interrupt_master_transmitter(struct i2c_adapter * i2c_adap, struct I2cBusObject * busptr, int i2c_state)
{
        switch (i2c_state)
        {
        case 0x10: /* Master Transmitter or Master Receiver: A repeated START condition has been transmitted */
            /* Note : A repeated START should not occur when in MasterReceiver mode in this implementation */
                if( ( busptr->mst_txbufindex < busptr->mst_txbuflen ) || ( busptr->mst_rxbufindex == busptr->mst_rxbuflen ) ) /* if bytes to transmit or no bytes to receive  ? */
                {
                        int myaddr = (busptr->mst_address & 0xFE);
                        dev_dbg(&i2c_adap->dev, "START TRANSMITTED, Read bit cleared addr = %x\n", myaddr);
                        write_IP0105_I2C_DAT(busptr, myaddr); /* Read bit cleared */
                        busptr->mode = MasterTransmitter;
                }
                else
                {
                        int myaddr = ((busptr->mst_address & 0xFE) | 0x01);
                        dev_dbg(&i2c_adap->dev, "START TRANSMITTED, Read bit set(%x)\n", myaddr);
                        write_IP0105_I2C_DAT(busptr, myaddr); /* Read bit set */
                        busptr->mode = MasterReceiver;
                }
                STAOUT( busptr, 0 );/* Clear STA flag */
                CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
                break;

        case 0x18: /* Master Transmitter: SLA+W has been transmitted; ACK has been received */
            /* fall through; */
        case 0x28: /* Master Transmitter: Data byte in I2DAT has been transmitted; ACK has been received */
                if( busptr->mst_txbufindex < busptr->mst_txbuflen )  /* transmit buffer not empty ? */
                {
                        write_IP0105_I2C_DAT(busptr, *( busptr->mst_txbuf + busptr->mst_txbufindex ));
                        busptr->mst_txbufindex++;
                        STAOUT( busptr, 0 ); /* Don't generate (repeated) START condition */
                }
                else
                {
                        if( busptr->mst_rxbufindex < busptr->mst_rxbuflen ) /* if bytes to receive  ? */
                        {
                                STAOUT( busptr, 1 ); /* Generate repeated START condition */
                        }
                        else
                        {
                                STAOUT( busptr, 0 ); /* Don't generate (repeated) START condition */
                                STOOUT( busptr, 1 ); /* Generate STOP condition */
                                busptr->mode = Idle;
                                busptr->isr_event |= evMasterReady;
                                wake_up_interruptible(&(busptr->iic_wait_master));
                        }
                }
                CLEAR_I2C_INTERRUPT( busptr ); /* reset interrupt */
                break;

        case 0x20: /* Master Transmitter: SLA+W has been transmitted; NOT ACK has been received */
                dev_dbg(&i2c_adap->dev, "Master Transmitter : SLA+W, NOT ACK has been received\n");
                /* fall through; */
        case 0x30: /* Master Transmitter: Data byte in I2DAT has been transmitted; NOT ACK has been received */
                STAOUT( busptr, 0 ); /* Don't generate (repeated) START condition */
                STOOUT( busptr, 1 ); /* Generate STOP condition */
                busptr->mode = Idle;
                busptr->mst_status = ( i2c_state == 0x20 ) ? errids_IsI2cDeviceError : errids_IsI2cWriteError;
                busptr->isr_event |= evMasterReady;
                CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
                wake_up_interruptible(&(busptr->iic_wait_master));
                break;

        case 0x38: /* Arbitration lost in SLA+R/~W or Data bytes */
                busptr->mode = Idle;
                STAOUT( busptr, 1 ); /* Retry the master operation */
                busptr->mst_txbufindex = 0;
                busptr->mst_rxbufindex = 0;
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after slave address reception */
                CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
                break;

        case 0x68: /* Arbitration lost in SLA+R/~W as master; Own SLA+W has been received, ACK has been returned */
                STAOUT( busptr, 1 ); /* Retry the master operation */
                busptr->mst_txbufindex = 0;
                busptr->mst_rxbufindex = 0;

                busptr->slv_bufindex = 1; /* reset buffer pointer !!! */
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after next byte reception */
                busptr->mode = SlaveReceiver;

//                Todo : Start timeout timer in tasklet
//                slave_device = i2c_adap;
//                tasklet_schedule(&IP0105_slave_tasklet);
                CLEAR_I2C_INTERRUPT(busptr); /* Don't reset i2c interrupt */
                break;

        case 0xB0: /* Arbitration lost in SLA+R/~W as master; Own SLA+R has been received, ACK has been returned */
                STAOUT( busptr, 1 ); /* Retry the master operation */
                busptr->mst_txbufindex = 0;
                busptr->mst_rxbufindex = 0;
                busptr->mode = SlaveTransmitter;

                busptr->isr_event |= evSlaveReadRequest;
                slave_device = i2c_adap;
                tasklet_schedule(&IP0105_slave_tasklet);
                /* Don't reset i2c interrupt */
                break;

        case 0x70: /* General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        case 0x78: /* Arbitration lost in SLA+R/~W as master; General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        default:
                recover_from_hardware_error( i2c_adap );
                break;
		}

}

static __inline void iic_interrupt_master_receiver(struct i2c_adapter * i2c_adap, struct I2cBusObject * busptr, int i2c_state)
{
        switch(i2c_state)
		{
		/* 0x10 : A repeated START should not occur when in MasterReceiver mode in this implementation */
        case 0x38: /* Arbitration lost in NOT ACK bit */
                busptr->mode = Idle;
                STAOUT( busptr, 1 ); /* Retry the master operation */
                busptr->mst_txbufindex = 0;
                busptr->mst_rxbufindex = 0;
                AAOUT( busptr, busptr->slv_enabled); /* ACK bit will be returned, or not, after slave address reception */
                CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
                break;

        case 0x50: /* Data byte has been received; ACK has been returned */
                if ( busptr->mst_rxbufindex < busptr->mst_rxbuflen ) /* bytes to receive ? */
                {
                        *( busptr->mst_rxbuf + busptr->mst_rxbufindex++ ) = read_IP0105_I2C_DAT(busptr);
                }
                else
                {
                        (void)read_IP0105_I2C_DAT(busptr); /* Ignore received byte, no storage space available */
                }
                /* fall through; */
        case 0x40: /* SLA+R has been transmitted; ACK has been received */
                if ( busptr->mst_rxbufindex < ( busptr->mst_rxbuflen - 1 ) )
                {
                        AAOUT( busptr, 1 ); /* ACK bit will be returned after byte reception */
                }
                else
                {
                        AAOUT( busptr, 0 ); /* NOT ACK bit will be returned after (last) byte reception */
                }
                STAOUT( busptr, 0 ); /* Don't generate (repeated) START condition */
                CLEAR_I2C_INTERRUPT( busptr ); /* reset interrupt */
                break;

        case 0x48: /* SLA+R has been transmitted; NOT ACK has been received */
                STAOUT( busptr, 0 ); /* Don't generate (repeated) START condition */
                STOOUT( busptr, 1 ); /* Generate STOP condition */
                busptr->mode = Idle;
                busptr->mst_status = errids_IsI2cDeviceError;
                busptr->isr_event |= evMasterReady;
                wake_up_interruptible(&(busptr->iic_wait_master));
                CLEAR_I2C_INTERRUPT( busptr ); /* reset interrupt */
                break;

        case 0x58: /* Data byte has been received; NOT ACK has been returned */
                if ( busptr->mst_rxbufindex < busptr->mst_rxbuflen ) /* bytes to receive ? */
                {
                        *( busptr->mst_rxbuf + busptr->mst_rxbufindex++ ) = read_IP0105_I2C_DAT(busptr);
dev_dbg(&i2c_adap->dev, "received byte %x\n", *(busptr->mst_rxbuf + busptr->mst_rxbufindex -1));
                }
                else
                {
                        (void)read_IP0105_I2C_DAT(busptr);
dev_dbg(&i2c_adap->dev, "ignore received byte\n");
                }
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after slave address reception */
                STAOUT( busptr, 0 ); /* Don't generate (repeated) START condition */
                STOOUT( busptr, 1 ); /* Generate STOP condition */
                busptr->mode = Idle;
                busptr->isr_event |= evMasterReady;
                CLEAR_I2C_INTERRUPT( busptr ); /* reset interrupt */
                wake_up_interruptible(&(busptr->iic_wait_master));
                break;

        case 0x68: /* Arbitration lost in SLA+R/~W as master; Own SLA+W has been received, ACK has been returned */
                STAOUT( busptr, 1 ); /* Retry the master operation */
                busptr->mst_txbufindex = 0;
                busptr->mst_rxbufindex = 0;

                busptr->slv_bufindex = 1; /* reset buffer pointer !!! */
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after next byte reception */
                busptr->mode = SlaveReceiver;

//                Todo : Start timeout timer in tasklet
//                tasklet_schedule(&IP0105_slave_tasklet);
                CLEAR_I2C_INTERRUPT(busptr); /* Don't reset i2c interrupt */
                break;

        case 0xB0: /* Arbitration lost in SLA+R/~W as master; Own SLA+R has been received, ACK has been returned */
                STAOUT( busptr, 1 ); /* Retry the master operation */
                busptr->mst_txbufindex = 0;
                busptr->mst_rxbufindex = 0;

                busptr->mode = SlaveReceiver;
                busptr->isr_event |= evSlaveReadRequest;
                slave_device = i2c_adap;
                tasklet_schedule(&IP0105_slave_tasklet);
                /* Don't reset i2c interrupt */
                break;

        case 0x70: /* General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        case 0x78: /* Arbitration lost in SLA+R/~W as master; General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        default:
				recover_from_hardware_error( i2c_adap );
				break;
		}
}

static __inline void iic_interrupt_slave_receiver(struct i2c_adapter * i2c_adap, struct I2cBusObject * busptr, int i2c_state)
{
        switch (i2c_state)
		{
        case 0x80: /* Previously addressed with own SLA; DATA byte has been received; ACK has been returned */
                if( busptr->slv_bufindex < busptr->slv_bufsize )
                {
                        *( busptr->slv_buf + busptr->slv_bufindex++ ) = read_IP0105_I2C_DAT(busptr);
                }
                else
                {
                        (void)read_IP0105_I2C_DAT(busptr);
                }
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after next byte reception */

                CLEAR_I2C_INTERRUPT( busptr );  /* Reset i2c interrupt */
				break;

        case 0x88: /* Previously addressed with own SLA; DATA byte has been received; NOT ACK has been returned */
                if ( busptr->slv_bufindex < busptr->slv_bufsize )
                {
                        *( busptr->slv_buf + busptr->slv_bufindex++ ) = read_IP0105_I2C_DAT(busptr);
                }
                else
                {
                        (void)read_IP0105_I2C_DAT(busptr);
                }

                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after slave address reception */
                CLEAR_I2C_INTERRUPT( busptr );    /* reset i2c interrupt */
				break;

        case 0xA0: /* A STOP condition or repeated START has been received while still address as SLV/REC or SLV/TRX */
				// Slave will be enabled again when data is out of the buffer
                AAOUT( busptr, 0 ); /* NOT ACK bit will be returned after slave address reception */

                busptr->isr_event |= evSlaveStopCondition;
                slave_device = i2c_adap;
                tasklet_schedule(&IP0105_slave_tasklet);
                CLEAR_I2C_INTERRUPT( busptr );   /* Reset i2c interrupt */
				break;

        case 0x70: /* General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        case 0x78: /* Arbitration lost in SLA+R/~W as master; General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        case 0x90: /* Previously addressed with General call; DATA byte has been received; ACK has been returned */
                /* fall through; */
        case 0x98: /* Previously addressed with General call; DATA byte has been received; NOT ACK has been returned */
                /* fall through; */
		default:
				recover_from_hardware_error( i2c_adap );
				break;
        }
}

static __inline void iic_interrupt_slave_transmitter(struct i2c_adapter * i2c_adap, struct I2cBusObject * busptr, int i2c_state)
{
        switch (i2c_state)
		{
		case 0xA0: /* A STOP condition or repeated START has been received while still address as SLV/TRX */
				// Slave will be enabled again when data is out of the buffer
                AAOUT( busptr, 0 ); /* NOT ACK bit will be returned after slave address reception */

                busptr->isr_event |= evSlaveStopCondition;
                slave_device = i2c_adap;
                tasklet_schedule(&IP0105_slave_tasklet);
                CLEAR_I2C_INTERRUPT( busptr );   /* Reset i2c interrupt */
                break;

        case 0xB8: /* Data byte in I2DAT has been transmitted; ACK has been received */
                if ( busptr->slv_bufindex < busptr->slv_bufsize )
                {
                        write_IP0105_I2C_DAT(busptr, *( busptr->slv_buf + busptr->slv_bufindex ));
                        busptr->slv_bufindex++;

                        if ( busptr->slv_bufindex < busptr->slv_buflen )
                        {
                                AAOUT( busptr, 1 ); /* Data byte will be transmitted */
                        }
                        else
                        {
                                AAOUT( busptr, 0 ); /* Last data byte will be transmitted */
                        }
                        CLEAR_I2C_INTERRUPT( busptr ); /* Reset interrupt */
                }
                else
                {
                        busptr->isr_event |= evSlaveReadRequest;
                        slave_device = i2c_adap;
                        tasklet_schedule(&IP0105_slave_tasklet);
                        /* Don't reset i2c interrupt */
                }
                break;

        case 0xC0: /* Data byte in I2DAT has been transmitted; NOT ACK has been received */
				/* fall through; */
        case 0xC8: /* Last data byte in I2DAT has been transmitted (AA=0); ACK has been received */
                AAOUT( busptr, busptr->slv_enabled ); /* ACK bit will be returned, or not, after slave address reception */

                busptr->isr_event |= evSlaveTransmitterReady;
                slave_device = i2c_adap;
                tasklet_schedule(&IP0105_slave_tasklet);
                /* Don't reset i2c interrupt */
                break;

        case 0x70: /* General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        case 0x78: /* Arbitration lost in SLA+R/~W as master; General call address (0x00) has been received; ACK has been returned */
                /* fall through; */
        default: /* Miscellaneous: No relevant state information available; SI=0 */
                recover_from_hardware_error( i2c_adap );
                break;
		}

}

static irqreturn_t IP0105_iic_interrupt(int irq, void *dev_id)
{
		// get the BusObject
		struct i2c_adapter * i2c_adap = (struct i2c_adapter *)dev_id;
        struct I2cBusObject * busptr = (struct I2cBusObject *)i2c_adap->algo_data;
        int i2c_state;

        // Get State of Iic Bus
        i2c_state = read_IP0105_I2C_STATUS(busptr);
        switch (busptr->mode)
		{
            case Idle:
                    iic_interrupt_idle(i2c_adap, busptr, i2c_state);
                    break;
            case MasterTransmitter:
                    iic_interrupt_master_transmitter(i2c_adap, busptr, i2c_state);
                    break;
            case MasterReceiver:
                    iic_interrupt_master_receiver(i2c_adap, busptr, i2c_state);
                    break;
            case SlaveReceiver:
                    iic_interrupt_slave_receiver(i2c_adap, busptr, i2c_state);
                    break;
            case SlaveTransmitter:
                    iic_interrupt_slave_transmitter(i2c_adap, busptr, i2c_state);
                    break;
		}
        return IRQ_HANDLED;
}

static void do_slave_tasklet(unsigned long unused)
{
        struct I2cBusObject * busptr = (struct I2cBusObject *)slave_device->algo_data;

        if (busptr->mode == SlaveReceiver)
        {
				if (busptr->isr_event & evSlaveRxIntermediateNtf)
				{
                        dev_dbg(&slave_device->dev, "Slave Receiver : Intermediate Ntf\n");
                        busptr->isr_event &= ~evSlaveRxIntermediateNtf;
                }
                if (busptr->isr_event & evSlaveReceiverReady)
                {
                        busptr->slv_bufindex = 1;
                        busptr->isr_event &= ~evSlaveReceiverReady;
                }
                if (busptr->isr_event & evSlaveReadRequest)
                {
                        dev_dbg(&slave_device->dev, "Slave Receiver : ReadRequest\n");
                        busptr->isr_event &= ~evSlaveReadRequest;
                }
                if (busptr->isr_event & evSlaveTransmitterReady)
                {
                        dev_dbg(&slave_device->dev, "Slave Receiver : TransmitterReady\n");
                        busptr->isr_event &= ~evSlaveTransmitterReady;
                }
                if (busptr->isr_event & evSlaveStopCondition)
                {
                        dev_dbg(&slave_device->dev, "Slave Receiver : SlaveStopCondition\n");
                        if (busptr->slv_usr_notify)
                        {
dev_dbg(&slave_device->dev, "Slave Receiver : Sending kill\n");
                                kill_fasync(busptr->slv_usr_notify, SIGIO, POLL_IN);
                        }
                        else  if (busptr->slv_enabled)
                        {
dev_dbg(&slave_device->dev, "Slave Receiver : AAOUT\n");
                                AAOUT(busptr, 1);
                        }

dev_dbg(&slave_device->dev, "Slave Receiver : Idle\n");
                        busptr->mode = Idle;
                        busptr->isr_event &= ~evSlaveStopCondition;
                }

        }
        else if (busptr->mode == SlaveTransmitter)
        {
                if (busptr->isr_event & evSlaveRxIntermediateNtf)
                        dev_dbg(&slave_device->dev, "Slave Transmitter : Intermediate Ntf\n");
                if (busptr->isr_event & evSlaveReceiverReady)
                        dev_dbg(&slave_device->dev, "Slave Transmitter : Ready\n");
                if (busptr->isr_event & evSlaveReadRequest)
                        dev_dbg(&slave_device->dev, "Slave Transmitter : ReadRequest\n");
                if (busptr->isr_event & evSlaveTransmitterReady)
                        dev_dbg(&slave_device->dev, "Slave Transmitter : TransmitterReady\n");
                if (busptr->isr_event & evSlaveStopCondition)
                        dev_dbg(&slave_device->dev, "Slave Transmitter : SlaveStopCondition\n");
        }

}

#define WAIT_IP0105INT()                do{ int t = TIMEOUT;                                       \
                                            while(!(read_IP0105_I2C_INT_STATUS(busptr) & IP0105_INTBIT)     \
                                                   && (t-->0)){}                               \
                                          }while(0)

#define WAIT_IP0105_STO_OR_INT()        do{ int t = TIMEOUT;                                       \
                                            while(!(read_IP0105_I2C_INT_STATUS(busptr) & IP0105_INTBIT)     \
                                                 &&(read_IP0105_I2C_CONTROL(busptr) & IP0105_STO)      \
                                                 &&(t-->0)){}                                  \
                                          }while(0)

/******************************************************************************
*   This function resets IP0105. The parameter "i2c_adap" indicates base address
*   of the IP0105 Block.
*******************************************************************************/
static void IP0105_reset(struct i2c_adapter *i2c_adap)
{
    struct I2cBusObject * busptr = (struct I2cBusObject *)i2c_adap->algo_data;

    dev_dbg(&i2c_adap->dev, "Reset the IP0105\n");

    DISABLE_I2C_INTERRUPT(busptr);
    AAOUT(busptr, 0);
    STAOUT(busptr, 0);

    if(1 /*read_IP0105_INTROG(busptr) & IP0105_INTRO_BB*/)
    {   /* Bus is busy */
        dev_dbg(&i2c_adap->dev, "I2C bus is busy\n");
        WAIT_IP0105INT();
dev_dbg(&i2c_adap->dev, "int_status = 0x%x\n", read_IP0105_I2C_INT_STATUS(busptr));
        if(read_IP0105_I2C_INT_STATUS(busptr) & IP0105_INTBIT)
        {/* Interrupt flag is set */
            unsigned int i2c_state = read_IP0105_I2C_STATUS(busptr);
            if((i2c_state == 0x08) || (i2c_state == 0x10))
            {
                write_IP0105_I2C_DAT(busptr, 0xEE);    /*  Transmit dummy address */
                CLEAR_I2C_INTERRUPT(busptr);           /* Clear I2C interrupt    */
            }
            else if((i2c_state == 0x40) || (i2c_state == 0x50))
            {   /* One byte must be read which should be NACKed */
                AAOUT(busptr, 0);                      /*  NACK next byte      */
                CLEAR_I2C_INTERRUPT(busptr);           /*  Clear I2C interrupt    */
            }
            else
            {
                /* For rest of the states just generating stop condition is enough */
            }
            WAIT_IP0105INT();
            STOOUT(busptr, 1);                         /*  Generate stop condition */
            CLEAR_I2C_INTERRUPT(busptr);               /*  Clear I2C interrupt: Not necessary but no harm */
            WAIT_IP0105_STO_OR_INT();
        }
        else
        {/* Interrupt flag did not set, May be due to clock stretching */
            STOOUT(busptr, 1);                         /*  Generate stop condition */
            CLEAR_I2C_INTERRUPT(busptr);               /*  Clear I2C interrupt: Not necessary but no harm */
            WAIT_IP0105_STO_OR_INT();
        }
    }
    else
    { /* Bus is free, do nothing */
        dev_dbg(&i2c_adap->dev, "I2C bus is free\n");
//        STOOUT(busptr, 1);                           /*  Generate stop condition */
//        CLEAR_I2C_INTERRUPT(busptr);                 /* Clear I2C interrupt: Not necessary but no harm */
//        WAIT_IP0105_STO_OR_INT();
    }

    if(read_IP0105_I2C_CONTROL(busptr) & IP0105_STO)
    {
        ASSERT(FALSE); /* Could not free I2C bus */
    }

    /* Set default values */
    DISABLE_I2C_CONTROLLER(busptr);      /*  Disable I2C controller */
    DISABLE_I2C_INTERRUPT(busptr);       /*  Disable I2C interrupts */
    AAOUT(busptr, 0);                    /*  Disable slave mode     */
    STAOUT(busptr, 0);                   /*  Remove start request   */
    CLEAR_I2C_INTERRUPT(busptr);         /*  Clear I2C interrupt    */

    dev_dbg(&i2c_adap->dev, "Reset done, re-init\n");
//    enable_irq(busptr->int_pin);
    /*  re-init again */
    ENABLE_I2C_CONTROLLER(busptr);
    ENABLE_I2C_INTERRUPT(busptr);
    if (busptr->slv_enabled == TRUE)
    {
        AAOUT(busptr, 1);
    }
    dev_dbg(&i2c_adap->dev, "re-init done\n");
}

/* Master ReadWrite interface function */
static unsigned int i2c_write_read(struct i2c_adapter * i2c_adap, int address, void *msgwram, int lenw, void *msgr, int lenr )
{
        struct I2cBusObject * busptr = (struct I2cBusObject *)i2c_adap->algo_data;

        int retval = 0;
        busptr->mst_status     = errids_Ok;

        busptr->mst_address    = address;
        busptr->mst_txbuflen   = lenw;
        busptr->mst_txbufindex = 0;
        busptr->mst_rxbuf      = msgr;
        busptr->mst_rxbufindex = 0;

        if( msgwram != NULL )
        {
            busptr->mst_txbuf = msgwram;
        }
        else
        {
            busptr->mst_txbuflen = 0; /* If both ptrs are NULL, do not write data */
        }

        if( msgr != NULL )
        {
            busptr->mst_rxbuflen = lenr;
        }
        else
        {
            busptr->mst_rxbuflen = 0;
        }

        STAOUT( busptr, 1 ); /* Generate START condition when selected bus is free and start interrupt driven machine */
        /* Suspend current task till the event flag is set */
        /* Wait for IIC transfer, result of action is written in I2cBusObject */
        retval = interruptible_sleep_on_timeout(&(busptr->iic_wait_master), (5*HZ));
        if (retval == 0)
        {
                dev_dbg(&i2c_adap->dev, "I2C Status 0%x, int stat %x\n",
                          read_IP0105_I2C_STATUS(busptr),
                          read_IP0105_I2C_INT_STATUS(busptr));
				pr_info("I2C0105 timeout\n");
                return errids_IsI2cHardwareError;
        }

        return busptr->mst_status;

}

static int IP0105_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
        struct i2c_msg *pmsg;
        void * wr_buf = NULL;
        void * rd_buf = NULL;
        int i, wr_len = 0, rd_len = 0;
        unsigned char addr = msgs[0].addr;
        int ret=0;

        dev_dbg(&i2c_adap->dev, "IP0105 xfer nr %d\n", num);
        ASSERT(num > 0 && num <= 2);
        for (i = 0; i < num; i++)
        {
                pmsg = &msgs[i];

                if (i == 0)
                {
                        if (pmsg->flags & I2C_M_TEN)
                        {
                                // addr = 10 bit addr, not supported yet
                        }
                        else
                        {
                                // addr = 7 bit addr
                                addr &= 0x7f;
                                addr <<= 1;
                        }
                }
                /*  wr_write handles all master read/write commands, including repeat start (I2C_M_NOSTART)
                */
                if (pmsg->flags & I2C_M_RD )
                {
                        /* read bytes into buffer*/
                        rd_buf = pmsg->buf;
                        rd_len = pmsg->len;
                }
                else
                {
                        /* write bytes from buffer */
                        wr_buf = pmsg->buf;
                        wr_len = pmsg->len;
                }
        }
        if (num != 0)
        {
                switch(ret = i2c_write_read(i2c_adap, addr, wr_buf, wr_len, rd_buf, rd_len))
                {
                case errids_Ok:
                        break;
                case errids_IsI2cHardwareError:
                        num = -1;
                        dev_dbg(&i2c_adap->dev, "Hardware error\n");
                        IP0105_reset(i2c_adap);
                        break;
                case errids_IsI2cDeviceError:
                        num = -1;
                        dev_dbg(&i2c_adap->dev, "Device error\n");
                        IP0105_reset(i2c_adap);
                        break;
                case errids_IsI2cWriteError:
                        num = -1;
                        dev_dbg(&i2c_adap->dev, "Write error\n");
                        IP0105_reset(i2c_adap);
                        break;
                default:
                        num = -1;
                        dev_dbg(&i2c_adap->dev, "Error Unkonwn\n");
                        IP0105_reset(i2c_adap);
                        break;
                }
        }

        return num;
}

static int algo_control(struct i2c_adapter *i2c_adap, unsigned int cmd, unsigned long arg)
{
        struct I2cBusObject * busptr =  (struct I2cBusObject *)i2c_adap->algo_data;
        switch (cmd)
        {
        case I2C_SET_SLAVE_ADDRESS:
                if (busptr->slv_enabled)
                        return -EBUSY;
                else
                {
                        write_IP0105_I2C_ADDRESS( busptr, I2CADDRESS( arg )); /* >> 1 */; /* 7 bits address, No General call support */
dev_dbg(&i2c_adap->dev, "Set Own adress to %x\n", read_IP0105_I2C_ADDRESS(busptr));
                }
                break;
        case I2C_SET_SLAVE_ENABLE:
                if (arg)
                {
                        if( !busptr->slv_enabled )
                        {
                               unsigned int len = (unsigned int)arg;
                               len++;  // Need an extra byte for the length
                               busptr->slv_buf = kmalloc(len, GFP_KERNEL);
                               if (NULL != busptr->slv_buf)
                               {
                                       busptr->slv_usr_notify = &i2c_adap->fasync;
                                       busptr->slv_bufindex = 1;
                                       busptr->slv_buflen = 0; // todo : slave transmitter, but we can only reveive
                                       busptr->slv_bufsize = len;

                                       busptr->slv_enabled = TRUE;
                                       AAOUT( busptr, 1 ); /* ACK bit will be returned after slave address reception */
                                       dev_dbg(&i2c_adap->dev, "Enable Slave\n");
                               }

                               else return -ENOMEM;
                        }
                        else return -EBUSY;
                }
                else
                {
                        busptr->slv_usr_notify = NULL;
                        busptr->slv_bufsize    = 0;
                        AAOUT( busptr, 0 ); /* ACK will not be sent after slave address reception */
                        busptr->slv_enabled = FALSE;

                        if (NULL != busptr->slv_buf)
                        {
                            kfree(busptr->slv_buf);  // kfree returns void, no check needed
                        }
                        busptr->slv_bufindex  = 0;
                        busptr->slv_buflen    = 0;

dev_dbg(&i2c_adap->dev, "Disable Slave\n");
                }
                break;
      case I2C_GET_SLAVE_DATA:
                {
                        unsigned long ret;
                        unsigned char * user_data = (unsigned char *)arg;
                        unsigned char nr_of_bytes = busptr->slv_bufindex -1;

                        busptr->slv_buf[0] = nr_of_bytes;
                        ret = copy_to_user(user_data, busptr->slv_buf, busptr->slv_bufindex);
                        busptr->slv_bufindex = 1;
                        if (busptr->slv_enabled == TRUE)
                        {
                                AAOUT( busptr, 1);  /* ACK bit will be returned after slave address reception */
                        }
                        if (0 == busptr->slv_buf[0])
                        {
                                return -ENODATA;
                        }
                }
                break;
        }
        return 0;
}

static u32 IP0105_func(struct i2c_adapter *adap)
{
        return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR |
               I2C_FUNC_PROTOCOL_MANGLING;
}

/* -----exported algorithm data: -------------------------------------  */

static struct i2c_algorithm IP0105_algo_0 = {
        .master_xfer = IP0105_xfer,
        .algo_control = algo_control,                   /* ioctl                */
        .functionality = IP0105_func,                     /* functionality        */
};
static struct i2c_algorithm IP0105_algo_1 = {
        .master_xfer = IP0105_xfer,
        .algo_control = algo_control,
        .functionality = IP0105_func,
};
static struct i2c_algorithm * IP0105_algo[NR_I2C_DEVICES] = { &IP0105_algo_0, &IP0105_algo_1 };


int i2c_IP0105_del_bus(struct i2c_adapter *i2c_adap)
{
        struct I2cBusObject *busptr;
        int res;

        dev_dbg(&i2c_adap->dev, "exit bus\n");
        busptr = (struct I2cBusObject *)i2c_adap->algo_data;

        if ((res = i2c_del_adapter(i2c_adap)) < 0)
                return res;
        DEB2(dev_dbg(&i2c_adap->dev, "i2c-algo-IP0105.o: adapter unregistered: %s\n",i2c_adap->name));

        return 0;
}

static struct i2c_adapter IP0105_ops_0 = {
       .name = "IP0105 0",                        // name
       .id = FAST_I2C_PORT_3,                // id
       .algo_data = &IP0105_i2cbus[0], // algo_data
};
static struct i2c_adapter IP0105_ops_1 = {
       .name = "IP0105 1",
       .id = FAST_I2C_PORT_4,
       .algo_data = &IP0105_i2cbus[1], // algo_data
};
static struct i2c_adapter * IP0105_ops[NR_I2C_DEVICES] = { &IP0105_ops_0, &IP0105_ops_1 };



int __init i2c_algo_IP0105_init (void)
{
    int device;
    printk("i2c-algo-IP0105.o: i2c IP0105 algorithm module\n");
    for (device = 0; device < NR_I2C_DEVICES; device++)
    {
        if (i2c_IP0105_add_bus(device, IP0105_ops[device]) < 0)
        {
            printk("i2c-algo-IP0105 %d: Unable to register with I2C\n", device);
            return -ENODEV;
        }
    }
    return 0;
}

void __exit i2c_algo_IP0105_exit(void)
{
    struct I2cBusObject *busptr;
    int device;

    for( device = 0; device < NR_I2C_DEVICES; device++ )
    {
        printk("exit bus %x\n", device);
        busptr = &IP0105_i2cbus[device];

        DISABLE_I2C_INTERRUPT( busptr );
        DISABLE_I2C_CONTROLLER( busptr ); /* Disable I2C controller */

        disable_irq(busptr->int_pin);   /* Enable i2c interrupt in Interrupt controller */
        free_irq(busptr->int_pin, (void *)&IP0105_i2cbus[device]);
        enable_irq(busptr->int_pin);   /* Enable i2c interrupt in Interrupt controller */

        i2c_IP0105_del_bus(IP0105_ops[device]);
    }
}

#endif
static __inline void enable_i2c_interrupt(resource_size_t offset)
{
        write_IP0105_I2C_INT_EN(offset, IP0105_INTBIT);
}

static __inline void enable_i2c_controller(resource_size_t offset)
{
        int val = read_IP0105_I2C_CONTROL(offset);

        val |= IP0105_EN;

        write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void clear_i2c_interrupt(resource_size_t offset)
{
        write_IP0105_I2C_INT_CLR(offset, 1);
}

static irqreturn_t i2c_ip0105_isr(int irq, void *dev_id)
{
	return 0;
}

static __inline void staout(resource_size_t offset, int p)
{
        int val = read_IP0105_I2C_CONTROL(offset);
        if (p)
		val |= IP0105_STA;
	else
		val &= ~IP0105_STA;

        write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void stoout(resource_size_t offset, int p)
{
        write_IP0105_I2C_STOP(offset, p);
}

static __inline void aaout(resource_size_t offset, int p)
{
        int val = read_IP0105_I2C_CONTROL(offset);
        if (p)
		val |= IP0105_AA;
	else
		val &= ~IP0105_AA;

        write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void setspeed100(resource_size_t offset)
{
        int val = read_IP0105_I2C_CONTROL(offset);

        val &= 0x00F0;
        val |= 0x00F4;

        write_IP0105_I2C_CONTROL(offset, val);
}

static __inline void disable_i2c_controller(resource_size_t offset)
{
        int val = read_IP0105_I2C_CONTROL(offset);

        val &= ~IP0105_EN;

	write_IP0105_I2C_CONTROL(offset, val);
}

static int i2c_ip0105_xfer(struct i2c_adapter *adapter,
		struct i2c_msg *msgs, int num)
{
	return 0;
}

static u32 i2c_ip0105_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

struct i2c_ip0105_struct {
	wait_queue_head_t	i2c_wait_master;
	resource_size_t		offset;
	int			irq;
	struct i2c_adapter	ip0105_adap;
	void __iomem		*base;
	struct resource		*res;
};

static struct i2c_algorithm i2c_ip0105_algo = {
	.master_xfer	= i2c_ip0105_xfer,
	.functionality	= i2c_ip0105_func,
};

static int __devinit i2c_ip0105_probe(struct platform_device *pdev) {
	struct i2c_ip0105_struct *i2c_ip0105;
	struct resource *res;
	resource_size_t res_size;
	void __iomem *base;
	int ret;

	printk("%s() %s:%d\n", __func__, __FILE__, __LINE__);
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

	i2c_ip0105->irq = platform_get_irq(pdev, 0);
	if (i2c_ip0105->irq < 0) {
		dev_err(&pdev->dev, "can't get irq number\n");
		return -ENOENT;
	}

	i2c_ip0105->offset = res->start;
	res_size = resource_size(res);

	if (!request_mem_region(res->start, res_size, pdev->name))
		return -EBUSY;

	base = ioremap(res->start, res_size);
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -EIO;
	}

	init_waitqueue_head(&(i2c_ip0105->i2c_wait_master));

	disable_i2c_controller(i2c_ip0105->offset);
	setspeed100(i2c_ip0105->offset);

	aaout(i2c_ip0105->offset, 0);	/* Slave mode disabled */
	stoout(i2c_ip0105->offset, 0);	/* Do not generate stop condition */
	staout(i2c_ip0105->offset, 0);	/* Do not generate Start condition */

	ret = request_irq(i2c_ip0105->irq, i2c_ip0105_isr, 0, pdev->name, (void *)i2c_ip0105);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n", i2c_ip0105->irq);
		return ret;
	}

	platform_set_drvdata(pdev, i2c_ip0105);

	clear_i2c_interrupt(i2c_ip0105->offset);
//	disable_irq(i2c_ip0105->irq);	/* disable i2c interrupt in Interrupt controller */
//	enable_irq(i2c_ip0105->irq);	/* enable i2c interrupt in Interrupt controller */

	enable_i2c_controller(i2c_ip0105->offset); /* Enable I2C Controller */
	enable_i2c_interrupt(i2c_ip0105->offset); /* Enable  both the SI and DMA interrupts */

	strcpy(i2c_ip0105->ip0105_adap.name, pdev->name);
	i2c_ip0105->ip0105_adap.owner		= THIS_MODULE;
	i2c_ip0105->ip0105_adap.algo		= &i2c_ip0105_algo;
	i2c_ip0105->ip0105_adap.dev.parent	= &pdev->dev;
	i2c_ip0105->ip0105_adap.nr		= pdev->id;
	i2c_ip0105->irq				= i2c_ip0105->irq;
	i2c_ip0105->base			= base;
	i2c_ip0105->res				= res;

	if ((ret = i2c_add_numbered_adapter(&i2c_ip0105->ip0105_adap)) < 0) {
		dev_dbg(&pdev->dev, "i2c-algo-IP0105: Unable to register with I2C\n");
		free_irq(i2c_ip0105->irq, i2c_ip0105);
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "Init i2c bus\n");

	return 0;
}

static int __devexit i2c_ip0105_remove(struct platform_device *pdev) {
	struct i2c_ip0105_struct *i2c_ip0105;
	
	printk("%s() %s:%d\n", __func__, __FILE__, __LINE__);
	i2c_ip0105 = platform_get_drvdata(pdev);
	free_irq(i2c_ip0105->irq, i2c_ip0105);

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
	printk("%s() %s:%d\n", __func__, __FILE__, __LINE__);
	return platform_driver_register(&i2c_ip0105_driver);
}

void __exit i2c_algo_ip0105_exit(void)
{
	printk("%s() %s:%d\n", __func__, __FILE__, __LINE__);
	platform_driver_unregister(&i2c_ip0105_driver);
}

MODULE_AUTHOR("Joel Thomas <jt@emlix.com>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for IP0105");
MODULE_LICENSE("GPL");

subsys_initcall(i2c_algo_ip0105_init);
module_exit(i2c_algo_ip0105_exit);


