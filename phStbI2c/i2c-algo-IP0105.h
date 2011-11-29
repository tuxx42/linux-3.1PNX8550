/* ------------------------------------------------------------------------- */
/* i2c-algo-IP3203.h i2c driver algorithms for VIPER2           		     */
/* ------------------------------------------------------------------------- */

/* $Id: $ */

#ifndef I2C_ALGO_IP3203_H
#define I2C_ALGO_IP3203_H 1

#include <linux/i2c.h>
#include <asm/io.h>

/* Local Macros for IP0105 */
#define read_IP0105_I2C_CONTROL(offset)      readl((u32*)((offset)+0x000000000))
#define read_IP0105_I2C_DAT(offset)               readl((u32*)((offset)+0x000000004))
#define read_IP0105_I2C_STATUS(offset)            readl((u32*)((offset)+0x000000008))
#define read_IP0105_I2C_ADDRESS(offset)           readl((u32*)((offset)+0x00000000C))
#define read_IP0105_I2C_STOP(offset)              readl((u32*)((offset)+0x000000010))
#define read_IP0105_I2C_PD(offset)                readl((u32*)((offset)+0x000000014))
#define read_IP0105_I2C_SET_PINS(offset)          readl((u32*)((offset)+0x000000018))
#define read_IP0105_I2C_OBS_PINS(offset)          readl((u32*)((offset)+0x00000001C))
#define read_IP0105_I2C_INT_STATUS(offset)        readl((u32*)((offset)+0x000000FE0))
#define read_IP0105_I2C_INT_EN(offset)            readl((u32*)((offset)+0x000000FE4))
#define read_IP0105_I2C_INT_CLR(offset)           readl((u32*)((offset)+0x000000FE8))
#define read_IP0105_I2C_INT_SET(offset)           readl((u32*)((offset)+0x000000FEC))
#define read_IP0105_I2C_POWERDOWN(offset)         readl((u32*)((offset)+0x000000FF4))
#define read_IP0105_MODULE_ID(offset)             readl((u32*)((offset)+0x000000FFC))

#define write_IP0105_I2C_CONTROL(offset, l)       writel(l,(u32*)((offset)+0x000000000))
#define write_IP0105_I2C_DAT(offset, l)                writel(l,(u32*)((offset)+0x000000004))
#define write_IP0105_I2C_STATUS(offset, l)             writel(l,(u32*)((offset)+0x000000008))
#define write_IP0105_I2C_ADDRESS(offset, l)            writel(l,(u32*)((offset)+0x00000000C))
#define write_IP0105_I2C_STOP(offset, l)          writel(l,(u32*)((offset)+0x000000010))
#define write_IP0105_I2C_PD(offset, l)                 writel(l,(u32*)((offset)+0x000000014))
#define write_IP0105_I2C_SET_PINS(offset, l)           writel(l,(u32*)((offset)+0x000000018))
#define write_IP0105_I2C_OBS_PINS(offset, l)           writel(l,(u32*)((offset)+0x00000001C))
#define write_IP0105_I2C_INT_STATUS(offset, l)         writel(l,(u32*)((offset)+0x000000FE0))
#define write_IP0105_I2C_INT_EN(offset, l)             writel(l,(u32*)((offset)+0x000000FE4))
#define write_IP0105_I2C_INT_CLR(offset, l)       writel(l,(u32*)((offset)+0x000000FE8))
#define write_IP0105_I2C_INT_SET(offset, l)            writel(l,(u32*)((offset)+0x000000FEC))
#define write_IP0105_I2C_POWERDOWN(offset, l)          writel(l,(u32*)((offset)+0x000000FF4))
#define write_IP0105_MODULE_ID(offset, l)              writel(l,(u32*)((offset)+0x000000FFC))


#define IP0105_AA                       (0x80) /* Bit in the register I2CCON     */
#define IP0105_EN                       (0x40) /* Bit in the register I2CCON     */
#define IP0105_STA                      (0x20) /* Bit in the register I2CCON     */
#define IP0105_STO                      (0x10) /* Bit in the register I2CCON     */
#define IP0105_CR2                      (0x03) /* Bits in the register I2CCON     */
#define IP0105_INTBIT                   (0x01) /* Bit in the register INT_STATUS */


struct i2c_algo_IP0105_data {
    void * baseAddress;
};

#define TMHW_I2C_MAX_SS_SPEED  100  // kHz
#define TMHW_I2C_MAX_FS_SPEED  400  // kHz
#define TMHW_I2C_MAX_HS_SPEED 3400  // kHz



#endif /* I2C_ALGO_IP3203_H */
