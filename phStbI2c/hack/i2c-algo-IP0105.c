/*
 * Provides I2C support for Philips PNX8550 boards.
 *
 * Authors: Joel Thomas <joelthomas@bdn.de>
 *
 * 2004-2006 (c) Joel Thomas. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/i2c-pnx.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>


static int __devinit i2c_ip0105_probe(struct platform_device *pdev) {return 0;}
static int __devexit i2c_ip0105_remove(struct platform_device *pdev) {return 0;}
static int i2c_ip0105_controller_suspend(struct platform_device *pdev, pm_message_t state) {return 0;}
static int i2c_ip0105_controller_resume(struct platform_device *pdev) {return 0;}

static struct platform_driver i2c_ip0105_driver = {
	.driver = {
		.name = "IP0105",
		.owner = THIS_MODULE,
	},
	.probe = i2c_ip0105_probe,
	.remove = __devexit_p(i2c_ip0105_remove),
	.suspend = i2c_ip0105_controller_suspend,
	.resume = i2c_ip0105_controller_resume,
};

static int __init i2c_algo_ip0105_init(void)
{
	return platform_driver_register(&i2c_ip0105_driver);
}

void __exit i2c_algo_ip0105_exit(void)
{
	platform_driver_unregister(&i2c_ip0105_driver);
}

MODULE_AUTHOR("Joel Thomas <joelthomas@bdn.de>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for IP0105");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ip0105-i2c");

subsys_initcall(i2c_algo_ip0105_init);
module_exit(i2c_algo_ip0105_exit);
