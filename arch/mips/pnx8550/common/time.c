/*
 * Copyright 2001, 2002, 2003 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 * Copyright (C) 2007 Ralf Baechle (ralf@linux-mips.org)
 *
 * Common time service routines for MIPS machines. See
 * Documents/MIPS/README.txt.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/smp.h>
#include <linux/kernel_stat.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/time.h>
#include <asm/hardirq.h>
#include <asm/div64.h>
#include <asm/debug.h>

#include <int.h>
#include <cm.h>

static unsigned long cpj;
cycle_t pnx8xxx_hpt_res;
long pnx8xxx_call_count = 0;
EXPORT_SYMBOL(pnx8xxx_hpt_res);
EXPORT_SYMBOL(pnx8xxx_call_count);


static inline void timer_ack(void)
{
	write_c0_compare(cpj);
}

static cycle_t hpt_read(struct clocksource *cs)
{

	pnx8xxx_hpt_res = read_c0_count2();

	return pnx8xxx_hpt_res;
}

static int pnx8xxx_set_next_event(unsigned long delta,
				struct clock_event_device *evt)
{
	unsigned long cnt;
	int res;

	//printk(KERN_INFO "pnx8xxx_set_next_event d: %lu c: %lu c+d: %lu\n", delta, cnt, cnt+delta);
	pnx8xxx_call_count++;
	cnt = read_c0_count();
	cnt += delta;
	write_c0_compare(delta);

	res = ((int)(read_c0_count() - cnt) >= 0) ? -ETIME : 0;

	return res;
}

static void pnx8xxx_set_mode(enum clock_event_mode mode, struct clock_event_device *cd) {
	printk(KERN_INFO "called pnx8xxx_set_mode(%i)\n", mode);
}

static struct clock_event_device pnx8xxx_clockevent = {
	.name		= "pnx8xxx_clockevent",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 300,
	.set_next_event = pnx8xxx_set_next_event,
	.set_mode	= pnx8xxx_set_mode,
};

static struct clocksource pnx_clocksource = {
	.name		= "pnx8xxx",
	.rating		= 200,
	.read		= hpt_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static irqreturn_t pnx8xxx_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	write_c0_compare(read_c0_compare());
	c->event_handler(c);

	return IRQ_HANDLED;
}


static struct irqaction pnx8xxx_timer_irq = {
	.handler	= pnx8xxx_timer_interrupt,
	.flags		= IRQF_DISABLED | IRQF_PERCPU | IRQF_TIMER,
	.name		= "pnx8xxx_timer",
	.dev_id		= &pnx8xxx_clockevent,
};

static irqreturn_t monotonic_interrupt(int irq, void *dev_id)
{
	write_c0_compare2(-1);
	return IRQ_HANDLED;
}

static struct irqaction monotonic_irqaction = {
	.handler = monotonic_interrupt,
	.flags = IRQF_DISABLED | IRQF_TIMER,
	.name = "Monotonic timer",
};



/*
void pnx8xxx_event_handler(struct clock_event_device *dev)
{
}
*/

__init void plat_time_init(void)
{
	unsigned int configPR;
	unsigned int n;
	unsigned int m;
	unsigned int p;
	unsigned int pow2p;
	unsigned int cpu = smp_processor_id();

        /* PLL0 sets MIPS clock (PLL1 <=> TM1, PLL6 <=> TM2, PLL5 <=> mem) */
        /* (but only if CLK_MIPS_CTL select value [bits 3:1] is 1:  FIXME) */

	/* read random registers */
        n = (PNX8550_CM_PLL0_CTL & PNX8550_CM_PLL_N_MASK) >> 16;
        m = (PNX8550_CM_PLL0_CTL & PNX8550_CM_PLL_M_MASK) >> 8;
        p = (PNX8550_CM_PLL0_CTL & PNX8550_CM_PLL_P_MASK) >> 2;
	pow2p = (1 << p);

	db_assert(m != 0 && pow2p != 0);

        /*
	 * Compute the frequency as in the PNX8550 User Manual 1.0, p.186
	 * (a.k.a. 8-10).  Divide by HZ for a timer offset that results in
	 * HZ timer interrupts per second.
	 */
	// bogomips 249.34
	mips_hpt_frequency = 27UL * ((1000000UL * n)/(m * pow2p));
	//pnx8xxx_clockevent.rating = 200 + mips_hpt_frequency / 10000000;
	cpj = DIV_ROUND_CLOSEST(mips_hpt_frequency, HZ);

	clockevents_calc_mult_shift(&pnx8xxx_clockevent, mips_hpt_frequency, 2);
	
	pnx8xxx_clockevent.max_delta_ns = clockevent_delta2ns(0x7fffffff, &pnx8xxx_clockevent);
	pnx8xxx_clockevent.min_delta_ns = clockevent_delta2ns(0x300, &pnx8xxx_clockevent);

	pnx8xxx_clockevent.cpumask = cpumask_of(cpu);
//	pnx8xxx_clockevent.event_handler = pnx8xxx_event_handler;

	printk("mips_hpt_frequency: %d\n", mips_hpt_frequency);
	printk("pnx_clocksource.rating: %d\n", pnx_clocksource.rating);
	printk("cpj: %ld\n", cpj);
	printk("pnx8xxx_clockevent.mult: %d\n", pnx8xxx_clockevent.mult);
	printk("pnx8xxx_clockevent.shift: %d\n", pnx8xxx_clockevent.shift);
	printk("pnx8xxx_clockevent.min_delta_ns: %llu\n",pnx8xxx_clockevent.min_delta_ns);
	printk("pnx8xxx_clockevent.max_delta_ns: %llu\n",pnx8xxx_clockevent.max_delta_ns);
	printk(KERN_INFO "clockevent %u %u %u registered\n", mips_hpt_frequency, pnx8xxx_clockevent.mult, pnx8xxx_clockevent.shift);

	/* Timer 1 start */
	configPR = read_c0_config7();
	configPR &= ~0x00000008;
	write_c0_config7(configPR);

	/* Timer 2 start */
	configPR = read_c0_config7();
	configPR &= ~0x00000010;
	write_c0_config7(configPR);

	/* Timer 3 stop */
	configPR = read_c0_config7();
	configPR |= 0x00000020;
	write_c0_config7(configPR);

	write_c0_count(0);
	timer_ack();

	/* Setup Timer 2 */
	write_c0_count2(0);
	write_c0_compare2(0xffffffff);

	clocksource_register_hz(&pnx_clocksource, mips_hpt_frequency);
	clockevents_register_device(&pnx8xxx_clockevent);

	setup_irq(PNX8550_INT_TIMER1, &pnx8xxx_timer_irq);
	setup_irq(PNX8550_INT_TIMER2, &monotonic_irqaction);
}
