/*
 * helper functions for IRQ control and handling
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <mips_irq.h>
#include <xc.h>

unsigned __attribute__((noinline, nomips16)) mips_di(void)
{
	unsigned previous_status;
	asm volatile("di   %0" : "=r"(previous_status));
	return previous_status;
}

unsigned __attribute__((noinline, nomips16)) mips_ei(void)
{
	unsigned previous_status;
	asm volatile("ei   %0" : "=r"(previous_status));
	return previous_status;
}

void __attribute__((noinline, nomips16)) mips_restore_irq(unsigned previous_status)
{
	if( previous_status & 1 ){
		asm volatile("ei");
	}else{
		asm volatile("di");
	}
}


/*
 * enable multi-vectored interrupts
 */
void __attribute__((noinline, nomips16)) mips_enable_mv_irq(void)
{
	unsigned val;
	/* set the CP0 cause IV bit */
	asm volatile("mfc0   %0,$13" : "=r"(val));
	val |= 0x00800000;
	asm volatile("mtc0   %0,$13" : "+r"(val));

	INTCONSET = _INTCON_MVEC_MASK;

	mips_ei();
}


/* end */
