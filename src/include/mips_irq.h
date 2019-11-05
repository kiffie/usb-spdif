/*
 * helper functions for IRQ control and handling
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#ifndef __MIPS_IRQ_H__
#define __MIPS_IRQ_H__

#ifdef __XC32__
#include <sys/attribs.h>
#endif

unsigned __attribute__((nomips16)) mips_di(void);

unsigned __attribute__((nomips16)) mips_ei(void);

void __attribute__ ((nomips16)) mips_restore_irq(unsigned previous_status);

void __attribute__((nomips16)) mips_enable_mv_irq(void);


/*
 * macros for Interrupt Service Routines (ISR)
 * to be used in combination with standard gcc toolchain
 */

#ifndef __XC32__
/* generate ISR dispatch code to be filled into the respective
 * exception vector section of IRQ vector number n_vector; generates
 * exception vector table entry for ISR isr_func
 */
#define __MIPS_ISR_DISPATCH__(n_vector, isr_func) \
	void __attribute__((section(".vector_"#n_vector), nomips16,optimize("Os"))) vector_##n_vector##_dispatch(void)\
	{ asm volatile("j " #isr_func); }

/* wrapper to enforce expansion of n_vector */
#define MIPS_ISR_DISPATCH(n_vector, isr_func) __MIPS_ISR_DISPATCH__(n_vector, isr_func)

/* function attributes for an ISR */
#define __ISR(...) __attribute__((interrupt,nomips16))


#define MIPS_ISR_DEFINE(n_vector) \
	MIPS_ISR_DISPATCH(n_vector, vector_##n_vector##_isr) \
	void __ISR() vector_##n_vector##_isr(void)

#endif

/*
 * macros for defining IRQ free code sections (for compatibility with avrlibc)
 */
static inline void __irq_cleanup_restore(unsigned *p){
     mips_restore_irq(*p);
}

static inline void __irq_cleanup_ei(unsigned *p){
    mips_ei();
}

#define ATOMIC_RESTORESTATE unsigned __irq_stat \
    __attribute__((__cleanup__(__irq_cleanup_restore))) = mips_di()

#define ATOMIC_FORCEON unsigned __irq_stat \
    __attribute__((__cleanup__(__irq_cleanup_ei))) = mips_di()

#define ATOMIC_BLOCK(type) for(type, __ToDo = 1; __ToDo; __ToDo = 0)


#endif
