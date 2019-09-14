/*
 * pic32_clock.h --- clock/oscillator control
 */

#include <xc.h>
#include <stdint.h>

#ifndef __PIC32CLK_H__

static inline uint32_t pic32clk_pb_clock(void){
#if defined(PB1DIV)
    return SYS_CLOCK / ((PB1DIV & _PB1DIV_PBDIV_MASK) + 1);
#elif defined(_OSCCON_PBDIV_MASK)
    return SYS_CLOCK>>OSCCONbits.PBDIV;
#else
    return SYS_CLOCK;
#endif
}

#endif
