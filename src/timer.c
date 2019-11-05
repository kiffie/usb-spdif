/*
 * timer.c --- timer for MIPS using CP0 counter register
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <timer.h>

#include <xc.h>
#include <mips_irq.h>
#include <sys/attribs.h>
#include <logger.h>

#ifndef LOGLEVEL_TIMER
#define LOGLEVEL_TIMER LOG_ERROR
#endif
#define LOGLEVEL LOGLEVEL_TIMER
#define LOG_PREFIX "TIMER"

#define TIMER_MIN_PERIOD 100 /* min period in ticks for IRQ */
#define TIMER_IRQ_PERIOD_MS 60000UL /* at least one IRQ per minute */


struct timer_event {
    unsigned used : 1;
    unsigned periodic : 1;
    Timer_handler handler;
    timer_time_t when;
    timer_time_t interval;
    void *arg;
};


static uint32_t timer_high;
static uint32_t last_timer_low;

static  inline void __attribute__((nomips16)) write_to_compare(uint32_t v){
    asm volatile("mtc0   %0, $11" : : "r"(v));
}

#ifdef ENABLE_TIMER_EVENTS

static struct timer_event timers[TIMER_NO_TIMERS];

static int timer_expires_next(void){
    int i, next= -1;
    timer_time_t now= timer_now();
    timer_time_t earliest= UINT64_MAX;
    for(i=0; i<TIMER_NO_TIMERS; i++){
        if( timers[i].used &&
            (timers[i].when >=  now + TIMER_MIN_PERIOD)  &&
            (timers[i].when < earliest) )
        {
            next= i;
            earliest= timers[i].when;
        }
    }
    return next;
}

int timer_add(Timer_handler h, timer_time_t t, int periodic, void *arg){
    int i=0, ndx=-1;
    while( i<TIMER_NO_TIMERS && ndx==-1 ){
        if( !timers[i].used ){
            ndx= i;
        }
        i++;
    }
    if( ndx>= 0){
        timers[ndx].used= 1;
        timers[ndx].handler= h;
        timers[ndx].when= timer_now()+t;
        timers[ndx].periodic= periodic;
        timers[ndx].interval= periodic? t : 0;
        return ndx;
    }else{
        return -1;
    }
}

void timer_del(int handle){
    if(handle>=0 && handle<TIMER_NO_TIMERS){
        timers[handle].used= 0;
    }
}

void timer_iteration(void) {
    int i;
    for (i= 0; i < TIMER_NO_TIMERS; i++) {
        if (timers[i].used) {
            struct timer_event *p = &timers[i];
            if (timer_now() >= p->when) {
                p->handler(p->arg, p->when);
                if( p->periodic){
                    p->when= timer_now() + p->interval;
                }else{
                    p->used= 0;
                }
            }
        }
    }
}


#endif

/*
 * initialize timer
 * multi-vectored interrupts must be enabled before calling this function
 */

void __attribute__((nomips16)) timer_init(void){
#ifdef ENABLE_TIMER_EVENTS
    int i;
    for(i=0; i<TIMER_NO_TIMERS; i++){
        timers[i].used= 0;
    }
#endif
    timer_high= 0;
    IEC0SET= _IEC0_CTIE_MASK;
    IPC0bits.CTIP= 1;
    uint32_t compare= timer_now() + TIMER_IRQ_PERIOD_MS*TIMER_TICKS_PER_MS;
    write_to_compare(compare);
    log_debug("timer initialized (SYS_CLOCK=%u, %u ticks per µs)\n",
	      SYS_CLOCK, TIMER_TICKS_PER_US);
}

/* inline this in in order to avoid function calls in ISR */
static inline void __attribute__((nomips16)) update_timer_high(void){
    uint32_t timer_low;
    asm volatile("mfc0   %0, $9" : "=r"(timer_low));
    if( timer_low < last_timer_low ){ /* detect overflow of timer */
        timer_high++;
	//log_debug("timer_high++ now==%u:%u\n", timer_high, timer_low);
    }
    last_timer_low= timer_low;
}

timer_time_t __attribute__((nomips16)) timer_now(){
    uint32_t timer_low;
    unsigned irq_state= mips_di();
    update_timer_high();
    asm volatile("mfc0   %0, $9" : "=r"(timer_low));
    timer_time_t now= (((timer_time_t)timer_high)<<32)|timer_low;
    mips_restore_irq(irq_state);
    return now;
}

void timer_wait_us(int us){
	timer_time_t then= timer_now() + TIMER_TICKS_PER_US * us;
	while( then > timer_now() );
}

void timer_wait_ms(int ms){
	timer_time_t then= timer_now() + TIMER_TICKS_PER_MS * ms;
	while( then > timer_now() );
        then= 0;
}

void __attribute__((nomips16)) timer_idle_ms(unsigned max_idletime){
    timer_time_t now= timer_now();
    uint32_t compare;
#ifdef ENABLE_TIMER_EVENTS
    int next_timer= timer_expires_next();
    if( next_timer >= 0 && timers[next_timer].when < now+TIMER_TICKS_PER_MS*max_idletime){
        compare= timers[next_timer].when;
    }else{
        compare= now + TIMER_TICKS_PER_MS*max_idletime;
    }
#else
    compare= now + TIMER_TICKS_PER_MS*max_idletime;
#endif
    write_to_compare(compare);
    asm volatile("nop");
    asm volatile("wait");
}

void __ISR(_CORE_TIMER_VECTOR, IPL1SOFT) core_timer_isr(void) {
    update_timer_high();
    
    /* schedule next IRQ */
    uint32_t timer_low;
    asm volatile("mfc0   %0, $9" : "=r"(timer_low));
    uint32_t compare= timer_low + TIMER_IRQ_PERIOD_MS*TIMER_TICKS_PER_MS;
    write_to_compare(compare);
    IFS0CLR= _IFS0_CTIF_MASK;
}

/* just a compatibility wrapper */
int gettimeofday(struct timeval *tv , void *tz){
    timer_time_t now= timer_now();
    tv->tv_sec= now / TIMER_TICKS_PER_SECOND;
    tv->tv_usec= (now - TIMER_TICKS_PER_SECOND*tv->tv_sec)/TIMER_TICKS_PER_US;
    return 0;
}
