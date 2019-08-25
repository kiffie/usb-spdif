/*
 * timer.h --- timer
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>
#include <sys/time.h>

#ifndef SYS_CLOCK
    #error "Macro SYS_CLOCK not defined (must be system clock im Hz)"
#endif


#ifndef TIMER_NO_TIMERS
#define TIMER_NO_TIMERS 16
#endif

#define TIMER_TICKS_PER_US (SYS_CLOCK/2000000)
#define TIMER_TICKS_PER_MS (TIMER_TICKS_PER_US*1000)
#define TIMER_TICKS_PER_SECOND (TIMER_TICKS_PER_US*1000000)

#define TIMER_EARLIEST 0
#define TIMER_LATEST UINT64_MAX

typedef uint64_t timer_time_t;

typedef void (* Timer_handler)(void *arg, timer_time_t when);

void timer_init(void);

__attribute__((nomips16)) timer_time_t timer_now();

/* busy waiting for an interval specified in microseconds */
void timer_wait_ms(int ms);
void timer_wait_us(int us);

/* put device into idle mode for at most max_idletime ms */
/* returns when an IRQ occurs or when at least one timer has expired */
__attribute__((nomips16)) void timer_idle_ms(unsigned max_idletime);


/* returns a handle >= 0 on success */
int timer_add(Timer_handler h, timer_time_t t, int periodic, void *arg);
#define timer_periodic(h, interval, arg) timer_add(h, interval, 1, arg)
void timer_del(int handle);

#ifdef ENABLE_TIMER_EVENTS
    void timer_iteration(void);
#else
    static inline __attribute__((always_inline)) void timer_iteration(void) {}
#endif

#endif
