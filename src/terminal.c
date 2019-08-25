/*
 * terminal functions using UART 2
 */

#ifndef TERM_TXBUFSIZE
#define TERM_TXBUFSIZE 0
#endif

#include "terminal.h"
#include <tinyprintf.h>
#include <xc.h>
#include <stdbool.h>

#if TERM_TXBUFSIZE > 0
#include <mips_irq.h>
#endif

#ifndef TERM_PORT
#define TERM_PORT 2
#endif

#ifndef TERM_IPL
#define TERM_IPL 7
#endif
#define TERM_IPL_TAG_STR(ipl_str) IPL ## ipl_str ## SOFT
#define TERM_IPL_TAG(ipl) TERM_IPL_TAG_STR(ipl)

#if TERM_PORT == 1
#   define UxMODE	U1MODE
#   define UxMODESET	U1MODESET
#   define UxSTA	U1STA
#   define UxTXREG	U1TXREG
#   define UxBRG	U1BRG
#   define UxVECTOR     _UART_1_VECTOR
#   if __PIC32_FEATURE_SET == 250 || __PIC32_FEATURE_SET == 270
#       define UxIPC        IPC8bits.U1IP
#       define UxTX_IFSCLR  IFS1CLR = _IFS1_U1TXIF_MASK
#       define UxTXIE       IEC1SET = _IEC1_U1TXIE_MASK
#       define UxTXDI       IEC1CLR = _IEC1_U1TXIE_MASK
#   elif __PIC32_FEATURE_SET == 470
#       define UxIPC        IPC7bits.U1IP
#       define UxTX_IFSCLR  IFS1CLR = _IFS1_U1TXIF_MASK
#       define UxTXIE       IEC1SET = _IEC1_U1TXIE_MASK
#       define UxTXDI       IEC1CLR = _IEC1_U1TXIE_MASK
#   endif
#elif TERM_PORT == 2
#   define UxMODE	U2MODE
#   define UxMODESET	U2MODESET
#   define UxSTA	U2STA
#   define UxTXREG	U2TXREG
#   define UxBRG	U2BRG
#   define UxVECTOR     _UART_2_VECTOR
#   if __PIC32_FEATURE_SET == 250 || __PIC32_FEATURE_SET == 270 || __PIC32_FEATURE_SET == 470
#       define UxIPC        IPC9bits.U2IP
#       define UxTX_IFSCLR  IFS1CLR = _IFS1_U2TXIF_MASK
#       define UxTXIE       IEC1SET = _IEC1_U2TXIE_MASK
#       define UxTXDI       IEC1CLR = _IEC1_U2TXIE_MASK
#   endif
#else
#   error "invalid UART number"
#endif

#ifndef SYS_CLOCK
    #error "Macro SYS_CLOCK not defined (must be system clock im Hz)"
#endif

#if TERM_TXBUFSIZE > 0

typedef struct ringbuf_ndx {
    //size_t capacity;
    size_t wr;
    size_t rd;
} ringbuf_ndx_t;

static inline size_t ringbuf_len(ringbuf_ndx_t *ndx, size_t capacity){
    size_t len;
    if(ndx->wr >= ndx->rd){
        len = ndx->wr - ndx->rd;
    }else{
        len = capacity - ndx->rd + ndx->wr;
    }
    return len;
}

static inline size_t ringbuf_free(ringbuf_ndx_t *ndx, size_t capacity){
    return capacity - ringbuf_len(ndx, capacity) - 1;
}

static inline size_t ringbuf_ndx_add(size_t a, size_t b, size_t capacity){
    return (a + b) % capacity;
}

static inline void ringbuf_ndx_inc(size_t *a, size_t capacity){
    *a = (*a + 1) % capacity;
}

#define RINGBUF_PUT(ndx, capacity, array, val)\
    if(ringbuf_free(ndx, capacity) > 0){\
        array[(ndx)->wr] = val;\
        ringbuf_ndx_inc(&(ndx)->wr, capacity);\
    }

static struct {
    char tx_buf[TERM_TXBUFSIZE];
    ringbuf_ndx_t tx_ndx;
    bool transmitting;
} term;


void term_write_char(unsigned char ch){
    unsigned irq_state;
    size_t free;
    do{
        irq_state = mips_di();
        free = ringbuf_free(&term.tx_ndx, TERM_TXBUFSIZE);
        mips_restore_irq(irq_state);
    }while(free == 0);
    irq_state = mips_di();
    if(term.transmitting){
        RINGBUF_PUT(&term.tx_ndx, TERM_TXBUFSIZE, term.tx_buf, ch)
    }else{
        UxTXREG = ch;
        UxTX_IFSCLR;
        UxTXIE;
        term.transmitting = true;
    }
    mips_restore_irq(irq_state);
}

void __ISR(UxVECTOR, TERM_IPL_TAG(TERM_IPL)) term_uart_isr( void ){

    if(ringbuf_len(&term.tx_ndx, TERM_TXBUFSIZE) == 0){
        UxTXDI;
        term.transmitting = false;
    }else{
        while(ringbuf_len(&term.tx_ndx, TERM_TXBUFSIZE) > 0 &&
              !(UxSTA & _U1STA_UTXBF_MASK))
        {
            UxTXREG = term.tx_buf[term.tx_ndx.rd];
            ringbuf_ndx_inc(&term.tx_ndx.rd, TERM_TXBUFSIZE);
        }
    }
    UxTX_IFSCLR;
}

void term_flush(void){
    size_t len;
    do {
        unsigned irq_state = mips_di();
        len = ringbuf_len(&term.tx_ndx, TERM_TXBUFSIZE);
        mips_restore_irq(irq_state);
    }while(len > 0);
}

#else

void term_write_char(unsigned char ch){
    while(UxSTA & _U1STA_UTXBF_MASK);
    UxTXREG= ch;
}

void term_flush(void){
    while(!(UxSTA & _U1STA_TRMT_MASK));
}

#endif

static void term_tfp_putc( void* p, char c){
    term_write_char(c);
}

#define TERM_BAUDRATE 115200

void term_init(void) {

#ifdef _OSCCON_PBDIV_POSITION
    unsigned pb_clock= SYS_CLOCK>>OSCCONbits.PBDIV;
#else
    unsigned pb_clock = SYS_CLOCK;
#endif

    UxMODE= _U1MODE_BRGH_MASK;  /* format: 8N1 */
    UxSTA= _U1STA_UTXEN_MASK |
            (0b10 << _U1STA_UTXISEL_POSITION); /* irq when buffer becomes/is empty */

    UxBRG= pb_clock/(4*TERM_BAUDRATE)-1;
    UxMODESET = _U1MODE_ON_MASK;
#if TERM_TXBUFSIZE > 0
    UxIPC = TERM_IPL;
#endif
    init_printf(NULL,term_tfp_putc);
}

void term_write_string(char *string) {
    while (*string) {
        if (*string == '\n')
            term_write_char('\r');
        term_write_char(*string++);
    }
    term_flush();
}

void term_writeln(void){
    term_write_char('\n');
    term_flush();
}


void term_write_hex(unsigned long num, unsigned int digits) {
    long i;
    unsigned long digit;
    for (i = (digits - 1)*4; i >= 0; i -= 4) {
        digit = (num >> i) & 0xf;
        if (digit < 10)
            term_write_char('0' + digit);
        else
            term_write_char('a' - 10 + digit);
    }
    term_flush();
}

void term_hexdump8(const void *buffer, int len){

        int ctr;
        unsigned char *buf= (unsigned char *) buffer;
        for( ctr= 0; ctr<len; ctr++ ){
                if( ctr % 16 == 0 ){
                        if( ctr!= 0 ){
                                term_writeln();
                        }
                        term_write_hex(ctr, 4);
                        term_write_string(": ");
                }
                term_write_hex(buf[ctr], 2);
                term_write_char(' ');
        }
        term_writeln();
}

/*
int term_kbhit(void)
{
   return MemoryRead(IRQ_STATUS) & IRQ_UART_READ_AVAILABLE;
}

int term_getch(void)
{
   while(!term_kbhit()) ;
   return MemoryRead(UART_READ);
}
 */
