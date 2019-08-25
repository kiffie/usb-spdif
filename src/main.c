/*
 * USB spdif --- Audio S/PDIF interface
 */

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

/* USB device stack */
#include <usb_audio.h>

#include "spdif_out.h"

#include <mips_irq.h>
#include <terminal.h>
#include <timer.h>

#include <logger.h>
#ifndef LOGLEVEL_MAIN
    #define LOGLEVEL_MAIN LOG_DEBUG
#endif
#define LOGLEVEL LOGLEVEL_MAIN
#define LOG_PREFIX "MAIN"


static void usb_device_data_handler(uint8_t ep_addr, void *data, unsigned len);

int main(void)
{

    BMXCONCLR = _BMXCON_BMXWSDRM_MASK;
    mips_enable_mv_irq();
#ifdef __32MX470F512H__
    RPF5R = 0b0001; /* U2TX for terminal (debug) output */
    CHECON = _CHECON_DCSZ_MASK | _CHECON_PREFEN_MASK | 1;
#elif defined(__32MX270F256D__)
    TRISBCLR = _TRISB_TRISB4_MASK; /* LED */
    RPB0R = 0b0010; /* U2TX for terminal (debug) output */
#else
    TRISBCLR = _TRISB_TRISB4_MASK; /* LED */
    RPB0R= 0b0010; /* U2TX for terminal (debug) output */
#endif
    term_init();
    timer_init();
    log_info("USB S/PDIF Interface\n");
#ifdef __32MX470F512H__
    log_debug("BMXCON = %08x\n", BMXCON);
    log_debug("CHECON = %08x\n", CHECON);
#endif
    spdif_out_init();
    usb_init();
    usb_audio_init();
    usb_audio_set_callback(0x01, usb_device_data_handler);
    timer_wait_ms(10); /* need time to ID pin state detection to work properly */

    while(1) {

        spdif_out_tasks();

    }
}

static void usb_device_data_handler(uint8_t ep_addr, void *data, unsigned len){
    spdif_out_tx_s16le(data, len/4);
}



/******************************************************************************/

void __attribute__((noinline, nomips16)) _nmi_handler(void){
    unsigned int dummy;
    log_error("************************************************************\n");
    log_error("unexpected NMI occurred.\n");
    if( RCONbits.WDTO ){
        log_error("reason: Watchdog timer overflow.\n");
    }
    log_error("performing software reset...\n");
    log_error("************************************************************\n");
    term_flush();
    mips_di();
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA; 
    RSWRSTSET = _RSWRST_SWRST_MASK;
    dummy = RSWRST;
    (void)dummy;
    while(1);
}

void __attribute__((noinline, nomips16)) _general_exception_handler(void){

    static const char cause_mnemonic [15][5] = {
         "Int", "MOD", "TLBL", "TLBS", "AdEL", "AdES", "IBE", "DBE",
         "Sys", "Bp", "RI", "CPU", "Ov", "TR", "???"
    };

    unsigned Cause, EPC, EXCCODE, mnemonic_ndx;
    log_error("************************************************************\n");
    log_error("GEN_EXCEPTION: unexpected exception -> restarting\n");
    asm volatile("mfc0   %0, $13"    : "=r"(Cause));
    asm volatile("mfc0   %0, $14"    : "=r"(EPC));
    EXCCODE = (Cause >> 2) & 0x1f;
    mnemonic_ndx = EXCCODE <= 13 ? EXCCODE : 14;
    log_error("\tCause    = %08x, EXCCODE = %02x (%s)\n"
              "\tEPC      = %08x\n",
              Cause, EXCCODE, cause_mnemonic[mnemonic_ndx], EPC);
    log_error("************************************************************\n");
    term_flush();
    mips_di();
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    RSWRST = _RSWRST_SWRST_MASK;
    RSWRST; /* read RSWRST */
    while(1);
}

