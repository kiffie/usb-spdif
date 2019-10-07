/*
 * USB spdif --- Audio S/PDIF interface
 */

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

#include <usb_audio.h>
#include <spdif_out.h>
#include <irhid.h>

#include <mips_irq.h>
#include <terminal.h>
#include <timer.h>

#include <logger.h>
#ifndef LOGLEVEL_MAIN
    #define LOGLEVEL_MAIN LOG_DEBUG
#endif
#define LOGLEVEL LOGLEVEL_MAIN
#define LOG_PREFIX "MAIN"

static bool bootloader_start = false;

static void enter_bootlader(void){
    mips_di();
    uint32_t *boot_magic = (uint32_t *)0xa0000000; /* start of SRAM */
    *boot_magic = 0x746f6f62; /* store magic value at start of SRAM */
    /* do a software reset */
    SYSKEY = 0;
    SYSKEY = 0xaa996655;
    SYSKEY = 0x556699aa;
    RSWRSTSET = _RSWRST_SWRST_MASK;
    RSWRST;
    while(true) { }; /* wait until reset happens */
}


static int nonstd_setup_handler(usb9_setup_data_t *sudata, void **inout_data){

    if(sudata->bmRequestType == USB9_RT_TYPE_VENDOR){
        if(sudata->bRequest == 0xf0 &&
           sudata->wValue == 0 &&
           sudata->wIndex == 0 &&
           sudata->wLength == 0)
        {
            bootloader_start = true;
            return 0;
        }else{
            return -1;
        }
    }else{
        return usb_audio_class_handler(sudata, inout_data);
    }
}

static void usb_device_data_handler(uint8_t ep_addr, void *data, unsigned len);

int main(void)
{
    term_init();
    timer_init();
    BMXCONCLR = _BMXCON_BMXWSDRM_MASK;
#if __PIC32_FEATURE_SET == 274
#if SYS_CLOCK > 54000000
    CHECON = 0b11 << _CHECON_PREFEN_POSITION | 3;   /* 3 wait states */
#elif SYS_CLOCK > 36000000
    CHECON = 0b11 << _CHECON_PREFEN_POSITION | 2;   /* 3 wait states */
#elif SYS_CLOCK > 18000000
    CHECON = 0b11 << _CHECON_PREFEN_POSITION | 1;   /* 3 wait state  */
#else
    CHECON = 0b11 << _CHECON_PREFEN_POSITION | 0;   /* 0 wait states */
#endif
    log_debug("CHECON = 0x%04x\n", CHECON);
#endif
    mips_enable_mv_irq();
#ifdef __32MX470F512H__
    RPF5R = 0b0001; /* U2TX for terminal (debug) output */
    CHECON = _CHECON_DCSZ_MASK | _CHECON_PREFEN_MASK | 1;
#else
    TRISBCLR = _TRISB_TRISB3_MASK | _TRISB_TRISB4_MASK; /* AUX LED, Act. LED */
    ANSELBCLR = _ANSELB_ANSB15_MASK; /* SCK2 */
    RPB0R= 0b0010; /* U2TX for terminal (debug) output */
    RPB2R = 0b0111; /* REFCLKO */
#endif
    log_info("USB S/PDIF Interface\n");
#ifdef SPDIF_REFCLKWIRE
    log_info("Reference clock wire: required\n");
#else
    log_info("Reference clock wire: not required\n");
#endif
#ifdef __32MX470F512H__
    log_debug("BMXCON = %08x\n", BMXCON);
    log_debug("CHECON = %08x\n", CHECON);
#endif
    spdif_out_init();
    usb_init();
    irhid_init();
    usb_audio_init();
    usb_set_nonstd_req_handler(nonstd_setup_handler, NULL);
    usb_audio_set_callback(0x01, usb_device_data_handler);

    while(1) {

        spdif_out_tasks();
        irhid_tasks();
        if(bootloader_start){
            log_info("entering bootloader\n");
            timer_wait_ms(10); /* enough time to complete USB transfer */
            term_flush();
            enter_bootlader();
        }
    }
}

static void usb_device_data_handler(uint8_t ep_addr, void *data, unsigned len){
#if PCM_SUBFRAMESIZE == 2
    spdif_out_tx_s16le(data, len/4);
#elif PCM_SUBFRAMESIZE == 3
    spdif_out_tx_s24le_packed(data, len/6);
#elif PCM_SUBFRAMESIZE == 4
    spdif_out_tx_s24le(data, len/8);
#else
    #error "invalid PCM_SUBFRAMESIZE"
#endif
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


