/*
 * irhid.c --- USB HID remote control keyboard
 *             uses TIMER2
 */

#include <irhid.h>
#include <usb.h>
#include <mips_irq.h>

#include <irmp.h>
#include <xc.h>
#include <pic32clk.h>
#include <sys/attribs.h>

#include <logger.h>

#ifndef LOGLEVEL_IRHID
#    define LOGLEVEL_IRHID LOG_DEBUG
#endif
#define LOGLEVEL LOGLEVEL_IRHID
#define LOG_PREFIX "IRHID"

typedef struct irhid_keymap {
    uint8_t rc_proto;
    uint16_t rc_address;
    uint32_t rc_command;
    /* key value is Usage ID as specified in USB HID Usage Tables Spec.,
     * Consumer Page, p. 74 sqq.,
     * see https://usb.org/sites/default/files/documents/hut1_12v2.pdf
     */
    uint16_t key;
} irhid_keymap_t;

static const irhid_keymap_t irhid_keymap[] = {

    /* RC5 code */
    {7, 0, 16, 0x00e9 },    /* Volume Up */
    {7, 0, 17, 0x00ea },    /* Volume Down */
    {7, 0, 13, 0x00e2 },    /* Mute */
    {7, 0, 53, 0x00b0 },    /* Play */
    {7, 0, 48, 0x00cd },    /* Play/Pause */
    {7, 0, 54, 0x00b7 },    /* Stop */
    {7, 0, 52, 0x00b5 },    /* Next Track (Fast Forward on RC-5)*/
    {7, 0, 50, 0x00b6 },    /* Previous Track (Fast Rewind on RC-5) */

    /* Denon RC-1173 (Net) */
    {5, 12884, 8202, 0x00cd },    /* Play/Pause */
    {5, 12884, 8203, 0x00b7 },    /* Stop */
    {5, 12884, 8204, 0x00b5 },    /* Next Track */
    {5, 12884, 8205, 0x00b6 },    /* Previous Track */

    /* End marker */
    {0xff, 0, 0, 0}
};

static struct {
    enum { TXS_IDLE=0, TXS_ARMED } tx_state;
    uint16_t key_report;
    uint16_t release_report;
} irhid;

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) timer2_isr(void){
    irmp_ISR();
    IFS0CLR = _IFS0_T2IF_MASK;
}

static uint16_t irhid_keymap_lookup(uint8_t proto,
                                    uint16_t address,
                                    uint32_t command)
{
    size_t i = 0;
    uint16_t report = 0;
    while((irhid_keymap[i].rc_proto) != 0xff){
        if(irhid_keymap[i].rc_proto == proto &&
           irhid_keymap[i].rc_address == address &&
           irhid_keymap[i].rc_command == command)
        {
            report = irhid_keymap[i].key;
            break;
        }
        i++;
    }
    return report;
}



static void irhid_in_complete(unsigned ep, unsigned pid, void *buffer, unsigned len)
{
    log_debug("USB IN transfer complete\n");
    irhid.tx_state = TXS_IDLE;
}


void irhid_init(void){
    irhid.tx_state = TXS_IDLE;
    irhid.release_report = 0;
    irmp_init();
    unsigned pb_clock= pic32clk_pb_clock();
    PR2 = pb_clock / F_INTERRUPTS;
    T2CON = _T2CON_ON_MASK;

    /* configure interrupts */
    IFS0CLR = _IFS0_T2IF_MASK;
    IEC0SET = _IEC0_T2IE_MASK;
    IPC2bits.T2IP = 5;


    /* USB */
    usb_init_endpoint(0x82,
                      USB9_EPDESC_ATTR_TYPE_INTERRUPT,
                      8,
                      irhid_in_complete);
}


void irhid_tasks(void){
    IRMP_DATA irmp_data;
    if(irmp_get_data(&irmp_data)){
        log_debug("data: proto = %u, addr = %u, command = %u, flags = %02x\n",
                  irmp_data.protocol,
                  irmp_data.address,
                  irmp_data.command,
                  irmp_data.flags);
        irhid.key_report = irhid_keymap_lookup(irmp_data.protocol,
                                               irmp_data.address,
                                               irmp_data.command);
        unsigned irq_state = mips_di();
        if(irhid.tx_state == TXS_IDLE &&
           !(irmp_data.flags & IRMP_FLAG_REPETITION) &&
           irhid.key_report != 0)
        {
            usb_arm_endpoint(0x82, &irhid.key_report, sizeof(irhid.key_report));
            usb_arm_endpoint(0x82, &irhid.release_report, sizeof(irhid.release_report));
            irhid.tx_state = TXS_ARMED;
        }
        mips_restore_irq(irq_state);
    }

}
