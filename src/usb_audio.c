/*
 * usb_audio --- USB audio output
 */

#include <usb_audio.h>
#include <usb.h>
#include <string.h>

#include <logger.h>

#ifndef LOGLEVEL_USB_AUDIO
#define LOGLEVEL_USB_AUDIO LOG_DEBUG
#endif
#define LOGLEVEL LOGLEVEL_USB_AUDIO
#define LOG_PREFIX "USB_AUDIO"

/*
 * Request codes
 */

#define USB_AUDIO_REQ_UNDEF     0x00
#define USB_AUDIO_REQ_SET_CUR   0x01
#define USB_AUDIO_REQ_GET_CUR   0x81
#define USB_AUDIO_REQ_SET_MIN   0x02
#define USB_AUDIO_REQ_GET_MIN   0x82
#define USB_AUDIO_REQ_SET_MAX   0x03
#define USB_AUDIO_REQ_GET_MAX   0x83
#define USB_AUDIO_REQ_SET_RES   0x04
#define USB_AUDIO_REQ_GET_RES   0x84
#define USB_AUDIO_REQ_SET_MEM   0x05
#define USB_AUDIO_REQ_GET_MEM   0x85
#define USB_AUDIO_REQ_GET_STAT  0xff

#define USB_AUDIO_REQ_IS_GET(req) (req & 0x80)

/*
 * control selectors
 */
#define USB_AUDIO_FU_MUTE_CONTROL       0x01
#define USB_AUDIO_FU_VOLUME_CONTROL     0x02
#define USB_AUDIO_FU_BASS_CONTROL       0x03
#define USB_AUDIO_FU_MID_CONTROL        0x04
#define USB_AUDIO_FU_TREBLE_CONTROL     0x05
#define USB_AUDIO_FU_EQUALIZER_CONTROL  0x06
#define USB_AUDIO_FU_AGC_CONTROL        0x07
#define USB_AUDIO_FU_DELAY_CONTROL      0x08
#define USB_AUDIO_FU_BASS_BOOST_CONTROL 0x09
#define USB_AUDIO_FU_LOUDNESS_CONTROL   0x0a


static struct {
    uint8_t mute;
    uint8_t audio_data[2][USB_AUDIO_ISO_EP_SIZE] __attribute__ ((aligned (4)));;
    usb_audio_data_cb_t rx_callback;
} usb_audio;


static void usb_audio_data_cb(unsigned ep, unsigned pid, void *buffer, unsigned len)
{
    if(usb_audio.rx_callback != NULL){
        if(usb_audio.mute){
            memset(buffer, 0, len);
        }
        usb_audio.rx_callback(0x01, buffer, len);
    }
    usb_arm_endpoint(0x01, buffer, USB_AUDIO_ISO_EP_SIZE);
}


int usb_audio_class_handler(usb9_setup_data_t *sudata, void **inout_data){

    int xfer_len = -1;
    log_debug("Class request: bmRequestType=%02x, bRequest=%02x, "
            "wValue=%04x, wIndex=%04x, wLength=%u\n",
            sudata->bmRequestType,
            sudata->bRequest,
            sudata->wValue,
            sudata->wIndex,
            sudata->wLength);

    /* sanity check */
    bool is_in = sudata->bmRequestType & USB9_RT_IN;
    bool is_get = USB_AUDIO_REQ_IS_GET(sudata->bRequest);
    if(is_in != is_get){
        return -1;
    }
    /* only interface related requests are supported */
    if((sudata->bmRequestType & USB9_RT_REC_MASK) != USB9_RT_REC_INTERFACE){
        return -1;
    }

    if(sudata->bRequest == USB_AUDIO_REQ_GET_CUR &&
       sudata->wValue == 0x0100 &&
       sudata->wIndex == ((ID_FEATURE_UNIT<<8) | 0x00) &&
       sudata->wLength == 1)
    {
        log_debug("GET_CUR Mute\n");
        xfer_len = 1;
        *inout_data = &usb_audio.mute;
    }else if(sudata->bRequest == USB_AUDIO_REQ_SET_CUR &&
       sudata->wValue == 0x0100 &&
       sudata->wIndex == ((ID_FEATURE_UNIT<<8) | 0x00) &&
       sudata->wLength == 1)
    {
        log_debug("SET_CUR Mute\n");
        xfer_len = 1;
        *inout_data = &usb_audio.mute;
    }
    return xfer_len;
}

void usb_audio_init(void){
    usb_audio.mute = false;
    usb_audio.rx_callback = NULL;
    usb_init_endpoint(0x01, USB9_EPDESC_ATTR_TYPE_ISO, USB_AUDIO_ISO_EP_SIZE, usb_audio_data_cb);
    usb_arm_endpoint(0x01, usb_audio.audio_data[0], USB_AUDIO_ISO_EP_SIZE);
    usb_arm_endpoint(0x01, usb_audio.audio_data[1], USB_AUDIO_ISO_EP_SIZE);
    log_debug("USB audio device application initialized\n");
}

/**
 * Set callback for received audio data. The callback may be executed from
 * an ISR.
 * @returns true on success
 */
bool usb_audio_set_callback(uint8_t ep_addr, usb_audio_data_cb_t callback){
    if(ep_addr != 0x01)
        return false;
    usb_audio.rx_callback = callback;
    return true;
}
