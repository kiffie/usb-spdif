/*
 * USB descriptors
 *
 * also contains usb_init(), which initializes the USB device with these
 * descriptors
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <usb_audio.h>

/* helper macros for encoding values > 255 */
#define BYTE0(word) (word & 0xff)
#define BYTE1(word) ((word >> 8) & 0xff)
#define BYTE2(word) ((word >> 16) & 0xff)

const uint8_t usb_hid_report_desc[] = {
    0x05, 0x0c,             // USAGE_PAGE (Consumer)
    0x09, 0x01,             // USAGE (consumer control)
    0xa1, 0x01,             // COLLECTION (Application)
    0x19, 0x00,             //   USAGE_MINIMUM
    0x2a, 0x9c, 0x02,       //   USAGE_MAXIMUM
    0x15, 0x00,             //   LOGICAL_MINIMUM
    0x26, 0x9c, 0x02,       //   LOGICAL_MAXIMUM
    0x95, 0x01,             //   REPORT_COUNT (1)
    0x75, 0x10,             //   REPORT_SIZE (16)
    0x81, 0x00,             //   INPUT (Data,Ary,Abs)
    0xc0                    // END_COLLECTION
};

const size_t USB_HID_REPORT_DESC_LEN = sizeof(usb_hid_report_desc);


static const struct usb9_device_descriptor usb_device_desc = {
    .bLength= 18,
    .bDescriptorType= USB9_DESC_DEVICE,
    .bcdUSB= 0x0200,
    .bDeviceClass= 0,
    .bDeviceSubClass= 0,
    .bDeviceProtocol= 0,
    .bMaxPacketSize0= 64,
    .idVendor= 0x16c0,  /* see USB-IDs-for-free.txt */
    .idProduct= 0x27e0,
    .bcdDevice= 0,
    .iManufacturer= 0,
    .iProduct= 1,
    .iSerialNumber= 0,
    .bNumConfigurations= 1
};

#define AUDIO_CONTROL_INTERFACE_ID 0x00
#define AUDIO_STREAMING_INTERFACE_ID 0x01
#define HID_INTERFACE_ID 0x02

#define USB_CONFIG_DESC_LEN 140

static const uint8_t usb_config_desc[USB_CONFIG_DESC_LEN] = {
    0x09,                               // bLength
    USB9_DESC_CONFIGURATION,            // bDescriptorType
    BYTE0(USB_CONFIG_DESC_LEN),         // wTotalLength
    BYTE1(USB_CONFIG_DESC_LEN),
    0x03,                               // Number of interfaces in this cfg
    0x01,                               // Index value of this configuration
    0x00,                               // Configuration string index
    0x80,                               // Attributes
    100,                                // Max power consumption (200 mA)

    /* Standard Audio Control Interface Descriptor */
    0x09,                               // bLength
    USB9_DESC_INTERFACE,                // bDescriptorType
    AUDIO_CONTROL_INTERFACE_ID,         // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    0x01,                               // bInterfaceClass: AUDIO_DEVICE
    0x01,                               // bInterfaceSubclass: AUDIO_CONTROL
    0x00,                               // bInterfaceProtocol
    0x00,                               // iInterface

    /* Class-specific Audio Control Interface Descriptor  */
    0x09,                               // bLength
    0x24,                               // bDescriptorType: CS_INTERFACE
    0x01,                               // bDescriptorSubtype
    0x00,0x01,                          // bcdADC
    0x27,0x00,                          // wTotalLength
    0x01,                               // bInCollection
    0x01,                               // baInterfaceNr

    /* Input Terminal Descriptor */
    0x0c,                               // bLength)
    0x24,                               // bDescriptorType: CS_INTERFACE
    0x02,                               // bDescriptorSubtype: INPUT_TERMINAL
    ID_INPUT_TERMINAL,                  // bTerminalID
    0x01,0x01,                          // wTerminalType
    0x00,                               // bAssocTerminal
    PCM_CHANNELS,                       // bNrChannels
    0x03,0x00,                          // wChannelConfig: Left Front and Right Front
    0x00,                               // iChannelNames
    0x00,                               // iTerminal

    /* Feature Unit Descriptor */
    0x09,                               // bLength
    0x24,                               // bDescriptorType
    0x06,                               // bDescriptorSubtype: FEATURE_UNIT
    ID_FEATURE_UNIT,                    // bUnitID
    ID_INPUT_TERMINAL,                  // bSourceID
    0x02,                               // bControlSize
    0x01,0x00,                          // bmaControls
    0x00,                               // iFeature

    /* Output Terminal Descriptor */
    0x09,                               // bLength
    0x24,                               // bDescriptorType: CS_INTERFACE
    0x03,                               // bDescriptorSubtype: OUTPUT_TERMINAL
    ID_OUTPUT_TERMINAL,                 // bTerminalID
    0x05, 0x06,                         // wTerminalType
    0x00,                               // bAssocTerminal
    ID_FEATURE_UNIT,                    // bSourceID
    0x00,                               // iTerminal

    /* Standard AS Interface Descriptor (Alt. Set. 0) */
    0x09,                               // bLength
    USB9_DESC_INTERFACE,                // bDescriptorType
    AUDIO_STREAMING_INTERFACE_ID,       // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    0x01,                               // bInterfaceClass: AUDIO_DEVICE
    0x02,                               // bInterfaceSubclass: AUDIOSTREAMING
    0x00,                               // bInterfaceProtocol
    0x00,                               // iInterface

    /* Standard AS Interface Descriptor (Alt. Set. 1) */
    0x09,                               // bLength
    USB9_DESC_INTERFACE,                // bDescriptorType
    AUDIO_STREAMING_INTERFACE_ID,       // bInterfaceNumber
    0x01,                               // bAlternateSetting
    0x01,                               // bNumEndpoints
    0x01,                               // bInterfaceClass: AUDIO_DEVICE
    0x02,                               // bInterfaceSubclass: AUDIOSTREAMING
    0x00,                               // bInterfaceProtocol
    0x00,                               // iInterface

    /*  Class-specific AS General Interface Descriptor */
    0x07,                               // bLength
    0x24,                               // bDescriptorType: CS_INTERFACE
    0x01,                               // bDescriptorSubtype: AS_GENERAL
    0x01,                               // bTerminalLink
    0x01,                               // bDelay
    0x01,0x00,                          // wFormatTag: PCM

    /*  Type 1 Format Type Descriptor */
    17,                                 // bLength
    0x24,                               // bDescriptorType: CS_INTERFACE
    0x02,                               // bDescriptorSubtype: FORMAT_TYPE
    0x01,                               // bFormatType: FORMAT_TYPE_1
    PCM_CHANNELS,                       // bNrChannels
    PCM_SUBFRAMESIZE,                   // bSubFrameSize
#if PCM_SUBFRAMESIZE == 2
    16,                                 // bBitResolution
#elif PCM_SUBFRAMESIZE == 3 || PCM_SUBFRAMESIZE == 4
    24,                                 // bBitResolution
#else
#   error "invalid subframe size"
#endif
    3,                                  // bSamFreqType
    BYTE0(44100),                       // tSamFreq
    BYTE1(44100),
    BYTE2(44100),
    BYTE0(48000),                       // tSamFreq
    BYTE1(48000),
    BYTE2(48000),
    BYTE0(96000),                       // tSamFreq
    BYTE1(96000),
    BYTE2(96000),

    /*  Standard Endpoint Descriptor */
    0x09,                               // bLength
    USB9_DESC_ENDPOINT,                 // bDescriptorType
    0x01,                               // bEndpointAddress
    0x0d,                               // bmAttributes: Isochronous, Synchronous
    BYTE0(USB_AUDIO_ISO_EP_SIZE),       // wMaxPacketSize
    BYTE1(USB_AUDIO_ISO_EP_SIZE),
    0x01,                               // bInterval
    0x00,                               // bRefresh
    0x00,                               // bSynchAddress

    /* Class-specific Isoc. Audio Data Endpoint Descriptor*/
    0x07,                               // bLength
    0x25,                               // bDescriptorType: CS_ENDPOINT
    0x01,                               // bDescriptorSubtype: GENERAL
    0x00,                               // bmAttributes
    0x00,                               // bLockDelayUnits
    0x00,0x00,                          // wLockDelay

    /* Interface 1 descriptor (HID device) */
    0x09,                               // bLength
    USB9_DESC_INTERFACE,                // bDescriptorType
    HID_INTERFACE_ID,                   // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndpoints
    0x03,                               // bInterfaceClass: HID
    0x00,                               // bInterfaceSubclass: No Subclass
    0x00,                               // bInterfaceProtocol
    0x00,                               // iInterface
    /* HID descriptor */
    0x09,                               // bLength
    USB9_DESC_HID,                      // bDescriptorType
    0x01, 0x01,                         // bcdHID
    0x00,                               // bCountryCode
    0x01,                               // bNumDescriptors
    USB9_DESC_HID_REPORT,               // bDescriptorType
    BYTE0(sizeof(usb_hid_report_desc)), // wDescriptorLength
    BYTE1(sizeof(usb_hid_report_desc)),

    /* Endpoint Descriptor */
    0x07,                               // bLength
    USB9_DESC_ENDPOINT,                 // bDescriptorType
    0x82,                               // bEndpointAddress
    USB9_EPDESC_ATTR_TYPE_INTERRUPT,    // bmAttributes
    BYTE0(8), BYTE1(8),                 // wMaxPacketSize
    10,                                 // bInterval
};

/* This must be adapted when configuration descriptor is changed */
#define HID_DESC_NDX (sizeof(usb_config_desc) - 7 - 9)

static const uint16_t usb_string_desc_0[] = {
    USB9_DESC_STRING_HEADER(1),
    USB9_LANGID_EN_US
};

static const uint16_t usb_string_desc_1[] = {
    USB9_DESC_STRING_HEADER(9),
    'U', 'S', 'B', '_', 'S', 'P', 'D', 'I', 'F'
};


static const usb_desc_table_elem_t usb_desc_table[] = {
    { 0, USB9_DESC_DEVICE, 0, &usb_device_desc, sizeof(usb_device_desc) },
    { 0, USB9_DESC_CONFIGURATION, 0, usb_config_desc, sizeof(usb_config_desc) },
    { 0, USB9_DESC_HID, 0, &usb_config_desc[HID_DESC_NDX], 9 },
    { 0, USB9_DESC_HID_REPORT, HID_INTERFACE_ID, usb_hid_report_desc, sizeof(usb_hid_report_desc) },
    { 0, USB9_DESC_STRING, 0, usb_string_desc_0, sizeof(usb_string_desc_0) },
    { 1, USB9_DESC_STRING, USB9_LANGID_EN_US, usb_string_desc_1, sizeof(usb_string_desc_1) },
    { 0, 0, 0, NULL, 0 }
};


void usb_init(void){
    usb_init_with_desc(usb_desc_table);
}
