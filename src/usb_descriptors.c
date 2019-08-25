/*
 * USB device and configuration descriptors
 */

#include <usb_audio.h>

/* helper macros for encoding values > 255 */
#define BYTE0(word) (word & 0xff)
#define BYTE1(word) ((word >> 8) & 0xff)
#define BYTE2(word) ((word >> 16) & 0xff)


static struct usb9_device_descriptor usb_device_desc = {
    .bLength= 18,
    .bDescriptorType= USB9_DESC_DEVICE,
    //.bcdUSB= 0x0110,
    .bcdUSB= 0x0200,
    .bDeviceClass= 0,
    .bDeviceSubClass= 0,
    .bDeviceProtocol= 0,
    .bMaxPacketSize0= 64,
    .idVendor= 0x6666,
    .idProduct= 1,
    .bcdDevice= 0,
    .iManufacturer= 0,
    .iProduct= 0,
    .iSerialNumber= 0,
    .bNumConfigurations= 1
};

#define AUDIO_CONTROL_INTERFACE_ID 0x00
#define AUDIO_STREAMING_INTERFACE_ID 0x01


static const uint8_t usb_config_desc[] = {
   /* USB Speaker Configuration Descriptor */
   0x09,                             // Size of this descriptor in bytes
   USB9_DESC_CONFIGURATION,          // CONFIGURATION descriptor type
   0x6D,0x00,                        // Total length of data for this cfg
   0x02,                             // Number of interfaces in this cfg
   0x01,                             // Index value of this configuration
   0x00,                             // Configuration string index
   0x80,                             // Attributes
   100,                              // Max power consumption (200 mA)

   /* Standard AC Interface Descriptor  */
   0x09,                             // Size of this descriptor in bytes (bLength)
   USB9_DESC_INTERFACE,              // INTERFACE descriptor type (bDescriptorType)
   0x00,                             // Interface Number  (bInterfaceNumber)
   0x00,                             // Alternate Setting Number (bAlternateSetting)
   0x00,                             // Number of endpoints in this intf (bNumEndpoints)
   0x01,                             // Class code  (bInterfaceClass): AUDIO_DEVICE
   0x01,                             // Subclass code (bInterfaceSubclass): AUDIO_CONTROL
   0x00,                             // Protocol code  (bInterfaceProtocol)
   0x00,                             // Interface string index (iInterface)

   /* Class-specific AC Interface Descriptor  */
    0x09,                            // Size of this descriptor in bytes (bLength)
    0x24,                            // CS_INTERFACE Descriptor Type (bDescriptorType)
    0x01,                            // HEADER descriptor subtype   (bDescriptorSubtype)
    0x00,0x01,                       // Audio Device compliant to the USB Audio specification version 1.00 (bcdADC)
    0x27,0x00,                       // wTotalLength
                                     // Includes the combined length of this descriptor header and all Unit and Terminal descriptors.
    0x01,                            // The number of AudioStreaming interfaces in the Audio Interface Collection to which this AudioControl interface belongs  (bInCollection)
    0x01,                            // AudioStreaming interface 1 belongs to this AudioControl interface. (baInterfaceNr(1))

    /* Input Terminal Descriptor */
    0x0C,                            // Size of the descriptor, in bytes (bLength)
    0x24,                            // CS_INTERFACE Descriptor Type (bDescriptorType)
    0x02,                            // INPUT_TERMINAL descriptor subtype (bDescriptorSubtype)
    ID_INPUT_TERMINAL,               // ID of this Terminal.(bTerminalID)
    0x01,0x01,                       // wTerminalType
    0x00,                            // No association (bAssocTerminal)
    PCM_CHANNELS,                    // bNrChannels
    0x03,0x00,                       // Left Front and Right front chanels (wChannelConfig)
    0x00,                            // Unused.(iChannelNames)
    0x00,                            // Unused. (iTerminal)

    /* Feature Unit Descriptor */
    0x09,                            // Size of the descriptor, in bytes (bLength)
    0x24,                            // CS_INTERFACE Descriptor Type (bDescriptorType)
    0x06,                            // FEATURE_UNIT  descriptor subtype (bDescriptorSubtype)
    ID_FEATURE_UNIT,                 // ID of this Unit ( bUnitID  ).
    ID_INPUT_TERMINAL,               // Input terminal is connected to this unit(bSourceID)
    0x02,                            // bControlSize
    0x01,0x00,                       // (bmaControls(0))
    0x00,                            //  iFeature

    /* Output Terminal Descriptor */
    0x09,                            // Size of the descriptor, in bytes (bLength)
    0x24,                            // CS_INTERFACE Descriptor Type (bDescriptorType)
    0x03,                            // OUTPUT_TERMINAL  descriptor subtype (bDescriptorSubtype)
    ID_OUTPUT_TERMINAL,              // ID of this Terminal.(bTerminalID)
    0x01,0x03,                       // (wTerminalType)See USB Audio Terminal Types.
    0x00,                            // No association (bAssocTerminal)
    ID_FEATURE_UNIT,                 // (bSourceID)
    0x00,                            // iTerminal (index of a string descriptor)

    /* Standard AS Interface Descriptor (Alt. Set. 0) */
    0x09,                            // Size of the descriptor, in bytes (bLength)
    USB9_DESC_INTERFACE,             // INTERFACE descriptor type (bDescriptorType)
    AUDIO_STREAMING_INTERFACE_ID,    // Interface Number  (bInterfaceNumber)
    0x00,                            // Alternate Setting Number (bAlternateSetting)
    0x00,                            // Number of endpoints in this intf (bNumEndpoints)
    0x01,                            // AUDIO_DEVICE Class code  (bInterfaceClass)
    0x02,                            // AUDIOSTREAMING Subclass code (bInterfaceSubclass)
    0x00,                            // Protocol code  (bInterfaceProtocol)
    0x00,                            // Interface string index (iInterface)

    /* Standard AS Interface Descriptor (Alt. Set. 1) */
    0x09,                            // Size of the descriptor, in bytes (bLength)
    USB9_DESC_INTERFACE,             // INTERFACE descriptor type (bDescriptorType)
    AUDIO_STREAMING_INTERFACE_ID,    // Interface Number  (bInterfaceNumber)
    0x01,                            // Alternate Setting Number (bAlternateSetting)
    0x01,                            // Number of endpoints in this intf (bNumEndpoints)
    0x01,                            // AUDIO_DEVICE Class code  (bInterfaceClass)
    0x02,                            // AUDIOSTREAMING Subclass code (bInterfaceSubclass)
    0x00,                            // Protocol code  (bInterfaceProtocol)
    0x00,                            // Interface string index (iInterface)

    /*  Class-specific AS General Interface Descriptor */
    0x07,                            // Size of the descriptor, in bytes (bLength)
    0x24,                            // CS_INTERFACE Descriptor Type (bDescriptorType)
    0x01,                            // AS_GENERAL subtype (bDescriptorSubtype)
    0x01,                            // Unit ID of the Output Terminal.(bTerminalLink)
    0x01,                            // Interface delay. (bDelay)
    0x01,0x00,                       // PCM Format (wFormatTag)

    /*  Type 1 Format Type Descriptor */
    0x0B,                            // Size of the descriptor, in bytes (bLength)
    0x24,                            // CS_INTERFACE Descriptor Type (bDescriptorType)
    0x02,                            // FORMAT_TYPE subtype. (bDescriptorSubtype)
    0x01,                            // FORMAT_TYPE_1. (bFormatType)
    PCM_CHANNELS,                    // bNrChannels
    0x02,                            // bSubFrameSize
    0x10,                            // bBitResolution
    0x01,                            // bSamFreqType
    BYTE0(PCM_FSAMPLE),              // tSamFreq
    BYTE1(PCM_FSAMPLE),
    BYTE2(PCM_FSAMPLE),

    /*  Standard Endpoint Descriptor */
    0x09,                            // Size of the descriptor, in bytes (bLength)
    USB9_DESC_ENDPOINT,              // ENDPOINT descriptor (bDescriptorType)
    0x01,                            // OUT Endpoint 1. (bEndpointAddress)
    0x0d,                            // bmAttributes: Isochronous, Synchronous
    BYTE0(USB_AUDIO_ISO_EP_SIZE),    // wMaxPacketSize
    BYTE1(USB_AUDIO_ISO_EP_SIZE),
    0x01,                            // bInterval
    0x00,                            // bRefresh
    0x00,                            // bSynchAddress

    /* Class-specific Isoc. Audio Data Endpoint Descriptor*/
    0x07,                            // Size of the descriptor, in bytes (bLength)
    0x25,                            // CS_ENDPOINT Descriptor Type (bDescriptorType)
    0x01,                            // GENERAL subtype. (bDescriptorSubtype)
    0x00,                            // No sampling frequency control, no pitch control, no packet padding.(bmAttributes)
    0x00,                            // bLockDelayUnits
    0x00,0x00                        // wLockDelay)
};

void usb_init(void){
    usb_init_with_desc(&usb_device_desc,
                       &usb_config_desc, sizeof(usb_config_desc));
}
