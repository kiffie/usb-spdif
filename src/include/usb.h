/*
 * simple USB device implementation for PIC32MX
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#ifndef __USB_H__
#define	__USB_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <usb-ch9.h>


#define USB_PID_OUT   0x1
#define USB_PID_IN    0x9
#define USB_PID_SETUP 0xd

#define USB_NO_ENDPOINTS 16   /* number of used endpoints (max. 16) */


typedef struct usb_desc_table_elem {
    uint8_t desc_index;
    uint8_t desc_type;
    uint16_t wIndex;
    const void *desc;
    uint16_t desc_len;
} usb_desc_table_elem_t;


typedef void (*usb_completion_cb_t)(unsigned ep,
                                    unsigned pid,
                                    void *buffer,
                                    unsigned len);

typedef int (*usb_crtl_setup_cb_t)(usb9_setup_data_t *setup_packet,
                                   void **inout_data);

typedef void (*usb_ctrl_data_complete_cb_t)(bool canceled);

void usb_init_with_desc(const usb_desc_table_elem_t *desc_table);

/* to be defined in application,
 * typically a wrapper around usb_init_with_desc()
 */
void usb_init(void);

#ifndef USB_DISABLE_NONSTD_CTRL
void usb_set_nonstd_req_handler(usb_crtl_setup_cb_t setup_cb,
                                usb_ctrl_data_complete_cb_t data_complete_cb);
#endif

void usb_shutdown(void);

void usb_ctrl_data_confirm(bool stall);

void usb_stall_endpoint(int ep, bool is_in);

/**
 * Initialize an endpoint. Needs to be called twice with different addresses for
 * bidirectional endpoints, in particular for EP0.
 * @param ep_addr address of the endpoint (MSB specifies direction)
 * @param bmAttributes attributes (to determine endpoint type)
 * @param ep_size maximum transaction size
 * @param handler transaction callback
 */
void usb_init_endpoint(uint8_t ep_addr,
                       uint8_t bmAttributes,
                       unsigned ep_size,
                       usb_completion_cb_t handler);


bool usb_arm_endpoint(uint8_t ep_addr, void *data, unsigned data_len);

#ifdef USB_DISABLE_IRQ
void usb_tasks(void);
#elif defined(USB_INDIRECT_ISR)
void usb_isr(void);
#else
static inline void usb_tasks(void){}
#endif

#endif	/* USB_H */
