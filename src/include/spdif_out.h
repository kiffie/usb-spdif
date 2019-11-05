/*
 * S/PDIF audio output
 * uses SPI2, DMACH3 and the reference clock generator
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#ifndef __SPDIF_OUT_H__
#define __SPDIF_OUT_H__

#include <stdint.h>

/* When this macro is defined then an external connection between the
 * reference clock output and the SPI clock pin (SCK2) is required
 */
#define SPDIF_REFCLKWIRE

/* LEDs */
#define SPDIF_OUT_AUX_LED LATBbits.LATB3
#define SPDIF_OUT_ACT_LED LATBbits.LATB4

void spdif_out_init(void);
void spdif_out_tx_s16le(int16_t *frames, unsigned n_frames);
void spdif_out_tx_s24le(int32_t *frames, unsigned n_frames);
void spdif_out_tx_s24le_packed(uint8_t *frames, unsigned n_frames);
int spdif_out_set_rate(unsigned frames_sec);
void spdif_out_tasks(void);

#endif

