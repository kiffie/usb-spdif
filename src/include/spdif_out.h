/*
 * S/PDIF audio output
 * uses SPI2, DMACH3 and the reference clock generator
 */

#ifndef __SPDIF_OUT_H__
#define __SPDIF_OUT_H__

#include <stdint.h>

#define SPDIF_REFCLKWIRE

void spdif_out_init(void);
void spdif_out_tx_s16le(int16_t *frames, unsigned n_frames);
void spdif_out_tx_s24le(int32_t *frames, unsigned n_frames);
int spdif_out_set_rate(unsigned frames_sec);
void spdif_out_tasks(void);

#endif

