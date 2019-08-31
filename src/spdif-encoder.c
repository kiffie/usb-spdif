/*
 * SPDIF encoder
 */

#include "spdif-encoder.h"

static uint16_t spdif_biphase_encode(int last, uint8_t data){
	int i;
	uint16_t result=0;

	for( i=0; i< 8; i++){
		result<<=2;
		if( data&1 ){
			result|= 0b10;
		}else{
			result|= 0b11;
		}
		if( last ){
			result^= 0b11;
		}
		last= result&1;
		data>>=1;
	}
	return result;
}

static uint16_t spdif_preamble_encode(int last, uint8_t data){

	uint16_t result=0;
	int i;
	switch( data & SPDIF_PREAMBLE_MASK ){
		case SPDIF_PREAMBLE_X:
			result= 0b11100010;
			break;
		case SPDIF_PREAMBLE_Y:
			result= 0b11100100;
			break;
		case SPDIF_PREAMBLE_Z:
			result= 0b11101000;
			break;
		}
	if( last ){
		result^= 0xff;
	}
	last= result&1;
	data>>=4;
	for(i=0; i<4; i++){
		result<<=2;
		if( data&1 ){
			result|= 0b10;
		}else{
			result|= 0b11;
		}
		if( last ){
			result^= 0b11;
		}
		last= result&1;
		data>>=1;
	}
	return result;
}

static void spdif_fast_encode(struct spdif_encoder *spdif,
			      void *encoded_buf, uint32_t subframe)
{
	uint32_t parity;
	uint32_t *encoded = (uint32_t *)encoded_buf;

	parity = subframe & ~SPDIF_PREAMBLE_MASK; /* exclude preamble bits */	
	parity ^= parity >> 16; /* slightly faster than calling __builtin_parity() */
	parity ^= parity >>  8;
	parity ^= parity >>  4;
	parity &= 0xf;
	parity =  0x69960000 << parity;
	parity &= 0x80000000;
	subframe |= parity;
	int last = spdif->last;

	uint32_t data1 = spdif->first_byte[subframe & 0xff];
	if(last){
		data1^= 0xffff;
	}
	last = data1&1;

	uint32_t data0 = spdif->byte[(subframe >> 8) & 0xff];
	if(last){
		data0^= 0xffff;
	}
	last= data0&1;
        encoded[0] = data1 << 16 | data0;

	data1 = spdif->byte[(subframe >> 16) & 0xff];
	if(last){
		data1^= 0xffff;
	}
	last = data1&1;

	data0 = spdif->byte[(subframe >> 24) & 0xff];
	if(last){
		data0^= 0xffff;
	}
	spdif->last= data0&1;
	encoded[1] = data1 << 16 | data0;
}


void spdif_encode_frame_generic(struct spdif_encoder *spdif,
				void *encoded,
				int left_shifted, int right_shifted)
{
	uint8_t *p= encoded;
	uint32_t subframe;
	subframe= spdif->frame_ctr == 0 ? SPDIF_PREAMBLE_Z : SPDIF_PREAMBLE_X;
	subframe|= left_shifted;
	spdif_fast_encode(spdif, p, subframe);
	p+= SPDIF_FRAMESIZE/2;
	subframe= SPDIF_PREAMBLE_Y | right_shifted;
	spdif_fast_encode(spdif, p, subframe);
	if( ++spdif->frame_ctr >= SPDIF_BLOCKSIZE ){
		spdif->frame_ctr= 0;
	}
}

void spdif_encoder_init(struct spdif_encoder *spdif){
	int i;
	for(i=0; i< 256; i++){
		spdif->first_byte[i]= spdif_preamble_encode(0, i);
		spdif->byte[i]= spdif_biphase_encode(0, i);
	}
}

/* end of SPDIF encoder */

