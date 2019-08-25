/*
 * S/PDIF audio output
 * uses SPI2, DMACH3 and the reference clock generator
 */


#include "spdif_out.h"
#include "spdif-encoder.h"
#include <timer.h>
#include <mips_irq.h>
#include <xc.h>
#include <sys/attribs.h>

#include <logger.h>
#ifndef LOGLEVEL_SPDIF_OUT
    #define LOGLEVEL_SPDIF_OUT LOG_DEBUG
#endif
#define LOGLEVEL LOGLEVEL_SPDIF_OUT
#define LOG_PREFIX "SPDIF_OUT"

#define SPDIF_BUFLEN (4*SPDIF_BLOCKSIZE)  /* buffer size in frames */

typedef struct spdif_context {
    unsigned tx_active:1;
    
    spdif_frame_t spdif_buf[SPDIF_BUFLEN];
    unsigned tx_ptr;
    unsigned tx_ctr, tx_ctr_last; /* used to check if buffer insertions have stopped */
    unsigned tx_ctr_brm;
    unsigned bov_ctr; /* buffer overflow counter */
    timer_time_t next_buffer_check;
    //timer_time_t next_staus_report;
    unsigned sampling_rate;
    int sampling_rate_adj;
    unsigned buflen_avg;
    spdif_encoder_t encoder;
} spdif_context_t;

static spdif_context_t spdif;

static inline unsigned int __attribute__((always_inline)) virt_to_phys(const void* p)
{
        return (int)p<0?((int)p&0x1fffffffL):(unsigned int)((unsigned char*)p+0x40000000L);
}

static inline void set_active_led(int active){
    PORTBbits.RB4 = active;
}

unsigned spdif_out_get_buflen(void){

    unsigned len;
    unsigned dma_ptr = DCH3SPTR/SPDIF_FRAMESIZE;
    if( spdif.tx_ptr >= dma_ptr ){
	len = spdif.tx_ptr - dma_ptr;
    }else{
	len = SPDIF_BUFLEN - dma_ptr + spdif.tx_ptr;
    }
    if( len > 0){
	len-= 1;
    }
    return len;
}

void spdif_out_tx_s16le(int16_t *frames, unsigned n_frames){
    int i;
    unsigned free;
    if(n_frames > 0){
        spdif.tx_ctr += n_frames;
        free =  SPDIF_BUFLEN - spdif_out_get_buflen();
        if( n_frames > free ){
            //log_error("S/PDIF buffer full: n_frames=%u, buflen=%u\n",
            //	      n_frames, free);
            spdif.bov_ctr++;
            n_frames = free;
        }
        for(i = 0; i< n_frames; i++){
            spdif_encode_frame_s16le(&spdif.encoder,
                                        &spdif.spdif_buf[spdif.tx_ptr++],
                                        &frames[2*i]);
            if( spdif.tx_ptr >= SPDIF_BUFLEN){
                spdif.tx_ptr = 0;
            }
        }
    }
}

#define RO_BASE_CLOCK SYS_CLOCK

int spdif_out_set_rate(unsigned frames_sec){
    
    unsigned bitclock = frames_sec * SPDIF_FRAMESIZE * 8;
    unsigned N =  (RO_BASE_CLOCK/2)/bitclock;
    unsigned M =  ((RO_BASE_CLOCK/2)%bitclock) * 512 / bitclock;
    
    REFOCON = 0;
   
    if( N == 0 || N > 32767){
	log_error("audio frame rate %u out of range\n", frames_sec);
	return -1;
    }
    spdif.sampling_rate = frames_sec;
    spdif.sampling_rate_adj = 0;
    log_debug("reference clock settings: N=%u, M=%u (%u Hz)\n", N, M, bitclock);
    
    REFOTRIM = M << _REFOTRIM_ROTRIM_POSITION;
    REFOCON = (N << _REFOCON_RODIV_POSITION) |
	      (0b0111 << _REFOCON_ROSEL_POSITION) | /* USB PLL output */
	      _REFOCON_ON_MASK;
    return 0;
}

static int spdif_out_tune_rate(int adj){
    
    unsigned irqstate;
    unsigned bitclock = spdif.sampling_rate * SPDIF_FRAMESIZE * 8;
    unsigned N =  (RO_BASE_CLOCK/2)/bitclock;
    int      M = ((RO_BASE_CLOCK/2)%bitclock) * 512 / bitclock - adj;

    if(M < 0){
	log_error("cannot further increase the clock frequency\n");
	return -1;
    }
    if(M > 511){
	log_error("cannot further decrease the clock frequency\n");
	return -1;
    }
    log_debug("ref. clock: N=%u, M=%d, adj=%d, freq=%u\n",
	      N, M, adj, (unsigned)(512LL*(RO_BASE_CLOCK/2) / (512 * N + M)));
    irqstate = mips_di();
    REFOTRIM = M << _REFOTRIM_ROTRIM_POSITION;
    REFOCONSET = _REFOCON_DIVSWEN_MASK;
    mips_restore_irq(irqstate);
    return 0;
}


void __ISR(_DMA_3_VECTOR, IPL4SOFT) spdif_out_dma_isr(void) {

    /* set active if there were insertions since the last IRQ */
    spdif.tx_active = spdif.tx_ctr_last != spdif.tx_ctr;
    spdif.tx_ctr_last = spdif.tx_ctr;
    if(spdif.tx_active){
	spdif.buflen_avg = (15*spdif.buflen_avg + spdif_out_get_buflen())/16;
    }
        
    DCH3INTCLR = 0x000000ff; /* clear all IRQ flags of the channel */
#if __PIC32_FEATURE_SET__ == 470
    IFS2CLR = _IFS2_DMA3IF_MASK; /* clear IRQ flag of the DMA controller */
#else
    IFS1CLR = _IFS1_DMA3IF_MASK; /* clear IRQ flag of the DMA controller */
#endif
}

static inline int spdif_out_round(float x){
    return x>=0 ? (long)((x)+0.5) : (long)((x)-0.5);
}

void spdif_out_tasks(void){
    int i;
    int old_adj;
#if 0
    static float e = 0;
    const float Tn = 5;
    float fs = 10; /* fsamp of I controller */

    if(timer_now() >= spdif.next_buffer_check){
	spdif.next_buffer_check = timer_now() + 100*TIMER_TICKS_PER_MS;
	if(spdif.tx_active){
	    //buflen = spdif_out_get_buflen();
	    /* I controller for clock frequency adjustment */
	    old_adj = spdif_out_round(10.0*spdif.sampling_rate_adj);
	    //e = (float)buflen - (float)(SPDIF_BUFLEN/2)/(float)SPDIF_BUFLEN;
	    e = (float)spdif.slack / (float)SPDIF_BUFLEN;
	    spdif.sampling_rate_adj += e / fs / Tn;

	    if(spdif.sampling_rate_adj > 100.0){
		spdif.sampling_rate_adj = 100.0;
	    }
	    if(spdif.sampling_rate_adj < -100.0){
		spdif.sampling_rate_adj = -100.0;
	    }
	    adj = spdif_out_round(10.0*spdif.sampling_rate_adj);
	    if( adj !=  old_adj){
		//spdif_out_tune_rate(adj);
	    }
	}
    }
#endif
    
    if(spdif.tx_active && timer_now() >= spdif.next_buffer_check){
	spdif.next_buffer_check = timer_now() + 1 * TIMER_TICKS_PER_SECOND;
	
	unsigned br = spdif.tx_ctr - spdif.tx_ctr_brm;
	spdif.tx_ctr_brm = spdif.tx_ctr;
	
	log_debug("overflows: %u, buflen=%u, buflen_avg=%u, adj=%d, br=%u\n",
		 spdif.bov_ctr, spdif_out_get_buflen(), spdif.buflen_avg,
		 spdif.sampling_rate_adj, br);
	old_adj = spdif.sampling_rate_adj;
	if( spdif.buflen_avg < SPDIF_BUFLEN/4){
	    spdif.sampling_rate_adj = -1;
	}else if( spdif.buflen_avg > SPDIF_BUFLEN*3/4){
	    spdif.sampling_rate_adj = 1;
	}
	if(old_adj != spdif.sampling_rate_adj){
	    spdif_out_tune_rate(spdif.sampling_rate_adj);
	}
    }

    if(!spdif.tx_active){
	/* fill S/PDIF buffer with silence */
	for(i = 0; i<SPDIF_BUFLEN; i++){
	    spdif_encode_frame_generic(&spdif.encoder, &spdif.spdif_buf[i], 0, 0);
	}
	spdif.bov_ctr = 0;
	spdif.sampling_rate_adj = 0.0;
	spdif.buflen_avg = SPDIF_BUFLEN/2;
    }
    set_active_led(spdif.tx_active);
}

void spdif_out_init(void){
    spdif.tx_ptr = 0;
    spdif.tx_ctr = 0;
    spdif.tx_ctr_last = 0;
    spdif.bov_ctr = 0;
    spdif.next_buffer_check = 0;
    spdif_encoder_init(&spdif.encoder);

    spdif_out_set_rate(48000);
    /* initialize SPI2 */
#ifdef __32MX470F512H__
    RPD3R = 0b0110; /* map SDO2 to RPD3 */
    ANSELDCLR = _ANSELD_ANSD3_MASK;
#else
    RPA4R = 0b0100; /* map SDO2 to pin RPA4 */
#endif
    SPI2CON = _SPI2CON_MCLKSEL_MASK |
	      _SPI2CON_ENHBUF_MASK | 
	      _SPI2CON_MODE32_MASK |
	      _SPI2CON_MSTEN_MASK |
	      _SPI2CON_DISSDI_MASK |
	      (0b11 << _SPI2CON_STXISEL_POSITION);
    SPI2BRG = 0;
    
    /* initialize and start DMA */
    DMACONSET = _DMACON_ON_MASK;
    
    DCH3ECONbits.CHSIRQ= _SPI2_TX_IRQ; /* DMA for TX on SPI2 */
    
    DCH3ECON = (_SPI2_TX_IRQ << _DCH3ECON_CHSIRQ_POSITION)|
	       _DCH3ECON_SIRQEN_MASK;
    
    DCH3SSA = virt_to_phys(spdif.spdif_buf);
    DCH3SSIZ = sizeof(spdif.spdif_buf);
    DCH3DSA= virt_to_phys((void *)&SPI2BUF);
    DCH3DSIZ= 4;
    DCH3CSIZ= 4;
    DCH3CON = _DCH3CON_CHEN_MASK |
	      _DCH3CON_CHAEN_MASK;
    /* enable DMA interrupt */
    DCH3INT= _DCH3INT_CHBCIE_MASK;
    
#if __PIC32_FEATURE_SET__ == 470
    IPC11bits.DMA3IP = 4;
    IEC2SET = _IEC2_DMA3IE_MASK;
#else
    IPC10bits.DMA3IP = 4;
    IEC1SET= _IEC1_DMA3IE_MASK;
#endif

    SPI2CONSET = _SPI2CON_ON_MASK;
    log_debug("audio interface initialized.\n");
}


