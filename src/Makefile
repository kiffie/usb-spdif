#
# Makefile for USB SPDIF output
# MCU: PIC32MX2xx
#

CC=pic32-gcc
BIN2HEX = pic32-bin2hex
OBJDUMP = pic32-objdump

PRG = usb-spdif

PROC = 32MX270F256B
#PROC = 32MX250F128B
#PROC = 32MX274F256B

CFLAGS = -Wall -g -mprocessor=$(PROC) -O2 -DSYS_CLOCK=48000000 -DI2C_X=1 \
         -DTERM_TXBUFSIZE=4096 \
         -Iinclude -Iirmp


LDFLAGS = $(CFLAGS) -Wl,-Map=$(PRG).map,--memorysummary,$(PRG).mem,--defsym=_min_heap_size=4096
LDFLAGS_APP := -Wl,--script=app_32MX270F256B.ld

OBJECTS = main.o pic32_config.o spdif_out.o spdif-encoder.o \
          mips_irq.o terminal.o tinyprintf.o timer.o \
          usb.o usb_descriptors.o usb_audio.o \
          irmp.o irhid.o

VPATH = irmp

DEPS    = $(patsubst %.o, %.d, $(notdir $(OBJECTS)))

.PHONY: all
all: $(PRG).hex $(PRG)_app.hex lst

%.o: %.c
	$(CC) -c $(CFLAGS) -MMD -o $@ $<

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@


$(PRG).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^
	
$(PRG).hex: $(PRG).elf
	$(BIN2HEX) $<

$(PRG)_app.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(LDFLAGS_APP) -o $@ $^

$(PRG)_app.hex: $(PRG)_app.elf
	$(BIN2HEX) $<

lst: $(PRG).lst

clean:
	rm -f $(OBJECTS) $(DEPS) $(PRG).elf $(PRG)_app.elf \
	      $(PRG).map $(PRG).map $(PRG).lst $(PRG).mem

-include $(DEPS)
