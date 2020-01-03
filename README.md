# usb-spdif
USB audio S/PDIF output

This project uses the microcontroller PIC32MX270 to read audio data from an USB host and to output a corresponding SPDIF audio stream over an SPI port. The SPDIF bitstream is generated in software. As a consequence, no special encoder chip is needed. An optical SPDIF transmitter can directly connected to the SPI/I2S port of the microcontroller.

The binary image for the microcontroller can be compiled with the [`ChipKIT toolchain`]. Simply change to the `src` directory and type `make`. It should also be possible to use the XC32 toolcain (requires chaging the first few lines of the Makefile); but this has not been tested.

[`ChipKIT toolchain`]: https://github.com/chipKIT32/chipKIT-compiler-builds/releases
