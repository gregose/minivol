/*
Copyright (c) 2008, Keenan Tims
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in
		the documentation and/or other materials provided with the
		distribution.
    * Neither the name Keenan Tims, nor the names of other contributors
	   may be used to endorse or promote products derived from this
		software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 This sourcecode is firmware for the Atmel ATtiny24/44/84 to control
 a TI PGA2310/2311/2320 digital volume control. See
 http://audio.gotroot.ca/minivol/ for the accompanying hardware
 details.

 Version: 1.0
 Author: Keenan Tims (ktims@gotroot.ca)
*/

#ifndef _PGA2320_H
#define _PGA2320_H

#include "spi.h"

#define PGA_MT_PORT	A
#define PGA_MT_PIN	PA3

#define PGA_CS_PORT	A
#define PGA_CS_PIN	PA5

/* undefine this to free up the pin */
#define PGA_ENABLE_MT	1

// end configuration //

/* convenience... */
#define PGA_MT_PORT_OUT	STR_CONCAT(PORT, PGA_MT_PORT)
#define PGA_CS_PORT_OUT	STR_CONCAT(PORT, PGA_CS_PORT)

#define PGA_MT_PORT_DDR	STR_CONCAT(DDR, PGA_MT_PORT)
#define PGA_CS_PORT_DDR	STR_CONCAT(DDR, PGA_CS_PORT)

/* void pga_init()
 *
 * Should be called prior to use of other pga_* functions. Initializes
 * internal state and configures I/O pins.
 */
void pga_init(void);

#if PGA_ENABLE_MT
	/* void pga_mute(uint8_t)
	 *
	 * Set or unset the dedicated mute line to PGA. Valid values:
	 *  1 - mute asserted
	 *  0 - mute not asserted
	 */
	void pga_set_mute(uint8_t muted);
#endif

/* void pga_set_volume(uint8_t, uint8_t)
 *
 * Set the current volume setting on left/right channels of PGA.
 * See PGA datasheet for details. Valid values 0-255.
 */
void pga_set_volume(uint8_t left, uint8_t right);

struct pga_status_t {
	uint8_t	left_vol,
				right_vol,
				muted;
};

/* struct pga_status_t pga_status
 *
 * Current status will be available in this structure
 */
volatile struct pga_status_t pga_status;

#endif
