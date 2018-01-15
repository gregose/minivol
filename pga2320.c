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

#include "pga2320.h"

void pga_init()
{
	// enable pull-ups for pins that idle high to avoid an unecessary edge
	PGA_CS_PORT_OUT |= _BV(PGA_CS_PIN);
	PGA_CS_PORT_DDR |= _BV(PGA_CS_PIN);

	#if PGA_ENABLE_MT
		PGA_MT_PORT_DDR |= _BV(PGA_MT_PIN);
		PGA_MT_PORT_OUT &= (uint8_t)~_BV(PGA_MT_PIN);
	#endif

	pga_status.left_vol = 0;
	pga_status.right_vol = 0;
	pga_status.muted = 0;
}

#if PGA_ENABLE_MT
	void pga_set_mute(uint8_t muted)
	{
		if (muted)
			PGA_MT_PORT_OUT &= (uint8_t)~_BV(PGA_MT_PIN);
		else
			PGA_MT_PORT_OUT |= _BV(PGA_MT_PIN);
		pga_status.muted = muted;
	}
#endif

void pga_set_volume(uint8_t left, uint8_t right)
{
	// assert CS
	PGA_CS_PORT_OUT &= (uint8_t)~_BV(PGA_CS_PIN);

	spi_write(right);		// right
	spi_write(left);		// left

	// deassert CS
	PGA_CS_PORT_OUT |= _BV(PGA_CS_PIN);

	pga_status.left_vol = left;
	pga_status.right_vol = right;
}
