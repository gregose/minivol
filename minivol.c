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

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <util/atomic.h>

#include "minivol.h"
#include "spi.h"
#include "pga2320.h"
#include "sn74hc595.h"

// CONSTANTS //

/*
* Lookup table to decode rotary encoder. Indexed by a 4-bit nibble
* representing the previous state in the 2 MSB and the current state
* in the 2 LSB. Gives the decoded change for the the indicated state:
*  0 = invalid state change
*  1 = up
*  2 = down
*/
uint8_t ENC_LUT[16] =
{
	0,	// 00->00
	2, // 00->01
	1, // 00->10
	0, // 00->11
	1, // 01->00
	0, // 01->01
	0, // 01->10
	2, // 01->11
	2, // 10->00
	0, // 10->01
	0, // 10->10
	1, // 10->11
	0, // 11->00
	1, // 11->01
	2, // 11->10
	0	// 11->11
};

// Fuse definitions for self-clocked ATtiny24/44/84
FUSES = {
	.low	= (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT1),
	.high	= (FUSE_SPIEN),
	.extended = EFUSE_DEFAULT
};

// Sets up I/O pin registers
// If necessary on tiny uCs, a few bytes could be saved by setting each with one
// instruction instead of using the macros.
void pin_setup() {
	// Set up data direction registers
	MLED_PORT_D |= _BV(MLED_PIN);
	ENC_PORT_D  &= ~_BV(ENC_PIN);
	GN_PORT_D	&= ~_BV(GN_PIN);
	BTN_PORT_D	&= ~(_BV(A_PIN) | _BV(B_PIN) | _BV(MT_PIN));

	// Set up pull-ups on inputs
	ENC_PORT_O	|= _BV(ENC_PIN);
	GN_PORT_O	|= _BV(GN_PIN);
	BTN_PORT_O	|= _BV(A_PIN) | _BV(B_PIN) | _BV(MT_PIN);

	// Read current status of encoder inputs and store them at startup
	status.enc_status = (((BTN_PORT_I & _BV(A_PIN)) >> A_PIN) << 1) | ((BTN_PORT_I & _BV(B_PIN)) >> B_PIN);
}

// Initialize interrupts
void int_setup() {
	GIMSK = _BV(PCIE0);
	PCMSK0 = (_BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2));

	// TIMER0 interrupt fires on compare match a
	TIMSK0 = _BV(OCIE0A);
	// TIMER1 interrupt fires on compare match a
	TIMSK1 = _BV(OCIE1A);

	// Turn off USI and ADC since they're not in use
	PRR |= PRUSI | PRADC;

	sei();
}

// Ramp smoothly between the current volume and the given target
void ramp_volume(uint8_t target_l, uint8_t target_r) {
	#if RAMP_DELAY == -1
		pga_set_volume(target_l, target_r);
	#else
		uint8_t new_l, new_r;
		while (pga_status.left_vol != target_l || pga_status.right_vol != target_r) {
			new_l = (target_l > pga_status.left_vol) ?
							pga_status.left_vol + 1 :
							(target_l != pga_status.left_vol) ?
								pga_status.left_vol - 1 :
								pga_status.left_vol;
			new_r = (target_r > pga_status.right_vol) ?
							pga_status.right_vol + 1 :
							(target_r != pga_status.right_vol) ?
								pga_status.right_vol - 1 :
								pga_status.right_vol;

			pga_set_volume(new_l, new_r);
			#if RAMP_DELAY
				_delay_ms(RAMP_DELAY);
			#endif
		}
	#endif
}

// THIS FUNCTION MUST BE CALLED ATOMICALLY OR WITH INTERRUPTS DISABLED!!!
// Places event 'opcode' on the event queue
void queue_event(uint8_t opcode) {
	if (status.queue_next == QUEUE_SIZE)	// Queue is full, ignore event request
		return;
	status.queue[status.queue_next].opcode = opcode;
	status.queue_next++;
}

// Process waiting events on the queue
void process_event()
{
	uint8_t current_op,i;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (status.queue_next == 0)
			return;

		current_op = status.queue[0].opcode;

		for (i=1;i<status.queue_next;i++)
			status.queue[i-1] = status.queue[i];

		status.queue_next--;
	}

	switch (current_op) {
		case OP_MUTE:
			if (status.sbits & _BV(SBITS_MUTED)) {
				ramp_volume(status.left_vol, status.right_vol);
				status.sbits &= ~_BV(SBITS_MUTED);
				MLED_PORT_O &= ~_BV(MLED_PIN);
			} else {
				#if MUTE_ATTN
					ramp_volume(
						(MUTE_ATTN > status.left_vol) ? 0 : status.left_vol - MUTE_ATTN,
						(MUTE_ATTN > status.right_vol) ? 0 : status.right_vol - MUTE_ATTN
					);
				#else
					ramp_volume(0, 0);
				#endif
				status.sbits |= _BV(SBITS_MUTED);
				MLED_PORT_O |= _BV(MLED_PIN);
			}
			start_ee_timer();
			break;

		case OP_VOL_INCR:
			if (status.sbits & _BV(SBITS_MUTED)) {
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
					queue_event(OP_MUTE);
					return;
				}
			}


			if (!(GN_PORT_I & _BV(GN_PIN))) { // gain cap disabled
				status.left_vol = (status.left_vol > (0xff - GAIN_STEP)) ? status.left_vol : status.left_vol + GAIN_STEP;
				status.right_vol = (status.right_vol > (0xff - GAIN_STEP)) ? status.right_vol : status.right_vol + GAIN_STEP;
			} else {
				status.left_vol = (status.left_vol + GAIN_STEP > GAIN_CAP) ? status.left_vol : status.left_vol + GAIN_STEP;
				status.right_vol = (status.right_vol + GAIN_STEP > GAIN_CAP) ? status.right_vol : status.right_vol + GAIN_STEP;
			}

			ramp_volume(status.left_vol, status.right_vol);
			start_ee_timer();
			break;

		case OP_VOL_DECR:
			if (status.sbits & _BV(SBITS_MUTED))
				return;

			status.left_vol = (status.left_vol > GAIN_STEP) ? status.left_vol - GAIN_STEP : 0;
			status.right_vol = (status.right_vol > GAIN_STEP) ? status.right_vol - GAIN_STEP : 0;

			ramp_volume(status.left_vol, status.right_vol);
			start_ee_timer();
			break;

		case OP_SLEEP:
			set_sleep_mode(SLEEP_MODE);
			sleep_mode();
			break;

		default:
			break;
	}
}

// Scan timer interrupt handler. Uses a col-major shift register to
// debounce inputs, then dispatches an event as necessary on the event
// queue.
// ref: http://www.ganssle.com/debouncing.pdf
ISR(TIM0_COMPA_vect) {
	uint8_t i, acc, changed;

	// Inputs are pulled up, so invert the input port
	db_status.state[db_status.index] = ~BTN_PORT_I & BTN_MASK;
	db_status.index++;

	acc = BTN_MASK;

	for(i=0; i<DB_CHECKS; i++)
		acc = acc & db_status.state[i];

	changed = acc ^ db_status.final_state;
	db_status.final_state = acc;

	if (db_status.index>=DB_CHECKS) {
		db_status.index = 0;
		if (acc & _BV(A_PIN))
			db_status.held_count_a++;
		if (acc & _BV(B_PIN))
			db_status.held_count_b++;
	}

	if (changed) {
		if (changed & _BV(MT_PIN)) {
			if (acc & _BV(MT_PIN)) {
				queue_event(OP_MUTE);
			}
		}
		if (!(ENC_PORT_I & _BV(ENC_PIN))) {
			// Shift left 2 bits so previous state moves to the proper spot
			status.enc_status <<= 2;
			// Store current state in the 2 LSB
			status.enc_status |= (((acc & _BV(A_PIN)) >> A_PIN) << 1) | ((acc & _BV(B_PIN)) >> B_PIN);
			// Clear the top 4 bits
			status.enc_status &= 0x0f;

			if (ENC_LUT[status.enc_status] == 1) {
				queue_event(OP_VOL_INCR);
			}
			if (ENC_LUT[status.enc_status] == 2) {
				queue_event(OP_VOL_DECR);
			}
			// if it's an invalid state, do nothing
		} else {
			if (changed & _BV(A_PIN) && !(changed & _BV(B_PIN))) {
				if (acc & _BV(A_PIN)) {
						queue_event(OP_VOL_INCR);
				}
				db_status.held_count_a = 0;
			}

			if (changed & _BV(B_PIN) && !(changed & _BV(A_PIN))) {
				if (acc & _BV(B_PIN)) {
						queue_event(OP_VOL_DECR);
				}
				db_status.held_count_b = 0;
			}
		}
	}


	// Only do this if set for button mode
	if (ENC_PORT_I & _BV(ENC_PIN)) {
		if (db_status.held_count_a == HELD_COUNT) {
			queue_event(OP_VOL_INCR);
			db_status.held_count_a = HELD_COUNT - REPEAT_COUNT;
		}
		if (db_status.held_count_b == HELD_COUNT) {
			queue_event(OP_VOL_DECR);
			db_status.held_count_b = HELD_COUNT - REPEAT_COUNT;
		}
	}
}

ISR(TIM1_COMPA_vect)
{
	// stop the timer
	TCCR1B = 0;
	TIMSK1 = 0;
	write_ee_state();
	// Put it back in sleep mode
	queue_event(OP_SLEEP);
}

ISR(PCINT0_vect)
{
	// do nothing, this was just to wake up the cpu
	return;
}

void write_ee_state()
{
	uint8_t cur[3];
	cur[0] = eeprom_read_byte(EEPROM_BASE + 0);
	cur[1] = eeprom_read_byte(EEPROM_BASE + 1);
	cur[2] = eeprom_read_byte(EEPROM_BASE + 2);

	if (cur[0] != status.left_vol)
		eeprom_write_byte(EEPROM_BASE + 0, status.left_vol);
	if (cur[1] != status.right_vol)
		eeprom_write_byte(EEPROM_BASE + 1, status.right_vol);
	if (cur[2] != status.sbits)
		eeprom_write_byte(EEPROM_BASE + 2, status.sbits);
}

void set_ee_state()
{
	uint8_t cur[3];
	eeprom_read_block(&cur, EEPROM_BASE, 3);

	status.left_vol = cur[0];
	status.right_vol = cur[1];
	status.sbits = cur[2];

	/* on first run, start at 0 volume unmuted */
	if (status.sbits & _BV(SBITS_FRUN)) {
		status.left_vol = 0;
		status.right_vol = 0;
		status.sbits = 0;
		pga_set_volume(status.left_vol, status.right_vol);
	} else if (status.sbits & _BV(SBITS_MUTED)) {
		// OP_MUTE will both call pga_set_volume and calculate proper
		// muted volume.
		queue_event(OP_MUTE);
	} else {
		pga_set_volume(status.left_vol, status.right_vol);
	}
}

void start_ee_timer()
{
	OCR1A  = INACT_TIMER_CMP;
	TIMSK1 = _BV(OCIE1A);
	TCNT1	 = 0;	// a new hit of this trigger resets the timer

	// start the timer
	TCCR1B = INACT_TIMER_PS;
}

void start_scan_timer() {
	OCR0A = SCAN_TIMER_CMP;
	TCCR0A = _BV(WGM01);
	TCCR0B = SCAN_TIMER_PS;
}

int main() {
	pin_setup();
	spi_init();
	pga_init();
	sn74hc595_init();
	int_setup();
	start_scan_timer();

	set_ee_state();
	pga_set_mute(0);

	for (;;) {
		if (status.queue_next == 0)
			_delay_ms(1);
		else
			process_event();
	}

	return 0;
}
