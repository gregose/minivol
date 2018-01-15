#ifndef _MINIVOL_H
#define _MINIVOL_H

/* convenience macros */
#ifndef STR_CONCAT_PRIM
#define STR_CONCAT_PRIM(a, b)	a ## b
#endif
#ifndef STR_CONCAT
#define STR_CONCAT(a, b)	STR_CONCAT_PRIM(a, b)
#endif


// BASIC OPTIONS //
#define GAIN_STEP	2   /* 1dB */
#define GAIN_CAP	192 /* 0dB */

// Uncomment below to use push button mode
//#define PUSH_BUTTON_MODE

// In milliseconds, how long to wait between each volume step while ramping. Set
// to -1 to disable ramping.
#define RAMP_DELAY 1
// How many steps to attenuate from the current level when muting. Set to 0 to mute
// fully.
#define MUTE_ATTN  0 /* full */
//#define MUTE_ATTN  72 /* 36dB */

// I/O PORT CONFIG //

#define MLED_PORT	B
#define MLED_PIN	PB0

#define GN_PORT	B
#define GN_PIN		PB2

// If you change these off of port A the interrupt code will need to change
// PCMSK0 must be set so that A, B and MT can trigger PCINT0

#define BTN_PORT	A
#define A_PIN		PA2
#define B_PIN		PA1
#define MT_PIN		PA0
#define BTN_MASK	(_BV(A_PIN) | _BV(B_PIN) | _BV(MT_PIN));

// number of times the input must match before a press is registered
#define DB_CHECKS	5		/* with default timer settings, 10ms */

// number of scan timer ticks / DB_CHECKS until a button is considered held
// counts each time the db_status.index == 0
#define HELD_COUNT 50 /* 0.5s */

// same as above, but for repeats after initial hold. must be <= held_count
#define REPEAT_COUNT 10 /* 0.1s */

// INTERRUPTS & TIMERS //

// TIMER0 - 8 bits
#define SCAN_TIMER_PS	(_BV(CS01) | _BV(CS00))		/* clk/64 */
#define SCAN_TIMER_CMP	125								/* 1ms with 8MHz/64 */

// TIMER1 - 16 bits
#define INACT_TIMER_PS	(_BV(CS12) | _BV(CS10))		/* clk/1024 */
#define INACT_TIMER_CMP	7813								/* 1s with 8MHz/64 */

#define SLEEP_MODE	SLEEP_MODE_PWR_DOWN

// MEMORY //
#define QUEUE_SIZE		5
#define EEPROM_BASE		((void *)0x00)	/* uses 3 bytes starting at this address */

// For convenience...
#define MLED_PORT_D		STR_CONCAT(DDR,MLED_PORT)
#define GN_PORT_D			STR_CONCAT(DDR,GN_PORT)
#define BTN_PORT_D		STR_CONCAT(DDR,BTN_PORT)

#define MLED_PORT_O		STR_CONCAT(PORT,MLED_PORT)
#define GN_PORT_O			STR_CONCAT(PORT,GN_PORT)
#define BTN_PORT_O		STR_CONCAT(PORT,BTN_PORT)

#define MLED_PORT_I		STR_CONCAT(PIN,MLED_PORT)
#define GN_PORT_I			STR_CONCAT(PIN,GN_PORT)
#define BTN_PORT_I		STR_CONCAT(PIN,BTN_PORT)

// OPCODE TABLE //
#define OP_NOP				0
#define OP_MUTE				1
#define OP_VOL_INCR		2
#define OP_VOL_DECR		3
#define OP_SLEEP			4

// SBITS bits //
#define SBITS_MUTED		0
#define SBITS_FRUN		1

// DATA TYPES //

// Store the queued events
struct minivol_queue_t {
	uint8_t	opcode;
};

/*
 * Store the status of the controller (doesn't coincide with PGA settings)
 * In particular, the volumes stored are not adjusted for mute status
 */
struct minivol_status_t {
	uint8_t	left_vol,
				right_vol,
				queue_next,
				sbits,
				enc_status; // Last read encoder state, see ENC_LUT comment

	volatile struct minivol_queue_t queue[QUEUE_SIZE];
};

/*
 * Switch debounce state machine
 */
struct db_status_t {
	uint8_t	final_state,
				state[DB_CHECKS],
				index;
	uint8_t  held_count_a,
	         held_count_b;
};

// GLOBALS //
volatile struct minivol_status_t status;
volatile struct db_status_t db_status;

// PROTOTYPES //
void pin_setup(void);
void int_setup(void);
void ramp_volume(uint8_t, uint8_t);
void set_volume(uint8_t, uint8_t);
void queue_event(uint8_t);
void process_event(void);
void start_ee_timer(void);
void write_ee_state(void);
void set_ee_state(void);
void start_scan_timer(void);
int main(void);

#endif
