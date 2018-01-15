#ifndef _SN74HC595_H
#define _SN74HC595_H

#include "spi.h"

/* Storage register clock
	HIGH -> LOW sets values into storage register, outputs to pins
	can be used same as CS in SPI
*/

#define SN74HC595_RCLK_PORT	B
#define SN74HC595_RCLK_PIN	PB1 // Use the ENC pin since it has a header

// end configuration //

#define SN74HC595_RCLK_PORT_OUT	STR_CONCAT(PORT, SN74HC595_RCLK_PORT)
#define SN74HC595_RCLK_PORT_DDR	STR_CONCAT(DDR, SN74HC595_RCLK_PORT)

void sn74hc595_init(void);

void sn74hc595_set_output(uint8_t byte);

#endif
