#include "sn74hc595.h"

void sn74hc595_init()
{
	// enable pull-ups for pins that idle high to avoid an unecessary edge
	SN74HC595_RCLK_PORT_OUT |= _BV(SN74HC595_RCLK_PIN);
	SN74HC595_RCLK_PORT_DDR |= _BV(SN74HC595_RCLK_PIN);
}

void sn74hc595_set_output(uint8_t output)
{
	// assert CS
	SN74HC595_RCLK_PORT_OUT &= (uint8_t)~_BV(SN74HC595_RCLK_PIN);

	spi_write(output); // put value in shift resister

	// deassert CS
	SN74HC595_RCLK_PORT_OUT |= _BV(SN74HC595_RCLK_PIN);
}
