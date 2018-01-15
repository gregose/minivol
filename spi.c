/* SPI code / bus shared by PGA2320 and SN74HC595
   device code responsible for asserting SS prior to writing
*/

#include "spi.h"

void spi_init()
{
  SPI_PORT_DDR |= _BV(SPI_CLK_PIN) | _BV(SPI_MOSI_PIN);
  SPI_PORT_OUT &= (uint8_t)~(_BV(SPI_CLK_PIN) | _BV(SPI_MOSI_PIN));
}

void spi_write(uint8_t byte)
{
	uint8_t i;

	for (i=0; i < 8; i++)
	{
		SPI_PORT_OUT &= (uint8_t)~_BV(SPI_CLK_PIN);

		if (0x80 & byte)	// MSB is set
			SPI_PORT_OUT |= _BV(SPI_MOSI_PIN);
		else
			SPI_PORT_OUT &= (uint8_t)~_BV(SPI_MOSI_PIN);

		SPI_PORT_OUT |= _BV(SPI_CLK_PIN);
		byte <<= 1;
	}
}
