/* Bit banged SPI shared between sn74hc595 and pga2320 */

#ifndef _SPI_H
#define _SPI_H

#include <avr/io.h>
#include "minivol.h"

/* SPI configuration
	It's just bit banged and should work on any port on any AVR
*/

#define SPI_PORT      A
#define SPI_CLK_PIN   PA4
#define SPI_MOSI_PIN  PA6

#define SPI_PORT_OUT  STR_CONCAT(PORT, SPI_PORT)
#define SPI_PORT_DDR	STR_CONCAT(DDR, SPI_PORT)

void spi_init(void);
void spi_write(uint8_t byte);

#endif
