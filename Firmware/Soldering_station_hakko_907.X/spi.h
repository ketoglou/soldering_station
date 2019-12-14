#ifndef SPI_H
#define	SPI_H

#define byte unsigned char

int spi_temperature;
byte byte_received;
byte flag_detached;
// Call it to initialize the module with interrupts
void SPI1_Init(void);
void SPI1_Master_RecieveOnly(void);

#endif	/* SPI_H */

