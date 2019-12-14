#ifndef I2C_H
#define I2C_H

#define I2C_Enable() (I2C1CON0bits.EN = 1)
#define I2C_Disable() (I2C1CON0bits.EN = 0)

#define byte unsigned char
#define I2C_TX_BUFFER_SIZE 10
#define I2C_RX_BUFFER_SIZE 10

//I2C COMMUNICATION VARIABLES
byte I2C_TX_COUNTER;
byte I2C_RX_COUNTER;
byte I2C_TX_BUFFER[I2C_TX_BUFFER_SIZE];
byte I2C_RX_BUFFER[I2C_RX_BUFFER_SIZE];
byte I2C_STOP_DETECTED;

// Call it to initialize the module with interrupts
void I2C_Init(void);

/* buffer has maximum I2C_BUFFER_SIZE size and contains the commands that will be send,
 * buffer_size is the number of commands in buffer and address is the address to be transmitted.
 * This function return 1 if everything goes well or 0 if I2C module is busy.
 */
unsigned char I2C_Transmit(byte *buffer,unsigned char buffer_size,byte address);

#endif