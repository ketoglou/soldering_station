#include "spi.h"
#include <pic18f27k42.h>

//Interrupt
void __interrupt(irq(IRQ_SPI1RX)) SPI_RX_ISR(void){
    if(byte_received == 0){
        spi_temperature = SPI1RXB;
        byte_received ++;
        SPI1TXB = 0x08;
    }else{
        byte_received = 0;
        byte data = SPI1RXB;
        if(data & 0x08){
            flag_detached = 1;
        }else{
            spi_temperature = spi_temperature << 8;
            spi_temperature = (spi_temperature | data) >> 4;
            flag_detached = 0;
        }
        PIE2bits.SPI1RXIE = 0; //Disable RX interrupt
        SPI1CON2bits.RXR = 0;
    }
}

void SPI1_Init(void){
    // Pins Configuration
    TRISC = 0x04; //RC0(CS),RC1(SCK) = output,RC2(DI) = input
    ANSELC = 0x00;    // Digital I/O;
    RC0PPS = 0x20;    // RC0->CS
    RC1PPS = 0x1E;    // RC1->SCK
    SPI1SDIPPS = 0x12; //RC2->DI
    
    //Interrupts
    IPR2bits.SPI1RXIP = 0; //Low priority interrupt
    
    // SPI Configuration
    SPI1CON1 = 0x94;  // CKE; CKP; SDI/SDO Polarity;
    SPI1CON2 = 0x01;  // TXR; RXR;
    SPI1BAUD = 7;  // SPI Baud Pre-Scaler = Fbaud = Fosc/(2xBaud)+1 = 4.26MHz
    SPI1CLK = 0x00;   // SPI Clock Select = Fosc
    SPI1TWIDTH = 0x00; //8 bits for each transfer
    SPI1CON0 = 0x83;  // SPI Enable; BMODE; Master/Slave; MSb/LSb;
    byte_received = 0;
    flag_detached = 0;
}

void SPI1_Master_RecieveOnly(void){
    spi_temperature = 0;
    SPI1CON2bits.TXR = 0; //Transmit Data-Required Bit;
    SPI1CON2bits.RXR = 1; // Receive FIFO Space-Required Bit;
    PIE2bits.SPI1RXIE = 1; //Enable RX interrupt
    SPI1TCNT = 2; // Load SPI Transfer Counter;
    SPI1TXB = 0x08; // Load a value into SPI transmit buffer;
}
