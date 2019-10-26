#include <xc.h>
#include "i2c.h"

void I2C_Init(void){
    //Configure SDA,SCL Pins
    TRISCbits.TRISC3 = 0; //SCL
    TRISCbits.TRISC4 = 0; //SDA
    LATCbits.LATC3 = 0; // Clear PORTC write latches
    LATCbits.LATC4 = 0;
    ANSELCbits.ANSELC3 = 0; //RC3 clear analog
    ANSELCbits.ANSELC4 = 0; //RC4 clear analog
    ODCONCbits.ODCC3 = 1; //RC3 as open-drain
    ODCONCbits.ODCC4 = 1; //RC4 as open-drain
    RC3I2C = 0x21;// Standard GPIO slew rate,Internal pull-ups x10,I2C specific thresholds
    RC4I2C = 0x21;
    WPUC = 0x18; //RC3,RC4 pull up enabled
    SLRCONCbits.SLRC3 = 0; // No slew rate limiting
    SLRCONCbits.SLRC4 = 0;
    I2C1SCLPPS = 0x13; //RC3 PPS Input
    I2C1SDAPPS = 0x14; //RC4 PPS Input
    RC3PPS = 0x21; //RC3 PPS Output
    RC4PPS = 0x22; //RC4 PPS Output
    
    I2C1CON0 = 0x04;
    I2C1CON1 = 0x80;
    I2C1CON2 = 0x24; //I2C Clock = I2C1CLK(500kHz)/4 = 125kHz
    I2C1CLK = 0x03; //MFINTOSC as clock (500kHz)
    I2C1CNT = 0;
           
    I2C1PIR = 0x00;
    I2C1ERR = 0x00; 
    
    I2C_STOP_DETECTED = 1;
    
    IPR3bits.I2C1TXIP = 0; //Transmit interrupt low priority
    IPR2bits.I2C1RXIP = 0; //Receive interrupt low priority
    PIR3bits.I2C1TXIF = 0; //Transmit interrupt flag clear
    PIR2bits.I2C1RXIF = 0; //Receive interrupt flag clear
    PIE3bits.I2C1TXIE = 1; //Transmit interrupt enable
    PIE2bits.I2C1RXIE = 1; //Receive interrupt enable
    
    //General interrupts
    I2C1PIRbits.PC1IF = 0;//Clear STOP interrupt flag
    I2C1PIEbits.PC1IE = 1; //Enable STOP interrupt
    IPR3bits.I2C1IP = 0; //General interrupt low priority
    PIR3bits.I2C1IF = 0; //General interrupt flag clear
    PIE3bits.I2C1IE = 1; //General interrupt enable
    
    I2C1CON0bits.EN = 1; //Enable I2C module
}

unsigned char I2C_Transmit(byte *buffer,unsigned char buffer_size,byte address){
    if(I2C_STOP_DETECTED && I2C1STAT0bits.BFRE && I2C1CNT == 0){
        I2C_STOP_DETECTED = 0;
        for(unsigned char i=0;i<(buffer_size-1);i++){
            I2C_TX_BUFFER[i] =  buffer[i+1];
        }
        I2C1ADB1 = address;
        I2C1CNT = buffer_size;
        I2C1TXB = buffer[0];
        I2C_TX_COUNTER = 0;
        I2C1CON0bits.S = 1;
        return 1;
    }
    return 0;
}