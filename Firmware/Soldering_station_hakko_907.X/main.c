/*
 * File:   main.c
 * Author: orion
 *
 * Created on October 25, 2019, 9:03 PM
 */


//Includes
#include <xc.h>
#include "config.h"
#include "i2c.h"
#include "oled_sdd1306.h"
#include "init_strings.h"

// This code is meant to run on a PIC running at 64 MHz.
#define _XTAL_FREQ 64000000
#define byte unsigned char
#define LED LATAbits.LA0

//Define functions
void timer0_init(void);
void external_interrupt_init(void);

//Define variables
int set_temp = 0;
byte int_flag = 0;
byte set_temperature[3];
byte temp_change = 1;

//------------------------------------------------------------------------------
void __interrupt(irq(IRQ_TMR0)) TIMER0_ISR(void){
    TMR0L= 0xDB;
    TMR0H= 0x0B;
    LED = !LED;
    PIR3bits.TMR0IF = 0; //Clear interrupt flag
}

void __interrupt(irq(IRQ_INT0)) INT0_ISR(void){
    PIR1bits.INT0IF = 0;
}

void __interrupt(irq(IRQ_INT1)) INT1_ISR(void){
    PIE7bits.INT2IE = 0;
    PIR7bits.INT2IF = 0;
    if(set_temp < 401)
        set_temp++;
    temp_change = 1;
    PIE5bits.INT1IE = 0;
    PIR5bits.INT1IF = 0;
}

void __interrupt(irq(IRQ_INT2)) INT2_ISR(void){
    PIE5bits.INT1IE = 0;
    PIR5bits.INT1IF = 0;
    if(set_temp > 0)
        set_temp--;
    temp_change = 1;
    PIE7bits.INT2IE = 0;
    PIR7bits.INT2IF = 0;
}

void __interrupt(irq(IRQ_I2C1TX)) I2C_TX_ISR(void){
    I2C1TXB = I2C_TX_BUFFER[I2C_TX_COUNTER];
    I2C_TX_COUNTER ++;
}

void __interrupt(irq(IRQ_I2C1RX)) I2C_RX_ISR(void){
    I2C_RX_BUFFER[I2C_RX_COUNTER] = I2C1RXB;
    I2C_RX_COUNTER ++;
}

void __interrupt(irq(IRQ_I2C1)) I2C_GENERAL_ISR(void){
    if(I2C1PIRbits.PC1IF)
        I2C_STOP_DETECTED = 1;
    I2C1PIR = 0x00;
}

void __interrupt(irq(default)) default_isr(void){
    //Unhandled interrupts go here
    Reset(); //In normal conditions we will never come here
}

//------------------------------------------------------------------------------
void main(void) {
    OSCEN = 0x80;
    while(!OSCSTATbits.EXTOR || !OSCSTATbits.PLLR);
    
    //IVTBASE* Registers must change if we use multiple IVTs(bootloader for instance)
    /*IVTBASEU = 0x00;
    IVTBASEH = 0x00;
    IVTBASEL = 0x00;*/
    IVTLOCK = 0x01; //IVTBASE Registers are locked and cannot be written(IVTWAY=1)
    
    //PPS Configuration
    PPSLOCKbits.PPSLOCKED = 0; //PPS selections
    
    TRISAbits.TRISA0 = 0; //LED
    
    INTCON0bits.IPEN = 1; //Enable interrupt priority
    INTCON0bits.GIEH = 1; //Enable high priority interrupts
    INTCON0bits.GIEL = 1; //Enable low priority interrupts
    
    external_interrupt_init();
    timer0_init();
    I2C_Init();
    OLED_Init();
    
    PPSLOCKbits.PPSLOCKED = 1; //PPS selections
    
    //WatchDog Configuration,no Window 
    WDTCON1 = 0x07; //LFINTOSC used,Windows is open at 100% of time
    WDTCON0 = 0x1B; //32 seconds watchdog time,Enable watchdog
    
    //Initialize screen display
    OLED_WriteString(keto_st,12,0,OLED_POSITION);
    OLED_WriteNumber(hakko_num,3,0,OLED_POSITION);
    OLED_WriteString(sold_station,16,1,0);
    OLED_WriteString(temperature,12,3,0);
    OLED_WriteString(set,11,5,0);
    OLED_WriteString(real,11,6,0);
    OLED_WriteString(celsious,2,5,112);
    OLED_WriteString(celsious,2,6,112);
    
    while(1){
        if(temp_change){
            temp_change = 0;
            int temp = set_temp/100;
            set_temperature[0] = temp;
            temp = (set_temp-temp*100)/10;
            set_temperature[1] = temp;
            temp = set_temp-(set_temp/100)*100-temp*10;
            set_temperature[2] = temp;
            OLED_WriteNumber(set_temperature,3,5,88);
            while(!PORTBbits.RB1 || !PORTBbits.RB2);
            PIE5bits.INT1IE = 1;
            PIR5bits.INT1IF = 0;
            PIE7bits.INT2IE = 1;
            PIR7bits.INT2IF = 0;
        }
        CLRWDT(); //Clear Watchdog timer
    }
}

//------------------------------------------------------------------------------

//********Timer functions*******************************************************
void timer0_init(void){
    T0CON0 = 0x10;
    T0CON1 = 0x48;
    TMR0L= 0xDB;
    TMR0H= 0x0B;
    IPR3bits.TMR0IP = 0; //Low priority interrupt
    PIR3bits.TMR0IF = 0; //Clear interrupt flag
    PIE3bits.TMR0IE = 1; //Enable interrupt
    T0CON0bits.EN = 1;
}

//------------------------------------------------------------------------------
void external_interrupt_init(void){
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    ANSELBbits.ANSELB0 = 0;
    ANSELBbits.ANSELB1 = 0;
    ANSELBbits.ANSELB2 = 0;
    
    //INTxPPS SELECT EXTERNAL INTERRUPT PIN
    INT0PPS = 0x08;
    INT1PPS = 0x09;
    INT2PPS = 0x0A;
    
    INTCON0bits.INT0EDG = 0; //Interrupt on falling edge of INT0 pin
    INTCON0bits.INT1EDG = 0; //Interrupt on falling edge of INT1 pin
    INTCON0bits.INT2EDG = 0; //Interrupt on falling edge of INT2 pin
    
    IPR1bits.INT0IP = 1; //High priority interrupt
    IPR5bits.INT1IP = 1; //High priority interrupt
    IPR7bits.INT2IP = 1; //High priority interrupt
    PIR1bits.INT0IF = 0; //Clear interrupt flag
    PIR5bits.INT1IF = 0; //Clear interrupt flag
    PIR7bits.INT2IF = 0; //Clear interrupt flag
    PIE1bits.INT0IE = 1; //Enable interrupt
    PIE5bits.INT1IE = 1; //Enable interrupt
    PIE7bits.INT2IE = 1; //Enable interrupt
}