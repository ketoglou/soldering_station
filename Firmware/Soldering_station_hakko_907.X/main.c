/*
 * File:   main.c
 * Author: orion
 *
 * Created on October 25, 2019, 9:03 PM
 */

/* EEPROM:
 * ADDRESS: 0                   1
 * DATA   : HIGH_BYTE_OLD_TEMP  LOW_BYTE_OLD_TEMP
 */

//Includes
#include <xc.h>
#include <pic18f27k42.h>
#include "config.h"
#include "i2c.h"
#include "oled_sdd1306.h"
#include "init_strings.h"

// This code is meant to run on a PIC running at 64 MHz.
#define _XTAL_FREQ 64000000
#define byte unsigned char
#define LED_STAT LATAbits.LA0
#define LED_READY LATAbits.LA1
#define MOSFET1 LATCbits.LC6
#define MOSFET2 LATCbits.LC7

//Define functions
void timer0_init(void);
void external_interrupt_init(void);
void write_eeprom(byte data,byte address[2]); //address[0]=high byte of address,address[1]=low byte of address
byte read_eeprom(byte address[2]);
void timer2PWM_init(void);
void ADC_Init(void);
void ADC_Start(byte pin);

//Define variables
int seconds_passed;
//byte max6675_read; //MAX6675 could not be expected to produce more than about 5 reads per second.
int set_temp = 180;
byte set_temperature[3];
byte real_temperature[3];
byte temp_change = 1;
int old_temp = 0;
byte eeprom_address[2];
int real_temp = 0;
int temporary_val; //temporary value
float dV,dV_average,Current,Vx,Vx_average,R,Temp;
byte measure_flag = 0;
byte timer2_pwm_counter = 0;
byte pulse_width;
int adc_result;
byte adc_flag;
//byte max6675_counter = 0;
CHAR detached_str[3] = {PAVLA,PAVLA,PAVLA};

//------------------------------------------------------------------------------
void __interrupt(irq(IRQ_TMR0)) TIMER0_ISR(void){
    TMR0L= 0xDB;
    TMR0H= 0x0B;
    seconds_passed++;
    measure_flag++;
    //max6675_read =1;
    LED_STAT = !LED_STAT;
    //if 10 minutes passed lower the temperature
    if(seconds_passed == 600){
        old_temp = set_temp;
        set_temp = 180;
        temp_change = 1;
    }else if(seconds_passed == 1800){ //if 30 minutes passed zero the temperature
        set_temp = 0;
        seconds_passed = 0;
        T0CON0bits.EN = 0; //Disable timer
        LED_STAT = 0;
        temp_change = 1;
    }
    PIR3bits.TMR0IF = 0; //Clear interrupt flag
}

//Period:0.01sec
//Interupt:0.0001sec
void __interrupt(irq(IRQ_TMR2)) TIMER2_ISR(void){
    timer2_pwm_counter++;
    if(timer2_pwm_counter >= pulse_width){
        MOSFET1 = 0;
        MOSFET2 = 0;
    }else{
        MOSFET1 = 1;
        MOSFET2 = 1;
    }
    if(timer2_pwm_counter == 100){
        timer2_pwm_counter = 0;
        MOSFET1 = 1;
        MOSFET2 = 1;
    }
    PIR4bits.TMR2IF = 0; //Clear interrupt flag
}

void __interrupt(irq(IRQ_AD)) ADC_ISR(void){
    adc_result = ADRESL;
    adc_result = adc_result | (ADRESH <<8);
    adc_flag = 1;
    PIR1bits.ADIF = 0; //Clear interrupt flag
}

void __interrupt(irq(IRQ_INT0)) INT0_ISR(void){
    if(!T0CON0bits.EN){
        set_temp = old_temp;
        T0CON0bits.EN = 1;
        temp_change = 1;
    }else if(set_temp == 180 && seconds_passed > 600){
        set_temp = old_temp;
        temp_change = 1;
    }
    seconds_passed = 0;
    PIR1bits.INT0IF = 0;
}

void __interrupt(irq(IRQ_INT1)) INT1_ISR(void){
    PIE7bits.INT2IE = 0;
    PIR7bits.INT2IF = 0;
    if((set_temp == 180 && seconds_passed > 600)|| !T0CON0bits.EN){
        set_temp = old_temp;
        if(!T0CON0bits.EN)
            T0CON0bits.EN = 1; //Enable timer
    }
    if(set_temp < 500)
        set_temp++;
    temp_change = 1;
    seconds_passed = 0;
    PIE5bits.INT1IE = 0;
    PIR5bits.INT1IF = 0;
}

void __interrupt(irq(IRQ_INT2)) INT2_ISR(void){
    PIE5bits.INT1IE = 0;
    PIR5bits.INT1IF = 0;
    if((set_temp == 180 && seconds_passed > 600)|| !T0CON0bits.EN){
        set_temp = old_temp;
        if(!T0CON0bits.EN)
            T0CON0bits.EN = 1; //Enable timer
    }
    if(set_temp > 180)
        set_temp--; 
    temp_change = 1;
    seconds_passed = 0;
    PIE7bits.INT2IE = 0;
    PIR7bits.INT2IF = 0;
}

void __interrupt(irq(IRQ_NVM)) EEPROM_ISR(void){
    NVMCON1bits.WREN = 0; //Disable writes
    PIR0bits.NVMIF = 0;
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

void __interrupt(irq(default)) DEFAULT_ISR(void){
    //Unhandled interrupts go here
    Reset(); //In normal conditions we will never come here
}

//------------------------------------------------------------------------------
void main(void) {
    //Oscillator Configuration(We use crystal oscillator)
    OSCCON1 = 0x20; //ExtOSC with 4xPLL
    OSCCON2 = 0x20;
    OSCCON3 = 0x00;
    OSCEN = 0x80;
    while(!OSCSTATbits.EXTOR || !OSCSTATbits.PLLR);
    
    //IVTBASE* Registers must change if we use multiple IVTs(bootloader for instance)
    /*IVTBASEU = 0x00;
    IVTBASEH = 0x00;
    IVTBASEL = 0x00;*/
    IVTLOCK = 0x01; //IVTBASE Registers are locked and cannot be written(IVTWAY=1)
    
    //PPS Configuration
    PPSLOCKbits.PPSLOCKED = 0; //PPS selections
    
    ANSELC = 0x00;
    TRISAbits.TRISA0 = 0; //LED STATUS
    TRISAbits.TRISA1 = 0; //LED READY
    TRISCbits.TRISC0 = 1; //Amplifier Input
    ANSELCbits.ANSELC0 = 1; //Read Analog Amplifier
    TRISCbits.TRISC1 = 1; //Vx Input
    ANSELCbits.ANSELC1 = 1; //Read Analog Vx
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    
    INTCON0bits.IPEN = 1; //Enable interrupt priority
    INTCON0bits.GIEH = 1; //Enable high priority interrupts
    INTCON0bits.GIEL = 1; //Enable low priority interrupts
    
    external_interrupt_init();
    I2C_Init();
    //SPI1_Init();
    ADC_Init();
    OLED_Init();
    timer0_init();
    timer2PWM_init();
    
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
    
    //Get previous temperature
    eeprom_address[0] = 0x00; //Address high byte
    eeprom_address[1] = 0x00; //Address low byte
    set_temp = read_eeprom(eeprom_address) << 8;
    eeprom_address[1] = 0x01; //Address low byte
    set_temp = set_temp | read_eeprom(eeprom_address);
    if(set_temp < 180)
        set_temp = 180;
    
    MOSFET1 = 1;
    MOSFET2 = 1;
    
    while(1){
        //Calculate encoder (set temperature)
        if(temp_change){
            temporary_val = set_temp/100;
            set_temperature[0] = temporary_val;
            temporary_val = (set_temp-temporary_val*100)/10;
            set_temperature[1] = temporary_val;
            temporary_val = set_temp-(set_temp/100)*100-temporary_val*10;
            set_temperature[2] = temporary_val;
            OLED_WriteNumber(set_temperature,3,5,88);
            //Write to EEPROM
            eeprom_address[0] = 0x00; //Address high byte
            eeprom_address[1] = 0x00; //Address low byte
            temporary_val = set_temp >> 8; //High temp byte
            write_eeprom(temporary_val,eeprom_address);
            temporary_val = set_temp; //Low temp byte
            eeprom_address[1] = 0x01; //Address low byte
            write_eeprom(temporary_val,eeprom_address);
            
            while(!PORTBbits.RB1 || !PORTBbits.RB2); //Wait here to suspend any voltage spikes from encoder
            
            temp_change = 0;
            PIR5bits.INT1IF = 0;
            PIR7bits.INT2IF = 0;
            PIE5bits.INT1IE = 1;
            PIE7bits.INT2IE = 1;
        }
        /*
         //Thermocouple Hakko 907 soldering iron
         if(!PIE2bits.SPI1RXIE && max6675_read){
            if(flag_detached){
                OLED_WriteString(detached_str,3,6,88);
                max6675_counter = 0;
            }else{
                if(!max6675_counter){
                    real_temp = 0.5+((float)spi_temperature*0.25);
                    max6675_counter = 1;
                }else if(max6675_counter){
                    real_temp = real_temp + (0.5+((float)spi_temperature*0.25));
                    real_temp = real_temp/2;
                    temporary_val = real_temp/100;
                    real_temperature[0] = temporary_val;
                    temporary_val = (real_temp-temporary_val*100)/10;
                    real_temperature[1] = temporary_val;
                    temporary_val = real_temp-(real_temp/100)*100-temporary_val*10;
                    real_temperature[2] = temporary_val;
                    OLED_WriteNumber(real_temperature,3,6,88);
                    max6675_counter = 0;
                }
            }
            SPI1_Master_RecieveOnly();
            max6675_read = 0;
        }*/
        
        //Thermistor Hakko 907 soldering iron
        //Calculate Temperature
        if(measure_flag == 2){
            //Measure current 10 times
            dV_average = 0;
            for(byte i=0;i<200;i++){
                while(ADCON0bits.GO);
                ADC_Start(0x10); //RC0=Amplifier Voltage
                while(!adc_flag);//Wait if an ADC process is still processing
                dV = adc_result * 0.00122; //Real voltage real
                dV = dV/20.0; //Because is amplified by x20 
                dV_average = dV_average + dV;
            }
            dV_average = dV_average/200;
            Current = dV_average/0.05; //Current across 0.05 Ohm resistance
            if(Current != 0){
                Vx_average = 0;
                for(byte i=0;i<10;i++){
                    while(ADCON0bits.GO);
                    ADC_Start(0x11); //RC1=Vx Voltage
                    while(!adc_flag);//Wait if an ADC process is still processing  
                    Vx = adc_result * 0.00122; //Real voltage real
                    Vx_average = Vx_average + Vx;
                }
                Vx_average = Vx_average/10;
                R = Vx_average/Current;
            }else
                R = 0;
            if(R > 48 && R < 200){
                Temp = 5.95*R - 281.3; //Based in least squares
                real_temp = (int)Temp;
                temporary_val = real_temp/100;
                real_temperature[0] = temporary_val;
                temporary_val = (real_temp-temporary_val*100)/10;
                real_temperature[1] = temporary_val;
                temporary_val = real_temp-(real_temp/100)*100-temporary_val*10;
                real_temperature[2] = temporary_val;
                OLED_WriteNumber(real_temperature,3,6,88);
            }else{
                OLED_WriteString(detached_str,3,6,88);
            }
            measure_flag = 0;
        }
        
        //Calculate Pulse
        if(set_temp > real_temp && (set_temp - real_temp) > 20)
            pulse_width = 40;
        else if(set_temp > real_temp)
            pulse_width = (((set_temp - real_temp)*25)/20)+5;
        else
            pulse_width = 4; 
        
        //READY LED
        if(set_temp > real_temp && (set_temp-real_temp) < 10)
            LED_READY = 1;
        else if(set_temp < real_temp && (real_temp-set_temp) < 10)
            LED_READY = 1;
        else if(set_temp == real_temp)
            LED_READY = 1;
        else
            LED_READY = 0;
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
    seconds_passed = 0;
    IPR3bits.TMR0IP = 0; //Low priority interrupt
    PIR3bits.TMR0IF = 0; //Clear interrupt flag
    PIE3bits.TMR0IE = 1; //Enable interrupt
    T0CON0bits.EN = 1;
}
//--------------------------------PWM-------------------------------------------

//Period:0.001sec
//Interupt:0.0001sec
void timer2PWM_init(void){
    TRISCbits.TRISC6 = 0; //MOSFET1 -> PWM5
    TRISCbits.TRISC7 = 0; //MOSFET2 -> PWM6
    ANSELCbits.ANSELC6 = 0;
    ANSELCbits.ANSELC7 = 0;
    
    T2CON= 0x60; //Prescale:64,Postscale:1
    T2PR = 25;
    T2CLK = 0x01; //CLK = Fosc/4 = 16MHz
    T2HLT = 0x00;
    IPR4bits.TMR2IP = 1; //High priority interrupt
    PIR4bits.TMR2IF = 0; //Clear interrupt flag
    PIE4bits.TMR2IE = 1; //Enable interrupt
    T2CONbits.ON = 1;
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

//------------------------------------------------------------------------------
void write_eeprom(byte data,byte address[2]){
    
    NVMCON1 = 0x00; //Access EEPROM memory locations,set REG bits to 00
    
    while(NVMCON1bits.WR); //Wait if we still write
    PIR0bits.NVMIF = 0; //Clear flag
    IPR0bits.NVMIP = 1; //High priority
    
    NVMADRH = 0x03 & address[0];
    NVMADRL = address[1];
    NVMDAT = data;
    
    NVMCON1bits.WREN = 1; //Enable writes
    INTCON0bits.GIE = 0; //Disable interrupts
    
    //Required unlock sequence
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
   
    NVMCON1bits.WR = 1; //Set WR bit to begin write  
    PIE0bits.NVMIE = 1; //Enable NVM interrupt
    INTCON0bits.GIE = 1; //Enable interrupts
}

byte read_eeprom(byte address[2]){
    byte temp;
    NVMCON1 = 0x00; //Access EEPROM memory locations,set REG bits to 00

    NVMADRH = 0x03 & address[0];
    NVMADRL = address[1];
    
    NVMCON1bits.RD = 1; //Read data
    temp = NVMDAT;
    return temp;
}

//------------------------ADC Functions-----------------------------------------

void ADC_Init(void){
    //Voltage measure
    ADCON0 = 0x94;
    ADCON1 = 0x00;
    ADCON2 = 0x00;
    ADREF = 0x00; //VDD,VSS as references
    IPR1bits.ADIP = 0; //Low priority
    PIE1bits.ADIE = 1; //Enable interrupt
    PIR1bits.ADIF = 0; //Clear interrupt flag
}

//0-7 =RA0-RA7,8-15=RB0-RB7,16-23=RC0-RC7
void ADC_Start(byte pin){
    if(!ADCON0bits.GO){
        ADPCH = pin;
        adc_result = 0;
        adc_flag = 0;
        ADCON0bits.GO = 1;
    }
}