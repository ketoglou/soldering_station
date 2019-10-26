#include <xc.h>
#include "oled_sdd1306.h"

//OLED FUNCTION 
//------------------------------------------------------------------------------
void OLED_SendCommand(byte data,DATA_TYPE type){
    OLED_COMMANDS[1] = data;
    if(type == COMMAND){
        OLED_COMMANDS[0] = 0x00;
    }else{
        OLED_COMMANDS[0] = 0x40;
    }
    while(!I2C_Transmit(OLED_COMMANDS,2,OLED_ADDRESS));
    while(!I2C_STOP_DETECTED);
}

void OLED_Init(void){
    OLED_SendCommand(OLED_DISPLAY_OFF,COMMAND);
    
    OLED_SendCommand(OLED_SET_DISPLAY_CLOCK_DIV,COMMAND);
    OLED_SendCommand(0x80,COMMAND); //The suggested ratio 0x80
    
    OLED_SendCommand(OLED_SET_MULTIPLEX_RATIO,COMMAND);
    OLED_SendCommand(0x3F,COMMAND);
    
    OLED_SendCommand(OLED_SET_DISPLAY_OFFSET,COMMAND);
    OLED_SendCommand(0x00,COMMAND); //No offset
    
    OLED_SendCommand(OLED_SET_DISPLAY_START_LINE,COMMAND);
     
    OLED_SendCommand(OLED_SET_MEMORY_ADDRESSING_MODE,COMMAND);
    OLED_SendCommand(0x02,COMMAND); // Page addressing
    
    OLED_SendCommand(OLED_SET_SEGMENT_REMAP,COMMAND);
    OLED_SendCommand(OLED_SET_COM_OUTPUT_SCAN_DIR,COMMAND);
    
    OLED_SendCommand(OLED_SET_COM_PINS_HW_CONF,COMMAND);
    OLED_SendCommand(0x12,COMMAND);
    
    OLED_SendCommand(OLED_SET_CONSTRAST_CMD,COMMAND);
    OLED_SendCommand(0xFF,COMMAND);
    
    OLED_SendCommand(OLED_SET_PRECHARGE_PERIOD,COMMAND);
    OLED_SendCommand(0xF1,COMMAND);
    
    OLED_SendCommand(OLED_SET_VCOM_DESELECT_LEVEL,COMMAND);
    OLED_SendCommand(0x40,COMMAND);
    
    OLED_SendCommand(OLED_RESUME_RAM,COMMAND);
    OLED_SendCommand(OLED_SET_NORMAL_DISPLAY,COMMAND);
    OLED_SendCommand(OLED_DISPLAY_ON,COMMAND);
    
    OLED_SendCommand(OLED_CHARGE_PUMP,COMMAND);
    OLED_SendCommand(0x14,COMMAND);
    
    OLED_ClearDisplay();   
}

//page range=0-7,pos range=0-119
void OLED_SetPageColumn(byte page,byte pos){
    OLED_SendCommand((OLED_SET_PAGE_START | page),COMMAND);
    byte lower_start_column = OLED_SET_LOWER_COLUMN | (0x0F & pos);
    OLED_SendCommand(lower_start_column,COMMAND); 
    byte upper_start_column = OLED_SET_HIGHER_COLUMN | ((0xF0 & pos)>>4);
    OLED_SendCommand(upper_start_column,COMMAND);
}

void OLED_ClearDisplay(void){
    for(byte j=0;j<8;j++){
        OLED_SetPageColumn(j,0);
        for(int i=0;i<128;i++){
            OLED_SendCommand(0x00,DATA);
        }
    }
}

void OLED_WriteChar(CHAR ch){
    for(byte i=0;i<8;i++){
        OLED_SendCommand(chars[ch][i],DATA);
    }
}

void OLED_WriteNum(byte num){
    for(byte i=0;i<8;i++){
        OLED_SendCommand(numbers[num][i],DATA);
    }
}

void OLED_WriteString(CHAR *st,int st_size,byte line,byte pos){
    if(pos > 119)
        pos = 0; //We start from the beginning
    OLED_POSITION = pos;
    OLED_SetPageColumn(line,pos);
    for(int i=0;i<st_size;i++){
        OLED_WriteChar(st[i]);
        OLED_POSITION = OLED_POSITION + 8;
    }      
}

void OLED_WriteNumber(byte *st,int st_size,byte line,byte pos){
    if(pos > 119)
        pos = 0; //We start from the beginning
    OLED_POSITION = pos;
    OLED_SetPageColumn(line,pos);
    for(int i=0;i<st_size;i++){
        OLED_WriteNum(st[i]);
        OLED_POSITION = OLED_POSITION + 8;
    } 
}