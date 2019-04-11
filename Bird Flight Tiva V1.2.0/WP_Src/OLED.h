#ifndef __OLED_H
#define __OLED_H

#define Bit_SET   1
#define Bit_RESET 0
#define SYSCTL_PERIPH_GPIO_OLED SYSCTL_PERIPH_GPIOA
#define GPIO_OLED   GPIO_PORTA_BASE 
#define LCD_DCout(data)		  GPIOPinWrite(GPIO_OLED, GPIO_PIN_2, data ? GPIO_PIN_2 : 0)//DC
#define LCD_RSTout(data)		GPIOPinWrite(GPIO_OLED, GPIO_PIN_3, data ? GPIO_PIN_3 : 0)//DC
#define LCD_SDAout(data)		GPIOPinWrite(GPIO_OLED, GPIO_PIN_4, data ? GPIO_PIN_4 : 0)//DC
#define LCD_SCLout(data)		GPIOPinWrite(GPIO_OLED, GPIO_PIN_5, data ? GPIO_PIN_5 : 0)//DC


#define READ_SDA   GPIOIntTypeGet(GPIO_OLED,GPIO_PIN_4)	//SDA 
#define IIC_SDA_H  GPIOPinWrite(GPIO_OLED,GPIO_PIN_4,GPIO_PIN_4)//SDA 
#define IIC_SDA_L  GPIOPinWrite(GPIO_OLED,GPIO_PIN_4,0)//SDA
#define IIC_SCL_H  GPIOPinWrite(GPIO_OLED,GPIO_PIN_5,GPIO_PIN_5)//SCL
#define IIC_SCL_L  GPIOPinWrite(GPIO_OLED,GPIO_PIN_5,0)//SCL

void LCD_WrDat(unsigned char dat);
void LCD_WrCmd(unsigned char cmd);
void OLED_WrCmd(unsigned char IIC_Data);
void OLED_WrDat(unsigned char IIC_Data);
void LCD_Set_Pos(unsigned char x, unsigned char y);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void LCD_P6x8Char(unsigned char x,unsigned char  y,unsigned char ch);
void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[]);
void write_6_8_number(unsigned char x,unsigned char  y,float number);
void LCD_P8x16Char(unsigned char x,unsigned char  y,unsigned char ch);
void LCD_P8x16Str(unsigned char x,unsigned char  y,unsigned char ch[]);
void write_8_16_number(unsigned char x,unsigned char  y,float number);
void LCD_clear_L(unsigned char x,unsigned char y);
void OLED_Init(void);
void OLED_Init_Fast(void);
void Draw_Logo(void);
void Draw_Logo1(void);
void LCD_CLS(void);
#endif



