#ifndef _lcd16x2_h_
#define _lcd16x2_h_

#include <main.h>

#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define RS_Port GPIOB
#define EN_Port GPIOB
#define D4_Port GPIOB
#define D5_Port GPIOB
#define D6_Port GPIOB
#define D7_Port GPIOB

void LCD_Enable();
void LCD_Send4Bit(unsigned char Data);
void LCD_SendCommand(unsigned char command);
void LCD_Clear();
void LCD_Init();
void LCD_Gotoxy(unsigned char x, unsigned char y);
void LCD_PutChar(unsigned char Data);
void LCD_Puts(char *s);

#endif