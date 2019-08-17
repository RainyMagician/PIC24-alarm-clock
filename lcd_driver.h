/* 
 * File:   lcd_driver.h
 * Author: alfre_ngjcdlk
 *
 * Created on March 20, 2018, 10:47 PM
 */

#ifndef LCD_DRIVER_H
#define	LCD_DRIVER_H

#ifndef FCY
#define FCY 4000000UL
#endif

#include "libpic30.h"
#include "string.h"
    
#define SINGLE_LINE     1
#define DOUBLE_LINE     2
#define LINE_DISPLAY    DOUBLE_LINE

/*------------------------------------------------------------------------------
 PIN CONFIG
------------------------------------------------------------------------------*/
#define D4   LATBbits.LATB0
#define D5   LATBbits.LATB1
#define D6   LATBbits.LATB2
#define D7   LATBbits.LATB4
#define RS   LATBbits.LATB5
#define E    LATBbits.LATB7

/*------------------------------------------------------------------------------
 INIT COMMANDS
------------------------------------------------------------------------------*/
#define LCD_INIT_CMD_1          0x30 /* 8bit Interface                        */
#define LCD_INIT_CMD_2          0x30 /* 8bit Interface                        */
#define LCD_INIT_CMD_3          0x30 /* 8bit Interface                        */
#define LCD_INIT_CMD_4          0x20 /* 4bit Interface                        */
#if LINE_DISPLAY==SINGLE_LINE
#define LCD_INIT_CMD_5          0x20 /* 4bit Interface                        */
#else
#define LCD_INIT_CMD_5          0x28 /* 4bit , 2 Line                         */
#endif
#define LCD_INIT_CMD_6          0x04 /* Display ON                            */
#define LCD_INIT_CMD_7          0x01 /* Clear LCD                             */
#define LCD_INIT_CMD_8          0x06 /* Entry Mode - No shift, Auto Increment */
#define LCD_INIT_CMD_9          0x0F /* Display ON, Cursor ON, Blink ON       */

#define LCD_INIT_DELAY          16
#define LCD_CMD_DELAY           40
#define LINE1                   0x80
#define LINE2                   0xC0
#define CLEAR                   0x01
#define HOME                    0x02

#define INST                    0
#define DATA                    1

void send_lcd(unsigned char byte, unsigned char type);
void lcd_init(void);
#define putc_lcd(c)             send_lcd(c, DATA)
void puts_lcd(char *s);
void putd_lcd(unsigned long d);
#define clear_lcd()             send_lcd(0x01, INST)
#define cursor(line_no, pos)    send_lcd(line_no+pos, INST)

    
void send_lcd(unsigned char byte, unsigned char type) {
    if (type==INST) {
        RS = 0;
    } else if (type==DATA) {
        RS = 1;
    }
    
    D7 = (byte & 0x80) >> 7;
    D6 = (byte & 0x40) >> 6;
    D5 = (byte & 0x20) >> 5;
    D4 = (byte & 0x10) >> 4;
    E = 1;
#if FCY > 2000000UL
    E = 1;
#endif
    E = 0;
   // __delay_us(LCD_CMD_DELAY);
    D7 = (byte & 0x08) >> 3;
    D6 = (byte & 0x04) >> 2;
    D5 = (byte & 0x02) >> 1;
    D4 = (byte & 0x01);
    E = 1;
#if FCY > 2000000UL
    E = 1;
#endif
    E = 0;
    
    if ((byte==0x01 || byte==0x02) && type==INST) {
        __delay_ms(LCD_INIT_DELAY);
    } else {
        __delay_us(LCD_CMD_DELAY);
    }
    
}

void lcd_init(void) {
    int i;
    unsigned char cmd[] = {
        LCD_INIT_CMD_1,
        LCD_INIT_CMD_2,
        LCD_INIT_CMD_3,
        LCD_INIT_CMD_4,
        LCD_INIT_CMD_5,
        LCD_INIT_CMD_6,
        LCD_INIT_CMD_7,
        LCD_INIT_CMD_8,
        LCD_INIT_CMD_9,
        0
    };
    for (i=0; cmd[i]; i++) {
        send_lcd(cmd[i], INST);
        __delay_ms(LCD_INIT_DELAY);
    }
}

static char *itoa(unsigned long d) {
    unsigned long i;
    int ndigit, iter;
    static char a[10];
    for (i=0; i<10; i++) {
        a[i] = 0;
    }
    for (i=d, ndigit=1, iter=0; i>0; i/=10) {
        if (iter) ndigit++;
        else iter = 1;
    }
    do {
        a[ndigit-1] += (d%10) + 0x30;
        ndigit--;
        d/=10;
    } while (ndigit);
    return a;
}

void puts_lcd(char *s) {
    int i;
    for (i=0; s[i]; i++) {
        putc_lcd(s[i]);
    }    
}

void putd_lcd(unsigned long d) {
    puts_lcd(itoa(d));
}

void create_custom_char(char address, char c[7]) {
    send_lcd(0x40 + (address<<3), INST);
    send_lcd(c[0], DATA);
    send_lcd(c[1], DATA);
    send_lcd(c[2], DATA);
    send_lcd(c[3], DATA);
    send_lcd(c[4], DATA);
    send_lcd(c[5], DATA);
    send_lcd(c[6], DATA);
}

#endif	/* LCD_DRIVER_H */

