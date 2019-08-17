/*
 * File:   main.c
 * Author: Alfred Chua
 * Co-author: Ken Sadiwa
 * 
 * Created on May 18, 2018, 10:31 AM
 * 
 * Title: Temperature Monitoring Watch
 * 
 * Description:
 * Temeperature is sampled every minute. The temperature alarm can only 
 * 
 * Instructions:
 * '*' is used to go back to the previous screen; except for setting the date
 *      and time.
 * '*' is also used to cancel an ongoing alarm. However, if the alarm is caused
 *      by the temperature, the alarm cannot be cancelled.
 * '#' is used as the O.K. value for most cases
 * 
 * States:
 *  Set date/time:
 *      '1', '3' - switch timeplace left/right
 *      '2', '0' - increment/decrement timeplace
 *      '#'      - finalize values and proceed
 *      '*'      - cancel alarm (if applicable)
 *  Clock (main screen)
 *      '1'      - Stopwatch mode
 *      '2'      - Alarm mode
 *      '3'      - Set date/time
 *      '*'      - cancel alarm (if applicable)
 *  Stopwatch
 *      '#'      - Start/Stop
 *      '0'      - Reset
 *      '*'      - cancel alarm / go back to Clock
 *  Alarm
 *      '2'      - scroll up options
 *      '0'      - scroll down options
 *      '#'      - proceed
 *      '*'      - cancel alarm / go back to Clock
 *  View Alarms
 *      '2','0'  - select alarm
 *      '3'      - go to Delete Alarm
 *      '#'      - enable/disable alarm
 *      '*'      - cancel alarm / go back to Alarm
 *  Set Alarm
 *      '1', '3' - switch timeplace or sched left/right
 *      '2', '0' - increment/decrement timeplace or toggle sched
 *      '#'      - proceed then go back to Alarm
 *      '*'      - cancel alarm / go back to Alarm
 *  Delete Alarm
 *      '1', '3' - select Y/N
 *      '2', '0' - select Y/N
 *      '#'      - proceed and go back to View Alarms
 *      '*'      - cancel alarm / go back to View Alarms
 */

// CONFIG4
#pragma config RTCOSC = LPRC
#pragma config DSWDTEN = OFF

// CONFIG3
#pragma config SOSCSEL = IO

// CONFIG2
#pragma config POSCMOD = NONE
#pragma config IOL1WAY = OFF
#pragma config OSCIOFNC = ON
#pragma config FCKSM = CSECMD
#pragma config FNOSC = FRC
#pragma config PLL96MHZ = OFF
#pragma config IESO = OFF

// CONFIG1
#pragma config FWDTEN = OFF
#pragma config JTAGEN = OFF


#include "xc.h"
#include "lcd_driver.h"

/*
 * Temperature calibration values
 */
// Alarm Temperature
#define DANGER_TEMP 380
// LM35 gain
#define TMP_SCALER 1
// (VDD / 1024) * 10000
#define VpDIV 33

/*------------------------------------------------------------------------------
 * PIN CONFIG
 *==============================================================================
 * PORTA
 *------------------------------------------------------------------------------
 *      0  IN  LM35 (AN0)
 *      2   IN  Keypad ROW1
 *      3   IN  Keypad ROW4
 *------------------------------------------------------------------------------
 * PORTB
 *------------------------------------------------------------------------------
 *      0   OUT LCD D4
 *      1   OUT LCD D5
 *      2   OUT LCD D6
 *      3   OUT RTCC clk
 *      4   OUT LCD D7
 *      5   OUT LCD RS
 *      7   OUT LCD E
 *      8   OUT Keypad COL1
 *      9   OUT Keypad COL2
 *      10  OUT Keypad COL3
 *      14  IN  T2clk (RP14 as T2CK)
 *      15  OUT Buzzer (GPIO)
 *----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 KEYPAD PIN CONFIG
------------------------------------------------------------------------------*/
#define ROW1 PORTAbits.RA2
#define ROW4 PORTAbits.RA3
#define ROWS ( (ROW4<<1) | ROW1 )
#define COL1 LATBbits.LATB8
#define COL2 LATBbits.LATB9
#define COL3 LATBbits.LATB10
#define COLS ( (COL3<<2) | (COL2<<1) | COL1 )

#define BUZZER LATBbits.LATB15

#define DEBOUNCE_MAX 10

#define KEYMAP keymap[keypad.row][keypad.col]
#define FRC 0
#define LPRC 5
#define ALL 0
#define MAX_ALARM 2
#define TEMP 1
#define ALARM 0
#define ALARM_SEC 31000
#define ALARM_HALFSEC 15500

struct date {
    char year[2], month[2], day[2];
    char wday;
} date;

struct time {
    char hour[2], minute[2], sec[2];
} time;

struct alarm {
    struct time time;
    char sched;
    char valid;
    char enable;
} alarms[MAX_ALARM];

struct __keypad {
    unsigned char isPressed;
    unsigned char row;
    unsigned char col;
    unsigned char buf;
    unsigned int  counter;
};

struct __keypad keypad = {0,0,0,0b11,0};

unsigned char state = 0;
unsigned char update_screen = 0;
char keypress;
unsigned int temperature = 0;
char buzzer = 0;
char alarm_source = 0;
char buzz_count = 0;
char tick = 0;

char keymap[4][3] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
};

char daymap[7][4] = {
    "Sun",
    "Mon",
    "Tue",
    "Wed",
    "Thu",
    "Fri",
    "Sat"
};

/*
 * Custom chars
 * 0 - down arrow
 * 1 - up arrow
 * 2 - Th
 * 3 - Su
 * 4 - empty box
 * 5 - crossed box
 */

char schedmap[7] = {3, 'M', 'T', 'W', 2, 'F', 'S'};

char max_day[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define inc_wrap(start,inc,lower,upper)                                         \
    do {                                                                        \
        if (inc > 0) {                                                          \
            if (start == upper) {                                               \
                start = lower;                                                  \
            } else {                                                            \
                start += inc;                                                   \
                if (start > upper) {                                            \
                    start = upper;                                              \
                }                                                               \
            }                                                                   \
        } else {                                                                \
            if (start == lower) {                                               \
                start = upper;                                                  \
            } else {                                                            \
                start += inc;                                                   \
                if (start < lower) {                                            \
                    start = lower;                                              \
                }                                                               \
            }                                                                   \
        }                                                                       \
    } while(0)


inline void clk_switch(char osc) {
    if (osc == FRC) {
        CLKDIVbits.RCDIV = 0;
        /*
        if (OSCCONbits.COSC != FRC) {
            asm volatile("disi #12");
            __builtin_write_OSCCONH(FRC);
            __builtin_write_OSCCONL(OSCCON | 0x01);            
        }
        */
    } else {
        CLKDIVbits.RCDIV = 7;
        /*
        if (OSCCONbits.COSC != LPRC) {
            asm volatile("disi #12");
            __builtin_write_OSCCONH(LPRC);
            __builtin_write_OSCCONL(OSCCON | 0x01);            
        }
        */
    }
}

inline void RTCC_WRLOCK(void) {
    asm volatile("push w7");
    asm volatile("push w8");
    asm volatile("disi #5");
    asm volatile("mov #0x55, w7");
    asm volatile("mov w7, _NVMKEY");
    asm volatile("mov #0xAA, w8");
    asm volatile("mov w8, _NVMKEY");
    asm volatile("bclr _RCFGCAL, #13"); //clr the RTCWREN bit
    asm volatile("pop w8");
    asm volatile("pop w7");
}

inline void RTCC_WRUNLOCK(void) {
    asm volatile("push w7");
    asm volatile("push w8");
    asm volatile("disi #5");
    asm volatile("mov #0x55, w7");
    asm volatile("mov w7, _NVMKEY");
    asm volatile("mov #0xAA, w8");
    asm volatile("mov w8, _NVMKEY");
    asm volatile("bset _RCFGCAL, #13"); //set the RTCWREN bit
    asm volatile("pop w8");
    asm volatile("pop w7");
}

void RP_init() {
    __builtin_write_OSCCONL(OSCCON & 0xBF);
        RPINR3bits.T2CKR = 14;
    __builtin_write_OSCCONL(OSCCON | 0x40);
    
}

void timer_init() {
    T2CONbits.TCKPS = 0;
    T2CONbits.TCS = 1;
    
    IEC0bits.T2IE = 1;
    IFS0bits.T2IF = 0;

    T4CONbits.TCKPS = 0;
    T4CONbits.TCS = 0;
    PR4 = 40000;
    
    IEC1bits.T4IE = 1;
    IFS1bits.T4IF = 0;
}

void RTCC_init() {
    RCFGCALbits.RTCOE = 1;
    
    PADCFG1bits.RTSECSEL = 0b10;
    
    ALCFGRPTbits.ALRMEN = 0;
    ALCFGRPTbits.CHIME = 1;
    ALCFGRPTbits.AMASK = 0x3;
    ALCFGRPTbits.ARPT = 0;
    ALCFGRPTbits.ALRMPTR = 0;
    ALRMVAL = 0;
    
    IEC3bits.RTCIE = 1;
    IFS3bits.RTCIF = 0;
}

void ADC_init(){            //setup ADC configuration bits and TRIS
    //setup ADC configuration bits and TRISB
    AD1CON1bits.ADON = 0;       // ADC OFF
    AD1CON1bits.ADSIDL = 1;     // Continue in idle mode
    AD1CON1bits.FORM = 0b00;    // int format
    AD1CON1bits.SSRC = 0b111;   // auto-convert
    AD1CON1bits.ASAM = 1;       // auto-sample
    AD1CON1bits.SAMP = 0;       // 
    AD1CON1bits.DONE = 0;       // 
    
    AD1CON2bits.CSCNA = 0;
    AD1CON2bits.SMPI = 0;      // interrupt after 16 conversions
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.ALTS = 0;
    
    AD1CON3bits.ADRC = 0;       // use system clk
    AD1CON3bits.SAMC = 1;       // ASAM = 1  TAD
    AD1CON3bits.ADCS = 1;       // ADCS = 2*Tcy
    
    AD1CHS  = 0x0000;           // use AN0 (RA0)
    
    AD1PCFG = 0xFFFE;           // use RA0 as AN0
    
    AD1CSSL = 0x0000;           // no scan        
    
    // ADC interrupt enable
    IEC0bits.AD1IE = 0;
    // Clear ADC interrupt flag
    IFS0bits.AD1IF = 0;

    /*
    AD1PCFG = 0xfffd;
    
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0b111;
    AD1CON1bits.ASAM = 1;
    AD1CON1bits.SAMP = 0;
    AD1CON1bits.DONE = 0;
    
    AD1CON2 = 0x0000;
    
    AD1CON3bits.ADRC = 0;
    AD1CON3bits.SAMC = 1;
    AD1CON3bits.ADCS = 1;
    
    AD1CHS = 0x0000;
    AD1CSSL = 0x0000;
    
    IEC0bits.AD1IE = 1;
    IFS0bits.AD1IF = 0;
    */ 
}

void disable_idle_modules() {
    // Timers
    T1CONbits.TSIDL = 1;
    //T2CONbits.TSIDL = 1;
    T3CONbits.TSIDL = 1;
    //T4CONbits.TSIDL = 1;
    T5CONbits.TSIDL = 1;
    
    // IC
    IC1CON1bits.ICSIDL = 1;
    IC2CON1bits.ICSIDL = 1;
    IC3CON1bits.ICSIDL = 1;
    IC4CON1bits.ICSIDL = 1;
    IC5CON1bits.ICSIDL = 1;
    
    // OC
    OC1CON1bits.OCSIDL = 1;
    OC2CON1bits.OCSIDL = 1;
    OC3CON1bits.OCSIDL = 1;
    OC4CON1bits.OCSIDL = 1;
    OC5CON1bits.OCSIDL = 1;
    
    // SPI
    SPI1STATbits.SPISIDL = 1;
    SPI2STATbits.SPISIDL = 1;
    
    // I2C
    I2C1CONbits.I2CSIDL = 1;
    I2C2CONbits.I2CSIDL = 1;
    
    // UART
    U1MODEbits.USIDL = 1;
    U2MODEbits.USIDL = 1;
    
    // PMP
    PMCONbits.PSIDL = 1;
    
    // CRC
    CRCCON1bits.CSIDL = 1;
    
    // ADC
    AD1CON1bits.ADSIDL = 1;
    
    // Comparators
    CMSTATbits.CMIDL = 1;
    
    // CT
    CTMUCONbits.CTMUSIDL = 1;
}

void keypad_pullup() {
    // RA1 pullup, CN int
    CNPU2bits.CN29PUE = 1;
    CNEN2bits.CN29IE = 1;
    // RA2 pullup, CN_int
    CNPU2bits.CN30PUE = 1;
    CNEN2bits.CN30IE = 1;

    // interrupt enable
    IEC1bits.CNIE = 1;
    IFS1bits.CNIF = 0;
    
    COL1 = 0;
    COL2 = 0;
    COL3 = 0;    
}

void init() {
    char c[][7] = {
    {
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b10101,
        0b01110,
        0b00100
    },
    {
        0b00100,
        0b01110,
        0b10101,
        0b00100,
        0b00100,
        0b00100,
        0b00100
    },
    {
        0b11100,
        0b01000,
        0b01000,
        0b01100,
        0b00100,
        0b00111,
        0b00101
    },
    {
        0b01100,
        0b10000,
        0b01000,
        0b00100,
        0b11101,
        0b00101,
        0b00111
    },
    {
        0b11111,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b11111
    },
    {
        0b11111,
        0b11001,
        0b10101,
        0b10101,
        0b10101,
        0b10011,
        0b11111
    }
    };
    
    disable_idle_modules();
    TRISA = 0x000F;
    TRISB = 0x4000;
    LATA = 0x000F;
    LATB = 0x0000;
    BUZZER = 0;
    ADC_init();
    keypad_pullup();
    RTCC_init();
    RP_init();
    timer_init();
    lcd_init();
    
    create_custom_char(0, c[0]);
    create_custom_char(1, c[1]); 
    create_custom_char(2, c[2]); 
    create_custom_char(3, c[3]); 
    create_custom_char(4, c[4]);
    create_custom_char(5, c[5]);
    
    AD1CON1bits.ADON = 1;
    while(!AD1CON1bits.DONE);
    temperature = ADC1BUF0;
    AD1CON1bits.ADON = 0;
    temperature = temperature*VpDIV;
    temperature = temperature/TMP_SCALER;
    temperature = temperature/10;
    
    RTCC_WRUNLOCK();
    RCFGCALbits.RTCEN = 1;
    RTCC_WRLOCK();
    ALCFGRPTbits.ALRMEN = 1;
    alarms[0].valid = 0;
    alarms[1].valid = 0;
    alarms[2].valid = 0;    
}

void get_dt(char field) {
    unsigned int temp, check;
    switch (field) {
        case 'y':
            RCFGCALbits.RTCPTR = 3;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 3;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 3;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 3;
                check = RTCVAL;        
            }
            date.year[0] = temp >> 4;
            date.year[1] = temp & 0xF;
            break;
        case 'M':
        case 'd':
            RCFGCALbits.RTCPTR = 2;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 2;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 2;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 2;
                check = RTCVAL;                
            }
            date.month[0] = temp >> 12;
            date.month[1] = (temp >> 8) & 0xF;
            date.day[0] = (temp >> 4) & 0xF;
            date.day[1] = temp & 0xF;
            break;
        case 'w':
        case 'h':
            RCFGCALbits.RTCPTR = 1;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 1;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 1;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 1;
                check = RTCVAL;                
            }
            date.wday = temp >> 8;
            time.hour[0] = (temp >> 4) & 0xF;
            time.hour[1] = temp & 0xF;
            break;
        case 'm':
        case 's':
            RCFGCALbits.RTCPTR = 0;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 0;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 0;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 0;
                check = RTCVAL;                
            }
            time.minute[0] = temp >> 12;
            time.minute[1] = (temp >> 8) & 0xF;
            time.sec[0] = (temp >> 4) & 0xF;
            time.sec[1] = temp & 0xF;
            break;
        case ALL:
            // year
            RCFGCALbits.RTCPTR = 3;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 3;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 3;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 3;
                check = RTCVAL;                
            }
            date.year[0] = temp >> 4;
            date.year[1] = temp & 0xF;

            // month, day
            RCFGCALbits.RTCPTR = 2;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 2;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 2;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 2;
                check = RTCVAL;                
            }
            date.month[0] = temp >> 12;
            date.month[1] = (temp >> 8) & 0xF;
            date.day[0] = (temp >> 4) & 0xF;
            date.day[1] = temp & 0xF;
            
            // wday, hour
            RCFGCALbits.RTCPTR = 1;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 1;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 1;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 1;
                check = RTCVAL;                
            }
            date.wday = temp >> 8;
            time.hour[0] = (temp >> 4) & 0xF;
            time.hour[1] = temp & 0xF;
            
            // minute, sec
            RCFGCALbits.RTCPTR = 0;
            temp = RTCVAL;
            RCFGCALbits.RTCPTR = 0;
            check = RTCVAL;
            while (temp != check) {
                RCFGCALbits.RTCPTR = 0;
                temp = RTCVAL;
                RCFGCALbits.RTCPTR = 0;
                check = RTCVAL;                
            }
            time.minute[0] = temp >> 12;
            time.minute[1] = (temp >> 8) & 0xF;
            time.sec[0] = (temp >> 4) & 0xF;
            time.sec[1] = temp & 0xF;
            break;
    }
}

void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void) {
    BUZZER = !BUZZER;
    buzz_count++;
    if (alarm_source == ALARM) {
        if (buzz_count >= 20) {
            buzzer = 0;
            buzz_count = 0;
            BUZZER = 0;
            T2CONbits.TON = 0;
            TMR2 = 0;
        }
    }
    IFS0bits.T2IF = 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void) {
    tick = 1;
    IFS1bits.T4IF = 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _RTCCInterrupt(void) {
    clk_switch(FRC);
    
    AD1CON1bits.ADON = 1;
    while (!AD1CON1bits.DONE);
    temperature = ADC1BUF0;
    AD1CON1bits.ADON = 0;
    // ADC -> adcV*k
    temperature = temperature*VpDIV;
    // adcV*k -> true V*c
    temperature = temperature/TMP_SCALER;
    // true V*c -> temperature*10
    temperature = temperature/10;
    
    if (temperature < DANGER_TEMP) {
        if (buzzer) {
            if (alarm_source = TEMP) {
                T2CONbits.TON = 0;
                BUZZER = 0;
                buzzer = 0;
                TMR2 = 0;
            }
        }
    } else {
        buzzer = 1;
        alarm_source = TEMP;
        // buzz every 0.5s
        PR2 = ALARM_HALFSEC;
        T2CONbits.TON = 1;
    }
    update_screen = 1;
    IFS3bits.RTCIF = 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _CNInterrupt(void){ //handle interrupts in this function
    clk_switch(FRC);
    if (ROWS != 0x3){ //check if any row is pressed
        keypad.counter = DEBOUNCE_MAX;
        //debounce and wait
        while ((ROWS != 0x3) && keypad.counter){
            keypad.counter--;
        }
        if (!keypad.counter) { //does not count as a press if not held for DEBOUNCEMAX
            keypad.buf = ROWS;
            keypad.isPressed = 1;
            COL1 = 1;
            for (keypad.counter==DEBOUNCE_MAX; keypad.counter; keypad.counter--);
            if (ROWS==keypad.buf) {
                COL2 = 1;
                for (keypad.counter==DEBOUNCE_MAX; keypad.counter; keypad.counter--);
                if (ROWS==keypad.buf) {
                    keypad.col = 2;
                } else {
                    keypad.col = 1;
                }
            } else {
                keypad.col = 0;
            }
            
            COL1 = 0;
            COL2 = 0;
            COL3 = 0;
            for (keypad.counter==DEBOUNCE_MAX; keypad.counter; keypad.counter--);
            switch (keypad.buf) {
                case 0b10:
                    keypad.row = 0;
                    break;
                case 0b01:
                    keypad.row = 3;
                    break;
                default:
                    keypad.row = 0;
            }
        }
        else keypad.isPressed = 0;
    }
    
    // Clear IRQ flag
    IFS1bits.CNIF = 0;
}

int set_date() {
    int year, month, day;
    int pos = 0;
    
    get_dt('y');
    get_dt('m');
    
    year = date.year[0]*10 + date.year[1];
    month = date.month[0]*10 + date.month[1];
    if (month == 0) {
        month = 1;
        date.month[1] = 1;
    }
    day = date.day[0]*10 + date.day[1];
    if (day == 0) {
        date.day[1] = 1;
    }
    
    clear_lcd();
    puts_lcd("Set Date 2");
    putc_lcd(1);
    puts_lcd(" 0");
    putc_lcd(0);
    cursor(LINE2,0);
    puts_lcd("20");
    putc_lcd(date.year[0] + '0');
    putc_lcd(date.year[1] + '0');
    putc_lcd('-');
    putc_lcd(date.month[0] + '0');
    putc_lcd(date.month[1] + '0');
    putc_lcd('-');
    putc_lcd(date.day[0] + '0');
    putc_lcd(date.day[1] + '0');
    cursor(LINE2,2);
    
    while(1) {
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '2':
                    switch (pos) {
                        case 0:
                            inc_wrap(year, 10, 0, 99);
                            date.year[0] = year/10;
                            date.year[1] = year%10;
                            putc_lcd(date.year[0] + '0');
                            putc_lcd(date.year[1] + '0');
                            cursor(LINE2,2);
                            break;
                        case 1:
                            inc_wrap(year, 1, 0, 99);
                            date.year[0] = year/10;
                            date.year[1] = year%10;
                            cursor(LINE2,2);
                            putc_lcd(date.year[0] + '0');
                            putc_lcd(date.year[1] + '0');
                            cursor(LINE2,3);
                            break;
                        case 2:
                            inc_wrap(month, 10, 1, 12);
                            date.month[0] = month/10;
                            date.month[1] = month%10;
                            putc_lcd(date.month[0] + '0');
                            putc_lcd(date.month[1] + '0');
                            cursor(LINE2,5);
                            break;
                        case 3:
                            inc_wrap(month, 1, 1, 12);
                            date.month[0] = month/10;
                            date.month[1] = month%10;
                            cursor(LINE2,5);
                            putc_lcd(date.month[0] + '0');
                            putc_lcd(date.month[1] + '0');
                            cursor(LINE2,6);
                            break;
                        case 4:
                            if (month == 2) {
                                if (year%4 == 0) {
                                    inc_wrap(day, 10, 1, 29);
                                } else {
                                    inc_wrap(day, 10, 1, 28);
                                }
                            } else {
                                inc_wrap(day, 10, 1, max_day[month]);
                            }
                            cursor(LINE2,8);
                            date.day[0] = day/10;
                            date.day[1] = day%10;
                            putc_lcd(date.day[0] + '0');
                            putc_lcd(date.day[1] + '0');
                            cursor(LINE2,8);
                            break;
                        case 5:
                            if (month == 2) {
                                if (year%4 == 0) {
                                    inc_wrap(day, 1, 1, 29);
                                } else {
                                    inc_wrap(day, 1, 1, 28);
                                }
                            } else {
                                inc_wrap(day, 1, 1, max_day[month]);
                            }
                            date.day[0] = day/10;
                            date.day[1] = day%10;
                            cursor(LINE2,8);
                            putc_lcd(date.day[0] + '0');
                            putc_lcd(date.day[1] + '0');
                            cursor(LINE2,9);
                            break;
                    }
                    break;
                case '0':
                    switch (pos) {
                        case 0:
                            inc_wrap(year, -10, 1, 99);
                            date.year[0] = year/10;
                            date.year[1] = year%10;
                            putc_lcd(date.year[0] + '0');
                            putc_lcd(date.year[1] + '0');
                            cursor(LINE2,2);
                            break;
                        case 1:
                            inc_wrap(year, -1, 1, 99);
                            date.year[0] = year/10;
                            date.year[1] = year%10;
                            cursor(LINE2,2);
                            putc_lcd(date.year[0] + '0');
                            putc_lcd(date.year[1] + '0');
                            cursor(LINE2,3);
                            break;
                        case 2:
                            inc_wrap(month, -10, 1, 12);
                            date.month[0] = month/10;
                            date.month[1] = month%10;
                            putc_lcd(date.month[0] + '0');
                            putc_lcd(date.month[1] + '0');
                            cursor(LINE2,5);
                            break;
                        case 3:
                            inc_wrap(month, -1, 1, 12);
                            date.month[0] = month/10;
                            date.month[1] = month%10;
                            cursor(LINE2,5);
                            putc_lcd(date.month[0] + '0');
                            putc_lcd(date.month[1] + '0');
                            cursor(LINE2,6);
                            break;
                        case 4:
                            if (month == 2) {
                                if (year%4 == 0) {
                                    inc_wrap(day, -10, 1, 29);
                                } else {
                                    inc_wrap(day, -10, 1, 28);
                                }
                            } else {
                                inc_wrap(day, -10, 1, max_day[month]);                                
                            }
                            cursor(LINE2,8);
                            date.day[0] = day/10;
                            date.day[1] = day%10;
                            putc_lcd(date.day[0] + '0');
                            putc_lcd(date.day[1] + '0');
                            cursor(LINE2,8);
                            break;
                        case 5:
                            if (month == 2) {
                                if (year%4 == 0) {
                                    inc_wrap(day, -1, 1, 29);
                                } else {
                                    inc_wrap(day, -1, 1, 28);
                                }
                            } else {
                                inc_wrap(day, -1, 1, max_day[month]);                                
                            }
                            date.day[0] = day/10;
                            date.day[1] = day%10;
                            cursor(LINE2,8);
                            putc_lcd(date.day[0] + '0');
                            putc_lcd(date.day[1] + '0');
                            cursor(LINE2,9);
                            break;
                    }
                    break;
                case '1':
                    if (pos > 0) {
                        pos--;
                        if (pos >= 4) {
                            cursor(LINE2,pos+4);
                        } else if (pos >= 2) {
                            cursor(LINE2,pos+3);
                        } else {
                            cursor(LINE2,pos+2);
                        }
                    }
                    break;
                case '3':
                    if (pos < 5) {
                        pos++;
                        if (pos >= 4) {
                            cursor(LINE2,pos+4);
                        } else if (pos >= 2) {
                            cursor(LINE2,pos+3);
                        } else {
                            cursor(LINE2,pos+2);
                        }
                    }
                    break;
                case '#':
                    RCFGCALbits.RTCPTR = 3;
                    RTCC_WRUNLOCK();
                    while(RCFGCALbits.RTCSYNC);
                    RTCVAL = (date.year[0] << 4) + date.year[1];
                    while(RCFGCALbits.RTCSYNC);
                    RTCVAL = (date.month[0] << 12) + (date.month[1] << 8) + (date.day[0] << 4) + date.day[1];
                    RTCC_WRLOCK();
                    return 0;
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        }
                    }
                    break;
            }
        }
        clk_switch(LPRC);
        Idle();
    }
    return 0;
}

char get_wday(int y, int m, int d) {
    return (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7;
}

int set_time() {
    char hour, minute, sec;
    char pos = 0;
    
    get_dt('h');
    get_dt('m');
    
    hour = time.hour[0]*10 + time.hour[1];
    minute = time.minute[0]*10 + time.minute[1];
    sec = time.sec[0]*10 + time.sec[1];
    
    clear_lcd();
    puts_lcd("Set Time 2");
    putc_lcd(1);
    puts_lcd(" 0");
    putc_lcd(0);
    cursor(LINE2,0);
    putc_lcd(time.hour[0] + '0');
    putc_lcd(time.hour[1] + '0');
    putc_lcd(':');
    putc_lcd(time.minute[0] + '0');
    putc_lcd(time.minute[1] + '0');
    putc_lcd(':');
    putc_lcd(time.sec[0] + '0');
    putc_lcd(time.sec[1] + '0');
    cursor(LINE2,0);
    
    while(1) {
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '2':
                    switch (pos) {
                        case 0:
                            inc_wrap(hour, 10, 0, 23);
                            time.hour[0] = hour/10;
                            time.hour[1] = hour%10;
                            putc_lcd(time.hour[0] + '0');
                            putc_lcd(time.hour[1] + '0');
                            cursor(LINE2,0);
                            break;
                        case 1:
                            inc_wrap(hour, 1, 0, 23);
                            time.hour[0] = hour/10;
                            time.hour[1] = hour%10;
                            cursor(LINE2,0);
                            putc_lcd(time.hour[0] + '0');
                            putc_lcd(time.hour[1] + '0');
                            cursor(LINE2,1);
                            break;
                        case 2:
                            inc_wrap(minute, 10, 0, 59);
                            time.minute[0] = minute/10;
                            time.minute[1] = minute%10;
                            putc_lcd(time.minute[0] + '0');
                            putc_lcd(time.minute[1] + '0');
                            cursor(LINE2,3);
                            break;
                        case 3:
                            inc_wrap(minute, 1, 0, 59);
                            time.minute[0] = minute/10;
                            time.minute[1] = minute%10;
                            cursor(LINE2,3);
                            putc_lcd(time.minute[0] + '0');
                            putc_lcd(time.minute[1] + '0');
                            cursor(LINE2,3);
                            cursor(LINE2,4);
                            break;
                        case 4:
                            inc_wrap(sec, 10, 0, 59);
                            time.sec[0] = sec/10;
                            time.sec[1] = sec%10;
                            putc_lcd(time.sec[0] + '0');
                            putc_lcd(time.sec[1] + '0');
                            cursor(LINE2,6);
                            break;
                        case 5:
                            inc_wrap(sec, 1, 0, 59);
                            time.sec[0] = sec/10;
                            time.sec[1] = sec%10;
                            cursor(LINE2,6);
                            putc_lcd(time.sec[0] + '0');
                            putc_lcd(time.sec[1] + '0');
                            cursor(LINE2,7);
                            break;
                    }
                    break;
                case '0':
                    switch (pos) {
                        case 0:
                            inc_wrap(hour, -10, 0, 23);
                            time.hour[0] = hour/10;
                            time.hour[1] = hour%10;
                            putc_lcd(time.hour[0] + '0');
                            putc_lcd(time.hour[1] + '0');
                            cursor(LINE2,0);
                            break;
                        case 1:
                            inc_wrap(hour, -1, 0, 23);
                            time.hour[0] = hour/10;
                            time.hour[1] = hour%10;
                            cursor(LINE2,0);
                            putc_lcd(time.hour[0] + '0');
                            putc_lcd(time.hour[1] + '0');
                            cursor(LINE2,1);
                            break;
                        case 2:
                            inc_wrap(minute, -10, 0, 59);
                            time.minute[0] = minute/10;
                            time.minute[1] = minute%10;
                            putc_lcd(time.minute[0] + '0');
                            putc_lcd(time.minute[1] + '0');
                            cursor(LINE2,3);
                            break;
                        case 3:
                            inc_wrap(minute, -1, 0, 59);
                            time.minute[0] = minute/10;
                            time.minute[1] = minute%10;
                            cursor(LINE2,3);
                            putc_lcd(time.minute[0] + '0');
                            putc_lcd(time.minute[1] + '0');
                            cursor(LINE2,4);
                            break;
                        case 4:
                            inc_wrap(sec, -10, 0, 59);
                            time.sec[0] = sec/10;
                            time.sec[1] = sec%10;
                            putc_lcd(time.sec[0] + '0');
                            putc_lcd(time.sec[1] + '0');
                            cursor(LINE2,6);
                            break;
                        case 5:
                            inc_wrap(sec, -1, 0, 59);
                            time.sec[0] = sec/10;
                            time.sec[1] = sec%10;
                            cursor(LINE2,6);
                            putc_lcd(time.sec[0] + '0');
                            putc_lcd(time.sec[1] + '0');
                            cursor(LINE2,7);
                            break;
                    }
                    break;
                case '1':
                    if (pos > 0) {
                        pos--;
                        if (pos >= 4) {
                            cursor(LINE2,pos+2);
                        } else if (pos >= 2) {
                            cursor(LINE2,pos+1);
                        } else {
                            cursor(LINE2,pos);
                        }
                    }
                    break;
                case '3':
                    if (pos < 5) {
                        pos++;
                        if (pos >= 4) {
                            cursor(LINE2,pos+2);
                        } else if (pos >= 2) {
                            cursor(LINE2,pos+1);
                        } else {
                            cursor(LINE2,pos);
                        }
                    }
                    break;
                case '#':
                    date.wday = get_wday(2000+date.year[0]*10+date.year[1], date.month[0]*10+date.month[1], date.day[0]*10+date.day[1]);
                    RCFGCALbits.RTCPTR = 1;
                    RTCC_WRUNLOCK();
                    while(RCFGCALbits.RTCSYNC);
                    RTCVAL = (date.wday << 8) + (time.hour[0] << 4) + time.hour[1];
                    while(RCFGCALbits.RTCSYNC);
                    RTCVAL = (time.minute[0] << 12) + (time.minute[1] << 8) + (time.sec[0] << 4) + time.sec[1];
                    RTCC_WRLOCK();
                    return 0;
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        }
                    }
                    break;
            }
        }
        clk_switch(LPRC);
        Idle();
    }
    
    return 0;
}

int clock() {
    int i;
    char what_to_update;
    struct date old_date;
    struct time old_time;
    
    get_dt(ALL);
    
    clear_lcd();
    putc_lcd(date.month[0] + '0');
    putc_lcd(date.month[1] + '0');
    putc_lcd('/');
    putc_lcd(date.day[0] + '0');
    putc_lcd(date.day[1] + '0');
    putc_lcd('/');
    putc_lcd('2');
    putc_lcd('0');
    putc_lcd(date.year[0] + '0');
    putc_lcd(date.year[1] + '0');
    puts_lcd("   ");
    puts_lcd(daymap[(int) date.wday]);
    
    cursor(LINE2,0);
    putc_lcd(time.hour[0] + '0');
    putc_lcd(time.hour[1] + '0');
    putc_lcd(':');
    putc_lcd(time.minute[0] + '0');
    putc_lcd(time.minute[1] + '0');
    puts_lcd("     ");
    if (temperature < 100) {
        putc_lcd(' ');
    }
    if (temperature >= 1000) {
        puts_lcd("99");
    } else {
        putd_lcd(temperature/10);
    }
    putc_lcd('.');
    putc_lcd((temperature%10) + '0');
    putc_lcd(0xDF);
    putc_lcd('C');
     
    while (1) {
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '1':
                    return 2;
                    break;
                case '2':
                    return 3;
                    break;
                case '3':
                    return 0;
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        }
                    }
                    break;
            }
        }
        if (update_screen) {
            update_screen = 0;            
            old_date = date;
            old_time = time;
            get_dt(ALL);
            
            if (old_date.year[0] != date.year[0]) {
                what_to_update = 9;
            } else if (old_date.year[1] != date.year[1]) {
                what_to_update = 8;
            } else if (old_date.month[0] != date.month[0]) {
                what_to_update = 7;
            } else if (old_date.month[1] != date.month[1]) {
                what_to_update = 6;
            } else if (old_date.day[0] != date.day[0]) {
                what_to_update = 5;
            } else if (old_date.day[1] != date.day[1]) {
                what_to_update = 4;
            } else if (old_time.hour[0] != time.hour[0]) {
                what_to_update = 3;
            } else if (old_time.hour[1] != time.hour[1]) {
                what_to_update = 2;
            } else if (old_time.minute[0] != time.minute[0]) {
                what_to_update = 1;
            } else {
                what_to_update = 0;
            }
            
            cursor(LINE2,10);
            if (temperature < 100) {
                putc_lcd(' ');
            }
            putd_lcd(temperature/10);
            cursor(LINE2,13);
            putc_lcd((temperature%10) + '0');           
            
            /* FALLTHROUGH in all cases intended*/
            switch (what_to_update) {
                case 9:
                    cursor(LINE1,8);
                    putc_lcd(date.year[0] + '0');
                    putc_lcd(date.year[1] + '0');
                case 8:
                    if (what_to_update <= 8) {
                        cursor(LINE1,9);
                        putc_lcd(date.year[1] + '0');
                    }
                case 7:
                    cursor(LINE1,0);
                    putc_lcd(date.month[0] + '0');
                    putc_lcd(date.month[1] + '0');
                case 6:
                    if (what_to_update <= 6) {
                        cursor(LINE1,1);
                        putc_lcd(date.month[1] + '0');
                    }
                case 5:
                    cursor(LINE1,3);
                    putc_lcd(date.day[0] + '0');
                    putc_lcd(date.day[1] + '0');
                case 4:
                    if (what_to_update <= 4) {
                        cursor(LINE1,4);
                        putc_lcd(date.day[1] + '0');
                    }
                    cursor(LINE1,13);
                    puts_lcd(daymap[(int) date.wday]);
                case 3:
                    cursor(LINE2,0);
                    putc_lcd(time.hour[0] + '0');
                    putc_lcd(time.hour[1] + '0');
                case 2:
                    if (what_to_update <= 2) {
                        cursor(LINE2,1);
                        putc_lcd(time.hour[1] + '0');
                    }
                case 1:
                    cursor(LINE2,3);
                    putc_lcd(time.minute[0] + '0');
                    putc_lcd(time.minute[1] + '0');
                default:
                    if (what_to_update == 0) {
                        cursor(LINE2,4);
                        putc_lcd(time.minute[1] + '0');
                    }
                    for (i=0; i<MAX_ALARM; i++) {
                        if (alarms[i].valid) {
                            if (alarms[i].enable) {
                                if (alarms[i].time.minute[1] == time.minute[1]) {
                                    if (alarms[i].time.minute[0] == time.minute[0]) {
                                        if (alarms[i].time.hour[1] == time.hour[1]) {
                                            if (alarms[i].time.hour[0] == time.hour[0]) {
                                                if (!alarms[i].sched) {
                                                    if (!buzzer) {
                                                        buzzer = 1;
                                                        alarm_source = ALARM;
                                                        PR2 = ALARM_SEC;
                                                        BUZZER = 1;
                                                        T2CONbits.TON = 1;
                                                    }
                                                    alarms[i].enable = 0;                                                        
                                                } else if ( alarms[i].sched & (1 << date.wday) ) {
                                                    if (!buzzer) {                                                        
                                                        buzzer = 1;
                                                        alarm_source = ALARM;
                                                        PR2 = ALARM_SEC;
                                                        BUZZER = 1;
                                                        T2CONbits.TON = 1;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }                                
                            }
                        }
                    }
            }
        }
        
        if (!keypad.isPressed && !update_screen) {
            if (OSCCONbits.COSC == FRC) {
                clk_switch(LPRC);
            }
            Idle();
        }
    }
    return 0;
}

#define NCHOICE 2-1
int pick_option() {
    int choice = 0;
    char menu_text[NCHOICE+1][16] = {
        "View Alarms",
        "Set Alarm"
    };
    
    clear_lcd();
    send_lcd(0x0C, INST);
    puts_lcd("Alarms");
    cursor(LINE2,0);
    puts_lcd("> ");
    puts_lcd(menu_text[0]);
    while(1) {
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '2':
                    if (choice < NCHOICE) {
                        choice++;
                    } else {
                        choice = 0;
                    }
                    cursor(LINE2,2);
                    puts_lcd("              ");
                    cursor(LINE2,2);
                    puts_lcd(menu_text[choice]);
                    break;
                case '0':
                    if (choice > 0) {
                        choice--;
                    } else {
                        choice = NCHOICE;
                    }
                    cursor(LINE2,2);
                    puts_lcd("              ");
                    cursor(LINE2,2);
                    puts_lcd(menu_text[choice]);
                    break;
                case '#':
                    return choice + 1;
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        } else {
                            return -1;
                        }
                    } else {
                        return -1;
                    }
                    break;
            }
        }
        clk_switch(LPRC);
        Idle();
    }
    
    return 0;
}

int view_alarms() {
    int i, j;
    char alarm_exists = 0;
    int printed = 0;
    char index[2] = {-1, -1};
    int select = 0;
    char pos = 0;
    
    clear_lcd();
    send_lcd(0x0F, INST);
    for (i=0; i<MAX_ALARM; i++) {
        if (alarms[i].valid) {
            alarm_exists = 1;
        }
    }
    if (alarm_exists) {
        for (i=0; i<MAX_ALARM; i++) {
            if (printed == 2) break;
            if (alarms[i].valid) {
                index[printed] = i;
                printed++;
                if (alarms[i].enable) {
                    putc_lcd(5);
                } else {
                    putc_lcd(4);
                }
                putc_lcd(' ');
                putc_lcd(alarms[i].time.hour[0] + '0');
                putc_lcd(alarms[i].time.hour[1] + '0');
                putc_lcd(':');
                putc_lcd(alarms[i].time.minute[0] + '0');
                putc_lcd(alarms[i].time.minute[1] + '0');
                putc_lcd(' ');
                
                for (j=0; j<7; j++) {
                    if ( alarms[i].sched & (1 << j) ) {
                        putc_lcd(schedmap[j]);
                    } else {
                        putc_lcd(' ');
                    }
                }
                if (printed) {
                    cursor(LINE2,0);
                }
            }
        }
        select = index[0];
        cursor(LINE1,0);
        while(1) {
            if (keypad.isPressed) {
                keypad.isPressed = 0;
                keypress = KEYMAP;
                switch (keypress) {
                    case '2':
                        pos = 0;
                        select = index[0];
                        cursor(LINE1,0);
                        break;
                    case '0':
                        if (printed == 2) {
                            pos = 1;
                            select = index[1];
                            cursor(LINE2,0);
                        }
                        break;
                    case '3':
                        return select;
                        break;
                    case '#':
                        alarms[select].enable = !alarms[select].enable;
                        if (alarms[select].enable) {
                            putc_lcd(5);
                        } else {
                            putc_lcd(4);
                        }
                        if (pos == 0) {
                            cursor(LINE1,0);
                        } else {
                            cursor(LINE2,0);
                        }
                        break;
                    case '*':
                        if (buzzer) {
                            if (alarm_source == ALARM) {
                                T2CONbits.TON = 0;
                                BUZZER = 0;
                                buzzer = 0;
                                buzz_count = 0;
                                TMR2 = 0;
                            } else {
                                return -1;
                            }
                        } else {
                            return -1;
                        }
                        break;
                }
            }
            clk_switch(LPRC);
            Idle();
        }
    } else {
        puts_lcd("No alarms set");
        while(1) {
            if (keypad.isPressed) {
                keypad.isPressed = 0;
                keypress = KEYMAP;
                if (keypress == '*') {
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        } else {
                            return -1;
                        }
                    } else {
                        return -1;
                    }
                }
            }
            clk_switch(LPRC);
            Idle();
        }
    }
    
    return -1;
}

int create_alarm() {
    char hour, minute;
    int i;
    int alarm_count = 0;
    char pos = 0;
    int free_index = 0;
    int mask = 1;
    
    clear_lcd();
    send_lcd(0x0F, INST);
    
    for (i=MAX_ALARM-1; i>=0; i--) {
        if (alarms[i].valid) {
            alarm_count++;
        } else {
            free_index = i;
        }
    }
    if (alarm_count == MAX_ALARM) {
        puts_lcd("No space");
        cursor(LINE2,0);
        puts_lcd("avaliable");
        while(1) {
            if (keypad.isPressed) {
                keypad.isPressed = 0;
                keypress = KEYMAP;
                if (keypress == '*') {
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        } else {
                            return 0;
                        }
                    } else {
                        return 0;
                    }
                }
            }
            clk_switch(LPRC);
            Idle();
        }
    }
    
    puts_lcd("Set Time Sched");
    cursor(LINE2,0);
    puts_lcd("00:00  |       |");
    cursor(LINE2,0);
    
    hour = 0;
    minute = 0;
    alarms[free_index].sched = 0;
    while(1) {
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '2':
                    switch (pos) {
                        case 0:
                            inc_wrap(hour, 10, 0, 23);
                            alarms[free_index].time.hour[0] = hour/10;
                            alarms[free_index].time.hour[1] = hour%10;
                            putc_lcd(alarms[free_index].time.hour[0] + '0');
                            putc_lcd(alarms[free_index].time.hour[1] + '0');
                            cursor(LINE2,0);
                            break;
                        case 1:
                            inc_wrap(hour, 1, 0, 23);
                            alarms[free_index].time.hour[0] = hour/10;
                            alarms[free_index].time.hour[1] = hour%10;
                            cursor(LINE2,0);
                            putc_lcd(alarms[free_index].time.hour[0] + '0');
                            putc_lcd(alarms[free_index].time.hour[1] + '0');
                            cursor(LINE2,1);
                            break;
                        case 2:
                            inc_wrap(minute, 10, 0, 59);
                            alarms[free_index].time.minute[0] = minute/10;
                            alarms[free_index].time.minute[1] = minute%10;
                            putc_lcd(alarms[free_index].time.minute[0] + '0');
                            putc_lcd(alarms[free_index].time.minute[1] + '0');
                            cursor(LINE2,3);
                            break;
                        case 3:
                            inc_wrap(minute, 1, 0, 59);
                            alarms[free_index].time.minute[0] = minute/10;
                            alarms[free_index].time.minute[1] = minute%10;
                            cursor(LINE2,3);
                            putc_lcd(alarms[free_index].time.minute[0] + '0');
                            putc_lcd(alarms[free_index].time.minute[1] + '0');
                            cursor(LINE2,4);
                            break;
                        case 4:
                        case 5:
                        case 6:
                        case 7:
                        case 8:
                        case 9:
                        case 10:
                            if (alarms[free_index].sched & mask) {
                                alarms[free_index].sched &= ~mask;
                                putc_lcd(' ');
                                cursor(LINE2,pos+4);
                            } else {
                                alarms[free_index].sched |= mask;
                                putc_lcd(schedmap[pos-4]);
                                cursor(LINE2,pos+4);
                            }
                            break;
                    }
                    break;
                case '0':
                    switch (pos) {
                        case 0:
                            inc_wrap(hour, -10, 0, 59);
                            alarms[free_index].time.hour[0] = hour/10;
                            alarms[free_index].time.hour[1] = hour%10;
                            putc_lcd(alarms[free_index].time.hour[0] + '0');
                            putc_lcd(alarms[free_index].time.hour[1] + '0');
                            cursor(LINE2,0);
                            break;
                        case 1:
                            inc_wrap(hour, -1, 0, 59);
                            alarms[free_index].time.hour[0] = hour/10;
                            alarms[free_index].time.hour[1] = hour%10;
                            cursor(LINE2,0);
                            putc_lcd(alarms[free_index].time.hour[0] + '0');
                            putc_lcd(alarms[free_index].time.hour[1] + '0');
                            cursor(LINE2,1);
                            break;
                        case 2:
                            inc_wrap(minute, -10, 0, 59);
                            alarms[free_index].time.minute[0] = minute/10;
                            alarms[free_index].time.minute[1] = minute%10;
                            putc_lcd(alarms[free_index].time.minute[0] + '0');
                            putc_lcd(alarms[free_index].time.minute[1] + '0');
                            cursor(LINE2,3);
                            break;
                        case 3:
                            inc_wrap(minute, -1, 0, 59);
                            alarms[free_index].time.minute[0] = minute/10;
                            alarms[free_index].time.minute[1] = minute%10;
                            cursor(LINE2,3);
                            putc_lcd(alarms[free_index].time.minute[0] + '0');
                            putc_lcd(alarms[free_index].time.minute[1] + '0');
                            cursor(LINE2,4);
                            break;
                        case 4:
                        case 5:
                        case 6:
                        case 7:
                        case 8:
                        case 9:
                        case 10:
                            if (alarms[free_index].sched & mask) {
                                alarms[free_index].sched &= ~mask;
                                putc_lcd(' ');
                                cursor(LINE2,pos+4);
                            } else {
                                alarms[free_index].sched |= mask;
                                putc_lcd(schedmap[pos-4]);
                                cursor(LINE2,pos+4);
                            }
                            break;
                    }
                    break;
                case '1':
                    if (pos > 0) {
                        pos--;
                        if (pos >= 4) {
                            mask = (1 << (pos-4));
                            cursor(LINE2,pos+4);
                        } else if (pos >= 2) {
                            cursor(LINE2,pos+1);
                        } else {
                            cursor(LINE2,pos);
                        }
                    }
                    break;
                case '3':
                    if (pos < 10) {
                        pos++;
                        if (pos >= 4) {
                            mask = (1 << (pos-4));
                            cursor(LINE2,pos+4);
                        } else if (pos >= 2) {
                            cursor(LINE2,pos+1);
                        } else {
                            cursor(LINE2,pos);
                        }
                    }
                    break;
                case '#':
                    alarms[free_index].valid = 1;
                    alarms[free_index].enable = 1;
                    return 0;
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        } else {
                            return 0;
                        }
                    } else {
                        return 0;
                    }
                    break;
            }
        }
        clk_switch(LPRC);
        Idle();
    }
    return 0;
}

int delete_alarm(int alarm_index) {
    char pos = 0;
    
    clear_lcd();
    puts_lcd("Delete alarm?");
    cursor(LINE2,0);
    puts_lcd("N / Y");
    cursor(LINE2,0);
    
    while(1) {
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '2':
                case '3':
                    if (pos != 1) {
                        pos = 1;
                        cursor(LINE2,4);
                    }
                    break;
                case '0':
                case '1':
                    if (pos != 0) {
                        pos = 0;
                        cursor(LINE2,0);
                    }
                    break;
                case '#':
                    if (pos == 1) {
                        alarms[alarm_index].valid = 0;
                    }
                    return 1;
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        } else {
                            return 1;
                        }
                    } else {
                        return 1;
                    }
                    break;
            }
        }
    }
    return 1;
}

void alarm_controller() {
    char menu = 0;
    int index = 0;
    while(menu >= 0) {
        switch (menu) {
            case 0:
                menu = pick_option();
                break;
            case 1:
                index = view_alarms();
                if (index < 0) {
                    menu = 0;
                } else {
                    menu = 3;
                }
                break;
            case 2:
                menu = create_alarm();
                break;
            case 3:
                menu = delete_alarm(index);
                break;
        }
    }
}

int stopwatch() {
    char h = 0;
    char m = 0;
    char s = 0;
    char ms = 0;
    char start = 0;
    
    clear_lcd();
    puts_lcd("00s 00");
    cursor(LINE2,0);
    puts_lcd("#:START 0:RESET");
    while(1) {
        if (tick) {
            tick = 0;
            if (ms < 99) {
                ms++;
                if (h) {
                    cursor(LINE1,12);
                } else if (m) {
                    cursor(LINE1,8);
                } else {
                    cursor(LINE1,4);
                }
                putc_lcd((ms/10) + '0');
                putc_lcd((ms%10) + '0');
            } else {
                ms = 0;
                if (s < 59) {
                    s++;
                    if (h) {
                        cursor(LINE1,8);
                    } else if (m) {
                        cursor(LINE1,4);
                    } else {
                        cursor(LINE1,0);
                    }
                    putc_lcd((s/10) + '0');
                    putc_lcd((s%10) + '0');
                    puts_lcd("s 00");
                } else {
                    s = 0;
                    if (m < 59) {
                        m++;
                        if (h) {
                            cursor(LINE1,4);
                        } else {
                            cursor(LINE1,0);
                        }
                        putc_lcd((m/10) + '0');
                        putc_lcd((m%10) + '0');
                        puts_lcd("m 00s 00");
                    } else {
                        m = 0;
                        inc_wrap(h, 1, 0, 99);
                        cursor(LINE1,0);
                        putc_lcd((h/10) + '0');
                        putc_lcd((h%10) + '0');
                        puts_lcd("h ");
                        putc_lcd((m/10) + '0');
                        putc_lcd((m%10) + '0');
                        puts_lcd("m ");
                        putc_lcd((s/10) + '0');
                        putc_lcd((s%10) + '0');
                        puts_lcd("s ");
                        putc_lcd((ms/10) + '0');
                        putc_lcd((ms%10) + '0');
                    }
                }
            }
        }
        if (keypad.isPressed) {
            keypad.isPressed = 0;
            keypress = KEYMAP;
            switch (keypress) {
                case '#':
                    if (start) {
                        T4CONbits.TON = 0;
                        start = 0;
                        TMR4--;
                        cursor(LINE2,2);
                        puts_lcd("START");
                    } else {
                        T4CONbits.TON = 1;
                        start = 1;
                        cursor(LINE2,2);
                        puts_lcd("STOP ");
                    }
                    break;
                case '0':
                    T4CONbits.TON = 0;
                    TMR4 = 0;
                    h = 0;
                    m = 0;
                    s = 0;
                    ms = 0;
                    tick = 0;
                    cursor(LINE1,0);
                    puts_lcd("00s 00          ");
                    break;
                case '*':
                    if (buzzer) {
                        if (alarm_source == ALARM) {
                            T2CONbits.TON = 0;
                            BUZZER = 0;
                            buzzer = 0;
                            buzz_count = 0;
                            TMR2 = 0;
                        } else {
                            return 0;
                        }
                    } else {
                        T4CONbits.TON = 0;
                        TMR4 = 0;
                        tick = 0;
                        return 0;
                    }
                    break;
            }
        }
        Idle();
    }
    
    return 0;
}

int main(void) {
    init();
    while(1) {
        switch (state) {
            case 0:
                send_lcd(0x0F, INST);
                set_date();
                set_time();
                state = 1;
                break;
            case 1:
                send_lcd(0x0C, INST);
                state = clock();
                clk_switch(FRC);
                break;
            case 2:
                send_lcd(0x0C, INST);
                stopwatch();
                state = 1;
                break;
            case 3:
                alarm_controller();
                state = 1;
                break;
        }
    }
    return 0;
}
