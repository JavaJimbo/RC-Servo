/*  RC Servo - For PIC 18F1330 using MPLABX
 * For the RC Servo Board
 *
 * 6/24/10  	Got PWM period set to 2 milliseconds for RC servos.
 * 9/3/10   	Cleaned up, eliminated unnecesary variables. Check robot ID# in interrupt routine.
 * 2/17/12  	New version, ASCII protocol.
 * 02-04-2013	Compiles with High Tech - gives boring COFF error.
 * 02-05-2013	Communicates using XBEE at 115200 baud.
 *  		Reads pot, controls duty cycle.
 *   		Works with Animator.
 * 2-7-2013    Got rid of MIDI stuff; simplified board enabling
 * 5-3-2013	Removed decoding, changed baud rate to 19200 to work with Memory board bit banged input.
 *		TODO: Check over interupt routine.
 *  eeprom_write(0,0);
 *  eeprom_read(0);
 *
 * 4-30-15  COMMAND/BOARD ID, SUBCOMMAND, LENGTH, DATA, CRC
 *          Baudrate = 57,600
 * 5-4-15   Reduced UART timeout to 1.7 ms.
 * 5-27-15  This board is #1
 * 
 *          Byte #0: BOARD ID
 *          Byte #1: First device being updated
 *          Byte #2: Number of devices being updated
 *          Byte #3: First data byte
 * 5-28-15  Simplified interrupt routine, moved stuff into processInBuffer().
 *          Copied interrupt and processInBuffer()from DMX Xbee Controller.
 * 7-21-16  One pot sets positions for All six servos.
 * 8-9-16   PWM outputs modified to increase servo range to 180 degrees
 *          PWM duty cycle optimized for Hitec HS-755HB 180 degree servos: 
 *          550 to 2350 microseconds approximately,
 *          and each servo refreshed every 14 milliseconds
 * 8-10-16  Modified AD initialization so FRC isn't used for AD clock.
 * 8-15-16  Continuous back and forth motion, delay at beginning.
 * 9-12-16  Created GitHub project.
 * 9-19-16  Eyeball open/close.
 * 
 */


#define EYELID_OPEN 302
#define EYELID_CLOSED 210

#define CENTER_UPDOWN 0

#include <plib.h>
#include "DELAY16.H"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include <math.h>
#include "DELAY16.H"

#define EYELID_SERVO 0
#define RIGHTLEFT_SERVO 1
#define UPDOWN_SERVO 2

#define TESTout PORTBbits.RB2

#define BOARD_ID 1
#define ANY_BOARD 0
#define STX '>'
#define ETX '\r'
#define DLE '/'
#define PLUS  '+'
#define MAXBUFFER 64

#define THIS_BOARD_NUMBER 0x81
#define SETBOARD 0x80

#define LED PORTAbits.RA4

#define false 0
#define true !false
#define TRUE true
#define FALSE false

unsigned short inLength = 0;
extern unsigned short CRCcalculate(unsigned char *message, unsigned short nBytes);
unsigned int convertDutyCycle(unsigned char servoPosition);
void initializePorts(void);
void putch(unsigned char byte);
unsigned short readAD(void);
void ADsetChannel(unsigned char channel);
void initializePorts(void);
void putch(unsigned char ch);
unsigned char processInBuffer(unsigned short inLength);

unsigned char dummy, ch, boardNumber = 0;
static unsigned int j = 0;
static unsigned char startFlag = false, endFlag = false, escapeFlag = false, LEDflag = false;
static unsigned char LEDcounter = 10;

unsigned int dutyCycle[6] = {288, 288, 288, 288, 288, 288};

union {
    unsigned char byte[2];
    unsigned int val;
} integer;


#define MAXDATA 64
unsigned char servoBuffer[MAXBUFFER];
#define MAXDEVICES 6
unsigned char DUTYbuffer[MAXDEVICES];
unsigned char numServosUpdated = 0;
unsigned char XBEERxBuffer[MAXDATA];

#define MAXDUTY 288 * 2
#define MINDUTY 288


// Configuration: High speed crystal (HS) oscillator, 
#pragma config OSC=HS, FCMEN=OFF, IESO=OFF,\
PWRT=OFF, BOR=OFF, BORV=3,\
WDT=OFF, WDTPS=32768, PWMPIN=ON, LPOL=HIGH, HPOL=HIGH,\
FLTAMX=RA5, T1OSCMX=HIGH, MCLRE=ON,\
STVREN=ON, BBSIZ=BB256,\
CP0=OFF, CP1=OFF, CPB=OFF, CPD=OFF,\
WRT0=OFF, WRT1=OFF, WRTC=OFF, WRTB=OFF, WRTD=OFF,\
EBTR0=OFF, EBTR1=OFF, EBTRB=OFF

union tag {
    unsigned char byte[2];
    unsigned short integer;
} convert;

unsigned char processInBuffer(unsigned short inLength) {
    unsigned char i, j, k, firstServo, numServosUpdated = 0;

    numServosUpdated = XBEERxBuffer[2];
    firstServo = XBEERxBuffer[1];
    for (i = 0; i < numServosUpdated; i++) {
        j = i + 3;
        k = i + firstServo;
        if (j < MAXBUFFER && k < MAXBUFFER)
            servoBuffer[k] = XBEERxBuffer[j];
        else {
            numServosUpdated = 0;
            break;
        }
    }

    if (numServosUpdated) {
        printf("\rUpdated: %d servos", numServosUpdated);
        for (i = 0; i < MAXDEVICES; i++) {
            DUTYbuffer[i] = servoBuffer[i];
        }
    }

    /*
    unsigned short result;
    unsigned char numBytes;

    convert.byte[0] = INbuffer[inLength - 2];
    convert.byte[1] = INbuffer[inLength - 1];

    numBytes = inLength - 2;
    result = CRCcalculate(INbuffer, numBytes);

    if (result != convert.integer) return (FALSE);  // ERROR: CRC doesn't match
     */
    return (numServosUpdated);
}

#define FORWARD 0
#define REVERSE 1
#define ONE_SECOND 71
#define TENTH_SECOND 7

void main(void) {
    LEDflag = true;
    unsigned short potValue = 255;
    unsigned char stroke = FORWARD;
    unsigned short Timer0Counter = 0;
    unsigned char angle = 127;
    unsigned char EyeballState = 0;
    unsigned char delayCounter = 0;
    unsigned short secondCounter = 0;

    initializePorts();
    DelayMs(100);
    printf("Testing servo commands\r");
    ADsetChannel(0);
    LED = 0;
    
    dutyCycle[EYELID_SERVO] = EYELID_CLOSED;
    delayCounter = ONE_SECOND;



    while (1) {

        if (TMR0IF) {
            TMR0IF = 0;
            
            potValue = readAD();
            //if (potValue < 16) potValue = 16;         
            //DUTYbuffer[0] = DUTYbuffer[1] = DUTYbuffer[2] = DUTYbuffer[3] = DUTYbuffer[4] = DUTYbuffer[5] = potValue;
            
            
            /*
            #define STROKE_ANGLE 127
            Timer0Counter++;
            if (Timer0Counter >= 4) {
                Timer0Counter = 0;
                if (stroke == FORWARD) {
                    if (angle < STROKE_ANGLE) angle++;
                    else stroke = REVERSE;
                } else {
                    if (angle > 0) angle--;
                    else stroke = FORWARD;
                }
            }                                        
            */
            
            if (Timer0Counter) Timer0Counter--;
            if (!Timer0Counter) {
              Timer0Counter = TENTH_SECOND;
                //DUTYbuffer[EYELID_SERVO] = potValue;
                //dutyCycle[0] = convertDutyCycle(DUTYbuffer[0]);
                // printf("\rSTATE: %d, POT: %d, DUTY: %d", EyeballState, potValue, dutyCycle[EYELID_SERVO]);     
                printf("\rSTATE: %d, DUTY: %d", EyeballState, dutyCycle[EYELID_SERVO]);
            }

            #define LAST_STATE 3
            if (delayCounter) delayCounter--;
            
            switch (EyeballState){
                case 0: 
                    dutyCycle[EYELID_SERVO] = EYELID_CLOSED;
                    if (!delayCounter) EyeballState++;
                    break;
                case 1:
                    if (dutyCycle[EYELID_SERVO] < EYELID_OPEN){
                        dutyCycle[EYELID_SERVO] = dutyCycle[EYELID_SERVO] + 1;                        
                    }
                    else {
                        EyeballState++;
                        delayCounter = ONE_SECOND * 2;
                    }
                    break;
                case 2:
                    if (!delayCounter){
                        EyeballState++;
                    }
                    break;
                case 3:
                    if (dutyCycle[EYELID_SERVO] > EYELID_CLOSED)
                        dutyCycle[EYELID_SERVO]--;
                    else {
                        EyeballState = 0;
                        delayCounter = ONE_SECOND * 2;
                    }                    
                    break;
                default:
                    EyeballState = 0;
                    break;
                    
            }
            
            if (OVDCOND == 0b00010101) {
                OVDCOND = 0b00101010;
                integer.val = dutyCycle[1];
                PDC0L = integer.byte[0];
                PDC0H = integer.byte[1];

                integer.val = dutyCycle[3];
                PDC1L = integer.byte[0];
                PDC1H = integer.byte[1];

                integer.val = dutyCycle[5];
                PDC2L = integer.byte[0];
                PDC2H = integer.byte[1];
            } else {
                OVDCOND = 0b00010101;
                integer.val = dutyCycle[0];
                PDC0L = integer.byte[0];
                PDC0H = integer.byte[1];

                integer.val = dutyCycle[2];
                PDC1L = integer.byte[0];
                PDC1H = integer.byte[1];

                integer.val = dutyCycle[4];
                PDC2L = integer.byte[0];
                PDC2H = integer.byte[1];
            }
            PTEN = 1; // Re-enable single shot PWM for next pulse

        }
    }
}

unsigned int convertDutyCycle(unsigned char servoPosition) {
    unsigned int temp;

    temp = servoPosition;
    temp = temp * 2;
    temp = temp + 178;
    return (temp);
}

void initializePorts(void) {
    INTCON = 0x00; // First, clear all interrupts
    PIE1 = 0; // Clear all peripheral interrupts

    // Initialize ports
    ADCON0 = 0b00000000; // Turn off A/D for now.
    ADCON1 = 0b000001110; // Set up 18F1330 for one analog input, use VCC and VSS for references.    
    ADCON2 = 0; // Clear A/D control register 2
    ADCON2bits.ADFM = 0; // Left justified A/D result
    ADCON2bits.ACQT2 = 1; // Acquisition time max
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 1;
    ADCON2bits.ADCS2 = 1; // Conversion time = Fosc/16
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;

    TRISA = 0b00001111; // Port A
    TRISB = 0b00000000; // Port B
    RBPU = 0; // Enable Port B pullups

    // TIMER 0: set up for 14 ms rollover
    T0CON = 0x00; // Clear everything
    T016BIT = 1; // 8 bit mode
    PSA = 0; // Use prescaler    
    T0PS0 = 0; // 1:128 prescaler
    T0PS1 = 1;
    T0PS2 = 1;
    T0CS = 0; // Use clock input
    T0SE = 0; // Not used
    TMR0ON = 1; // Enable timer
    TMR0IF = 0;
    // TIMER 1: disabled for now
    T1CON = 0; // Clear
    T1RD16 = 1; // Enable 16 bit operation.
    TMR1CS = 0; // Use internal clock
    T1CKPS0 = 0; // 1:1 prescale
    T1CKPS1 = 0;
    TMR1ON = 0; // Disabled

    // SET UP PEM
    PIE3 = 0; // Disable time bas interrupts
    FLTCONFIG = 0; // Disable faults
    PTCON0 = 0; // Clear PWM Timer Control Register 0
    PTCKPS0 = 1; // 1:64 Prescale
    PTCKPS1 = 1;
    PTMOD1 = 0; // Single shot mode
    PTMOD0 = 1;

    PTCON1 = 0; // Clear PWM Timer Control Register 1
    PTEN = 0; // PWM Time Base is disabled for now
    // Default time base counts UP

    PWMCON0 = 0; // Clear PWM COntrol Register 0
    PWMEN0 = 0; // Enable all PWM outputs
    PWMEN1 = 0;
    PWMEN2 = 1;
    PMOD0 = 1; // All PWM's in independent mode
    PMOD1 = 1;
    PMOD2 = 1;

    PWMCON1 = 0; // PWM Control Register 1 not used

    PTPERH = 0; // Set PWM period to about 2.4 milliseconds for 18.432 Mhz clock
    PTPERL = 170;

    // PDC0L = PDC0H = PDC1L = PDC1H = PDC2L = PDC2H = 0;  Clear duty cycles 

    OVDCOND = 0b00101010; // To start with, set outputs on PWM0, PWM2, PWM4
    OVDCONS = 0b00000000; // When PWM's are off, keep outputs low.

    BRGH = 1; // high speed baud rate    
    SPBRG = 19; // Set the baud rate to 57600 for 18.432 Mhz clock    

    SYNC = 0; // asynchronous
    SPEN = 1; // enable serial port pins
    CREN = 1; // enable reception
    SREN = 0; // no effect
    TXIE = 0; // disable tx interrupts
    RCIE = 1; // Enable rx interrupts
    TX9 = 0; // 8 bit transmission
    RX9 = 0; // 8 bit reception
    TXEN = 1; // enable the transmitter
    TXIE = 0; // Disable UART Tx interrupts
    RCIE = 0; // Enabled UART Rx interrupts
    PEIE = 1; // Enable peripheral interrupts.
    TMR1IE = 0; // Disable timer 1 interrupts.
    TMR0IE = 0; // Disable Timer 0 interrupts
    GIE = 1; // Enable global interrupts
}


// This transmits one character over the RS485 bus.
// It enables RS485 transmission for the first character sent,
// and disables if a carriage return '\r' is tranmitted.

void putch(unsigned char ch) {
    while (!TXIF) // Wait for transmit buffer to be empty
        continue;
    TXREG = ch;
}

void ADsetChannel(unsigned char channel) {
    ADCON0 = (channel << 2) + 0x01; // enable ADC, RC osc.
}

unsigned short readAD(void) {
    int ADresult;
    GODONE = 1;
    while (GODONE)
        continue; // wait for conversion complete
    ADresult = (int) ADRESH;

    return (ADresult);
}

static void interrupt isr(void) {
    unsigned char BoardID, ch;
    static unsigned char buffIndex = 0;
    static unsigned char escapeFlag = false;
    unsigned char i, j, k, firstServo;

    if (RCIF == 1) { // If RX interrupt occurs:
        RCIF = 0;

        if (RCSTAbits.OERR) { // If overrun occurs, reset receive enable.
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }

        if (RCSTAbits.FERR) // If frame error occurs, flush buffer
            ch = RCREG;
        ch = RCREG;

        if (ch == DLE && !escapeFlag)
            escapeFlag = true;
        else if (ch == STX && !escapeFlag)
            buffIndex = 0;
        else if (ch == ETX && !escapeFlag) {
            BoardID = XBEERxBuffer[0];
            if (BoardID == BOARD_ID)
                inLength = buffIndex;
            buffIndex = 0;
        } else {
            escapeFlag = false;
            if (buffIndex < MAXBUFFER) XBEERxBuffer[buffIndex++] = ch;
        }
    }
}

