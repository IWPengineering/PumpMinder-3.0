/* 
 * File:   Water42.0.c
 * Author: Shawn Bordner
 *
 * Created on April 13, 2017
 */

#include <p24Fxxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <xc.h>
#include <string.h>
#include <stdbool.h>
//#include "display.h"
#include "utilities.h"





// PIC24F16KA101 Configuration Bit Settings

// 'C' source line config statements



// FBS
#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)

// FOSCSEL
#pragma config FNOSC = FRC              // Fast RC oscilator (FRC)
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-Speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx2              // ICD Pin Placement Select bits (PGC2/PGD2 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

const int pulseWidthThreshold = 20; // The value to check the pulse width against (2048)
static volatile int buttonFlag = 0; // alerts us that the button has been pushed and entered the inerrupt subroutine
static bool isButtonTicking = false;
static volatile int buttonTicks = 0;

//prototypes
void sendMessage(char message[160]);

void initAdc(void); // forward declaration of init adc

/*********************************************************************
 * Function: initialization()
 * Input: None
 * Output: None
 * Overview: configures chip to work in our system (when power is turned on, these are set once)
 * Note: Pic Dependent
 * TestDate: 06-03-14
 ********************************************************************/
void initialization(void) {
    //IO port control
    AD1PCFG = 0xFFFF; //Turn off analog function on all pins
    TRISA = 0xFFFF; // Make PORTA all inputs
    //TRISB = 0x0DC0; //0xCEE0; // Set LCD outputs as outputs
    TRISB = 0b1101110111111111; //RB2 and RB7 outputs
    // Timer control (for WPS)
    T1CONbits.TCS = 0; // Source is Internal Clock (8MHz)
    T1CONbits.TCKPS = 0b11; // Prescalar to 1:256
    T1CONbits.TON = 1; // Enable the timer (timer 1 is used for the water sensor)

    //H2O sensor config
    // WPS_ON/OFF pin 7 RA2
    TRISAbits.TRISA2 = 0; //makes water presence sensor pin an output.
    PORTAbits.RA2 = 1; //turns on the water presence sensor.
    
    TRISAbits.TRISA6 = 1; // RA6 as an input 
    CNEN1bits.CN8IE = 1; // enable interrupt for pin 15 RB12*/
    IEC1bits.CNIE = 1; // enable change notification interrupt

    initAdc();
}

/*********************************************************************
 * Function: readWaterSensor
 * Input: None
 * Output: pulseWidth
 * Overview: RB5 is one water sensor, start at beginning of positive pulse
 * Note: Pic Dependent
 * TestDate: Not tested as of 03-05-2015
 ********************************************************************/
int readWaterSensor(void) 
{
    // WPS_OUT - pin 3 RA1
    if (PORTAbits.RA1) 
    {
        while (PORTAbits.RA1){
        }; //make sure you start at the beginning of the positive pulse
    }
    
    while (!PORTAbits.RA1) 
    {
    }; //wait for rising edge
    
    uint32_t prevICTime = TMR1; //get time at start of positive pulse
    while (PORTAbits.RA1) 
    {
    };
    uint32_t currentICTime = TMR1; //get time at end of positive pulse
    uint32_t pulseWidth = 0;
    if (currentICTime >= prevICTime) 
    {
        pulseWidth = (currentICTime - prevICTime);
    } 
    else 
    {
        pulseWidth = (currentICTime - prevICTime + 0x100000000);
    }
    //Check if this value is right
    return (pulseWidth <= pulseWidthThreshold);
}

/*********************************************************************
 * Function: initAdc()
 * Input: None
 * Output: None
 * Overview: Initializes Analog to Digital Converter
 * Note: Pic Dependent
 * TestDate: 06-02-2014
 ********************************************************************/
void initAdc(void) 
{
    // 10bit conversion
    AD1CON1 = 0; // Default to all 0s
    AD1CON1bits.ADON = 0; // Ensure the ADC is turned off before configuration
    AD1CON1bits.FORM = 0; // absolute decimal result, unsigned, right-justified
    AD1CON1bits.SSRC = 0; // The SAMP bit must be cleared by software
    AD1CON1bits.SSRC = 0x7; // The SAMP bit is cleared after SAMC number (see
    // AD3CON) of TAD clocks after SAMP bit being set
    AD1CON1bits.ASAM = 0; // Sampling begins when the SAMP bit is manually set
    AD1CON1bits.SAMP = 0; // Don't Sample yet
    // Leave AD1CON2 at defaults
    // Vref High = Vcc Vref Low = Vss
    // Use AD1CHS (see below) to select which channel to convert, don't
    // scan based upon AD1CSSL
    AD1CON2 = 0;
    // AD3CON
    // This device needs a minimum of Tad = 600ns.
    // If Tcy is actually 1/8Mhz = 125ns, so we are using 3Tcy
    //AD1CON3 = 0x1F02; // Sample time = 31 Tad, Tad = 3Tcy
    AD1CON3bits.SAMC = 0x1F; // Sample time = 31 Tad (11.6us charge time)
    AD1CON3bits.ADCS = 0x2; // Tad = 3Tcy
    // Conversions are routed through MuxA by default in AD1CON2
    AD1CHSbits.CH0NA = 0; // Use Vss as the conversion reference
    AD1CSSL = 0; // No inputs specified since we are not in SCAN mode
    // AD1CON2
}

/*********************************************************************
 * Function: readAdc()
 * Input: channel
 * Output: adcValue
 * Overview: check with accelerometer
 * Note: Pic Dependent
 * TestDate:
 ********************************************************************/
int readAdc(int channel) //check with accelerometer
{
    switch (channel) 
    {
        case 4:
            /*ANSBbits.ANSB2 = 1; // AN4 is analog*/
            TRISBbits.TRISB2 = 1; // AN4 is an input
            AD1CHSbits.CH0SA = 4; // Connect AN4 as the S/H input
            break;
    }
    AD1CON1bits.ADON = 1; // Turn on ADC
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE) 
    {
    }
    // Turn off the ADC, to conserve power
    AD1CON1bits.ADON = 0;
    return ADC1BUF0;
}

void __attribute__((__interrupt__, __auto_psv__)) _DefaultInterrupt() 
{ 
    // We should never be here
}

void __attribute__((interrupt, auto_psv)) _CNInterrupt(void) { //button interrupt
    if (IFS1bits.CNIF && PORTAbits.RA6){
        //sendMessage("Interrupt Has Happened");
     // If the button is pushed and we're in the right ISR
        buttonFlag = 1;
    }
    

    // Always reset the interrupt flag
    IFS1bits.CNIF = 0;
}

static void adjustHours(float hours, int *p_hours, int *p_decimal_hours)
{
    hours *= 1.602564;
    
    *p_hours = ((int)hours);
    *p_decimal_hours = (hours * 1000) - (*p_hours * 1000);
}

/*void hoursToAsciiDisplay(int hours, int decimalHour) 
{
    int startLcdView = 0;
    DisplayTurnOff();
    unsigned char aryPtr[] = "H: ";
    DisplayDataAddString(aryPtr, sizeof ("H: "));
    //    DisplayDataAddCharacter(49); // can we cycle power, or ones mixed up.

    float realHours = (hours * 1000) + decimalHour;
    realHours /= 1000;
    
    int h, dh;
    adjustHours(realHours, &h, &dh);
    
    hours = h;
    decimalHour = dh;
    
    if (hours == 0) 
    {
        DisplayDataAddCharacter(48);
    } 
    else 
    {
        if (startLcdView || (hours / 10000 != 0)) 
        {
            DisplayDataAddCharacter(hours / 10000 + 48);
            startLcdView = 1;
            hours = hours - ((hours / 10000) * 10000); // moving the decimal point - taking advantage of int rounding
        }
        
        if (startLcdView || hours / 1000 != 0) 
        {
            DisplayDataAddCharacter(hours / 1000 + 48);
            startLcdView = 1;
            hours = hours - ((hours / 1000) * 1000);
        }
        
        if (startLcdView || hours / 100 != 0) 
        {
            DisplayDataAddCharacter(hours / 100 + 48);
            startLcdView = 1;
            hours = hours - ((hours / 100) * 100);
        }
        
        if (startLcdView || hours / 10 != 0) 
        {
            DisplayDataAddCharacter(hours / 10 + 48);
            startLcdView = 1;
            hours = hours - ((hours / 10) * 10);
        }
        
        DisplayDataAddCharacter(hours + 48);
    }
    
    DisplayDataAddCharacter('.');
    startLcdView = 0;
    
    if (decimalHour == 0) 
    {
        DisplayDataAddCharacter(48);
    } 
    else 
    {   
        if (startLcdView || decimalHour / 100 != 0) 
        {
            DisplayDataAddCharacter(decimalHour / 100 + 48);
            startLcdView = 1;
            decimalHour = decimalHour - ((decimalHour / 100) * 100);
        }
        else
        {
            DisplayDataAddCharacter(48);
        }
        
        if (startLcdView || decimalHour / 10 != 0) 
        {
            DisplayDataAddCharacter(decimalHour / 10 + 48);
            startLcdView = 1;
            decimalHour = decimalHour - ((decimalHour / 10) * 10);
        }
        else
        {
            DisplayDataAddCharacter(48);
        }
        
        DisplayDataAddCharacter(decimalHour + 48);
    }

    DisplayLoop(15, true);
}*/

static int countdownPos = 0;
const unsigned char countdownArray[] = { '5', '5', '4', '4', '3', '3', '2', '2', '1', '1', '0', '0' };
const unsigned char countdownResetArray[] = "Reset In ";
/*static void DisplayCountdown(void)
{
    DisplayTurnOff();
    DisplayDataAddString((unsigned char *)&countdownResetArray, sizeof(countdownResetArray));    
    DisplayDataAddCharacter(countdownArray[countdownPos++]);
    DisplayLoop(15, true);
}*/

static void ResetDisplayCountdown(void)
{
    countdownPos = 0;
    buttonTicks = 0;
}
void UART_init(void) {
    // UART config

    U1STA = 0;
    U1MODE = 0x8000; //enable UART for 8 bit data//no parity, 1 stop bit
    U1MODEbits.BRGH = 0; // or BRGH = 0 and then BRG = 25
    U1BRG = 25; // Set baud to 9600, FCY = 4MHz (#pragma config FNOSC = FRC)


    U1STAbits.UTXEN = 1; //enable transmit
}

int stringLength(char *string) {
    int i = 0;
    //Checks for the terminating character
    while (string[i] != '\0') {
        i++;
    }
    return i;
}

void sendMessage(char message[160]) {
    int stringIndex = 0;
    

    while (stringIndex < stringLength(message)) { 
        while (U1STAbits.UTXBF == 1){
            //do nothing
        }

        U1TXREG = message[stringIndex];
        stringIndex++;
        
    }
    
    
    
}


#define delayTime                   500
#define msHr                        (uint32_t)3600000
#define hourTicks                   (msHr / delayTime)
#define BUTTON_TICK_COUNTDOWN_THRESHOLD          5
#define BUTTON_TICK_RESET_THRESHOLD              10

int main(void)
{   
    resetCheckRemedy();
    
    initialization();

    UART_init();
    
    initSleep();
    sendMessage("Initialization Complete\r\n");
    
    uint16_t tickCounter = 0;
    uint16_t hourCounter = 0;
 
    while (1){
        
        //sleepForPeriod(HALF_SECOND);
        delayMs(delayTime);

        if (readWaterSensor()){
            tickCounter++;

            if (tickCounter >= hourTicks) 
            {
                hourCounter++;
                tickCounter = 0;
            }
        }
        
        if(isButtonTicking){
            if(PORTAbits.RA6){
               sendMessage("button held down\r\n");
               buttonTicks++; 
               if(buttonTicks > BUTTON_TICK_COUNTDOWN_THRESHOLD)
               {
               }
               if(buttonTicks > BUTTON_TICK_RESET_THRESHOLD)
               {
                   sendMessage("Resetting\r\n");
                   tickCounter = 0;
                   hourCounter = 0;
                   isButtonTicking = false;
               }
            }
            else
            {
                buttonTicks = 0;
                isButtonTicking = false;
            }
        }

        if (buttonFlag){ // If someone pushed the button
            
            buttonFlag = 0;
            char hourStr[15];
            char decimalStr[15];
            int hours = hourCounter;
            int decimalHour = (tickCounter / hourTicks /  1000);
            
            
            
            float realHours = (hourCounter * 1000) + decimalHour ;
            realHours /= 1000;
            int h, dh;
            adjustHours(realHours, &h, &dh);

            hours = h;
            decimalHour = dh;
            
            sprintf(hourStr, "%d", hours);
            sprintf(decimalStr, "%d", decimalHour);
            
            sendMessage("H:");
            sendMessage(hourStr);
            sendMessage(".");
            sendMessage(decimalStr);
            sendMessage("\r\n");
            isButtonTicking = true;
            
        }
    }

    return -1;
}
/** Notes for Dr. Fish:
 *  Need to add the code for writing to EEPROM to store daily values
 *  Need to update code to tranmit that array of data points.
 *  Need to update Sleep code so that it doesn't go to sleep mid UART transmission
 *  Verify the overall usage of the WPS and also the ticks to hour and decimal hours conversion is correct.
 */