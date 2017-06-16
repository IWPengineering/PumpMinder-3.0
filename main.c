/* 
 * File:   PumpMinder-3.0.c
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
///#pragma config FNOSC = FRC              // Fast RC oscilator (FRC)
#pragma config FNOSC = LPFRC            // Oscillator Select (500 kHz Low-Power FRC oscillator with divide-by-N (LPFRCDIV), default div 1:1)
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-Speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
//#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output disabled; pin functions as port I/O)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
///#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)

// FICD
#pragma config ICS = PGx1              // ICD Pin Placement Select bits (PGC1/PGD1 are used for programming and debugging the device)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
//#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
#pragma config RTCOSC = LPRC            // RTCC Reference Clock Select bit (RTCC uses LPRC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

const int pulseWidthThreshold = 20; // The value to check the pulse width against (2048)
static volatile int buttonFlag = 0; // alerts us that the button has been pushed and entered the inerrupt subroutine
static bool isButtonTicking = false;
static volatile int buttonTicks = 0;

//prototypes
void sendMessage(char message[160]);

void initAdc(void); // forward declaration of init adc


void _ISR _T1Interrupt( void){
// The T1 interrupt is used to wake the PIC up from SLEEP.

// Note:  The timer is reset to zero once it matches its Preset value
 
_T1IF = 0;  //clear the interrupt flag
} //T1Interrupt

void ConfigTimerT1WithInt(int num_ms_sleep){
    _T1IP = 4; // confirm interrupt priority is the default value
    TMR1 = 0; // clear the timer

    // configure Timer1 module to
    // use LPRC as its clock so it is still running during sleep
    // set prescale to 1:1
    // continue running in IDLE mode and start running
    T1CON = 0x8202;

    // set the period register for num_ms_sleep milliseconds 
    // Since oscillator = LPRC it is 32khz
    // so one count every 31.25usec
    // 
    PR1 = ((float) num_ms_sleep *1000)/31.25;

// Timer1 Interrupt control bits
    _T1IF = 0; // clear the interrupt flag, before enabling interrupt
    _T1IE = 1; // enable the T1 interrupt source
}
void ConfigTimerT2NoInt(){
// Timer 2 is used to measure the HIGH and LOW times from the WPS
    // Also used in delayMs function.
    TMR2 = 0; // clear the timer
    // Configure Timer2 
    //   As a 16bit counter running from Fosc/2 with a 1:1 prescale
    //   shut off in IDLE, Don't enable yet
    T2CON = 0x2000; // Don't prescale
    // Assume the period register PR2 will be set by the function using Timer2

    // init the Timer2 Interrupt control bits
    _T2IF = 0; // clear the interrupt flag, this can be used to see if Timer2 got to PR2
    _T2IE = 0; // disable the T2 interrupt source
}

void UART_init(void) {
    // UART config

    U1STA = 0;
    //U1MODE = 0x8000; //enable UART for 8 bit data//no parity, 1 stop bit
    
    // If Fosc = 8Mhz, use the following two settings
    //U1MODEbits.BRGH = 0; // or BRGH = 0 and then BRG = 25
    //U1BRG = 25; // Set baud to 9600, FCY = 4MHz (#pragma config FNOSC = FRC)
  
    //U1MODEbits.PDSEL = 0; // 8 bits no parity
    //U1MODEbits.STSEL = 0; // 1 stop bit
            
    // Since Fosc = 500khz now we need to use BRGH = 1 and BRG = 6 (we would like 5.5) to get 9600 BAUD
    U1MODEbits.BRGH = 1;
    // U1BRG = 6;
    U1BRG = 12;
    
    U1MODEbits.UARTEN = 1;  // Turn on the UART
    U1STAbits.UTXEN = 1; //enable transmit
}


/*********************************************************************
 * Function: initialization()
 * Input: None
 * Output: None
 * Overview: Configures RA, RB pins as input or output 
 *           Configures Timer 1 to increment at 8Mhz/256 = 31.25khz (32usec)
 *           Turns ON the Water Presence Sensor
 *           Turns OFF the Battery Voltage Sensor
 *           Turns OFF the Low Battery Indicator
 *           Enables a Change Notification Interrupt for the STATE Debug/Settings pin
 *           Initializes A/D, UART and SLEEP mode
 * Note: Pic Dependent
 * TestDate: 06-03-14
 ********************************************************************/
void initialization(void) {
    //IO port control
    AD1PCFG = 0xFFFF; //Turn off analog function on all pins
    TRISA = 0xFFFF; // Make PORTA all inputs (NOTE:  Port A outputs configured later)
    TRISB = 0xFFFF; // Make PORTB all inputs (NOTE: Port B outputs configured later)
    
    // Configure Serial Communication Pins. 
    // this is the default TRISBbits.TRISB2=1; //U1Rx 
    TRISBbits.TRISB7 = 0; // U1Tx = Output
    // THIS IS SHAWN'S CODE TRISB = 0b1101110111111111; //RB2 and RB7 outputs [looks like RB13 and RB9??] (why are both RX and TX outputs?)
    
 
    //T1CONbits.TCS = 0; // Source is Internal Clock Fosc/2 so 250khz
    //T1CONbits.TCKPS = 0b01; // Prescalar to 1:8 (now TC1 clock is 31.25khz)
    //T1CONbits.TON = 1; // Enable the timer (timer 1 is used for the water sensor)

    //H2O sensor config
    // WPS_ON/OFF pin 7 RA2 (WPS input is RA1 - pin 3)
    TRISAbits.TRISA2 = 0; //makes water presence sensor enable pin an output.
    PORTAbits.RA2 = 1; //turn on the water presence sensor.
    
    // Battery Voltage Check (enable = B4, battery voltage A3)
    TRISBbits.TRISB4 = 0; // make battery voltage check enable an output
    PORTBbits.RB4 = 0;    // Disable the battery voltage check
    
    // Low battery Indicator
    TRISAbits.TRISA4 = 0; // Make low battery indicator (pin 10 A4 an output)
    PORTAbits.RA4 = 0;    // Turn off the low battery indicator
    
    // Debug/Setting Pins
    // Button pin (RA6) is already an input
    // STATE pin (RB12) is already an input 
    CNEN1bits.CN8IE = 1; // enable interrupt for pin 15 RB12*/
    IEC1bits.CNIE = 1; // enable change notification interrupt
    
    // Unused Pins
    // RB13, 14, 15 are already inputs

    initAdc();    // Initialize the A/D converter
    UART_init();  // Initialize the UART for communication via RJ45
  // I think this assumes use of RTCC to sleep.  We will use Timer 1  initSleep();  // Initialize SLEEP mode
    
    // Timer control (for WPS)
    ConfigTimerT2NoInt();    // used by readWaterSensor to time the WPS_OUT pulse
    
    //void setRTCC(char sec, char min, char hr, char wkday, char date, char month, char year)
    //        internal RTCC expects 0-6 lets call Sunday 0
    setRTCC(0,10,14,5,9,06,17); //Friday June 9 2:10 PM
    // debug RTCC
    debugVar = GetRTCCmonth();
    debugVar = GetRTCCday();
    debugVar = GetRTCChour();


}

/*********************************************************************
 * Function: readWaterSensor
 * Input: None
 * Output: pulseWidth
 * Overview: RA1 is the water sensor, start at beginning of positive pulse
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
 * Function: readWaterSensor2
 * Input: None
 * Output: pulseWidth indicates Water present True or False if no water
 *         this decision is made by checking the frequency of the square wave
 *         WPSOUT.  Measurements made on PumpMinder show no water as 65hz and 
 *         tap water to be 550hz - 1300hz.  This is very different than IWP
 *         which expected water to be at least 2khz.
 *         This routine is hard coded to assume that anything slower than
 *         400 hz means that there is no water.
 * Overview: RA1 is water sensor 555 square wave
 * Note: Pic Dependent
 * TestDate: Not tested as of 03-05-2015 ?????????????????
 ********************************************************************/
int readWaterSensor2(void) // RB8 is one water sensor
{
    int WaterPresent = true;  // initialize WPS variable
    
    // LEAVE it on or you need to allow time for it to turn on
    //           PORTAbits.RA2 = 1; // Turn on the 555 Timer
    
    // System clock is 500khz clock  500khz/2 = 250khz => 4us period
    // Choose 400hz as min threshold for water present
    //     so High or Low last no longer than 1.250ms
    // PR2 1.25ms/4us = 312.5
    PR2 = 312;
    
    
    TMR2 = 0;  //Clear Timer2
    _T2IF = 0; //Clear Timer2 Interrupt Flag
    T2CONbits.TON = 1;  //turn on Timer2
    
    //Must get through both a HIGH and a LOW of short enough duration
    //to be considered the frequency (2khz min) representing water is present
    while(PORTAbits.RA1 && WaterPresent){
        if(_T2IF){
            WaterPresent=false;
            _T2IF = 0;
        } //high too long, no water
    }
    TMR2 = 0;  //Restart Timer2
    while(!PORTAbits.RA1 && WaterPresent){
        if(_T2IF){
            WaterPresent=false;
            _T2IF = 0;
        } //low too long, no water
    }
    TMR2 = 0;  //Restart Timer2
    while(PORTAbits.RA1 && WaterPresent){
        if(_T2IF){
            WaterPresent=false;
            _T2IF=0;
        } //high too long, no water
    }
    T2CONbits.TON = 0;  //turn off Timer2
    // Don't turn off the 555 Timer.  You need to wait to turn it on 
    // and its not worth the power savings
    //           PORTAbits.RA2 = 0; // Turn off the 555 Timer
    //WaterPresent variable is high if the freq detected is fast enough (PR2 value)
    return (WaterPresent);
    

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

//int stringLength(char *string) {
//    int i = 0;
    //Checks for the terminating character
//    while (string[i] != '\0') {
//        i++;
//    }
//    return i;
//}

//void sendMessage(char message[160]) {
    
//    int stringIndex = 0;
    

//    while (stringIndex < stringLength(message)) { 
//        while (U1STAbits.UTXBF == 1){
            //do nothing
//        }

//        U1TXREG = message[stringIndex];
//        stringIndex++;
        
//    }
    
    
    
//}


#define delayTime                   500 // main loop duration (including SLEEP) in milliseconds
#define msHr                        (uint32_t)3600000
//#define hourTicks                   (msHr / delayTime)
#define hourTicks                   5 // simulate 1hr every 2.5sec DEBUG
#define BUTTON_TICK_COUNTDOWN_THRESHOLD          5
#define BUTTON_TICK_RESET_THRESHOLD              10

int main(void)
{   
    resetCheckRemedy();
    // change the postscaler on the system clock to 1:1 rather than default 2:1
    CLKDIVbits.RCDIV = 0b000; 
    
    initialization();
    sendMessage("Initialization Complete\n");

   // does not work ConfigTimerT1WithInt(delayTime); // used to wake up from SLEEP every delayTime milli-seconds
    
    uint16_t tickCounter = 0;
    uint16_t hourCounter = 0;
    int dayCounter = 24;
    int loopCounter = 0;
    Day = 0;
 
     
    TRISBbits.TRISB15 = 0;  // for debug, make unused pin 18 an output
       
    while (1){
         PORTBbits.RB15 = 0;  // about to go to sleep
        //sleepForPeriod(HALF_SECOND);
   //// DEBUG     Sleep(); //sleep until Timer1 wakes us up every delayTime ms
        /// delayMs(delayTime);
         delayMs(500);  // debug
         PORTBbits.RB15 = 1;  // just woke up, sleep duration is time LOW
         
         // Update the day
         loopCounter++;
         if(loopCounter >= hourTicks){
             dayCounter--;
             sendMessage("One hour gone by\n");  // Debug
             loopCounter = 0;
             if(dayCounter == 0){
                 Day++;
                 dayCounter = 24;
                 sendMessage("    NEXT DAY \n");  // Debug
             }
         }
         if(Day > PrevDay){//Save daily water hours to EEPROM
            int EEPROMaddrs = PrevDay*2;
            int decimalHour=0;
            EEProm_Write_Int(EEPROMaddrs,hourCounter);
            hourCounter = 0; // reset for the new day 
            decimalHour = ((float)tickCounter/hourTicks)*1000;  // The value of decimalHour is the number of mHours of water
            EEPROMaddrs++;
            EEProm_Write_Int(EEPROMaddrs,decimalHour);
            decimalHour = 0; //reset for the new day
            tickCounter = 0; //reset for the new day
            PrevDay = Day;
            
         }
        if (readWaterSensor2()){
            tickCounter++;  // keeps track of fractional hours pumped

            if (tickCounter >= hourTicks) 
            {
                hourCounter++;  // keeps track of integer hours pumped
                tickCounter = 0;
                sendMessage("another hour of water\n");  // Debug
            }
        }
        
        if(isButtonTicking){
            if(PORTAbits.RA6){
               sendMessage("button held down\n");
               buttonTicks++; 
               if(buttonTicks == (BUTTON_TICK_RESET_THRESHOLD - 3)) // Warn 1.5sec in advance
               { 
                   sendMessage("About to RESET\n");
               }
               if(buttonTicks > BUTTON_TICK_RESET_THRESHOLD)
               {
                   sendMessage("Resetting\n");
                   tickCounter = 0;
                   hourCounter = 0;
                   buttonTicks = 0;
                   Day = 0;
                   PrevDay = Day; 
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
            int decimalHour = 0;
            buttonFlag = 0;
            
            // Save the current pumping hours to EEPROM
            int EEPROMaddrs = Day*2;
            EEProm_Write_Int(EEPROMaddrs,hourCounter);
             
            decimalHour = ((float)tickCounter/hourTicks)*1000;  // The value of decimalHour is the number of mHours of water
            EEPROMaddrs++;
            EEProm_Write_Int(EEPROMaddrs,decimalHour);
            
            ReportHoursOfPumping();  // This will send today and all previously saved days to the RJ45 connection
            isButtonTicking = true;
            
        }
    }

    return -1;
}
/** Notes for Dr. Fish:
 *  Need to add the code for writing to EEPROM to store daily values (done not tested)
 *  Need to update code to tranmit that array of data points.  (done not tested)
 *  Need to update Sleep code so that it doesn't go to sleep mid UART transmission (not necessary.  Only start SLEEP when main() commands is
 *  Verify the overall usage of the WPS and also the ticks to hour and decimal hours conversion is correct.
 */