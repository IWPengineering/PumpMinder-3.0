/* 
 * File:   PumpMinder-3.0.c
 * Author: Shawn Bordner
 * And Perfecter: Randy Fish
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
static volatile int buttonFlag = 0; // alerts us that the button has been pushed and entered the interrupt subroutine
static bool isButtonTicking = false;
static volatile int buttonTicks = 0;


void ConfigTimerT1NoInt(){
    _T1IP = 4; // confirm interrupt priority is the default value
    TMR1 = 0; // clear the timer
 
    // Configure Timer1 
    //   As a 16bit counter running from Fosc/2 
    //   continue running in IDLE, Don't enable yet
    T1CON = 0x2000; 
    T1CONbits.TCKPS = 1; // select 1:8 prescale so Timer Clock = 250khz/8 = 31.25khz 
    // Assume the period register PR1 will be set by the function using Timer1

    // init the Timer1 Interrupt control bits
    _T1IF = 0; // clear the interrupt flag, this can be used to see if Timer2 got to PR2
    _T1IE = 0; // disable the T2 interrupt source
 
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
    ConfigTimerT1NoInt();    // used to control total time for each outer loop
    ConfigTimerT2NoInt();    // used by readWaterSensor to time the WPS_OUT pulse
    
    //void setRTCC(char sec, char min, char hr, char wkday, char date, char month, char year)
    //        internal RTCC expects 0-6 lets call Sunday 0
    setRTCC(0,0,12,6,25,011,17); //Saturday Nove 25 17 12:00 PM
    CurrentDay = GetRTCCday();
    PrevDay = CurrentDay;
    CurrentHour = GetRTCChour();
    PrevHour = CurrentHour;
    CheckBattery();
  
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



#define delayTime                   500 // main loop duration (including SLEEP) in milliseconds
//#define msHr                        (uint32_t)3600000
//#define hourTicks                   (msHr / delayTime)
//#define hourTicks                   5 // simulate 1hr every 2.5sec DEBUG
//#define BUTTON_TICK_COUNTDOWN_THRESHOLD          5
#define BUTTON_TICK_RESET_THRESHOLD              10

void DeepSleep() {

    PORTAbits.RA4 = 0; //Turn of low battery LED
    PORTAbits.RA2 = 0; //Turn off WPS

    asm(
    BSET DSCON, #15; //Enable Deep Sleep
    PWRSAV #SLEEP_MODE ; //Put the device into Deep Sleep mode
    );
}


int main(void)
{   
    resetCheckRemedy();
    // change the postscaler on the system clock to 1:1 rather than default 2:1
    CLKDIVbits.RCDIV = 0b000; 
    
    initialization();
    
    sendMessage("Initialization Complete\r\n");
    uint32_t decimalHour = 0;
    uint16_t hourCounter = 0;

    //If there is no unread data, day should be zero. if there is unread data, the day should be the next day after what has already been saved.
    int EEPROMaddrs = 0;
    Day = EEProm_Read_Int(EEPROMaddrs);
    if (Day > 0) { // This is the number of days saved but unread. 
        Day++;
    }
    
      
    _T1IF = 0; //clear interrupt flag
    TMR1 = 0; // clear timer 
    T1CONbits.TON = 1;  //turn on Timer1 
    PR1 = delayTime * 31.25;  // Timer 1 clock = 31.25khz so 31.25 clocks/1ms


    while(1) {
        if(PORTAbits.RA6) {
            DeepSleep();
        }

    }

    while (1){
        // Just wait until Timer1 has gotten to delayTime since last loop start
        //
        // For our current selections, this means that we go around this loop 1 every second
        
         while(!_T1IF){ // just wait until delayTime has gone by
         }
        _T1IF = 0; //clear T1 interrupt flag
        TMR1 = 0; // clear timer T1

         // Update the hour and day
         CurrentHour = GetRTCChour();
         //sprintf(debugString, "%d", CurrentHour);
         //sendMessage(debugString);
         //sendMessage("\r\n");
         if(CurrentHour != PrevHour){
             // CurrentDay = GetRTCCday();
             CheckBattery(); // Once the battery is found to be low, the LED will flash on and off
             PrevHour = CurrentHour;
         }
         // Flash Low Battery LED if battery is Low
         if(LowBatteryDetected){
             FlashBatteryCounter++;
             if((!PORTAbits.RA4)&&(FlashBatteryCounter > 3)){
                 PORTAbits.RA4 = 1; // Turn Low Battery LED On for 1 sec
                 FlashBatteryCounter = 0; // reset counter
             }
             else{
                 PORTAbits.RA4 = 0;  // Turn Low Battery LED Off
             }   
         }
         
         CurrentDay = GetRTCCday();
         if(CurrentDay != PrevDay){//Save daily water hours to EEPROM
             // If we are in the middle of pumping we will just ignore any pumping
             // that took place right before the day rolled over to the next
             pumping = 0;
            int EEPROMaddrs = 0; //first location is used for the number of days since system was RESET
            EEProm_Write_Int(EEPROMaddrs,Day);
            EEPROMaddrs = 1+(Day*2); //first location for new day's data
            EEProm_Write_Int(EEPROMaddrs,hourCounter);
            hourCounter = 0; // reset for the new day 
            EEPROMaddrs++;
            EEProm_Write_Int(EEPROMaddrs,decimalHour);
            decimalHour = 0; //reset for new day
            Day++;
            PrevDay = CurrentDay;  
         }

          if (readWaterSensor2()){
            if (pumping == 0){ //Is this the start of a pumping event?
                pumping = 1; //sets flag saying that pumping is in progress
                hourInit = GetRTCChour(); //Gets the current hour
                // We want to read minute and second at the same time so we don't 
                // have a problem like the time is 1:59 when we read seconds so we 
                // end up getting seconds =  59 and min = 2;
                
                //Set the pointer to 0b01 so that reading starts at at minutes - seconds
                _RTCPTR = 0b00; // decrements with read or write
                _RTCWREN = 0; //don't want to write, just want to read
                long binaryMinuteSec = RTCVAL; // write minute & second to variable
                char secBCD = binaryMinuteSec & 0b0000000011111111;
                secondInit = BcdToDec(secBCD);  //Gets the current second
                char minuteBCD = (binaryMinuteSec >> 8) & 0b0000000011111111; 
                minuteInit = BcdToDec(minuteBCD); // Gets the current minute
               
            }
           
        }
        if ((pumping == 1) && !readWaterSensor2()){ // We just stopped pumping
            hourEnd = GetRTCChour();
            // We want to read minute and second at the same time so we don't 
                // have a problem like the time is 1:59 when we read seconds so we 
                // end up getting seconds =  59 and min = 2;
                
                //Set the pointer to 0b01 so that reading starts at at minutes - seconds
                _RTCPTR = 0b00; // decrements with read or write
                _RTCWREN = 0; //don't want to write, just want to read
                long binaryMinuteSec = RTCVAL; // write minute & second to variable
                char secBCD = binaryMinuteSec & 0b0000000011111111;
                secondEnd = BcdToDec(secBCD);  //Gets the current second
                char minuteBCD = (binaryMinuteSec >> 8) & 0b0000000011111111; 
                minuteEnd = BcdToDec(minuteBCD); // Gets the current minute
                
                   
            // The goal here is to calculate the number of hours and decimal hours
            // in this most recent pumping event and add it to our growing total pumping
            // time (hourCounter, decimalHour) which will be saved at the end of the day
            // or when the REPORT button is pressed
            
            // Need to take into account times when sec,min,hr roll over to next min,hr,day
            int secondTOT = secondEnd - secondInit;
            if(secondTOT<0){  // Correct for second overflow
                secondTOT=secondTOT+60;
                minuteEnd--;  
            }
            
            int minuteTOT = minuteEnd - minuteInit; //gives you the total amount of minutes pumped
            if(minuteTOT<0){   // Correct for minute overflow
                minuteTOT=minuteTOT+60;
                hourEnd--;
            }
            
            int hourTOT = hourEnd - hourInit; //gives you the total number of hours pumped during this last pumping event    
            if(hourTOT<0){hourTOT= hourTOT+24;} // The start may have been just before midnight
           
            // Now start updating our running totals
            long PumpingEventMilliSeconds = 60*minuteTOT + secondTOT;
            PumpingEventMilliSeconds *= 1000;       // This number can never exceed 3599000
            if(PumpingEventMilliSeconds > 3599000){
                sendMessage("Error Calculating Pumping Seconds\r\n"); /// DEBUG
            }
            // Now add to our running sum of mill=hours which we call decimalHour
            
            long decimalHourMilliSeconds = decimalHour*3600; //convert to number of milliseconds
                                                             // the max is 24bits and decimalHour is 31
            decimalHourMilliSeconds = decimalHourMilliSeconds+PumpingEventMilliSeconds;
            if(decimalHourMilliSeconds > 3600000){
                hourTOT++;
                decimalHourMilliSeconds = decimalHourMilliSeconds - 3600000;
            }
            decimalHour = decimalHourMilliSeconds/3600; //convert back to milli-hours
            // Update hourCounter
            hourCounter = hourCounter+hourTOT;
            
            pumping = 0; // clears the pumping flag
        }
        
        if(isButtonTicking){
            if(PORTAbits.RA6){
               buttonTicks++; 
               if(buttonTicks == (BUTTON_TICK_RESET_THRESHOLD / 2)) // Warn before resetting
               { // Divide variables by three to account for granularity error in time calculation
                   sendMessage("About to RESET ");
               }
               if(buttonTicks >(BUTTON_TICK_RESET_THRESHOLD / 2) ){
                   sendMessage("! ");
               }
               if(buttonTicks > BUTTON_TICK_RESET_THRESHOLD) //reset after 5 seconds
               { 
                   sendMessage("\r\n Resetting\r\n");
                   decimalHour = 0;
                   hourCounter = 0;
                   buttonTicks = 0;
                   Day = 0;  // Day is used to show the #day since last reset
                   //PrevDay = Day; 
                   int EEPROMaddrs = 0; //first location saved for the day
                   EEProm_Write_Int(EEPROMaddrs,Day);
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
            /////////////////pumping = 0; // clears the pumping flag
            
            // Save the current pumping hours to EEPROM
            int EEPROMaddrs = 1 +(Day*2);
            EEProm_Write_Int(EEPROMaddrs,hourCounter);
            
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