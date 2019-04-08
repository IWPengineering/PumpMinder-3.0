/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef UTILITIES_H__
#define	UTILITIES_H__

#include <xc.h> // include processor files - each processor file is guarded.  

// ****************************************************************************
// *** Constants **************************************************************
// ****************************************************************************




// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************

extern int MessageBeingTransmitted;  //set to 1 when the UART is sending a message
extern float EEFloatData; //to be used when trying to write a float to EEProm EEFloatData = 123.456 then pass as &EEFloatData
extern int Day; // Used to keep track of which day (since saved water hours was last read) is currently in progress
//                 this is not the day from the RTCC which will run from 0-6 and restart
extern int PrevDay; //Based upon RTCC
extern int CurrentDay; //Based upon RTCC
extern int CurrentHour; //Based upon RTCC
extern int PrevHour; //Based upon RTCC
extern int CurrentMin; //Based upon RTCC
extern int PrevMin; //Based upon RTCC
extern int CurrentSec; //Based upon RTCC
extern int PrevSec; //Based upon RTCC
extern int debugVar;  // used during debug to read different values
extern int LowBatteryDetected; // Set to 1 when battery is less than threshold
extern int FlashBatteryCounter; //Used to flash low battery LED 0.5s/5sec
extern char debugString[15]; 
extern int pumping;
extern int hourInit;
extern int hourEnd;
extern int minuteInit;
extern int minuteEnd;
extern int secondInit;
extern int secondEnd;

typedef enum 
{
            HALF_SECOND = 0b0000,
            ONE_SECOND  = 0b0001,
            TEN_SECOND  = 0b0010,
            ONE_MINUTE  = 0b0011,
            TEN_MINUTE  = 0b0100,
            ONE_HOUR    = 0b0101,
            ONE_DAY     = 0b0110,
            ONE_WEEK    = 0b0111,
            ONE_MONTH   = 0b1000,
            ONE_YEAR    = 0b1001
} sleepLength_t;




// ****************************************************************************
// *** Functions  *************************************************************
// ****************************************************************************

void delayMs(int ms);
void setRTCC(char sec, char min, char hr, char wkday, char date, char month, char year); 
void ReportHoursOfPumping();
int getLowerBCDAsDecimal(int bcd);
int getUpperBCDAsDecimal(int bcd);
char BcdToDec(char val);
char DecToBcd(char val);
int GetRTCCmonth(void);
int GetRTCCday(void);
int GetRTCChour(void);
int GetRTCCminute(void);
int GetRTCCsecond(void);
void sendMessage(char message[750]);
int receiveMessage(void);
void initAdc(void);
int readAdc(int channel) ;
void deepSleep();

void CheckBattery(void);
int readBatteryPin(void); //returns the AD reading on the pin 8 VWATCH

/**
 * void initSleep(void)
 * 
 * This function call inits the sleep mode for the PIC.
 * This will be used in future sleepMs calls
 * 
 * This function uses the RTCC.
 */
void initSleep(void);

/**
 * void sleepms(sleepLength_t length)
 * 
 * @param length - enum which represents length of sleep
 * 
 * This function puts the device to sleep for the specified period.
 * This function uses the RTCC Alarm
 */
void sleepForPeriod(sleepLength_t length);

/**
 * This function checks every possible reset condition, individually.
 * It isolates each possibility to its own if() statement, where it can
 *  be handled.
 * 
 * As of 3/24, this function just clears the reset bits, it doesn't handle
 *  any specific implementation. This should be fixed later.
 */
void resetCheckRemedy(void);


void EEProm_Write_Int(int addr, int newData);
int EEProm_Read_Int(int addr);
void EEProm_Read_Float(unsigned int ee_addr, void *obj_p);
void EEProm_Write_Float(unsigned int ee_addr, void *obj_p);
void sendMessage(char message[160]);
int stringLength(char *string);

void ClearEEProm(void); // Note, this currently clears 21 consecutive float values in EEPROM, If you want to change that, you must modify the function

#endif	/* UTILITIES_H__ */

