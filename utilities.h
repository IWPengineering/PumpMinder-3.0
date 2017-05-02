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

void delayMs(int ms);

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


#endif	/* UTILITIES_H__ */

