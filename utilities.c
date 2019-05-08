/*
 * File:   utilities.c
 * Author: KSK0419
 *
 * Created on March 22, 2016, 10:45 AM
 */


//#include "xc.h"
#include "utilities.h"

#include <p24Fxxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <xc.h>
#include <string.h>
#include <stdbool.h>
#include <p24F16KA101.h>
#include <string.h>




int __attribute__ ((space(eedata))) eeData; // Global variable located in EEPROM


// ****************************************************************************
// *** Constants **************************************************************
// ****************************************************************************






// ****************************************************************************
// *** Global Variables *******************************************************
// ****************************************************************************

int MessageBeingTransmitted = 0;  //set to 1 when the UART is sending a message
float EEFloatData = 0;  // to be used when trying to write a float to EEProm, ie. EEFloatData = 123.456 then pass as &EEFloatData
int Day = 0; // Used to keep track of which day (since saved water hours was last read) is currently in progress
int PrevDay = 0;
int CurrentDay = 0;
int CurrentHour = 0; 
int PrevHour = 0; 
int debugVar = 0;
int LowBatteryDetected = 0; // =1 when battery voltage drops below minimum
int FlashBatteryCounter = 0;
char debugString[15]; 
int CurrentMin;
int PrevMin;
int CurrentSec;
int PrevSec;
int pumping;
int hourInit;
int hourEnd;
int minuteInit;
int minuteEnd;
int secondInit;
int secondEnd;

// ****************************************************************************
/************************* Utility FUNCTIONS ********************************/
// ****************************************************************************


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
    // This device needs a minimum of Tad = 600ns. (according to an old note)
    // Our Tcy is 1/250khz = 4us so we can use Tad = Tcy
    AD1CON3bits.ADCS = 0; // Tad = Tcy = 4us.  ADCS (conversion clock) = (TAD/TCY)-1
    // Assume we are going to use abut 12us charge time (IWP says 11.2us)
    // This would be 3*Tad
    AD1CON3bits.SAMC = 3; 
    
    // Conversions are routed through MuxA by default in AD1CON2
    // So only need to configure the MuxA conversion reference
    AD1CHSbits.CH0NA = 0; // Use Vss as the conversion reference
    AD1CSSL = 0; // No inputs specified since we are not in SCAN mode
    // AD1CON2
    
    // Configure the pin for AN5 which is where VWATCH is connected as an analog pin
    AD1PCFGbits.PCFG5 = 0;  
    
    
    TRISAbits.TRISA3 = 1;  // input
    
    // Not sure about this
    AD1CHSbits.CH0SA = 5; //AN5 is where VWATCH is connected
}

/* Unused Function
 * Deep sleep caused issues on wake up
 */
void deepSleep(){ //Put PIC into Deep Sleep mode and turn off WPS and any other unnecessary power draws
    int Abits;
    int Bbits;
    
    /*
    SRbits.IPL = 0; // Set CPU interrupt to max priority.
    IEC0bits.INT0IE = 1; // Enable Interrupt Zero (INT0)
    IPC0bits.INT0IP = 0; // default is highest priority, Sets interrupt priority
    INTCON2bits.INT0EP = 0; // 1 = negative edge, 0 = positive edge]
    TRISBbits.TRISB7 = 0; // Pin 10 A4 input. INT0 vibration sensor output, high upon vibration
    */
    
    //Read from ports, Write to Latches
    // Use shadow register    
    //LATAbits.LATA4 = 1; //Vibration Sensor power
    //LATAbits.LATA2 = 0; //WPS
    Abits = LATA;
    Abits = Abits | 0b10000; //Vibration sensor power
    Abits = Abits & 0b011; //WPS off
    LATA = Abits;
            
    //LATBbits.LATB14 = 0; //Test Pin
    //LATBbits.LATB4 = 0; //Turn off Battery Voltage Sensor
    Bbits = LATB;
    Bbits = Bbits & 0b011111111101111; //Test pin and battery voltage sensor off.
    LATB = Bbits;
            
    PMD1 = PMD1 | 0xFFFF;       //bulk disable Timers I2C,UARTS,SPI,ADC's
    PMD2 = PMD2 | 0xFFFF;       //bulk turn off Input Capture and Output compare
  
    asm("BSET DSCON, #15;"); //Enable Deep Sleep
    asm("NOP;");
    asm("PWRSAV #0"); //Put the device into Deep Sleep mode

}

 //Put PIC into Sleep mode and turn off WPS and any other unnecessary power draws
void sleepyTime(){
    int Abits;
    int Bbits;
    
    // Enabled something wrong? Doesn't sleep with the interrupts enabled must always be waking up?
    // Sleeps fine when using WDT and waking up cyclically 
    /****************************************************
    IFS0 = 0; // Clear interrupt flags.
    SRbits.IPL = 0; // Set CPU interrupt to max priority.
    IEC0bits.INT0IE = 1; // Enable Interrupt Zero (INT0)
    IPC0bits.INT0IP = 0; // default is highest priority, Sets interrupt priority
    INTCON2bits.INT0EP = 0; // 1 = negative edge, 0 = positive edge (expecting this)
    TRISBbits.TRISB7 = 1; // INT0 input is sensor output, high upon vibration
    CNPD2bits.CN23PDE = 1; // Internal pull-down resistor enabled for INT0
    CNPU2bits.CN23PUE = 0; // Internal pull-up resistor disabled for INT0
    TRISAbits.TRISA4 = 0; //Pin 10 A4 output
    LATAbits.LATA4 = 1; //Pin 10 A4 high/powered
    *****************************************************/
    //Read from ports, Write to Latches
    // Use shadow register    
    //LATAbits.LATA4 = 1; //Vibration Sensor power
    //LATAbits.LATA2 = 0; //WPS
    Abits = LATA;
    Abits = Abits | 0b0000000000010000; //Vibration sensor power
    Abits = Abits & 0b1111111111111011; //WPS off
    LATA = Abits;
    //LATBbits.LATB14 = 0; //Test Pin
    //LATBbits.LATB4 = 0; //Turn off Battery Voltage Sensor
    Bbits = LATB;
    Bbits = Bbits & 0b1011111111101111; //Test pin and battery voltage sensor off.
    LATB = Bbits;
            
    PMD1 = PMD1 | 0xFFFF;       //bulk disable Timers I2C,UARTS,SPI,ADC's
    PMD2 = PMD2 | 0xFFFF;       //bulk turn off Input Capture and Output compare
    
    RCONbits.SWDTEN = 1; // Enable WDT
   
    asm("PWRSAV #0"); //Enter sleep mode
    
    RCONbits.SWDTEN = 0; //Disable WDT
    initialization();
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

/*********************************************************************
 * Function: CheckBattery
 * Input: none
 * Output: none
 * Overview: Checks the voltage on the battery.  If it is lower than MinVoltage
 *           which is a Global setting, it lights the Low Battery LED
 * Note: Library
 * TestDate: not tested as of 6/15/2017
 ********************************************************************/
void CheckBattery(void){
    
    int batteryLevel = 0;
    // char BatStr[15]; // for DEBUG
     
    PORTBbits.RB4 = 1;  // Enable the battery voltage check
    LATBbits.LATB4 = 1; // Enable the battery voltage check
    delayMs(10); //Give things time to settle out
    
    batteryLevel = readBatteryPin();
    // VWATCH = (BatteryVoltage - drop across FET)*10K/30k
    // returned batteryLevel = VWATCH/(3.2V/2^10) or VWATCH/3.125mV
    //    or it can be written batteryLevel = (Vbattery - FET)*106.7
    // so batteryVoltage = (batteryLevel/107)+FET
    float battVolts = ((float)batteryLevel/107)+0.14; // assume FET = 0.14 based on testing
    
    PORTBbits.RB4 = 0;  // Disable the battery voltage check
    if(battVolts < 4.4){ // After 1V (some 1.2), each battery starts a steep voltage drop as charge is pulled
        // turn on LED
        LowBatteryDetected = 1;     
    }
    else{
        LowBatteryDetected = 0;
        //PORTAbits.RA4 = 0;  // turn off LED
    }
    
    // For DEBUG let's report reading
    //sprintf(BatStr, "%f", (double)battVolts);
    //sendMessage("Battery = ");
    //sendMessage(BatStr);
    //sendMessage("\r\n");
    
}

/*********************************************************************
 * Function: readBatteryPin()
 * Input: none - assumes we are using pin 8 which is where VWATCH is connected
 * Output: adcValue
 * Overview: read the value from the A/D for pin 8 does not convert to voltage
 * Note: Pic Dependent
 * TestDate:
 ********************************************************************/
int readBatteryPin(void) //check pin 8 
{
    AD1CON1bits.ADON = 1; // Turn on ADC
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE) 
    {
    }
    // Turn off the ADC, to conserve power
    AD1CON1bits.ADON = 0;
    return ADC1BUF0;
}

//This function converts a BCD to DEC
//Input: BCD Value
//Returns: Hex Value

char BcdToDec(char val) {
    return ((val / 16 * 10) + (val % 16));
}


//This function converts HEX to BCD
//Input: Hex Value
//Returns: BCD Value

char DecToBcd(char val) {
    return ((val / 10 * 16) + (val % 10));
}


/*********************************************************************
 * Function: getLowerBCDAsDecimal
 * Input: int bcd
 * Output: Decimal version of BCD found in (lower byte)
 * Overview: Returns the decimal value for the lower 8 bits in a 16 bit BCD (Binary Coded Decimal)
 * Note: Library
 * TestDate: 06-04-2014
 ********************************************************************/
int getLowerBCDAsDecimal(int bcd) //Tested 06-04-2014
{
    //Get the tens digit (located in the second nibble from the right)
    //by shifting off the ones digit and anding
    int tens = (bcd >> 4) & 0b0000000000001111;
    //Get the ones digit (located in the first nibble from the right)
    //by anding (no bit shifting)
    int ones = bcd & 0b0000000000001111;
    //Returns the decimal value by multiplying the tens digit by ten
    //and adding the ones digit
    return (tens * 10) +ones;
}


/*********************************************************************
 * Function: getUpperBCDAsDecimal
 * Input: int bcd
 * Output: Decimal verision of BCD found in (upper byte)
 * Overview: Returns the decimal value for the Upper 8 bits in a 16 bit BCD (Binary Coded Decimal)
 * Note: Library
 * TestDate: 06-04-2014
 ********************************************************************/
int getUpperBCDAsDecimal(int bcd) //Tested 06-04-2014
{
    //Get the tens digit (located in the first nibble from the left)
    //by shifting off the rest and anding
    int tens = (bcd >> 12) & 0b0000000000001111;
    //Get the ones digit (located in the second nibble from the left)
    //by shifting off the rest and anding
    int ones = (bcd >> 8) & 0b0000000000001111;
    //Returns the decimal value by multiplying the tens digit by ten
    //and adding the ones digit
    return (tens * 10) +ones;
}


/*********************************************************************
 * Function: setRTCC()
 * Input: SS MM HH WW DD MM YY
 * Output: None
 * Overview: Initializes values for the internal RTCC
 * Note: 
 ********************************************************************/
void setRTCC(char sec, char min, char hr, char wkday, char date, char month, char year){
 
    __builtin_write_RTCWEN(); //does unlock sequence to enable write to RTCC, sets RTCWEN
    
    RCFGCALbits.RTCWREN = 1; // Allow user to change RTCC values
    RCFGCALbits.RTCPTR = 0b11; //Point to the top (year) register
    
    RTCVAL = DecToBcd(year); // RTCPTR decrements automatically after this
    RTCVAL = DecToBcd(date) + (DecToBcd(month) << 8);
    RTCVAL = DecToBcd(hr) + (DecToBcd(wkday) << 8);
    RTCVAL = DecToBcd(sec) + (DecToBcd(min) << 8); // = binaryMinuteSecond;
 
    _RTCEN = 1; // = 1; //RTCC module is enabled
    _RTCWREN = 0; // = 0; // disable writing
 
}

/*********************************************************************
 * Function: GetRTCCmonth()
 * Input: None
 * Output: int value of the month from internal RTCC
 * Overview: Reads both the current month and day from the internal RTCC
 *           and returns the month
 * Note: 
 ********************************************************************/
int GetRTCCmonth(void){
    char value = 0;
    //Set the pointer to 0b10 so that reading starts at month - day
    _RTCPTR = 0b10; // decrements with read or write
    _RTCWREN = 0; //don't want to write, just want to read
    long binaryMonthDay = RTCVAL; // write month & day to variable
    value = (binaryMonthDay >> 8) & 0b0000000000011111; 
    return BcdToDec(value);
}
/*********************************************************************
 * Function: GetRTCCday()
 * Input: None
 * Output: int value of the day (date not day of week) from internal RTCC
 * Overview: Reads both the current month and day from the internal RTCC
 *           and returns the month
 * Note: 
 ********************************************************************/
int GetRTCCday(void){
    char value = 0;
    //Set the pointer to 0b10 so that reading starts at month - day
    _RTCPTR = 0b10; // decrements with read or write
    _RTCWREN = 0; //don't want to write, just want to read
    long binaryMonthDay = RTCVAL; // write month & day to variable
    value = binaryMonthDay & 0b0000000000111111; 
    return BcdToDec(value);
}

/*********************************************************************
 * Function: GetRTCChour()
 * Input: None
 * Output: int value of the current hour from internal RTCC
 * Overview: Reads both the current month and day from the internal RTCC
 *           and returns the month
 * Note: 
 ********************************************************************/
int GetRTCChour(void){
    char value = 0;
    //Set the pointer to 0b01 so that reading starts at weekday - hours
    _RTCPTR = 0b01; // decrements with read or write
    _RTCWREN = 0; //don't want to write, just want to read
    long binaryWkDayHours = RTCVAL; // write month & day to variable
    value = binaryWkDayHours & 0b0000000011111111; 
    return BcdToDec(value);
}
/*********************************************************************
 * Function: GetRTCCminute()
 * Input: None
 * Output: int value of the current minute from internal RTCC
 * Overview: Reads both the current minute and second from the internal RTCC
 *           and returns the minute
 * Note: 
 ********************************************************************/
int GetRTCCminute(void){
    char value = 0;
    //Set the pointer to 0b00 so that reading starts at at minutes - seconds
    _RTCPTR = 0b00; // decrements with read or write
    _RTCWREN = 0; //don't want to write, just want to read
    long binaryMinuteSec = RTCVAL; // write minute & second to variable
    value = (binaryMinuteSec >> 8) & 0b0000000011111111; 
    return BcdToDec(value);
}
/*********************************************************************
 * Function: GetRTCCsecond()
 * Input: None
 * Output: int value of the current second from internal RTCC
 * Overview: Reads both the current minute and second from the internal RTCC
 *           and returns the second
 * Note: 
 ********************************************************************/
int GetRTCCsecond(void){
    char value = 0;
    //Set the pointer to 0b00 so that reading starts at minutes - seconds
    _RTCPTR = 0b00; // decrements with read or write
    _RTCWREN = 0; //don't want to write, just want to read
    long binaryMinuteSec = RTCVAL; // write minutes & seconds to variable
    value = binaryMinuteSec & 0b0000000011111111; 
    return BcdToDec(value);
}

int stringLength(char *string) {
    int i = 0;
    //Checks for the terminating character
    while (string[i] != '\0') {
        i++;
    }
    return i;
}

void sendMessage(char message[750]) {
    int stringIndex = 0;
    int msg_length = 0;
   
    msg_length = stringLength(message);
    
  

    while (stringIndex < msg_length) {
        while (U1STAbits.UTXBF == 1){
            //do nothing
        }

        U1TXREG = message[stringIndex];
        stringIndex++;

    }
}

int receiveMessage(void){
    char message;
    
    if (_U1RXIF){ //Something is available to read 
        _U1RXIF = 0;
        message = U1RXREG; //Read the RX data register
    }
    
    if(message == 0b01000111){ //if message is equal to "G"
        return 2;
        
    }else if (message == 0b01000011){ //if message is equal to "C"
        return 1; //Message Received clears data, return command to reset data.
    }
    
    return 0;
    
}

/**********************************************************************
 * Function:  ReportHoursOfPumping(void)
 * Input:  none
 * Output: none
 * Overvies: Sends the value of hourCounter and tickCounter most recently saved to EEPROM
 *           which should be the amount so far today to the RJ45 connection
 *           the current values are sent first.  Then the values of previous days
 *           stored in EEPROM are sent until the day reaches zero
 * TestDate: not tested as of May 29 2017
 * *********************************************************************/
void ReportHoursOfPumping(){
    
    int report_hours;
    long int report_decimalHour;
    int report_tenth;
    int report_cent;
    int report_mil;
    char strMessage[750] = "GETDATA: "; //initializes the Message string with the format code fore the app.
    
    CheckBattery(); //Run CheckBattery() function only when transmitting data.)
    
    if(LowBatteryDetected == 0){
        sendMessage("GETBATT: Battery is OK\r\n");
    }else{
        sendMessage("GETBATT: Change Batteries\r\n");
    }
    
    int Dayptr = 0;
    while(Dayptr <= Day){ // I think this should make the data count up not down?
        int EEPROMaddrs = 1 + (Dayptr*2);
        report_hours = EEProm_Read_Int(EEPROMaddrs);
        EEPROMaddrs++;
        report_decimalHour = EEProm_Read_Int(EEPROMaddrs);
        // need to deal with leading zeros
        report_tenth = report_decimalHour/100;
        report_decimalHour = report_decimalHour - (report_tenth*100);
        report_cent = report_decimalHour/10;
        report_mil = report_decimalHour - (report_cent*10);
        
        char timeStr[15];        
        
        //The message string should have the final format of GETDATA:X.XXX,X.XXX,X.XXX, etc.
        sprintf(timeStr, "%d.%d%d%d", report_hours, report_tenth, report_cent, report_mil);
        strcat(strMessage, timeStr);
        if(Dayptr < Day){
            strcat(strMessage, ",");
        }else if (Dayptr >= Day){
            strcat(strMessage, "\r\n");
        }
        Dayptr++;
    } 
    sendMessage(strMessage);
}
/*********************************************************************
 * Function: EEProm_Write_Int(int addr, int newData)
 * Input: addr - the location to write to relative to the start of EEPROM
 *        newData - - Floating point value to write to EEPROM
 * Output: none
 * Overview: The value passed by newData is written to the location in EEPROM
 *           which is multiplied by 2 to only use addresses with even values
 *           and is then offset up from the start of EEPROM
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/
void EEProm_Write_Int(int addr, int newData){
    unsigned int offset;
    NVMCON = 0x4004;
    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (2* addr & 0x01ff); // Initizlize lower word of address
    __builtin_tblwtl(offset, newData); // Write EEPROM data to write latch
    asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while(NVMCONbits.WR==1); // Optional: Poll WR bit to wait for
    // write sequence to complete
}
/*********************************************************************
 * Function: int EEProm_Read_Int(int addr);
 * Input: addr - the location to read from relative to the start of EEPROM
 * Output: int value read from EEPROM
 * Overview: A single int is read from EEPROM start + offset and is returned
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/
int EEProm_Read_Int(int addr){
    int data; // Data read from EEPROM
    unsigned int offset;

    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (2* addr & 0x01ff); // Initialize lower word of address
    data = __builtin_tblrdl(offset); // Write EEPROM data to write latch
    return data;
}
/*********************************************************************
 * Function: EEProm_Read_Float(unsigned int ee_addr, void *obj_p)
 * Input: ee_addr - the location to read from relative to the start of EEPROM
 *                  it is assumed that you are referring to the # of the float 
 *                  you want to read.  The first is 0, the next is 1 etc.
 *        *obj_p - the address of the variable to be updated (assumed to be a float)
 * Output: none
 * Overview: A single float is read from EEPROM start + offset.  This is done by
 *           updating the contents of the float address provided, one int at a time
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/


void EEProm_Read_Float(unsigned int ee_addr, void *obj_p)
 {
     unsigned int *p = obj_p;  //point to the float to be updated
     unsigned int offset;
     
     ee_addr = ee_addr*4;  // floats use 4 address locations

     // Read and update the first half of the float
    // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address
    *p = __builtin_tblrdl(offset); // Write EEPROM data to write latch
      // First half read is complete
    
    p++;
    ee_addr = ee_addr+2;
      
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initialize lower word of address
    *p = __builtin_tblrdl(offset); // Write EEPROM data to write latch
      // second half read is complete
      
 }
/*********************************************************************
 * Function: EEProm_Write_Float(unsigned int ee_addr, void *obj_p)
 * Input: ee_addr - the location to write to relative to the start of EEPROM
 *                  it is assumed that you are referring to the # of the float 
 *                  you want to write.  The first is 0, the next is 1 etc.
 *        *obj_p - the address of the variable which contains the float
 *                  to be written to EEPROM
 * Output: none
 * Overview: A single float is written to EEPROM start + offset.  This is done by
 *           writing the contents of the float address provided, one int at a time
 * Note: Library
 * TestDate: 12-26-2016
 ********************************************************************/
 void EEProm_Write_Float(unsigned int ee_addr, void *obj_p)
 {
    unsigned int *p = obj_p;
    unsigned int offset;
    NVMCON = 0x4004;
    ee_addr = ee_addr*4;  // floats use 4 address locations
    
    // Write the first half of the float
     // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initizlize lower word of address
    __builtin_tblwtl(offset, *p); // Write EEPROM data to write latch
     asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while(NVMCONbits.WR==1); // Optional: Poll WR bit to wait for
    // first half of float write sequence to complete
    
    // Write the second half of the float
    p++;
    ee_addr = ee_addr + 2;
    TBLPAG = __builtin_tblpage(&eeData); // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (ee_addr & 0x01ff); // Initizlize lower word of address
    __builtin_tblwtl(offset, *p); // Write EEPROM data to write latch
     asm volatile ("disi #5"); // Disable Interrupts For 5 Instructions
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while(NVMCONbits.WR==1); // Optional: Poll WR bit to wait for
    // second half of float write sequence to complete
    
 }
/*********************************************************************
 * Function: ClearEEProm(void)
 * Input: none
 * Output: none
 * Overview: This function writes a 0 to the first 21 float locations
 *           in EEProm.  It should be called the first time a board
 *           is programmed but NOT every time we Initialize since we don't want 
 *           to lose data saved prior to shutting down because of lost power
 * Note: Library
 * TestDate: 1-5-2017
 ********************************************************************/
void ClearEEProm(void){
    EEFloatData = 0;
    EEProm_Write_Float(0, &EEFloatData);
    EEProm_Write_Float(1, &EEFloatData);
    EEProm_Write_Float(2, &EEFloatData);
    EEProm_Write_Float(3, &EEFloatData);
    EEProm_Write_Float(4, &EEFloatData);
    EEProm_Write_Float(5, &EEFloatData);
    EEProm_Write_Float(6, &EEFloatData);
    EEProm_Write_Float(7, &EEFloatData);
    EEProm_Write_Float(8, &EEFloatData);
    EEProm_Write_Float(9, &EEFloatData);
    EEProm_Write_Float(10, &EEFloatData);
    EEProm_Write_Float(11, &EEFloatData);
    EEProm_Write_Float(12, &EEFloatData);
    EEProm_Write_Float(13, &EEFloatData);
    EEProm_Write_Float(14, &EEFloatData);
    EEProm_Write_Float(15, &EEFloatData);
    EEProm_Write_Float(16, &EEFloatData);
    EEProm_Write_Float(17, &EEFloatData);
    EEProm_Write_Float(18, &EEFloatData);
    EEProm_Write_Float(19, &EEFloatData); 
    EEProm_Write_Float(20, &EEFloatData); 
}

/*********************************************************************
 * Function: delayMs()
 * Input: milliseconds
 * Output: None
 * Overview: Delays the specified number of milliseconds
 * Note: This routine assumes that System clock is 500khz clock  
 *       500khz/2 = 250khz => 4us period for Timer 2
 *       Therefor we need to count 250 for each ms
 *       Actually measuring the time points to the need to have PR = 229 
 * TestDate: 05-20-14
 ********************************************************************/
void delayMs(volatile int ms) 
{
    PR2 = 229;
    while (ms > 0) 
    {
        TMR2 = 0;  //Clear Timer2
        _T2IF = 0; //Clear Timer2 Interrupt Flag
        T2CONbits.TON = 1;  //turn on Timer2
        while(!_T2IF){
        }
        T2CONbits.TON = 0; // turn off Timer2
        ms--;
    }
}


void __attribute__ ((interrupt, auto_psv)) _RTCCInterrupt(void)
{
    _RTCIF = 0; // Clear the interrupt flag
    // Go back to wherever we were executing from - the goal here is
    //  just to sleep for some time
}


void initSleep(void)
{
    //RCFGCALbits.RTCWREN = 1; // enable writing to the RTCC control registers
    asm volatile ("push w7");
    asm volatile ("push w8");
    asm volatile ("disi #5");
    asm volatile ("mov #0x55, w7");
    asm volatile ("mov w7, _NVMKEY");
    asm volatile ("mov #0xAA, w8");
    asm volatile ("mov w8, _NVMKEY");
    asm volatile ("bset _RCFGCAL, #13"); //set the RTCWREN bit
    asm volatile ("pop w8");
    asm volatile ("pop w7");

    RCFGCAL  = 0x2200;
    //RTCPWC   = 0x0400;
    /*RTCPWCbits.RTCCLK = 0b01; */
    ALCFGRPTbits.CHIME = 0;
    //ALCFGRPTbits.AMASK = 0b0000;
    ALCFGRPTbits.ALRMEN = 0;
    RCFGCALbits.RTCEN = 1;
    RCFGCALbits.RTCWREN = 0;
    
    /* Set interrupt priority to lowest available while still being enabled
     Note: This is important! If the interrupt is of a higher or equal
     priority to a CPU interrupt, then the device will generate either
     a hard fault or address fault on wakeup from sleep.
     */
    _RTCIP = 0b001;
    IPC15bits.RTCIP = 1;
    IFS3bits.RTCIF = 0;
    IEC3bits.RTCIE = 1;
}

void sleepForPeriod(sleepLength_t length)
{
    //PMD3bits.RTCCMD = 0;
    asm volatile ("push w7");
    asm volatile ("push w8");
    asm volatile ("disi #5");
    asm volatile ("mov #0x55, w7");
    asm volatile ("mov w7, _NVMKEY");
    asm volatile ("mov #0xAA, w8");
    asm volatile ("mov w8, _NVMKEY");
    asm volatile ("bset _RCFGCAL, #13"); //set the RTCWREN bit
    asm volatile ("pop w8");
    asm volatile ("pop w7");
    
    _RTCEN = 1;
    ALCFGRPTbits.AMASK = length;    // shouldn't it be configured to half a second> 0000
    
    ALCFGRPTbits.ALRMPTR = 0b0010;
    ALRMVAL = 0x0000;
    ALRMVAL = 0x0000;
    ALRMVAL = 0x0000;
    ALCFGRPTbits.ARPT = 0;  
    ALCFGRPTbits.ALRMEN = 1;
    _RTCWREN = 0;
    
    // Go to sleep
    Sleep();
    asm volatile("NOP");
    
    asm volatile ("push w7");
    asm volatile ("push w8");
    asm volatile ("disi #5");
    asm volatile ("mov #0x55, w7");
    asm volatile ("mov w7, _NVMKEY");
    asm volatile ("mov #0xAA, w8");
    asm volatile ("mov w8, _NVMKEY");
    asm volatile ("bset _RCFGCAL, #13"); //set the RTCWREN bit
    asm volatile ("pop w8");
    asm volatile ("pop w7");
    
    _RTCEN = 0;
    _RTCPTR = 0b00; // get ready to set seconds/minutes
    RTCVAL = 0x0000; // set minutes and secs back to 0.
    
    _RTCWREN = 0;
}

void resetCheckRemedy(void)
{
    if(_TRAPR)
    {
        // Trap Reset Conflict has occured
        _TRAPR = 0;
    }
    
    if(_IOPUWR)
    {
        // Illegal Opcode, illegal address mode, or
        //  W reg used as address pointer caused a reset
        _IOPUWR = 0;
    }

   
    if(_DPSLP)
    {
        // Woke up from Deep Sleep Mode
        _DPSLP = 0;
    }
    
    /*if(RCONbits.CM)
    {
        // Configuration word mismatch occurred
        RCONbits.CM = 0;
    }*/
    
    if(_EXTR)
    {
        // External Reset (MCLR Pin) has occurred
        _EXTR = 0;
    }
    
    if(_SWR)
    {
        // Software Reset has occurred
        _SWR = 0;
    }
    
    if(_WDTO)
    {
        // Watchdog timeout reset has occurred
        _WDTO = 0;
    }
    // Using flag in main.c resets there.
//    if(_SLEEP)
//    {
//        // Woke up from sleep mode
//        _SLEEP = 0;
//    }
    
    if(_IDLE)
    {
        // Woke up from idle mode
        _IDLE = 0;
    }
    
    if(_BOR)
    {
        // Brownout caused reset
        _BOR = 0;
    }
    
    if(_POR)
    {
        // Power-up reset has occurred
        _POR = 0;
    }
    
    if(_U1RXIF)
    {
        // There is data to read in the U1RXREG
        _U1RXIF = 0;
    }
}