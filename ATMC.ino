/*
Sets and controls a temperature and acquires and logs temperatures using MAX31856 thermocouple interface chips.

Temperature Control:

Using Timer 1 PWM to drive Pin 3 at 25 kHz with 10 bit dutycycle

Pin Assignments:

PWM out on HEATER_PIN Pin 3
Temperature acquisition LED on Pin 11
MAX31856_CS1 Pin 4 SPI CS for first MAX31856 thermocouple interface
MAX31856_CS2 Pin 5 SPI CS for second MAX31856 thermocouple interface
MAX31856_CS3 Pin 6 SPI CS for third MAX31856 thermocouple interface
MAX31856_CS4 Pin 7 SPI CS for fourth MAX31856 thermocouple interface
MAX31856_CS5 Pin 8 SPI CS for fifth MAX31856 thermocouple interface
MAX31856_CS6 Pin 9 SPI CS for sixth MAX31856 thermocouple interface

Copyright (c) 2017 ATMC.ino 

S&T and the University of Missouri Board of Curators 
license to you the right to use, modify, copy, and distribute this 
code subject to the MIT license:

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. 

Acknoledgements:

I used the MAX31856 SPI code provided by Rx7man on this post:
https://forum.arduino.cc/index.php?topic=390824.0

All other functions used the standard Arduino libraries

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

The authors kindly request that any publications benefitting from the use
of this software include the following citation: 

@Misc{ATMC2017 ,
author =   {Stutts, Daniel S.},
title = {{TempDAQandControl.ino: Arduino Application to Acquire and Control Temperatures }},
howpublished = {\url{https://github.com/dsstutts/ATMC.git}},
year = {2017}}
  
*/
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include "TimerFive.h"
#include <Wire.h>
#include "RTClib.h"
#include <stdarg.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <string.h>

#define BS 8 //Backspace character
#define CR 13// Carrage return
#define LF 10// Line feed
#define DEL 127//Delete character
#define ECHO_//Comment out if you don't want to echo input chars
#define MAXPOINTS 2000 // Maximum number of data points to store in a file
#define MAXFILES 10000 // Maximum number of data files to create
#define HEATER_PIN 3 // PWM output pin
///////// Chip Select Pins //////
#define SDCS 10// SPI CS for SD card
#define MAX31856_CS1 4// SPI CS for first MAX31856 thermocouple interface
#define MAX31856_CS2 5// SPI CS for second MAX31856 thermocouple interface
#define MAX31856_CS3 6// SPI CS for third MAX31856 thermocouple interface
#define MAX31856_CS4 7// SPI CS for fourth MAX31856 thermocouple interface
#define MAX31856_CS5 8// SPI CS for fifth MAX31856 thermocouple interface
#define MAX31856_CS6 9// SPI CS for sixth MAX31856 thermocouple interface
#define NUM_TCs 6 // Number of thermocouples
#define NUM31856REGs 10// Number of special function registers on the MAX31856
#define TYPE_K 0x03
#define TYPE_T 0x07
#define NOP __asm__ __volatile__ ("nop");// Inline no-operation ASM for MAX31856 SPI communication.
#define DATAREAD_LED 11
////////////////////////////////

///////// Globals ////////////.
// These control the data acquisition rate from 200 ms to 1 s:
double updateIntervals[] = {100.0,200.0,300.0,400.0,500.0,600.0,700.0,800.0, 900.0, 1000.0};
double *updateIntPtr = updateIntervals;
volatile char inbuff[200];
volatile char *inbuffPtr = inbuff;
volatile unsigned int InputBufferIndex;
unsigned long numDataPoints = 0;// Counts the number of datapoints acquired
boolean parseCommands = false;
char HourStr[6];
char MinuteStr[6];
char SecondStr[6];
char kpStr[10];
char kiStr[10];
char kdStr[10];
char YearStr[6];
char MonthStr[6];
char DayStr[6];

File DataFile;
String FileName = "";
//char dataString[50];
String dataString = "";
String *dataStringPtr;
String comma = ",";
//byte RegisterValues[] = {0x90,  0x03,   0xFC,   0x7F,   0xC0,   0x07,     0xFF,     0x80,     0x00,     0x00 };//Type K Thermocouple
byte RegisterValues[] =   {0x90,  0x07,   0xFC,   0x7F,   0xC0,   0x07,     0xFF,     0x80,     0x00,     0x00 };// Type T Thermocouple
String RegisterNames[] =  {"CR0", "CR1", "MASK", "CJHF", "CHLF", "LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO"};
byte RegisterAddresses[] = {0x00,  0x01,   0x02,   0x03,   0x04,   0x04,     0x06,     0x07,     0x08,     0x09 };
int CSs[] = {MAX31856_CS1,MAX31856_CS2, MAX31856_CS3, MAX31856_CS4, MAX31856_CS5, MAX31856_CS6};

boolean tellTime = false;// Output time stamp flag
boolean set_Time = false;
boolean set_Hour = false;
boolean set_Minute = false;
boolean set_Second = false;
boolean set_Year = false;
boolean set_Month = false;
boolean set_Day = false;
boolean setGains = false;
boolean setKp = false;
boolean setKi = false;
boolean setKd = false;
boolean setPID = false;
boolean eeGainsSet = false;
boolean reportGains = false;
boolean createFile = false;// Data file creation flag
boolean readTemp = false;
boolean record = false;
boolean ledState = LOW;
boolean saveData = false;
boolean allOff = false;
boolean powerOn = false;
boolean ls = false;// List SD card directory flag
unsigned int Interval = 9;
boolean printAcqRate = false;
boolean setAcqRate = false;
double  acqInterval = 0;
double temp1;
double temp2;
double temp3;
double temp4;
double temp5;
double temp6;
double SetTemp = 50.0;
double ttime = 0;// Temperature time
double Ts = 200.0;// Default data acquisition rate.
double Err[] = {0.0, 0.0, 0.0};//current error, last error, error before last 
double derr=0.0, ierr=0.0;
double Kp = 200.0;
double DC = 0.0;
int iDC = 0;
boolean KpSet = false;

//double Ki = 2.0;
double Ki = 10.0;
boolean KiSet = false;

//double Kd = 120.0;
double Kd = 60.0;
boolean KdSet = false;
boolean pidUpdate = false;
unsigned int NumDataPoints = 0;
unsigned long writeSDtime = 0;
boolean PowerOn = false;
unsigned int i, j, numPts = 0;

volatile char InChar = '\0'; // serial input character
char myChar;
boolean Help = false;

const char fileCreateerror[] PROGMEM = "Couldn't create file\n";//Store in program memory
const char HelpText[] PROGMEM = {"THC supports the following commands:\r\n \\
  A -- Control, acquire and store data\r\n \\
  a -- Stop everything and save data; wait until restart.\r\n \\
  h -- List of supported commands\r\n \\
  o -- Turn power off\r\n \\
  ss -- Print acquisition rate\r\n \\
  s# -- Set acquisition rate\r\n \\
  s# -- Set data acquisition interval where # is an integer option denoting:\r\n \\
    0 --> .1 sec (not implemented) \r\n \\  
    1 --> .2 sec\r\n \\
    2 --> .3 sec\r\n \\
    3 --> .4 sec\r\n \\
    4 --> .5 sec\r\n \\
    5 --> .6 sec\r\n \\
    6 --> .7 sec\r\n \\
    7 --> .8 sec\r\n \\
    8 --> .9 sec\r\n \\
    9 --> 1.0 sec\r\n \\
  T# -- Set the x = 0 boundary temperature to # degrees C (currently not implemented)\r\n \\
  thms -- set time where h,m, and s are hours, minutes, seconds integers \r\n \\
  tt -- Show the current time.\r\n\ \\
  gg -- Report current PID gains.\r\n \\
  gp#i#d# -- Set PID gains, where # denotes floating point numbers.\r\n \\
  Note that while you may set one or two at a time, you must enter the PID gains and times in order!\r\n \\
  Z -- Not implemented.\r\n\ \\
  W -- Not implemented.\r\n"};
//12 byte PID gains structure:
struct PID_Gains{
  double Kp;
  double Ki;
  double Kd;
};
unsigned int Year;
byte Month;
byte Day;
byte Hour;
byte Minute;
byte Second;
struct PID_Gains pidGains;
///////// EEPROM Addresses //////
byte eeGainSetAddr = 0;// 1 byte
byte eepidGainsAddr = 1;// 12 bytes
byte eeAcqRateAddr = 13;// 2 bytes
/////////////////////////////////
////////////////// End of Global Variables ////////

////////// Object Instantiations //////////////////
//SD card objects:
Sd2Card card;
SdVolume volume;
SdFile root;
File logfile;
// Real time clock object
RTC_PCF8523 rtc;
////////////// End of Object Instantiations //////////

////////////// Function Definitions ////////////

///////// Interrupt Service Routines (ISRs) ///////////////
void serialEvent()
{
  while(Serial.available())
  {
  
    InChar = Serial.read();

    if((InChar==DEL)&&(InputBufferIndex!=0))
    {
      inbuff[--InputBufferIndex] = '\0';//Replace the erroneous byte with a NUL.
      Serial.print(BS);//Echo backspace character.
    }
    else
    {
    inbuff[InputBufferIndex] = InChar;
    InputBufferIndex++;
    }
    if(InChar == '\n' or InChar == 0xD)
    {
      parseCommands = true;
      InputBufferIndex = 0;
    }
  }
}
void ReadData(void)
{
  if(ledState) ledState = false;
  else
  ledState = true;
  digitalWrite(DATAREAD_LED, ledState);
  readTemp = true;
}
///////////// End of ISR definitions ////////

////////// MAX31856 Functions ////////////////
 byte ReadSingleRegister(int Pin, byte Register) {
    digitalWrite(Pin, LOW);
    NOP;
    SPI.transfer(Register & 0x7F); //set bit 7 to 0 to ensure a read command
    NOP;
    byte data = SPI.transfer(0);
    digitalWrite(Pin, HIGH);
    return data;
  }

  unsigned long ReadMultipleRegisters(int Pin, byte StartRegister, int count) {
    //reads up to 4 sequential registers
    digitalWrite(Pin, LOW);
    unsigned  long data = 0;
    SPI.transfer(StartRegister & 0x7F); //force bit 7 to 0 to ensure a read command
    NOP;

    for (int i = 0; i<count; i++){
      data = (data<<8) | SPI.transfer(0);  //bitshift left 8 bits, then add the next register
    }
    digitalWrite(Pin, HIGH);
    return data;
  }

  void WriteRegister(int Pin, byte Register, byte Value) {
    byte Address = Register | 0x80; //Set bit 7 high for a write command
    digitalWrite(Pin, LOW);
    NOP;
    SPI.transfer(Address);
    NOP;
    SPI.transfer(Value);
    digitalWrite(Pin, HIGH);
  }

  double ReadColdJunction(int Pin){
    
    double temperature;

    long data, temperatureOffset;

    data = ReadMultipleRegisters(Pin, 0x08, 4);

    // Register 9 is the temperature offset
    temperatureOffset = (data & 0x00FF0000) >> 16;

    // Is this a negative number?
    if (temperatureOffset & 0x80)
    temperatureOffset |= 0xFFFFFF00;

    // Strip registers 8 and 9, taking care of negative numbers
    if (data & 0x8000)
    data |= 0xFFFF0000;
    else
    data &= 0x0000FFFF;

    // Remove the 2 LSB's - they aren't used
    data = data >> 2;

    // Add the temperature offset to the temperature
    temperature = data + temperatureOffset;

    // Convert to Celsius
    temperature *= 0.015625;
    

    // Return the temperature
    return (temperature);


  }
  double ReadTemperature(int Pin){

    double temperature;
    long data;

    data = ReadMultipleRegisters(Pin,0x0C, 4);

    // Strip the unused bits and the Fault Status Register
    data = data >> 13;

    // Negative temperatures have been automagically handled by the shift above :-)

    // Convert to Celsius
    temperature = (double) data * 0.0078125;
    

    // Return the temperature
    return (temperature);
  }

void initializeMAX31856Pins() {
  Serial.print("Initializing SPI chip-select pins");
  for (int i = 0; i < NUM_TCs; i++) {
    Serial.print(", ");
    Serial.print(CSs[i]);
    pinMode(CSs[i], OUTPUT);
    digitalWrite(CSs[i], HIGH);
  }
  Serial.println("  Done");
}

void InitializeChannel(int Pin) {
  Serial.print("Initializing channel on pin ");
  Serial.println(Pin);
  for (int i = 0; i < NUM31856REGs; i++) {
    WriteRegister(Pin, i, RegisterValues[i]);
  }
  //Serial.print("Finished in Initialize Channel\n");  
}

void VerifyData(int CS) {
    int ErrorCount = 0; 
  for (int i = 0; i < NUM31856REGs; i++) {
    byte RegVal = ReadSingleRegister(CS, i);
    if(RegVal != RegisterValues[i]){
        Serial.print(RegisterNames[i]);
    Serial.print("\t has 0x");
    Serial.print(RegVal, HEX);
    Serial.print(" and should have 0x");
    Serial.println(RegisterValues[i],HEX);
    ErrorCount++;
    }
  }
  if (ErrorCount == 0){
    Serial.println("No discrepancies found");
  }
}

////////// End of MAX31856 Functions /////////
void printGains(void){
  Serial.print("Kp = ");
  Serial.print('\t');
  Serial.print(pidGains.Kp);
  Serial.print('\t');
  Serial.print("Ki = ");
  Serial.print('\t');
  Serial.print(pidGains.Ki);
  Serial.print('\t');
  Serial.print("Kd = ");
  Serial.print(pidGains.Kd);
  Serial.print('\n');
}

void file_err(void)
{

 int errLength = strlen_P(fileCreateerror);
    for(j = 0; j < errLength; j++)
    {
      myChar = pgm_read_word_near(fileCreateerror + j);
      Serial.print(myChar);
    } 
    Timer3.pwm(HEATER_PIN, 0);//Set DC to zero!
    while(1);//Stop here.
}

void PID_Control(void)
{  
  
  //Err[0] = temp1 - SetTemp;//If tracking a cold temp on a Peltier cooler. 
  Err[0] = SetTemp - temp1;
  derr = Err[0]-Err[1];
  ierr = ierr + Err[0];
  if (ierr >= 250.0) ierr = 250.0;// Saturate integral error for anti-windup.
  if (ierr <= -250.0) ierr = -250.0;
  Err[1] = Err[0];
  DC = pidGains.Kp*Err[0]+pidGains.Ki*ierr*Ts+pidGains.Kd*derr/Ts;
  iDC = (int)DC;// Cast to int
  if(iDC >= 1023)iDC = 1023;
  if(iDC <= 0)iDC = 0;
  Timer3.pwm(HEATER_PIN, iDC);//Set DC
}

void WriteToSD(void)
{
      

    dataString +=ttime+comma+temp1+comma+temp2+comma+temp3+comma+temp4+comma+temp5+comma+temp6;
    logfile.println(dataString);
    dataString = "";
     
//  if (logfile.writeError || !logfile.sync()) {
//    error("write header");
//  }
}

//////////////Parser//////////
void parseSerialInput(void) {
    boolean NEG = 0;
    boolean hourSet = false;
    boolean minuteSet = false;
    boolean secondSet = false;
    boolean monthSet = false;
    boolean daySet = false;
    boolean yearSet = false;
    boolean kpSet = false;
    boolean kiSet = false;
    boolean kdSet = false;
    char dataStr[2] = {'\0','\0'};
    unsigned int data = 0;
    double SetTemp;
    byte i = 0;
    byte k = 0;
    byte l = 0;
    byte j = 0;
    byte o = 0;
    byte q = 0;
    byte p = 0;
//
// Read the first command character and parse accordingly:
//

if(*inbuffPtr == 'h'){// Print help string.
  Help = true;
  return;
}

if(*inbuffPtr == 'o'){// Turn off power, but still log data.
  powerOn = false;
  return;
}

if(*inbuffPtr == 'A'){//Start controlling temperature and saving data.
   saveData = true;
   powerOn = true;
   pidUpdate = true;
   readTemp = true;
   createFile = true;
   numDataPoints = 0;
   ttime = 0.0;
   return;
}

if(*inbuffPtr == 'a'){//Stop everything and save data.
   saveData = false;
   allOff = true;
   return;
}

if(*inbuffPtr == 's'){
   *inbuffPtr++;//increment pointer to second character
   if(*inbuffPtr++ == 's') {//Set print settings flag and return.
    printAcqRate = true;
    return;
  }
  else
  {
    *inbuffPtr--;// Decrement pointer
    setAcqRate = true;
     while ((*inbuffPtr != '\0')) {
        if ((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) {// Make sure it's numeric!        
            dataStr[0] = *inbuffPtr;
        }
        *inbuffPtr++;//Increment buffer pointer.
        i++;
     }
     Interval = (unsigned int)atoi(dataStr);//Prevent invalid intervals.
   }//End else
}// End if s

// Time functions:
if(*inbuffPtr=='t'){//Time functions
  *inbuffPtr++;//increment pointer to second character
  if(*inbuffPtr++=='t') {//Tell time and return
    tellTime = true;
    return;
  }
  else
  {
  *inbuffPtr--;// Decrement pointer
    set_Time = true;

    while ((*inbuffPtr != '\0')) {
        switch (*inbuffPtr) {
            case 'Y':
            case 'y':
                yearSet = true;
                set_Year = true;
                break;
            case 'R':
            case 'r':
                monthSet = true;
                set_Month = true;
                break;
            case 'D':
            case 'd':
                daySet = true;
                set_Day = true;
                break;
            case 'h':
            case 'H':
                hourSet = true;
                set_Hour = true;
                break;
            case 'M':
            case 'm':
                minuteSet = true;
                set_Minute = true;
                break;

            case 'S':
            case 's':
                secondSet = true;
                set_Second = true;
                break;
            default: 
    if ((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) {// Make sure they're numeric!
 
            if(secondSet&&i<6){
                SecondStr[i] = *inbuffPtr;
                yearSet = false;
                monthSet = false;
                daySet = false;
                hourSet = false;
                minuteSet = false;
                i++;
            }
            else if (i>=6||(atoi(SecondStr)>59))//Trap some potential errors.
            {
              Serial.print("Erroneous second setting!\n");
              set_Time = false;
              return;
            }
            if(minuteSet&&j<6){
                MinuteStr[j] = *inbuffPtr;
                yearSet = false;
                monthSet = false;
                daySet = false;
                hourSet = false;
                secondSet = false;
                j++;
            }
            else if (j>=6||(atoi(MinuteStr)>59))
            {
              Serial.print("Erroneous minute setting!\n");
              set_Time = false;
              return;
            }
            if(hourSet&&(atoi(HourStr)<25)&&k<6){
                HourStr[k] = *inbuffPtr;
                yearSet = false;
                monthSet = false;
                daySet = false;
                minuteSet = false;
                secondSet = false;
                k++;
            }
            else if (k>=6||(atoi(HourStr)>24))
            {
              Serial.print("Erroneous hour setting!\n");
              set_Time = false;
              return;
            } 
            if(daySet&&(atoi(DayStr)<32)&&p<6){
              DayStr[p] = *inbuffPtr;
              yearSet = false;
              monthSet = false;
              hourSet = false;
              minuteSet = false;
              secondSet = false;
              p++;
            }
            else if (p>=6||(atoi(DayStr)>31))
            {
              Serial.print("Erroneous day setting!\n");
              set_Time = false;
              return;
            }
            if(monthSet&&(atoi(MonthStr)<13)&&o<6){
              MonthStr[o] = *inbuffPtr;
              yearSet = false;
              daySet = false;
              hourSet = false;
              minuteSet = false;
              secondSet = false;
              o++;
            }
            else if (o>=6||(atoi(MonthStr)>12))
            {
              Serial.print("Erroneous month setting!\n");
              set_Time = false;
              return;
            }
            if(yearSet&&(atoi(YearStr)<100)&&q<6){
              YearStr[q] = *inbuffPtr;
              monthSet = false;
              daySet = false;
              hourSet = false;
              minuteSet = false;
              secondSet = false;
              q++;
            }
            else if (q>=6||(atoi(YearStr)>99))
            {
              Serial.print("Erroneous year setting!\n");
              set_Time = false;
              return;
              
              
              
            }
    }
     break;
     }//End of main switch
        //i++;// Increment input buffer index if not using pointer arithmetic
        *inbuffPtr++;//Increment the buffer pointer (this generates a warning from
                      // the gcc compiler because the result is not assigned to
                      // a variable.)
   }//End of while
  }// End of 'tt' else clase
 }// End of if 't' clause
 // End of time functions.
    
// PID control gain setting:
  if(*inbuffPtr=='g') {//Control Setting case
    *inbuffPtr++;
      
    if(*inbuffPtr++=='g') {//Set report_Gains and return
          reportGains = true;
          return;
      }
    else
    {
      *inbuffPtr--;// Decrement pointer
      setGains = true;
      while (*inbuffPtr != '\0') {
   
    switch (*inbuffPtr) {
      case 'p':
        setKp = true;
        kpSet = true;
      break;
      case 'i':
        setKi = true;
        kiSet = true;
      break;
      case 'd':
        setKd = true;
        kdSet = true;
      break;
      default:
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9'))||(*inbuffPtr=='.')) {// Make sure they're numeric!
      if(kdSet){
        kdStr[k] = *inbuffPtr;
        kpSet = false;
        kiSet = false;
        k++;
      }
      if(kiSet){
        kiStr[j] = *inbuffPtr;
        kpSet = false;
        kdSet = false;
        j++;
      }
      if(kpSet){
      
        kpStr[i] = *inbuffPtr;
        kiSet = false;
        kdSet = false;
        i++;
      }
    }//End if numeric
      break;
   }//End PID switch
    *inbuffPtr++;
  }// End PID while
 } // End of if cc
}// End of if c
    
}// End of parseSerialInput

////////////End of Parser////

void setup()
{
  
  Ts = updateIntervals[1]/1000.0;// Set default sampling rate.
  // put your setup code here, to run once:
  pinMode(DATAREAD_LED, OUTPUT);//Set the data-read activity LED pin to output.
  digitalWrite(DATAREAD_LED,false);
  
  //delay(100);
 
  InputBufferIndex = 0;
  //Timer1.initialize(200000);// 200 ms 
  Timer1.initialize(((long)updateIntervals[1])*1000);// Set default update interval.
  Timer1.attachInterrupt(ReadData);
  Timer3.initialize(40); // 40 us => 25 kHz PWM frequency
  Timer3.pwm(HEATER_PIN, 0);//Start with zero duty cycle
  //Timer5.initialize(500000); // May use later...
  //Timer5.attachInterrupt(callback function of some use...);
  while (!Serial); // for Leonardo/Micro/Zero
   Serial.begin(250000);
  //Serial.begin(2000000);
   //Serial.begin(500000);
  //Serial.begin(1000000);// Works!
  //Serial.begin(2000000);// Works too!  Holy Cow!
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  //SPI.setDataMode(SPI_MODE0);// SPI Mode 0 for SD card.
  
  Serial.println("READY!  \n");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.initialized()) 
  {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   }

    Serial.print("\nInitializing SD card...");
    if(!card.init(SPI_FULL_SPEED, SDCS)) {  //Try full speed first...
  //if (!card.init(SPI_HALF_SPEED, SDCS)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the SDCS pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }
   
  if(!SD.begin(SDCS))
  {
    Serial.println("SD card failed or not present.  ");
    //while(1);//Might as well stop here...
  }
  else
  Serial.println("SD card initialized.");

  if (!volume.init(card)) 
  {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card\n");
    return;
  }
  SPI.end();
  byte SPIERROR = SPSR;//Read SPI status reg to clear errors; doesn't work.
  delay(100);
  SPI.setClockDivider(SPI_CLOCK_DIV2);//Reset to 7.8 MHz and Mode 3 for MAX31856
  //SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE3);
   Serial.print("Initalizing Pins\n");
  initializeMAX31856Pins();
   for (int i = 0; i < NUM_TCs; i++) { //usually done for each channel..
    InitializeChannel(CSs[i]);     
    delay(10);
    VerifyData(CSs[i]); 
  }
 //noInterrupts();//The EEPROM driver apparently code uses interrupts, 
  EEPROM.get(eeGainSetAddr, eeGainsSet);// so you can't disable all of them.
 if(eeGainsSet = 1){
  EEPROM.get(eepidGainsAddr, pidGains);
 }
 Serial.print("eeGainsSet = :");
 Serial.print(eeGainsSet);
 Serial.print("\n");
 EEPROM.get(eeAcqRateAddr, Interval);
 //interrupts();
 Serial.print("Kp = :");
 Serial.print(pidGains.Kp);
 Serial.print("\n");
 Serial.print("Ki = :");
 Serial.print(pidGains.Ki);
 Serial.print("\n");
 Serial.print("Kd = :");
 Serial.print(pidGains.Kd);
 Serial.print("\n");
 EEPROM.get( eeAcqRateAddr, Interval);
 Serial.print(Interval);
 Serial.print("\n");
 Ts = updateIntervals[Interval]/1000.0;// Set default sampling rate.
 //Timer1.initialize(200000);// 200 ms 
 Timer1.initialize(((long)updateIntervals[Interval])*1000);// Set default update interval.
 Timer1.attachInterrupt(ReadData);
 Timer3.initialize(40); // 40 us => 25 kHz PWM frequency
 Timer3.pwm(HEATER_PIN, 0);//Start with zero duty cycle
 delay(100);
}
///////////// End setup ////////////
///////////// Main Loop ////////////
void loop()
{
 // All other function calls should occur here...
 
  if(parseCommands)
  {
    parseSerialInput();
    for(i = 0; i < sizeof(inbuff); i++)// Flush buffer.
    {
      inbuff[i] = '\0';
    }
    inbuffPtr = inbuff;// Point the input string pointer back to the beginning.
    parseCommands = false;
  }

  if(setAcqRate){
    Timer1.initialize(((long)updateIntervals[Interval])*1000);
    Ts = updateIntervals[Interval]/1000.0;// Set default sampling rate.
    EEPROM.put(eeAcqRateAddr, Interval);// Save setting.
    setAcqRate = false;
  }

  if(Help)
  {
    int HelpLength = strlen_P(HelpText);
    for(j = 0; j < HelpLength; j++)
    {
      myChar = pgm_read_byte_near(HelpText + j);
      Serial.print(myChar);
    }
    Help = false;
  }

  if(ls)
  {
    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
     root.openRoot(volume);

  // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
    ls = false;
  }

  if(printAcqRate){
    acqInterval = updateIntervals[Interval];
    Serial.print("Acqisition Interval = ");
    Serial.print(acqInterval);
    Serial.print(" mS");
    Serial.print("\n");
    printAcqRate = false;
  }
  if(reportGains){
    printGains();
    reportGains = false;
  }
  if(setGains){
   
    if(setKp){
       pidGains.Kp = atof(kpStr);
      for (j = 0; j<sizeof(kpStr); j++)kpStr[j] = '\0'; //Flush the gain buffers
      setKp = false;
   }
   if(setKi){
      pidGains.Ki = atof(kiStr);
      for (j = 0; j<sizeof(kiStr); j++)kiStr[j] = '\0';
      setKi = false;
   }
   if(setKd){
    pidGains.Kd = atof(kdStr);
    for (j = 0; j<sizeof(kdStr); j++)kdStr[j] = '\0';
    setKd = false;
   }
   noInterrupts();//Don't let this transaction be interrupted!
   EEPROM.put(eepidGainsAddr, pidGains);
   EEPROM.put(eeGainSetAddr, 1);//Store the fact that the gains are stored in EPROM.
   interrupts();
    setGains = false;
  }

  if(createFile&&saveData)
  {
   SPI.setDataMode(SPI_MODE0);//Switch to SPI_MODE0 for SD access.
    // Create a new file
   char filename[] = "LOGGER00.CSV";
   DateTime now = rtc.now();//This call must occur before noInterrupts()!
   noInterrupts();
  for (i = 0; i < MAXFILES; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
   
      if (!SD.exists(filename)) 
      {
      // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        logfile.print(now.year(), DEC);
        logfile.print('/');
        logfile.print(now.month(), DEC);
        logfile.print('/');
        logfile.print(now.day(), DEC);
        logfile.print('_');
        logfile.print(now.hour(), DEC);
        logfile.print(':');
        logfile.print(now.minute(), DEC);
        logfile.print(':');
        logfile.print(now.second(), DEC);
        logfile.println();
        logfile.print('\n');
        break;  // leave the loop!
      }
       
    }
  
  if (!logfile) 
    {
      file_err();
    }
    createFile = false;
    SPI.setDataMode(SPI_MODE3);//Reset to MAX31856 SPI mode.
    interrupts();
  }

  if(tellTime)
  {
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('_');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    Serial.print('\n');
    tellTime = false;
  }

  if(set_Time){
    // Currently we can only set hour, minute and second...
    DateTime now = rtc.now();
    if(set_Year){
      rtc.adjust(DateTime(atoi(YearStr), now.month(), now.day(), now.hour(), now.minute(), now.second()));
      set_Year = false;
    } 
    if(set_Month){
      rtc.adjust(DateTime(now.year(),atoi(MonthStr), now.day(), now.hour(), now.minute(), now.second()));
      set_Month = false;
    }
    if(set_Day){
      rtc.adjust(DateTime(now.year(), now.month(),atoi(DayStr), now.hour(), now.minute(), now.second()));
      set_Day = false;
    }
    if(set_Hour){
      rtc.adjust(DateTime(now.year(), now.month(), now.day(), atoi(HourStr), now.minute(), now.second()));
      set_Hour = false;
    }
    if(set_Minute){
     // rtc.adjust(DateTime(Year, Month, Day, Hour, atoi(MinuteStr), now.second()));
     rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), atoi(MinuteStr), now.second()));
      set_Minute = false;
    }
    if(set_Second){
      rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), atoi(SecondStr)));
      set_Second = false;
    }
    set_Time = false;   
  }

  if(readTemp)
  {
     temp1 = ReadTemperature(CSs[0]);
     temp2 = ReadTemperature(CSs[1]);
     temp3 = ReadTemperature(CSs[2]);
     temp4 = ReadTemperature(CSs[3]);
     temp5 = ReadTemperature(CSs[4]);
     temp6 = ReadTemperature(CSs[5]);
 
    if(temp1>=90){  // Shut down if control temp > 90 degrees C.
      Timer3.pwm(HEATER_PIN, 0);//Set DC to zero!
      noInterrupts();
      logfile.close();
      while(1);//Stop here now!
    }
   //  if(record){ // May cache data for block (512 byte) write later...
       //TC1_temp[numDataPoints] = temp1;
       //TC2_temp[numDataPoints] = temp2;
   //  }//Always print the data out for now...
   //  else{
       Serial.print(ttime);
       Serial.print('\t');
       Serial.print(temp1);
       Serial.print('\t');
       Serial.print(temp2);
       Serial.print('\t');
       //Serial.print(DC);
       Serial.print(temp3);
        Serial.print('\t');
        Serial.print(temp4);
        Serial.print('\t');
        Serial.print(temp5);
        Serial.print('\t');
        Serial.print(temp6);
       Serial.print('\n');
  //     }
     
     numDataPoints++;
     readTemp = false;
     pidUpdate = true;

     if(saveData){
        noInterrupts();
        SPI.setDataMode(SPI_MODE0);
        WriteToSD();
        interrupts();
     }
     SPI.setDataMode(SPI_MODE3);
     ttime = ttime + Ts;
  }
 if(powerOn){//Safety wrapper around PWM output
  if(pidUpdate&&saveData)
  {
    PID_Control();
    pidUpdate = false;
  }
 }
 else
 Timer3.pwm(HEATER_PIN, 0);//Set DC to zero!
 
  if(allOff)
  {
    powerOn = false;
    saveData = false;
    Timer3.pwm(HEATER_PIN, 0);//Set DC to zero!
    noInterrupts();
    SPI.setDataMode(SPI_MODE0);
    logfile.close();
    while(allOff);//Stop here now!
  }
}
////////////  End Main Loop /////////////////



