#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Wire.h>
//#include "RTClib.h"
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
#define ECHO_//Comment out if you don't want to echo input chars to the serial monitor
#define MAXPOINTS 2000 // Maximum number of data points to store in a file
#define MAXFILES 10000 // Maximum number of data files to create

///////// Chip Select Pins //////
#define SDCS 53// SPI CS for SD card
#define HEATER_PIN_A 3 // PWM output pin 1
#define HEATER_PIN_B 2 // PWM output pin 2
//#define HEATER_PIN_C 4 // PWM output pin 3

#define TC1 16 // SPI CS for first MAX31856 thermocouple interface
#define TC2 14
#define TC3 5
#define TC4 7
#define TC5 22
#define TC6 24
#define TC7 26
#define TC8 28
#define TC9 30
#define TC10 35
#define TC11 37
#define TC12 33
#define TC13 41
#define TC14 43
#define TC15 39
#define TC16 47
#define TC17 49
#define TC18 45
#define NUM_TCs 18 // Number of thermocouples
int CSs[] = {TC1,TC2,TC3,TC4,TC5,TC6,TC7,TC8,TC9,TC10,TC11,TC12,TC13,TC14,TC15,TC16,TC17,TC18};
#define NUM_TCs_Sensing 18-NUM_TCs // Number of thermocouples
int CSs_sensing[] = {};

double temp[NUM_TCs];
double temp2[NUM_TCs];
#define NUM31856REGs 10// Number of special function registers on the MAX31856
#define TYPE_K 0x03
#define TYPE_T 0x07
#define NOP __asm__ __volatile__ ("nop");// Inline no-operation ASM 
#define DATAREAD_LED 11//     for inserting a 62.5 ns delay used  for MAX31856
//                            SPI communication and for H-bridge deadtime. 
//The following executes 10 NOPs in a row for a 625 ns delay:
#define NOP10 __asm__ __volatile__ ("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"\
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
int DRArray[18], DRnum=0;     //DataReady
////////////////////////////////

///////// Globals ////////////
// These control the data acquisition rate from 200 ms to 6 s:
double updateIntervals[] = {100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0,                                              
800.0, 200.0, 1000.0, 4000.0};
double *updateIntPtr = updateIntervals;
volatile char inbuff[200];
volatile char *inbuffPtr = inbuff;
volatile unsigned int InputBufferIndex;
unsigned long numDataPoints = 0;// Counts the number of datapoints acquired
boolean parseCommands = false;
boolean monitorData = true;
char dcStr[5];
char tempStr[10];
String *dcStrPtr;
char kpStr[10];
char kiStr[10];
char kdStr[10];
String FileName = "";
//char dataString[100];
String dataString = "";
String *dataStringPtr;
String comma = ",";
// Shift register configuration for chips internal cold junction compensation
/*
//byte RegisterValues[] = {0x90,  0x03,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };//Type K Thermocouple
byte RegisterValues[] =   {0x90,  0x07,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };// Type T Thermocouple
String RegisterNames[] =  {"CR0", "CR1", "MASK", "CJHF", "CHLF", \
"LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO"};
byte RegisterAddresses[] = {0x00,  0x01,   0x02,   0x03,   0x04,   0x05,     \
0x06,     0x07,     0x08,     0x09 };
*/
// Shift register configuration for writing cold junction compensation. Set cold junction temp to zero. Speeds up conversion by 25ms.
//byte RegisterValues[] = {0x90,  0x03,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };//Type K Thermocouple
byte RegisterValues[] =   {0x98,  0x07,   0xFC,   0x7F,   0xC0,   0x7F,     \
0xFF,     0x80,     0x00,     0x00 ,     0x00,     0x00};// Type T Thermocouple
String RegisterNames[] =  {"CR0", "CR1", "MASK", "CJHF", "CHLF", \
"LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO", "CJTH","CJTL"};
byte RegisterAddresses[] = {0x00,  0x01,   0x02,   0x03,   0x04,   0x05,     \
0x06,     0x07,     0x08,     0x09,     0x0A,     0x0B};
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
boolean saveData = false;
boolean allOff = false;
boolean controlErr1 = false;
boolean powerOn = false;
boolean ls = false;// List SD card directory flag
unsigned int Interval = 4;//default Interval
boolean printAcqRate = false;
boolean setAcqRate = false;
double  acqInterval = 0;
double ttime = 0;// Temperature time
double Ts = 200.0;// Default data acquisition rate.
double Err[] = {0.0, 0.0, 0.0};//current error, last error, error before last
double derr = 0.0, ierr = 0.0;
double setTemp = 50.0;// Default target temperature
double DCset = 0;

unsigned int NumDataPoints = 0;
unsigned long writeSDtime = 0;
boolean openLoop = false;//Open loop flag
boolean logData = false;//Start saving data flag
unsigned int i, j, numPts = 0;
double Kp = 200.0;
double DC = 0.0;
int iDC = 0;
int fanDC = 0;
boolean KpSet = false;

//double Ki = 2.0;
double Ki = 10.0;
boolean KiSet = false;

//double Kd = 120.0;
double Kd = 60.0;
boolean KdSet = false;
boolean pidUpdate = false;

boolean PowerOn = false;
boolean setDC = false;//Manual open loop duty cycle setting flag.
boolean controlOn = false;//PID control flag.
volatile char InChar = '\0'; // serial input character
char myChar;
boolean Help = false;

const char fileCreateerror[] PROGMEM = "Couldn't create file\n";//Store in program memory
const char controlError1[] PROGMEM = "Invalid control Interval\n";
const char HelpText[] PROGMEM = {"THC supports the following commands:\r\n \\
  A -- Control, acquire and store data\r\n \\
  a -- Stop everything and save data; wait until restart.\r\n \\
  h -- List of supported commands\r\n \\
  L -- Log data; use 'a' to stop and save.\r\n \\
  o -- Turn power off\r\n \\
  ss -- Print acquisition rate\r\n \\
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
    10 --> 4 sec\r\n \\
  T# -- Set the x = 0 boundary temperature to # degrees C, turn power on, and log data\r\n \\
  q# -- Set the boundary temperature to # degrees C\r\n \\
  th#m#s# -- set time where h,m, and s are hours, minutes, seconds and #, integers \r\n \\
  ty## -- set year where ## are the last two digits \r\n \\
  tr## -- set month where ## are the digits of the month \r\n \\
  td## -- set day \r\n \\
  tt -- Show the current time.\r\n\ \\
  gg -- Report current PID gains.\r\n \\
     gp#i#d# -- Set PID gains, where # denotes floating point numbers.\r\n \\
     Note that while you may set one or two at a time, you must enter the PID \
     gains and times in order!\r\n \\
z -- Monitor data by printing to the serial monitor -- toggle logic.\r\n\ \\
W#### -- Simultaneously set duty cycle and log data, where #### represents the \
duty cycle from 0 to 1023.\r\n"
};
///////// EEPROM Addresses //////
struct PID_Gains {
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
// Total EEPROM bytes stored: 15 so far...
/////////////////////////////////

////////////////// End of Global Variables ////////

////////// Object Instantiations //////////////////
//SD card objects:
Sd2Card card;
SdVolume volume;
SdFile root;
File logfile;
////////////// End of Object Instantiations //////////
void getValueWithDataReady(int pin1, int dr1, int pin2=666, int dr2=666, int pin3=666, int dr3=666, int pin4=666, int dr4=666);   
///////// Interrupt Service Routines (ISRs) ///////////////
void serialEvent()
{
  while (Serial.available())
  {

    InChar = Serial.read();

    if ((InChar == DEL) && (InputBufferIndex != 0))
    {
      inbuff[--InputBufferIndex] = '\0';//Replace the erroneous byte with a NUL.
      Serial.print(BS);//Echo backspace character.
    }
    else
    {
      inbuff[InputBufferIndex] = InChar;
      InputBufferIndex++;
    }
    if (InChar == '\n' or InChar == 0xD)
    {
      parseCommands = true;
      InputBufferIndex = 0;
    }
  }
}
void ReadData(void)
{
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

  for (int i = 0; i < count; i++) {
    data = (data << 8) | SPI.transfer(0); //bitshift left 8 bits, 
  }//                                     then add the next register
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

double ReadColdJunction(int Pin) {

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

double ReadTemperature(int Pin) {
  double temperature;
  long data;

  data = ReadMultipleRegisters(Pin, 0x0C, 4);

  // Strip the unused bits and the Fault Status Register
  data = data >> 13;

  // Negative temperatures have been automagically handled by the shift above :-)

  // Convert to Celsius
  temperature = (double) data * 0.0078125;


  // Return the temperature
  return (temperature);
}

void initializeMAX31856Pins() {
  Serial.print("Initializing SPI chip-select pins\n");
  Serial.print("Initializing Sensing pins\n");
  for (int i = 0; i < NUM_TCs; i++) {
    Serial.print(", ");
    Serial.print(CSs[i]);
    pinMode(CSs[i], OUTPUT);
    digitalWrite(CSs[i], HIGH);
  }
  Serial.println("  Done");
  Serial.print("Initializing NonSensing pins\n");
  for (int i = 0; i < NUM_TCs_Sensing; i++) {
    Serial.print(", ");
    Serial.print(CSs_sensing[i]);
    pinMode(CSs_sensing[i], OUTPUT);
    digitalWrite(CSs_sensing[i], HIGH);
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
    if (RegVal != RegisterValues[i]) {
      Serial.print(RegisterNames[i]);
      Serial.print("\t has 0x");
      Serial.print(RegVal, HEX);
      Serial.print(" and should have 0x");
      Serial.println(RegisterValues[i], HEX);
      ErrorCount++;
    }
  }
  if (ErrorCount == 0) {
    Serial.println("No discrepancies found");
  }
}

////////// End of MAX31856 Functions /////////
void printGains(void) {
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
  for (j = 0; j < errLength; j++)
  {
    myChar = pgm_read_word_near(fileCreateerror + j);
    Serial.print(myChar); delay(1000);
  }
  Timer3.pwm(HEATER_PIN_A, 0);//Set DC to zero!
  Timer3.pwm(HEATER_PIN_B, 0);//Set DC to zero!
  //Timer3.pwm(HEATER_PIN_C, 0);//Set DC to zero!
  while (1); //Stop here.
}

void stopAll()
{
    Timer3.pwm(HEATER_PIN_A, 0);//Set DC to zero!
    Timer3.pwm(HEATER_PIN_B, 0);//Set DC to zero!
    //Timer3.pwm(HEATER_PIN_C, 0);//Set DC to zero!
    noInterrupts();
    SPI.setDataMode(SPI_MODE0);
    for (int i = 0; i < NUM_TCs; i++) {
    digitalWrite(CSs[i], HIGH);
    }
    logfile.close();
    while (allOff); //Stop here now!
}

void PID_Control(void)
{
  //Err[0] = temp1 - setTemp;//If tracking a cold temp on a Peltier cooler.
  Err[0] = setTemp - temp[0];
  derr = Err[0] - Err[1];
  ierr = ierr + Err[0];
  if (ierr >= 250.0) ierr = 250.0;// Saturate integral error for anti-windup.
  if (ierr <= -250.0) ierr = -250.0;
  Err[1] = Err[0];
  DC = pidGains.Kp * Err[0] + pidGains.Ki * ierr * Ts + pidGains.Kd * derr / Ts;
  iDC = (int)DC;// Cast to int
  if (iDC >= 1023)iDC = 1023;
  if (iDC <= 0)iDC = 0;
  Timer3.pwm(HEATER_PIN_A, iDC);//Set DC
  //Timer3.pwm(HEATER_PIN_B, iDC);//Set DC
  //Timer3.pwm(HEATER_PIN_C, iDC);//Set DC
}


void WriteToSD(void)
{
  //Write time, temperatures, and current duty cycle to the SD card.
  dataString += ttime;
  for (i = 0; i < NUM_TCs; i++) // record temperatures
  {
    dataString += comma + temp[i];
  }
  dataString += comma + iDC;
  for (i = 0; i < NUM_TCs; i++) // record temperatures
  {
  dataString += comma + temp2[i];
  }
  logfile.println(dataString);
  dataString = "";
}
bool getValueWithDataReady(int (DRArray)[18], int DRnum){
  int allTrue=0, checkTemp=0;
  bool check[18]={0};
  const byte REGISTER=0x0C; 
  while(allTrue!=DRnum)
  {
    for(int n=0; n<DRnum; n++)
    {
      if(!check[n])
      {
        checkTemp=!(ReadSingleRegister(DRArray[n], REGISTER));
        if (checkTemp)
        {
          allTrue++;
          check[n]=true;
          //Serial.print(n); 
          //Serial.print('\n'); 
        }

      }
    }
  }
  return true;
}
void defineDRArray(int (&DRArray)[18], int &DRnum){
  int N=1000; //N is dummy variable
  
  DRArray[0]=17;
  DRArray[1]=15;
  DRArray[2]=6;
  DRArray[3]=8;
  DRArray[4]=23;
  DRArray[5]=25;
  DRArray[6]=27;
  DRArray[7]=29;
  DRArray[8]=31;
  DRArray[9]=34;
  DRArray[10]=36;
  DRArray[11]=38;
  DRArray[12]=40;
  DRArray[13]=42;
  DRArray[14]=38;
  DRArray[15]=46;
  DRArray[16]=48;
  DRArray[17]=44;
  for (int n=0; n!=18; n++)
  {
    if (DRArray[n]!= N) DRnum++; 
  }
  return;
}
//////////////Parser//////////
void parseSerialInput(void) {
  boolean NEG = 0;
  char dataStr[2] = {'\0', '\0'};
  boolean kpSet = false;
  boolean kiSet = false;
  boolean kdSet = false;
  unsigned int data = 0;
  byte i = 0;
  byte k = 0;
  byte l = 0;
  byte j = 0;
  byte o = 0;
  byte q = 0;
  byte p = 0;

  if (*inbuffPtr == 'h') { // Print help string.
    Help = true;
    return;
  }
  if(*inbuffPtr == 'z')//Toggle monitorData flag and return
  {
      if(monitorData) {
        monitorData = false;
      }
      else
      {
        monitorData = true;
      }
      return;
  }
  if (*inbuffPtr == 'o') { // Turn off power, but still log data.
    powerOn = false;
    return;
  }
  if (*inbuffPtr == 'A') { //Start controlling default temperature and saving data.
    saveData = true;
    powerOn = true;
    pidUpdate = true;
    controlOn = true;
    readTemp = true;
    createFile = true;
    numDataPoints = 0;
    ttime = 0.0;
    if(Interval>9) controlErr1 = true;
    return;
  }
  if (*inbuffPtr == 'a') { //Stop everything and save data.
    saveData = false;
    allOff = true;
    return;
  }
    if (*inbuffPtr == 's') {
    *inbuffPtr++;//increment pointer to second character
    if (*inbuffPtr++ == 's') { //Set print settings flag and return.
      printAcqRate = true;
      return;
    }
    else
    {
      *inbuffPtr--;// Decrement pointer
      setAcqRate = true;
      while ((*inbuffPtr != '\0')) {
        if ((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) {
          dataStr[i] = *inbuffPtr;//Make sure they're numeric!  
        }
        *inbuffPtr++;//Increment buffer pointer.
        i++;
      }
      Interval = (unsigned int)atoi(dataStr);//
      if(controlOn&&(Interval>10))controlErr1 = true;//Trap invalid control update interval error
    }//End else
  }// End if s
  if (*inbuffPtr == 'q') { //settemp
    while (*inbuffPtr != '\0') {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.')) { // Make 
        tempStr[i] = *inbuffPtr;//                  sure they're numeric!
        i++;
      }//End if numeric
      *inbuffPtr++;
    }// End duty cycle set while
    setTemp = atof(tempStr);//convert to float
    for (j = 0; j < sizeof(tempStr); j++)tempStr[j] = '\0'; //Flush buffer
    return;
  }

  // Set target temperature and log data:
  if (*inbuffPtr == 'T') {
    saveData = true;
    powerOn = true;
    pidUpdate = true;
    controlOn = true;
    readTemp = true;
    createFile = true;
    numDataPoints = 0;
    ttime = 0.0;
    while (*inbuffPtr != '\0') {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.')) { // Make 
        tempStr[i] = *inbuffPtr;//                  sure they're numeric!
        i++;
      }//End if numeric
      *inbuffPtr++;
    }// End duty cycle set while
    setTemp = atof(tempStr);//convert to float
    for (j = 0; j < sizeof(tempStr); j++)tempStr[j] = '\0'; //Flush buffer
    return;
  }

  //Open loop duty cycle setting:
  if (*inbuffPtr == 'C') { //Control Setting case
    readTemp = true;// Always read temp when power on to shut down if T >= 90 C.
    saveData = true;
    powerOn = true;
    createFile = true;
    numDataPoints = 0;
    ttime = 0.0;
    *inbuffPtr++;
    openLoop = true;
    setDC = true;
    while (*inbuffPtr != '\0') {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.')) { 
        dcStr[i] = *inbuffPtr;// Make sure they're numeric!        
        i++;
      }//End if numeric
      *inbuffPtr++;
    }// End duty cycle set while
    return;
  } // End of if C

  //Open loop duty cycle setting with data acquisition:
  if (*inbuffPtr == 'W') {
    saveData = true;
    setDC = true;
    openLoop = true;
    readTemp = true;
    createFile = true;
    numDataPoints = 0;
    ttime = 0.0;
    *inbuffPtr++;
    while (*inbuffPtr != '\0') {
      if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.')) {  
        dcStr[i] = *inbuffPtr;// Make sure they're numeric!  
        i++;
      }//End if numeric
      *inbuffPtr++;
    }// End duty cycle set while
    return;
  } // End of if W

  if (*inbuffPtr == 'c') { 
    saveData = false;
    allOff = true;
    return;
  }

  if (*inbuffPtr == 'L') { //Just start logging data
    createFile = true;
    saveData = true;
    readTemp = false;
    numDataPoints = 0;
    ttime = 0.0;
    return;
  }

  if(*inbuffPtr == 'l')
    {
      ls = true;
      return;
    }
  if (*inbuffPtr == 'g') { //Control Setting case
    *inbuffPtr++;

    if (*inbuffPtr++ == 'g') { //Set report_Gains and return
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
            if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.')) { 
              if (kdSet) {// Make sure they're numeric!
                kdStr[k] = *inbuffPtr;
                kpSet = false;
                kiSet = false;
                k++;
              }
              if (kiSet) {
                kiStr[j] = *inbuffPtr;
                kpSet = false;
                kdSet = false;
                j++;
              }
              if (kpSet) {

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
    } // End of if else
  }// End of if g

}// End of parseSerialInput

////////////End of Parser////

void setup()
{
  Timer3.initialize(40); // 40 us => 25 kHz PWM frequency  
  Timer3.pwm(HEATER_PIN_A, 0);
  Timer3.pwm(HEATER_PIN_B, 0);
  
  //Timer3.pwm(HEATER_PIN_C, 0);
  EEPROM.get(eeAcqRateAddr, Interval);// Below we handle invalid cases:
  if ((Interval <= 0) || (Interval > 9) || isnan(Interval)) Interval = 8; //Set default update index.
  // Reading an empty (not yet assigned a value) EEPROM register returns NAN.

  Ts = updateIntervals[Interval] / 1000.0; // Set sampling rate.
  InputBufferIndex = 0;
  Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set default update interval.
  Timer1.attachInterrupt(ReadData);

  while (!Serial); // for Leonardo/Micro/Zero
  Serial.begin(250000);
  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //SPI.setDataMode(SPI_MODE0);// SPI Mode 0 for SD card.

  Serial.println("READY!  \n");
  Serial.print("\nInitializing SD card...");
  if (!card.init(SPI_FULL_SPEED, SDCS)) { //Try full speed first...
    //if (!card.init(SPI_HALF_SPEED, SDCS)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the SDCS pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (!SD.begin(SDCS))
  {
    Serial.println("SD card failed or not present.  ");
    //while(1);//Might as well stop here...
  }
  else
    Serial.println("SD card initialized.");

  if (!volume.init(card))
  {
    Serial.println("Could not find FAT16/FAT32 partition.\
    \nMake sure you've formatted the card\n");
    return;
  }
  SPI.end();
  
  //SPI.setClockDivider(SPI_CLOCK_DIV2);//Reset to 7.8 MHz and Mode 3 for MAX31856
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setDataMode(SPI_MODE3);
  
  Serial.print("Initalizing Pins\n");
  initializeMAX31856Pins();
  for (int i = 0; i < NUM_TCs; i++) { //usually done for each channel..
    InitializeChannel(CSs[i]);
    delay(10);
    Serial.print("VerifyData");
    VerifyData(CSs[i]);
    Serial.print("done verifying");
  }
  EEPROM.get(eeGainSetAddr, eeGainsSet);// so you can't disable all of them.
  if (eeGainsSet = 1) {
    EEPROM.get(eepidGainsAddr, pidGains);
  }
  Serial.print("eeGainsSet = :");
  Serial.print(eeGainsSet);
  Serial.print("\n");
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
  Serial.print("Interval = ");
  Serial.print(Interval);
  Serial.print("\n");
  Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set  
  Timer1.attachInterrupt(ReadData);                           // update interval.

  delay(120);
}
///////////// End setup ////////////
///////////// Main Loop ////////////
void loop()
{    
  // All other function calls should occur here...
  if (parseCommands)
  {
    parseSerialInput();
    for (i = 0; i < sizeof(inbuff); i++) // Flush buffer.
    {
      inbuff[i] = '\0';
    }
    inbuffPtr = inbuff;// Point the input string pointer back to the beginning.
    parseCommands = false;

     if(controlErr1)// Trap invalid control update rate error
   {
    int errLen = strlen_P(controlError1);
    for (j = 0;j < errLen;j++)
    {
    myChar = pgm_read_byte_near(controlError1 + j);
    Serial.print(myChar);
    }
    stopAll();
   }
  }

  if (setAcqRate) {
   Serial.print("Interval = \n");
   Serial.print(Interval);
   Serial.print("\n");
    Timer1.initialize(((long)updateIntervals[Interval]) * 1000);
    Ts = updateIntervals[Interval] / 1000.0; // Set default sampling rate.
    EEPROM.put(eeAcqRateAddr, Interval);// Save setting.
    setAcqRate = false;
  }

  if (Help)
  {
    int HelpLength = strlen_P(HelpText);
    for (j = 0; j < HelpLength; j++)
    {
      myChar = pgm_read_byte_near(HelpText + j);
      Serial.print(myChar);
    }
    Help = false;
  }

  if (ls)
  {
    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
    root.openRoot(volume);

    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
    ls = false;
  }

  if (printAcqRate) {
    acqInterval = updateIntervals[Interval];
    Serial.print("Acqisition Interval = ");
    Serial.print(acqInterval);
    Serial.print(" mS");
    Serial.print("\n");
    printAcqRate = false;
  }
  if (reportGains) {
    printGains();
    reportGains = false;
  }
  if (setGains) {

    if (setKp) {
      pidGains.Kp = atof(kpStr);
      for (j = 0; j < sizeof(kpStr); j++)kpStr[j] = '\0'; //Flush the gain buffers
      setKp = false;
    }
    if (setKi) {
      pidGains.Ki = atof(kiStr);
      for (j = 0; j < sizeof(kiStr); j++)kiStr[j] = '\0';
      setKi = false;
    }
    if (setKd) {
      pidGains.Kd = atof(kdStr);
      for (j = 0; j < sizeof(kdStr); j++)kdStr[j] = '\0';
      setKd = false;
    }
    noInterrupts();//Don't let this transaction be interrupted!
    EEPROM.put(eepidGainsAddr, pidGains);
    EEPROM.put(eeGainSetAddr, 1);//Store the fact that the gains are stored in EPROM.
    interrupts();
    setGains = false;
  }
  if (createFile && saveData)
  {
    SPI.setDataMode(SPI_MODE0);//Switch to SPI_MODE0 for SD access.
    // Create a new file
    char filename[] = "LOGGER00.CSV";
    //DateTime now = rtc.now();//This call must occur before noInterrupts()!
    noInterrupts();
    for (i = 0; i < MAXFILES; i++) {
      filename[6] = i / 10 + '0';
      filename[7] = i % 10 + '0';

      if (!SD.exists(filename))
      {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE);
        logfile.print("Lauren");
        logfile.println();
        logfile.print('\n');
        break;  // leave the loop!
      }
    }

    if (!logfile)
    {
      file_err();
    }
    Serial.print("filing\n");
    createFile = false;
    SPI.setDataMode(SPI_MODE3);//Reset to MAX31856 SPI mode.
    readTemp = true;
    interrupts();
  }
  if (setDC)
  {
    iDC = atoi(dcStr);
    if (iDC < 0)iDC = 0;// Saturate duty cycles below zero or above 1023.
    if (iDC > 1023) iDC = 1023;
    Timer3.pwm(HEATER_PIN_A, iDC);//Set DC
    Timer3.pwm(HEATER_PIN_B, iDC);//Set DC
    //Timer3.pwm(HEATER_PIN_C, iDC);//Set DC
    setDC = false;
    for (i = 0; i < sizeof(dcStr); i++) // Flush dcStr buffer.
    {
      dcStr[i] = '\0';
    }
  }
  
  if (readTemp)
  {
    digitalWrite(SDCS, HIGH);//Low
    for (i = 0; i < NUM_TCs; i++) // Read Temperatures
    {
    temp[i] = ReadTemperature(CSs[i]);
    temp2[i]=ReadColdJunction(CSs[i]);
    }
  if(monitorData)  
  {   
    // Uncomment to enable dataready for all thermocouples being read.
    /*         
    bool DataReady=false;
    DataReady=getValueWithDataReady(DRArray, DRnum); 
    Serial.print(DataReady);
    Serial.print("\n");
    */                                                                                                            
    Serial.print(ttime);
        for (i = 0; i < NUM_TCs; i++) // post temperatures
    {
    Serial.print('\t');
    Serial.print(temp[i]);
    //Serial.print('\t');
    //Serial.print(temp2[i]);
    }
    Serial.print('\t'); 
    Serial.print(iDC);//Current duty cycle
    Serial.print('\n');
  }
    numDataPoints++;
    readTemp = false;

    if (saveData) {
      noInterrupts();
      SPI.setDataMode(SPI_MODE0);
      for (int i = 0; i < NUM_TCs; i++) {
      digitalWrite(CSs[i], HIGH);
      }
      WriteToSD();
      interrupts();
    }
    SPI.setDataMode(SPI_MODE3);
    ttime = ttime + Ts;
  }
   if (allOff)
 {
    Timer3.pwm(HEATER_PIN_A, 0);//Set DC to zero!
    Timer3.pwm(HEATER_PIN_B, 0);//Set DC to zero!
    //Timer3.pwm(HEATER_PIN_C, 0);//Set DC to zero!
    noInterrupts();
    SPI.setDataMode(SPI_MODE0);
    for (int i = 0; i < NUM_TCs; i++) {
    digitalWrite(CSs[i], HIGH);
    }
    logfile.close();
    while (allOff); //Stop here now!
  }
}


////////////  End Main Loop /////////////////
