#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <TimerOne.h>
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
#define MAX31856_CS1 28// SPI CS for second MAX31856 thermocouple interface
#define MAX31856_CS2 29
#define MAX31856_CS3 30
#define NUM_TCs 3 // Number of thermocouples
#define NUM_TCs_Sensing 0 // Number of thermocouples
#define max1 53
/*
#define MAX31856_CS1 53// SPI CS for first MAX31856 thermocouple interface 16
#define max1 14// SPI CS for second MAX31856 thermocouple interface
# max2 5
#define max3 7
#define max4 22
#define max5 24
#define max6 26
#define max7 28
#define max8 30
#define max9 35
#define max10 37
#define max11 33
#define max12 41
#define max13 43
#define max14 39
#define max15 47
#define max16 49
#define max17 45

#define NUM_TCs 1 // Number of thermocouples
#define NUM_TCs_Sensing 17 // Number of thermocouples
*/

#define NUM31856REGs 10// Number of special function registers on the MAX31856
#define TYPE_K 0x03
#define TYPE_T 0x07
#define NOP __asm__ __volatile__ ("nop");// Inline no-operation ASM 
#define DATAREAD_LED 11//     for inserting a 62.5 ns delay used  for MAX31856
//                            SPI communication and for H-bridge deadtime. 
//The following executes 10 NOPs in a row for a 625 ns delay:
#define NOP10 __asm__ __volatile__ ("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"\
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
////////////////////////////////

///////// Globals ////////////
// These control the data acquisition rate from 200 ms to 6 s:
double updateIntervals[] = {100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0,                                              
800.0, 900.0, 1000.0, 4000.0};
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
File DataFile;
String FileName = "";
//char dataString[100];
String dataString = "";
String *dataStringPtr;
String comma = ",";

//byte RegisterValues[] = {0x90,  0x03,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };//Type K Thermocouple
byte RegisterValues[] =   {0x90,  0x07,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };// Type T Thermocouple
String RegisterNames[] =  {"CR0", "CR1", "MASK", "CJHF", "CHLF", \
"LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO"};
byte RegisterAddresses[] = {0x00,  0x01,   0x02,   0x03,   0x04,   0x04,     \
0x06,     0x07,     0x08,     0x09 };

int CSs[] = {MAX31856_CS1,MAX31856_CS2,MAX31856_CS3};
int CSs2[] = {};
//int CSs2[] = {max1,max2,max3,max4,max5,max6,max7,max8,max9,max10,max11,max12,max13,max14,max15,max16,max17};

double temp1;
double temp2;
double temp3;


boolean createFile = false;// Data file creation flag
boolean readTemp = false;
boolean record = false;
boolean saveData = false;
boolean allOff = false;
boolean controlErr1 = false;
boolean powerOn = false;
boolean ls = false;// List SD card directory flag
unsigned int Interval = 9;//default Interval
boolean printAcqRate = false;
boolean setAcqRate = false;
double  acqInterval = 0;
double ttime = 0;// Temperature time
double Ts = 200.0;// Default data acquisition rate.

unsigned int NumDataPoints = 0;
unsigned long writeSDtime = 0;
boolean openLoop = false;//Open loop flag
boolean logData = false;//Start saving data flag
unsigned int i, j, numPts = 0;

volatile char InChar = '\0'; // serial input character
char myChar;
boolean Help = false;

const char fileCreateerror[] PROGMEM = "Couldn't create file\n";//Store in program memory
const char controlError1[] PROGMEM = "Invalid control Interval\n";
const char HelpText[] PROGMEM = {"THC supports the following commands:\r\n \\
  a -- Stop everything and save data; wait until restart.\r\n \\
  h -- List of supported commands\r\n \\
  L -- Log data; use 'a' to stop and save.\r\n \\
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
z -- Monitor data by printing to the serial monitor -- toggle logic.\r\n\ \\"
};
///////// EEPROM Addresses //////
byte eeGainSetAddr = 0;// 1 byte
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
  Serial.print("Initializing SPI chip-select pins");
  for (int i = 0; i < NUM_TCs; i++) {
    Serial.print(", ");
    Serial.print(CSs[i]);
    pinMode(CSs[i], OUTPUT);
    digitalWrite(CSs[i], HIGH);
  }
  Serial.println("  Done");
  Serial.print("Initializing NonSensing pins");
  for (int i = 0; i < NUM_TCs_Sensing; i++) {
    Serial.print(", ");
    Serial.print(CSs2[i]);
    pinMode(CSs2[i], OUTPUT);
    digitalWrite(CSs2[i], HIGH);
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
void file_err(void)
{

  int errLength = strlen_P(fileCreateerror);
  for (j = 0; j < errLength; j++)
  {
    myChar = pgm_read_word_near(fileCreateerror + j);
    Serial.print(myChar);
  }
  while (1); //Stop here.
}

void stopAll()
{
    noInterrupts();
    SPI.setDataMode(SPI_MODE0);
    logfile.close();
    while (allOff); //Stop here now!
}

void WriteToSD(void)
{
  //Write time, temperatures, and current duty cycle to the SD card.

dataString += ttime + comma + temp1 + comma + temp2 + comma + temp3;

  logfile.println(dataString);
  dataString = "";
}

//////////////Parser//////////
void parseSerialInput(void) {
  boolean NEG = 0;
  char dataStr[2] = {'\0', '\0'};
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
  if (*inbuffPtr == 'a') { //Stop everything and save data.
    saveData = false;
    allOff = true;
    return;
  }
  if (*inbuffPtr == 'L') { //Just start logging data
    saveData = true;
    readTemp = true;
    createFile = true;
    numDataPoints = 0;
    ttime = 0.0;
    return;
  }

  if(*inbuffPtr == 'l')
    {
      ls = true;
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
    }//End else
  }// End if s

}// End of parseSerialInput

////////////End of Parser////

void setup()
{
  

  EEPROM.get(eeAcqRateAddr, Interval);// Below we handle invalid cases:
  if ((Interval <= 0) || (Interval > 9) || isnan(Interval)) Interval = 1; //Set default update index.
  // Reading an empty (not yet assigned a value) EEPROM register returns NAN.

  Ts = updateIntervals[Interval] / 1000.0; // Set sampling rate.
  InputBufferIndex = 0;
  Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set default update interval.
  Timer1.attachInterrupt(ReadData);
  while (!Serial); // for Leonardo/Micro/Zero
  Serial.begin(250000);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);// SPI Mode 0 for SD card.

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
  EEPROM.get( eeAcqRateAddr, Interval);
  Serial.print("Interval = ");
  Serial.print(Interval);
  Serial.print("\n");
  Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set  
  Timer1.attachInterrupt(ReadData);                           // update interval.


  delay(100);
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
  if (readTemp)
  {
    temp1 = ReadTemperature(CSs[0]);
    temp2 = ReadTemperature(CSs[1]);
    temp3 = ReadTemperature(CSs[2]);
  if(monitorData)  
  {                                                                                                                        
    Serial.print(ttime);
    Serial.print('\t');
    Serial.print(temp1);
    Serial.print('\t');
    Serial.print(temp2);
    Serial.print('\t');
    Serial.print(temp3);
    Serial.print('\n');
  }
    numDataPoints++;
    readTemp = false;

    if (saveData) {
      noInterrupts();
      SPI.setDataMode(SPI_MODE0);
      WriteToSD();
      interrupts();
    }
    SPI.setDataMode(SPI_MODE3);
    ttime = ttime + Ts;
  }
   if (allOff)
 {
    noInterrupts();
    SPI.setDataMode(SPI_MODE0);
    logfile.close();
    while (allOff); //Stop here now!
  }
}


////////////  End Main Loop /////////////////
