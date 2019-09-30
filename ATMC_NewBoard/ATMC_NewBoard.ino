#include <EEPROM.h>
#include <TimerOne.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>

///////// Chip Select Pins //////
//#define MAX31856_CS1 53// SPI CS for first MAX31856 thermocouple interface 16
#define MAX31856_CS1 16// SPI CS for second MAX31856 thermocouple interface
#define NUM_TCs 1 // Number of thermocouples
#define NUM_TCs_Sensing 0 // Number of thermocouples
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
//The following executes 10 NOPs in a row for a 625 ns delay:
#define NOP10 __asm__ __volatile__ ("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"\
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
////////////////////////////////

///////// Globals ////////////.
// These control the data acquisition rate from 200 ms to 6 s:
double updateIntervals[] = {100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, \
800.0, 900.0, 1000.0, 4000.0};
double *updateIntPtr = updateIntervals;
boolean monitorData = true;

//byte RegisterValues[] = {0x90,  0x03,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };//Type K Thermocouple
byte RegisterValues[] =   {0x90,  0x07,   0xFC,   0x7F,   0xC0,   0x07,     \
0xFF,     0x80,     0x00,     0x00 };// Type T Thermocouple
String RegisterNames[] =  {"CR0", "CR1", "MASK", "CJHF", "CHLF", \
"LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO"};
byte RegisterAddresses[] = {0x00,  0x01,   0x02,   0x03,   0x04,   0x04,     \
0x06,     0x07,     0x08,     0x09 };

int CSs[] = {MAX31856_CS1};
int CSs2[] = {};
//int CSs2[] = {max1,max2,max3,max4,max5,max6,max7,max8,max9,max10,max11,max12,max13,max14,max15,max16,max17};
double temp1;
double temp2;
boolean readTemp = false;
unsigned int Interval = 9;//default Interval
boolean printAcqRate = false;
boolean setAcqRate = false;
double  acqInterval = 0;
double ttime = 0;// Temperature time
double Ts = 200.0;// Default data acquisition rate.

///////// EEPROM Addresses //////
byte eeAcqRateAddr = 13;// 2 bytes
// Total EEPROM bytes stored: 15 so far...
/////////////////////////////////
////////////////// End of Global Variables ////////

////////////// Function Definitions ////////////

///////// Interrupt Service Routines (ISRs) ///////////////
void ReadData(void)
{
  readTemp = true; //removed LED
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
  Serial.print("Finished in Initialize Channel\n");
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
void setup()
{
  EEPROM.get(eeAcqRateAddr, Interval);// Below we handle invalid cases:
  if ((Interval <= 0) || (Interval > 9) || isnan(Interval)) Interval = 1; //Set default update index.
  // Reading an empty (not yet assigned a value) EEPROM register returns NAN.

  Ts = updateIntervals[Interval] / 1000.0; // Set sampling rate.
  // put your setup code here, to run once:
  Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set default update interval.
  Timer1.attachInterrupt(ReadData);
  while (!Serial); // for Leonardo/Micro/Zero
  Serial.begin(250000);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  Serial.println("READY!  \n");
  byte SPIERROR = SPSR;//Read SPI status reg to clear errors; doesn't work.
  delay(1000);
  //SPI.setClockDivider(SPI_CLOCK_DIV2);//Reset to 7.8 MHz and Mode 3 for MAX31856
  SPI.setClockDivider(SPI_CLOCK_DIV4);// 4 MHz
  SPI.setDataMode(SPI_MODE3);
  Serial.print("Initalizing Pins\n");
  Serial.print("Initalizing sensing Pins\n");
  initializeMAX31856Pins();
  for (int i = 0; i < NUM_TCs; i++) { //usually done for each channel..
    InitializeChannel(CSs[i]);
    delay(10);
    Serial.print("VerifyData");
    VerifyData(CSs[i]);
    Serial.print("done verifying");
  }
  Serial.print("Initalizing nonsensing Pins\n");
  initializeMAX31856Pins();

  //noInterrupts();//The EEPROM driver apparently code uses interrupts,
  EEPROM.get( eeAcqRateAddr, Interval);
  Serial.print("Interval = ");
  Serial.print(Interval);
  Serial.print("\n");
  Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set  
  Timer1.attachInterrupt(ReadData);                           // update interval.
}
///////////// End setup ////////////
///////////// Main Loop ////////////
void loop()
{
  // All other function calls should occur here...
  if (setAcqRate) {
   Serial.print("Interval = \n");
   Serial.print(Interval);
   Serial.print("\n");
    Timer1.initialize(((long)updateIntervals[Interval]) * 1000);
    Ts = updateIntervals[Interval] / 1000.0; // Set default sampling rate.
    EEPROM.put(eeAcqRateAddr, Interval);// Save setting.
    setAcqRate = false;
  }
  if (printAcqRate) {
    acqInterval = updateIntervals[Interval];
    Serial.print("Acqisition Interval = ");
    Serial.print(acqInterval);
    Serial.print(" mS");
    Serial.print("\n");
    printAcqRate = false;
  }

  if (readTemp)
  {
    temp1 = ReadTemperature(CSs[0]);
    //temp2 = ReadTemperature(CSs[1]);
  if(monitorData)
  {
    Serial.print(ttime);
    Serial.print('\t');
    Serial.print(temp1);
    //Serial.print('\t');
    //Serial.print(temp2);
    Serial.print('\n');

  }
    readTemp = false;
    SPI.setDataMode(SPI_MODE3);
    ttime = ttime + Ts;
  }
}
////////////  End Main Loop /////////////////
