#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Wire.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

////////// Chip Select Pins //////////
#define SDCS 10// SPI CS for SD card
#define HEATER_PIN_A 3 // PWM output pin 1
#define HEATER_PIN_B 2 // PWM output pin 2
#define HEATER_PIN_C 4 // PWM output pin 3

//Thermocouple Type: Uncomment the type of themocouple used. 
#define TypeT
//#define TypeK

//Cold Junction Compensation: Uncomment if you would like to use the internal cold junction compensation.
//#define ColdJunctionComp

#define TC1 4 // SPI CS for first MAX31856 thermocouple interface
#define TC2 12
#define TC3 6
#define TC4 7
#define TC5 8
#define TC6 9

#define NUM_TCs 6 // Number of thermocouples
int CSs[] = {TC1,TC2,TC3,TC4,TC5,TC6}; //List of thermocouples to read
double temp[NUM_TCs];
#define NOP __asm__ __volatile__ ("nop");// Inline no-operation ASM 

#define BS 8 //Backspace character
#define CR 13// Carrage return
#define LF 10// Line feed
#define DEL 127//Delete character
#define ECHO_//Comment out if you don't want to echo input chars to the serial monitor
#define MAXFILES 10000 // Maximum number of data files to create
////////////////////////////////

////////// Globals //////////
// These control the data acquisition rate from 100 ms to 4 s:
double updateIntervals[] = {100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0,                                              
800.0, 900.0, 1000.0, 4000.0};
volatile char inbuff[200];
volatile char *inbuffPtr = inbuff;
volatile unsigned int InputBufferIndex;
unsigned long numDataPoints = 0;// Counts the number of datapoints acquired
boolean parseCommands = false;
boolean monitorData = true;
char dcStr[5];
char tempStr[10];
String fileheading = "LOGGER";
String filename = "";
String filetype = ".CSV";
String dataString = "";
String dataStringHeader = "";
String comma = ",";
boolean createFile = false;// Data file creation flag
boolean readTemp = false;
boolean saveData = false;
boolean allOff = false;
boolean ls = false;// List SD card directory flag
unsigned int Interval = 3;//default Interval
boolean printAcqRate = false;
boolean setAcqRate = false;
double  acqInterval = 0;
double ttime = 0;// Temperature time
double Ts = 200.0;// Default data acquisition rate.
double DCset = 0;
int iDC = 0;
unsigned int NumDataPoints = 0;
boolean logData = false;//Start saving data flag
unsigned int i, j, numPts = 0;
boolean setDC = false;//Manual open loop duty cycle setting flag.
volatile char InChar = '\0'; // serial input character
char myChar;
boolean Help = false;

const char fileCreateerror[] PROGMEM = "Couldn't create file\n";//Store in program memory
const char HelpText[] PROGMEM = R"(THC supports the following commands:
  a -- Stop everything and save data; wait until restart.
  h -- List of supported commands
  L -- Log data
  ss -- Print acquisition rate
  s# -- Set data acquisition interval where # is an integer option denoting:
    0 --> 0.1 sec
    1 --> 0.2 sec
    2 --> 0.3 sec
    3 --> 0.4 sec
    4 --> 0.5 sec
    5 --> 0.6 sec
    6 --> 0.7 sec
    7 --> 0.8 sec
    8 --> 0.9 sec
    9 --> 1.0 sec
    10 --> 4.0 sec
  W#### -- Simultaneously set duty cycle and log data, where #### represents  
           the duty cycle from 0 to 1023.
)";
#ifdef ColdJunctionComp
// Shift register configuration for chips internal cold junction compensation
#define NUM31856REGs 10// Number of special function registers on the MAX31856
#ifdef TypeK
byte RegisterValues[] = {0x90,0x03,0xFC,0x7F,0xC0,0x07,0xFF,0x80,0x00,0x00};//Type K Thermocouple
#else
byte RegisterValues[] = {0x90,0x07,0xFC,0x7F,0xC0,0x07,0xFF,0x80,0x00,0x00};// Type T Thermocouple
#endif
String RegisterNames[] = {"CR0","CR1","MASK","CJHF","CHLF","LTHFTH","LTHFTL","LTLFTH","LTLFTL","CJTO"};
byte RegisterAddresses[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
#else
// Shift register configuration for writing cold junction compensation. Set cold junction temp to zero. Speeds up conversion by 25ms.
// Temperature is given at temp above ambient (Excess Temperature)
#define NUM31856REGs 12// Number of special function registers on the MAX31856
#ifdef TypeK
byte RegisterValues[] = {0x90,0x03,0xFC,0x7F,0xC0,0x07,0xFF,0x80,0x00,0x00,0x00,0x00};//Type K Thermocouple
#else
byte RegisterValues[] = {0x98,0x07,0xFC,0x7F,0xC0,0x7F,0xFF,0x80,0x00,0x00,0x00,0x00};// Type T Thermocouple
#endif
String RegisterNames[] = {"CR0","CR1","MASK","CJHF","CHLF","LTHFTH","LTHFTL","LTLFTH","LTLFTL","CJTO","CJTH","CJTL"};
byte RegisterAddresses[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B};
#endif
////////// EEPROM Addresses //////////
byte eeAcqRateAddr = 13;// 2 bytes
////////// Object Instantiations //////////
//////////SD card objects //////////
Sd2Card card;
SdVolume volume;
SdFile root;
File logfile;
////////// End of Object Instantiations //////////
////////// Interrupt Service Routines (ISRs) //////////
void serialEvent() {
	while (Serial.available()) {
		InChar = Serial.read();
		if ((InChar == DEL) && (InputBufferIndex != 0)) {
			inbuff[--InputBufferIndex] = '\0';//Replace the erroneous byte with a NUL.
			Serial.print(BS);//Echo backspace character.
		}
		else{
			inbuff[InputBufferIndex] = InChar;
			InputBufferIndex++;
		}
		if (InChar == '\n' or InChar == 0xD) {
			parseCommands = true;
			InputBufferIndex = 0;
		}	
	}
}
void ReadData(void) {
  readTemp = true;
}
////////// End of ISR definitions //////////

////////// MAX31856 Functions //////////
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
	unsigned long data = 0;
	SPI.transfer(StartRegister & 0x7F); //force bit 7 to 0 to ensure a read command
	NOP;
	for (int i = 0; i < count; i++) {
		data = (data << 8) | SPI.transfer(0); //bitshift left 8 bits, 
	}										  //then add the next register
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

double ReadTemperature(int Pin) {
	double temperature;
	long data;
	data = ReadMultipleRegisters(Pin, 0x0C, 4);
	data = data >> 13;// Strip the unused bits and the Fault Status Register
	temperature = (double) data * 0.0078125;// Convert to Celsius
	return (temperature);// Return the temperature
}

void initializeMAX31856Pins() {
	Serial.print("Initializing SPI chip-select pins\n");
	for (int i = 0; i < NUM_TCs; i++) {
		Serial.print(CSs[i]);
		Serial.print(", ");
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
////////// End of MAX31856 Functions //////////

void file_err(void) {
	int errLength = strlen_P(fileCreateerror);
	for (j = 0; j < errLength; j++) {
		myChar = pgm_read_word_near(fileCreateerror + j);
		Serial.print(myChar); delay(1000);
	}
	Timer3.pwm(HEATER_PIN_A, 0);//Set DC to zero!
	while (1); //Stop here.
}

void stopAll() {
	Timer3.pwm(HEATER_PIN_A, 0);//Set DC to zero!
    noInterrupts();
    SPI.setDataMode(SPI_MODE0);
    for (int i = 0; i < NUM_TCs; i++) {
		digitalWrite(CSs[i], HIGH);
    }
    logfile.close();
    while (allOff);//Stop here now!
}

void WriteToSD(void) {//Write time, temperatures, and current duty cycle to the SD card.
	dataString += ttime;
	for (i = 0; i < NUM_TCs; i++) { // record temperatures
		dataString += comma + temp[i];
	}
	dataString += comma + iDC;
	logfile.println(dataString);
	dataString = "";
}

////////// Parser //////////
void parseSerialInput(void) {
	boolean NEG = 0;
	char dataStr[2] = {'\0', '\0'};
	unsigned int data = 0;
	byte i = 0;
	if (*inbuffPtr == 'h') { // Print help string.
		Help = true;
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
		else {
			*inbuffPtr--;// Decrement pointer
			while ((*inbuffPtr != '\0')) {
				if ((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) {
					dataStr[i] = *inbuffPtr;//Make sure they're numeric!  
				}
				*inbuffPtr++;//Increment buffer pointer.
				i++;
			}
			if ((unsigned int)atoi(dataStr) > 10) {
				Serial.print("Invalid Interval \n");
			}
			else {
				setAcqRate = true;
				Interval = (unsigned int)atoi(dataStr);//
			}
		}// End else
	}// End if s
	if (*inbuffPtr == 'W') { // Open loop duty cycle setting with data acquisition:
		saveData = true;
		setDC = true;
		readTemp = true;
		createFile = true;
		numDataPoints = 0;
		ttime = 0.0;
		*inbuffPtr++;
		while (*inbuffPtr != '\0') {
			if (((*inbuffPtr >= '0') && (*inbuffPtr <= '9')) || (*inbuffPtr == '.')) {  
				dcStr[i] = *inbuffPtr;// Make sure they're numeric!  
				i++;
			}// End if numeric
			*inbuffPtr++;
		}// End duty cycle set while
		return;
	}// End of if W
	if (*inbuffPtr == 'L') { // Just start logging data
		createFile = true;
		saveData = true;
		readTemp = false;
		numDataPoints = 0;
		ttime = 0.0;
		return;
	}
	if(*inbuffPtr == 'l'){
		ls = true;
		return;
	}
}// End of parseSerialInput
////////// End of Parser //////////

void setup() {
	Timer3.initialize(40); // 40 us => 25 kHz PWM frequency  
	Timer3.pwm(HEATER_PIN_A, 0);// Initialize heaters to 0
	Timer3.pwm(HEATER_PIN_B, 0);
	Timer3.pwm(HEATER_PIN_C, 0);
	EEPROM.get(eeAcqRateAddr, Interval);// Below we handle invalid cases:
	if ((Interval <= 0) || (Interval > 9) || isnan(Interval)) Interval = 8;// Set default update index.
	// Reading an empty (not yet assigned a value) EEPROM register returns NAN.
	Ts = updateIntervals[Interval] / 1000.0;// Set sampling rate.
	InputBufferIndex = 0;
	Timer1.initialize(((long)updateIntervals[Interval]) * 1000);// Set default update interval.
	Timer1.attachInterrupt(ReadData);
	while (!Serial); // for Leonardo/Micro/Zero
	Serial.begin(250000);
	while (!Serial) {
		;// wait for serial port to connect. Needed for native USB port only
	}
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV128);
	Serial.println("READY!  \n");
	Serial.print("Initializing SD card...");
	if (!card.init(SPI_FULL_SPEED, SDCS)) { // Try full speed first...
		if (!card.init(SPI_HALF_SPEED, SDCS)) {
			Serial.println("initialization failed. Things to check:");
			Serial.println("* is a card inserted?");
			Serial.println("* is your wiring correct?");
			Serial.println("* did you change the SDCS pin to match your shield or module?");
			return;
		}
	}
	else {
		Serial.println("Wiring is correct and a card is present.");
	}
	if (!SD.begin(SDCS)) {
		Serial.println("SD card failed or not present.  ");
	}
	else {
		Serial.println("SD card initialized.");
	}
	if (!volume.init(card)) {
		Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card\n");
		return;
	}
	SPI.end();
	SPI.setClockDivider(SPI_CLOCK_DIV128);
	SPI.setDataMode(SPI_MODE3);
	Serial.print("Initalizing Pins\n");
	initializeMAX31856Pins();
	for (int i = 0; i < NUM_TCs; i++) { // Usually done for each channel.
		InitializeChannel(CSs[i]);
		delay(10);
		Serial.print("Verifying Data. ");
		VerifyData(CSs[i]);
		Serial.print("Done Verifying\n");
	}
	EEPROM.get( eeAcqRateAddr, Interval);
	Serial.print("Interval = ");
	Serial.print(Interval);
	Serial.print("\n");
	Timer1.initialize(((long)updateIntervals[Interval]) * 1000); // Set update interval.
	Timer1.attachInterrupt(ReadData);
	delay(120);
}
////////// End setup //////////
////////// Main Loop //////////
void loop() { // All other function calls occur here.
	if (parseCommands) {
		parseSerialInput();
		for (i = 0; i < sizeof(inbuff); i++) { // Flush buffer.
			inbuff[i] = '\0';
		}
		inbuffPtr = inbuff;// Point the input string pointer back to the beginning.
		parseCommands = false;
	}
	if (setAcqRate) {
		Serial.print("Interval = \n");
		Serial.print(Interval);
		Serial.print("\n");
		Timer1.initialize(((long)updateIntervals[Interval]) * 1000);
		Ts = updateIntervals[Interval] / 1000.0;// Set default sampling rate.
		EEPROM.put(eeAcqRateAddr, Interval);// Save setting.
		setAcqRate = false;
	}
	if (Help) {
		int HelpLength = strlen_P(HelpText);
		for (j = 0; j < HelpLength; j++) {
			myChar = pgm_read_byte_near(HelpText + j);
			Serial.print(myChar);
		}
		Help = false;
	}
	if (ls) {
		Serial.println("\nFiles found on the card (name, date and size in bytes): ");
		root.openRoot(volume);
		root.ls(LS_R | LS_DATE | LS_SIZE);// list all files in the card with date and size
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
  
	if (createFile && saveData) {
		SPI.setDataMode(SPI_MODE0);//Switch to SPI_MODE0 for SD access.
		// Create a new file
		noInterrupts();
		for (i = 0; i < MAXFILES; i++) {
			filename = fileheading;
			filename.concat(i);
			filename.concat(filetype);
			if (!SD.exists(filename)) { // only open a new file if it doesn't exist
				Serial.print(filename);
				Serial.print("\t");
				logfile = SD.open(filename, FILE_WRITE);
				for (i = 0; i < NUM_TCs; i++) { // record temperatures
					dataStringHeader += comma + "TC";
					dataStringHeader.concat(i+1);
				}
				dataStringHeader += comma + "Power (1023 Max)";
				logfile.println("Time"+dataStringHeader);
				break;// leave the loop!
			}
		}
		if (!logfile) {
			file_err();
		}
		Serial.print("filing\n");
		createFile = false;
		SPI.setDataMode(SPI_MODE3);// Reset to MAX31856 SPI mode.
		readTemp = true;
		interrupts();
	} 
	if (setDC) {
		iDC = atoi(dcStr);
		if (iDC < 0)iDC = 0;// Saturate duty cycles below zero or above 1023.
		if (iDC > 1023) iDC = 1023;
		Timer3.pwm(HEATER_PIN_A, iDC);//Set DC
		setDC = false;
		for (i = 0; i < sizeof(dcStr); i++) { // Flush dcStr buffer.
			dcStr[i] = '\0';
		}
	}
	if (readTemp) {
		digitalWrite(SDCS, HIGH);
		for (i = 0; i < NUM_TCs; i++) { // Read Temperatures
			temp[i] = ReadTemperature(CSs[i]);
		}
		if (monitorData) {                                                                                             
			Serial.print(ttime);
			for (i = 0; i < NUM_TCs; i++) { // Post temperatures
				Serial.print('\t');
				Serial.print(temp[i]);
			}
			Serial.print('\t'); 
			Serial.print(iDC);// Current duty cycle
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
	if (allOff) {
		Timer3.pwm(HEATER_PIN_A, 0);// Set DC to zero!
		noInterrupts();
		SPI.setDataMode(SPI_MODE0);
		for (int i = 0; i < NUM_TCs; i++) {
			digitalWrite(CSs[i], HIGH);
		}
		logfile.close();
		while (allOff);// Stop here now!
	}
}
//////////  End Main Loop //////////
