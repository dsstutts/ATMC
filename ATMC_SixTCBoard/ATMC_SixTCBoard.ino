/*
  ATMC Six Thermocouple Board -- portable/safe revision

  Major portability changes from the original sketch:
    - No avr/pgmspace.h, TimerOne, or TimerThree dependency.
    - Data acquisition timing uses millis(), so the sketch is much more portable
      across AVR, ESP32, SAMD, SAM/Due, etc.
    - Heater PWM uses a small abstraction layer. User commands still use a
      logical 10-bit duty cycle, 0..1023. Boards without 10-bit analogWrite
      support are scaled to 8-bit analogWrite internally.
    - Serial command parsing is bounded and does not use pointer post-increment
      side effects.
    - Help text is printed line-by-line with F() literals, which is efficient
      on AVR and harmless on most other Arduino-style cores.
    - Avoids heap-fragmenting String concatenation in the data logging path.
    - Adds optional RTClib/DS3231 RTC support, version reporting, compile-time
      RTC setting, and descriptive CSV metadata headers.
    - Scans the SD root directory at startup so log-file numbering resumes after
      the highest existing LOG#####.CSV file instead of starting from zero.
    - Adds lowercase w#### duty-cycle command to change heater duty while
      continuing an existing log timeline without resetting elapsed_time_s.

  Notes:
    - The previous Timer3-based 25 kHz PWM is not portable. This version favors
      portability. ESP32 uses the LEDC API when available.
    - Arduino Due does not have built-in EEPROM. If EEPROM is unavailable, the
      acquisition interval still works but is not persisted across resets.
    - RTC support is compiled only when RTClib.h is installed and THC_ENABLE_RTC
      is true. If no RTC is present, the sketch still compiles and runs.
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef THC_ENABLE_RTC
  #define THC_ENABLE_RTC 1
#endif

// RTC selection.  The original ATMC sketch used Adafruit RTClib's
// RTC_PCF8523 type, which is common on Adafruit data-logging shields.
// Change THC_RTC_TYPE to THC_RTC_DS3231 if your board uses a DS3231.
#define THC_RTC_NONE    0
#define THC_RTC_DS3231  1
#define THC_RTC_PCF8523 2

#ifndef THC_RTC_TYPE
  #define THC_RTC_TYPE THC_RTC_PCF8523
#endif

#if THC_ENABLE_RTC && (THC_RTC_TYPE != THC_RTC_NONE)
  // Intentionally include RTClib directly instead of relying on __has_include,
  // because some Arduino AVR toolchains/preprocessors do not define
  // __has_include reliably.  If you do not have RTClib installed, either
  // install "RTClib by Adafruit" from Library Manager or set
  // THC_ENABLE_RTC to 0 above.
  #include <Wire.h>
  #include <RTClib.h>
  #define THC_HAS_RTC 1
#else
  #define THC_HAS_RTC 0
#endif

#if defined(ARDUINO_ARCH_ESP32)
  #if defined(__has_include)
    #if __has_include(<esp_arduino_version.h>)
      #include <esp_arduino_version.h>
    #endif
  #endif
  #ifndef ESP_ARDUINO_VERSION_MAJOR
    // Older Arduino-ESP32 cores may not publish the version macro. Treat them
    // as pre-3.x for LEDC API compatibility.
    #define ESP_ARDUINO_VERSION_MAJOR 2
  #endif
#endif

// EEPROM support. Arduino Mega/AVR has real EEPROM, but some Arduino
// preprocessor configurations do not reliably expose __has_include. Therefore
// include EEPROM.h directly for known EEPROM-capable cores, then fall back to
// __has_include for other compatible cores.
#if defined(ARDUINO_ARCH_SAM)
  // Arduino Due/SAM does not have built-in EEPROM.
  #define THC_HAS_EEPROM 0
#elif defined(ARDUINO_ARCH_AVR)
  #include <EEPROM.h>
  #define THC_HAS_EEPROM 1
#elif defined(ARDUINO_ARCH_ESP32)
  #include <EEPROM.h>
  #define THC_HAS_EEPROM 1
#elif defined(__has_include)
  #if __has_include(<EEPROM.h>)
    #include <EEPROM.h>
    #define THC_HAS_EEPROM 1
  #endif
#endif

#ifndef THC_HAS_EEPROM
  #define THC_HAS_EEPROM 0
#endif

// ----------------------------- User configuration -----------------------------

static const uint32_t SERIAL_BAUD = 250000UL;

// Give USB/serial adapters and the Arduino Serial Monitor time to settle before
// printing the startup banner. This is especially helpful on boards such as
// the Mega, where the bootloader/reset sequence can otherwise leave the first
// few characters looking like baud-rate gibberish even though later text is OK.
#ifndef THC_SERIAL_STARTUP_DELAY_MS
  #define THC_SERIAL_STARTUP_DELAY_MS 1500UL
#endif

#ifndef THC_SERIAL_READY_TIMEOUT_MS
  #define THC_SERIAL_READY_TIMEOUT_MS 3000UL
#endif

#ifndef THC_PRINT_STARTUP_DIAGNOSTICS
  #define THC_PRINT_STARTUP_DIAGNOSTICS 1
#endif

// Arduino IDE 2.x Serial Monitor autoscroll can miss very large startup bursts.
// This small pace delay makes the boot text easier to follow on a Mega without
// introducing delay() into the data-acquisition loop. Set to 0 to disable.
#ifndef THC_SERIAL_STARTUP_PACE_MS
  #define THC_SERIAL_STARTUP_PACE_MS 2UL
#endif

// Program/version metadata. Increment these whenever the sketch behavior changes.
#define THC_PROGRAM_NAME       "ATMC Six Thermocouple Board"
#define THC_PROGRAM_SHORT_NAME "ATMC_SixTCBoard"
#define THC_VERSION_MAJOR      2
#define THC_VERSION_MINOR      4
#define THC_VERSION_PATCH      2
#define THC_VERSION_STRING     "2.4.2"
#define THC_BUILD_DATE         __DATE__
#define THC_BUILD_TIME         __TIME__
#define THC_BUILD_STAMP        __DATE__ " " __TIME__

// If true, the sketch sets a DS3231 RTC to the compile/build time on the first
// boot after a new firmware upload when EEPROM-like storage is available. If
// EEPROM is unavailable, use command tb to set the RTC to the build time.
#ifndef THC_SET_RTC_FROM_BUILD_ON_NEW_FIRMWARE
  #define THC_SET_RTC_FROM_BUILD_ON_NEW_FIRMWARE 1
#endif

// If true, the sketch forces the RTC to the build time every boot. Leave false
// for normal use, or every reset will move the clock backward to compile time.
#ifndef THC_FORCE_RTC_BUILD_TIME_ON_EVERY_BOOT
  #define THC_FORCE_RTC_BUILD_TIME_ON_EVERY_BOOT 0
#endif

// SPI chip-select pins.
static const uint8_t SDCS = 10;
static const uint8_t TC1  = 4;
static const uint8_t TC2  = 12;
static const uint8_t TC3  = 6;
static const uint8_t TC4  = 7;
static const uint8_t TC5  = 8;
static const uint8_t TC6  = 9;

// PWM output pins.
static const uint8_t HEATER_PIN_A = 3;
static const uint8_t HEATER_PIN_B = 2;
static const uint8_t HEATER_PIN_C = 4;
static const uint8_t ACTIVE_PWM   = HEATER_PIN_A;

// Only ACTIVE_PWM is driven by default. In the original pin map, HEATER_PIN_C
// shares pin 4 with TC1, so B/C are intentionally disabled unless you assign
// unique, known-safe pins and set these true.
static const bool ENABLE_HEATER_PIN_B = false;
static const bool ENABLE_HEATER_PIN_C = false;

// Thermocouple type. Leave exactly one enabled.
#define TypeT
// #define TypeK

// Cold junction compensation. Uncomment to use MAX31856 internal CJC.
// #define ColdJunctionComp

// Echo command input to the serial monitor.
#define ECHO_INPUT 1

// Logical command range for duty cycle. The PWM wrapper handles board scaling.
static const uint8_t  PWM_BITS    = 10;
static const uint16_t PWM_MAX     = (1U << PWM_BITS) - 1U;  // 1023
static const uint32_t PWM_FREQ_HZ = 25000UL;                // Used where supported.

static const uint32_t MAX_LOG_FILES = 100000UL;
static const uint8_t  NUM_TCS       = 6;
static const uint8_t  CSs[NUM_TCS]  = {TC1, TC2, TC3, TC4, TC5, TC6};

// Acquisition interval options, indexed by the s# command.
static const uint16_t updateIntervalsMs[] = {
  100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 4000
};
static const uint8_t NUM_INTERVAL_OPTIONS =
  sizeof(updateIntervalsMs) / sizeof(updateIntervalsMs[0]);

// EEPROM storage. One byte is enough for interval index 0..10. Four more bytes
// are used to remember which build stamp has already initialized the RTC.
static const uint16_t EEPROM_SIZE_BYTES  = 64;
static const uint16_t eeAcqRateAddr      = 13;
static const uint16_t eeBuildStampAddr   = 20;

// Serial command buffer.
static const uint8_t INPUT_BUFFER_LEN = 200;

// MAX31856 SPI settings. The original sketch used SPI_MODE3; keep that here.
// If your MAX31856 board/library expects MODE1, change SPI_MODE3 to SPI_MODE1.
static SPISettings max31856SpiSettings(1000000UL, MSBFIRST, SPI_MODE3);

// ------------------------------ Global state ----------------------------------

static char inputBuffer[INPUT_BUFFER_LEN];
static uint8_t inputIndex = 0;
static bool commandReady = false;
static bool inputOverflow = false;

static float tempC[NUM_TCS];

static bool readTempNow      = false;
static bool monitorData      = true;
static bool saveData         = false;
static bool createFile       = false;
static bool listFilesFlag    = false;
static bool stopAllFlag      = false;
static bool printAcqRateFlag = false;
static bool setAcqRateFlag   = false;
static bool setDutyFlag      = false;

static uint8_t intervalIndex = 8;   // Default: 0.9 s, matching original fallback.
static uint16_t samplePeriodMs = updateIntervalsMs[8];
static float samplePeriodS = 0.9f;
static uint32_t nextSampleMs = 0;

static uint32_t numDataPoints = 0;
static float elapsedLogTimeS = 0.0f;

static uint16_t requestedDuty = 0;
static uint16_t currentDuty = 0;

static bool sdReady = false;
static bool logFileOpen = false;
static File logfile;
static char currentLogFilename[13] = "";
static bool csvHeaderWritten = false;
static uint32_t nextLogFileNumber = 0;
static bool logFileScanDone = false;

#if THC_HAS_RTC
  #if THC_RTC_TYPE == THC_RTC_DS3231
    static RTC_DS3231 rtc;
  #elif THC_RTC_TYPE == THC_RTC_PCF8523
    static RTC_PCF8523 rtc;
  #else
    #error "Unsupported THC_RTC_TYPE. Use THC_RTC_DS3231, THC_RTC_PCF8523, or THC_RTC_NONE."
  #endif
#endif
static bool rtcReady = false;

static void printStartupDiagnostics();
static void clearPendingSerialInput();
static bool scanLogFileNumbers();
static void printNextLogFilename(Stream &out);

// --------------------------- MAX31856 register setup ---------------------------

#if defined(TypeT) && defined(TypeK)
  #error "Select only one thermocouple type: TypeT or TypeK."
#endif

#if !defined(TypeT) && !defined(TypeK)
  #define TypeT
#endif

#ifdef ColdJunctionComp
  static const uint8_t NUM31856REGS = 10;
  #ifdef TypeK
    static const uint8_t RegisterValues[NUM31856REGS] = {
      0x90, 0x03, 0xFC, 0x7F, 0xC0, 0x07, 0xFF, 0x80, 0x00, 0x00
    };
  #else
    static const uint8_t RegisterValues[NUM31856REGS] = {
      0x90, 0x07, 0xFC, 0x7F, 0xC0, 0x07, 0xFF, 0x80, 0x00, 0x00
    };
  #endif
  static const char * const RegisterNames[NUM31856REGS] = {
    "CR0", "CR1", "MASK", "CJHF", "CHLF", "LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO"
  };
  static const uint8_t RegisterAddresses[NUM31856REGS] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
  };
#else
  static const uint8_t NUM31856REGS = 12;
  #ifdef TypeK
    static const uint8_t RegisterValues[NUM31856REGS] = {
      0x90, 0x03, 0xFC, 0x7F, 0xC0, 0x07, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00
    };
  #else
    static const uint8_t RegisterValues[NUM31856REGS] = {
      0x98, 0x07, 0xFC, 0x7F, 0xC0, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00
    };
  #endif
  static const char * const RegisterNames[NUM31856REGS] = {
    "CR0", "CR1", "MASK", "CJHF", "CHLF", "LTHFTH", "LTHFTL", "LTLFTH", "LTLFTL", "CJTO", "CJTH", "CJTL"
  };
  static const uint8_t RegisterAddresses[NUM31856REGS] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
  };
#endif

// ------------------------------ Utility helpers --------------------------------

static void serialStartupPace() {
#if THC_SERIAL_STARTUP_PACE_MS > 0
  Serial.flush();
  delay((uint32_t)THC_SERIAL_STARTUP_PACE_MS);
#endif
}

static void printLine(Stream &out, const __FlashStringHelper *text) {
  out.println(text);
}

static void printLine(Stream &out, const char *text) {
  out.println(text);
}

static void printHelp(Stream &out) {
  printLine(out, F("THC supports the following commands:"));
  printLine(out, F("  a -- Stop everything, save data, close the file, and wait until restart."));
  printLine(out, F("  h -- List supported commands."));
  printLine(out, F("  v -- Print program, version, build, board, PWM, and RTC status."));
  printLine(out, F("  d -- Print startup/runtime diagnostics."));
  printLine(out, F("  l -- List files on the SD card and show the next log filename."));
  printLine(out, F("  L -- Log data to a new CSV file."));
  printLine(out, F("  tt -- Tell RTC time, if an RTC is available."));
  printLine(out, F("  tb -- Set RTC to this sketch's compile/build date and time."));
  printLine(out, F("  tYYYY-MM-DD HH:MM:SS -- Set RTC, e.g. t2026-04-28 13:45:00."));
  printLine(out, F("  tY2026R04D28H13M45S00 -- Set RTC using fields. R denotes month."));
  printLine(out, F("  ss -- Print acquisition interval."));
  printLine(out, F("  s# -- Set acquisition interval, where # is:"));
  printLine(out, F("    0  --> 0.1 sec"));
  printLine(out, F("    1  --> 0.2 sec"));
  printLine(out, F("    2  --> 0.3 sec"));
  printLine(out, F("    3  --> 0.4 sec"));
  printLine(out, F("    4  --> 0.5 sec"));
  printLine(out, F("    5  --> 0.6 sec"));
  printLine(out, F("    6  --> 0.7 sec"));
  printLine(out, F("    7  --> 0.8 sec"));
  printLine(out, F("    8  --> 0.9 sec"));
  printLine(out, F("    9  --> 1.0 sec"));
  printLine(out, F("    10 --> 4.0 sec"));
  printLine(out, F("  W#### -- Set 0..1023 duty cycle and start/restart logging, e.g. W512."));
  printLine(out, F("  w#### -- Set 0..1023 duty cycle without resetting the active log timeline."));
}

static char *trimCommand(char *s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }

  char *end = s + strlen(s);
  while (end > s) {
    char c = *(end - 1);
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      *(--end) = '\0';
    } else {
      break;
    }
  }

  return s;
}

static bool parseUnsignedInRange(const char *s, uint16_t minValue, uint16_t maxValue, uint16_t &value) {
  if (s == NULL || *s == '\0') {
    return false;
  }

  uint32_t accum = 0;
  while (*s != '\0') {
    if (*s < '0' || *s > '9') {
      return false;
    }
    accum = accum * 10UL + (uint32_t)(*s - '0');
    if (accum > maxValue) {
      return false;
    }
    ++s;
  }

  if (accum < minValue || accum > maxValue) {
    return false;
  }

  value = (uint16_t)accum;
  return true;
}

static uint32_t fnv1a32(const char *s) {
  uint32_t hash = 2166136261UL;
  while (*s != '\0') {
    hash ^= (uint8_t)(*s++);
    hash *= 16777619UL;
  }
  return hash;
}

static bool isLeapYear(uint16_t year) {
  return ((year % 4U) == 0U && ((year % 100U) != 0U || (year % 400U) == 0U));
}

static uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month < 1 || month > 12) {
    return 0;
  }
  if (month == 2 && isLeapYear(year)) {
    return 29;
  }
  return days[month - 1];
}

static bool validateDateTimeParts(uint16_t year, uint8_t month, uint8_t day,
                                  uint8_t hour, uint8_t minute, uint8_t second) {
  if (year < 2000U || year > 2099U) { return false; }
  if (month < 1U || month > 12U) { return false; }
  if (day < 1U || day > daysInMonth(year, month)) { return false; }
  if (hour > 23U) { return false; }
  if (minute > 59U) { return false; }
  if (second > 59U) { return false; }
  return true;
}

static void formatDateTimeParts(char *buffer, size_t bufferLen,
                                uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second) {
  snprintf(buffer, bufferLen, "%04u-%02u-%02u %02u:%02u:%02u",
           year, month, day, hour, minute, second);
}

#if THC_HAS_EEPROM
static uint32_t eepromReadU32(uint16_t addr) {
  uint32_t value = 0;
  for (uint8_t k = 0; k < 4; ++k) {
    value |= ((uint32_t)EEPROM.read(addr + k)) << (8U * k);
  }
  return value;
}

static void eepromWriteU32(uint16_t addr, uint32_t value) {
  bool changed = false;
  for (uint8_t k = 0; k < 4; ++k) {
    uint8_t b = (uint8_t)((value >> (8U * k)) & 0xFFU);
    if (EEPROM.read(addr + k) != b) {
      EEPROM.write(addr + k, b);
      changed = true;
    }
  }
  #if defined(ARDUINO_ARCH_ESP32)
    if (changed) { EEPROM.commit(); }
  #else
    (void)changed;
  #endif
}
#endif

static const __FlashStringHelper *rtcTypeName() {
#if !THC_HAS_RTC
  return F("none / not compiled");
#elif THC_RTC_TYPE == THC_RTC_DS3231
  return F("DS3231");
#elif THC_RTC_TYPE == THC_RTC_PCF8523
  return F("PCF8523");
#else
  return F("unknown");
#endif
}

static void printVersionInfo(Stream &out) {
  out.print(F("Program: "));
  out.println(F(THC_PROGRAM_NAME));
  out.print(F("Version: "));
  out.println(F(THC_VERSION_STRING));
  out.print(F("Build: "));
  out.print(F(THC_BUILD_DATE));
  out.print(' ');
  out.println(F(THC_BUILD_TIME));
  out.print(F("Board architecture: "));
#if defined(ARDUINO_ARCH_AVR)
  out.println(F("AVR"));
#elif defined(ARDUINO_ARCH_ESP32)
  out.println(F("ESP32"));
#elif defined(ARDUINO_ARCH_SAM)
  out.println(F("SAM/Due"));
#elif defined(ARDUINO_ARCH_SAMD)
  out.println(F("SAMD"));
#elif defined(TEENSYDUINO)
  out.println(F("Teensy"));
#else
  out.println(F("unknown/generic Arduino"));
#endif
  out.print(F("PWM command range: 0.."));
  out.println(PWM_MAX);
  out.print(F("Requested PWM frequency: "));
  out.print(PWM_FREQ_HZ);
  out.println(F(" Hz"));
  out.print(F("RTC compiled: "));
  out.println(THC_HAS_RTC ? F("yes") : F("no"));
  out.print(F("RTC type: "));
  out.println(rtcTypeName());
  out.print(F("RTC detected: "));
  out.println(rtcReady ? F("yes") : F("no"));
}

#if THC_HAS_RTC
static void formatDateTime(char *buffer, size_t bufferLen, const DateTime &dt) {
  formatDateTimeParts(buffer, bufferLen, dt.year(), dt.month(), dt.day(),
                      dt.hour(), dt.minute(), dt.second());
}

static bool parseIsoDateTime(const char *s, uint16_t &year, uint8_t &month, uint8_t &day,
                             uint8_t &hour, uint8_t &minute, uint8_t &second) {
  unsigned int yy = 0, mo = 0, dd = 0, hh = 0, mm = 0, ss = 0;
  char sep = 0;
  int matched = sscanf(s, "%u-%u-%u%c%u:%u:%u", &yy, &mo, &dd, &sep, &hh, &mm, &ss);
  if (matched != 7 || (sep != ' ' && sep != 'T' && sep != 't')) {
    return false;
  }
  if (!validateDateTimeParts((uint16_t)yy, (uint8_t)mo, (uint8_t)dd,
                             (uint8_t)hh, (uint8_t)mm, (uint8_t)ss)) {
    return false;
  }
  year = (uint16_t)yy;
  month = (uint8_t)mo;
  day = (uint8_t)dd;
  hour = (uint8_t)hh;
  minute = (uint8_t)mm;
  second = (uint8_t)ss;
  return true;
}

static bool parseUIntToken(const char *begin, const char *end, uint16_t minValue,
                           uint16_t maxValue, uint16_t &value) {
  if (begin == NULL || end == NULL || begin >= end) {
    return false;
  }
  uint32_t accum = 0;
  const char *p = begin;
  while (p < end) {
    if (*p < '0' || *p > '9') {
      return false;
    }
    accum = accum * 10UL + (uint32_t)(*p - '0');
    if (accum > maxValue) {
      return false;
    }
    ++p;
  }
  if (accum < minValue || accum > maxValue) {
    return false;
  }
  value = (uint16_t)accum;
  return true;
}

static bool parseFieldDateTime(const char *s, uint16_t &year, uint8_t &month, uint8_t &day,
                               uint8_t &hour, uint8_t &minute, uint8_t &second) {
  DateTime base = rtcReady ? rtc.now() : DateTime(THC_BUILD_DATE, THC_BUILD_TIME);
  uint16_t yy = base.year();
  uint16_t mo = base.month();
  uint16_t dd = base.day();
  uint16_t hh = base.hour();
  uint16_t mi = base.minute();
  uint16_t ss = base.second();
  bool sawField = false;

  const char *p = s;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t' || *p == ',' || *p == ';') {
      ++p;
    }
    if (*p == '\0') { break; }

    char key = *p++;
    const char *numStart = p;
    while (*p >= '0' && *p <= '9') {
      ++p;
    }
    const char *numEnd = p;
    uint16_t v = 0;
    if (!parseUIntToken(numStart, numEnd, 0, 9999, v)) {
      return false;
    }

    switch (key) {
      case 'Y': case 'y':
        if (v < 100U) { v = (uint16_t)(2000U + v); }
        yy = v;
        break;
      case 'R': case 'r':  // Month; M/m is reserved for minute.
        mo = v;
        break;
      case 'D': case 'd':
        dd = v;
        break;
      case 'H': case 'h':
        hh = v;
        break;
      case 'M': case 'm':
        mi = v;
        break;
      case 'S': case 's':
        ss = v;
        break;
      default:
        return false;
    }
    sawField = true;
  }

  if (!sawField) {
    return false;
  }

  if (!validateDateTimeParts(yy, (uint8_t)mo, (uint8_t)dd,
                             (uint8_t)hh, (uint8_t)mi, (uint8_t)ss)) {
    return false;
  }

  year = yy;
  month = (uint8_t)mo;
  day = (uint8_t)dd;
  hour = (uint8_t)hh;
  minute = (uint8_t)mi;
  second = (uint8_t)ss;
  return true;
}

static bool setRtcDateTime(uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t second) {
  if (!rtcReady) {
    Serial.println(F("RTC is not available; time was not set."));
    return false;
  }
  if (!validateDateTimeParts(year, month, day, hour, minute, second)) {
    Serial.println(F("Invalid date/time. Expected range: years 2000..2099, valid month/day, 00:00:00..23:59:59."));
    return false;
  }
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  char stamp[24];
  formatDateTimeParts(stamp, sizeof(stamp), year, month, day, hour, minute, second);
  Serial.print(F("RTC set to "));
  Serial.println(stamp);
  return true;
}

static bool setRtcToBuildTime() {
  if (!rtcReady) {
    Serial.println(F("RTC is not available; build time was not written."));
    return false;
  }
  DateTime buildTime(THC_BUILD_DATE, THC_BUILD_TIME);
  rtc.adjust(buildTime);
  char stamp[24];
  formatDateTime(stamp, sizeof(stamp), buildTime);
  Serial.print(F("RTC set to build time "));
  Serial.println(stamp);
  return true;
}

static void tellRtcTime(Stream &out) {
  if (!rtcReady) {
    out.println(F("RTC is not available."));
    return;
  }
  DateTime now = rtc.now();
  char stamp[24];
  formatDateTime(stamp, sizeof(stamp), now);
  out.print(F("RTC time: "));
  out.println(stamp);
}

static bool handleTimeCommand(const char *cmd) {
  if (strcmp(cmd, "tt") == 0) {
    tellRtcTime(Serial);
    return true;
  }

  if (strcmp(cmd, "tb") == 0 || strcmp(cmd, "tB") == 0) {
    setRtcToBuildTime();
    return true;
  }

  if (cmd[0] != 't') {
    return false;
  }

  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
  const char *payload = cmd + 1;

  if (parseIsoDateTime(payload, year, month, day, hour, minute, second) ||
      parseFieldDateTime(payload, year, month, day, hour, minute, second)) {
    setRtcDateTime(year, month, day, hour, minute, second);
  } else {
    Serial.println(F("Invalid time command. Use tt, tb, tYYYY-MM-DD HH:MM:SS, or tY2026R04D28H13M45S00."));
  }
  return true;
}

static void initRtc() {
  Wire.begin();

  if (!rtc.begin()) {
    rtcReady = false;
    Serial.print(F("RTC type configured: "));
    Serial.println(rtcTypeName());
    Serial.println(F("RTC not detected. Time commands and log-start timestamps are disabled."));
    Serial.println(F("RTC diagnostics: check SDA/SCL wiring, 5V/GND, I2C address, and that THC_RTC_TYPE matches the installed chip."));
    return;
  }

  rtcReady = true;
  Serial.print(F("RTC detected: "));
  Serial.println(rtcTypeName());

  bool shouldSetToBuildTime = false;

#if THC_RTC_TYPE == THC_RTC_PCF8523
  if (!rtc.initialized()) {
    Serial.println(F("RTC reports that it has not been initialized; setting RTC to build time."));
    shouldSetToBuildTime = true;
  }
#endif

  if (rtc.lostPower()) {
    Serial.println(F("RTC reports lost power; setting RTC to build time."));
    Serial.println(F("RTC diagnostic: if this message appears after every power cycle, replace/check the RTC coin cell."));
    shouldSetToBuildTime = true;
  }

#if THC_RTC_TYPE == THC_RTC_PCF8523
  // Harmless if the oscillator is already running; important after first setup
  // or after the oscillator has been stopped.
  rtc.start();
#endif

#if THC_FORCE_RTC_BUILD_TIME_ON_EVERY_BOOT
  Serial.println(F("THC_FORCE_RTC_BUILD_TIME_ON_EVERY_BOOT is enabled."));
  shouldSetToBuildTime = true;
#elif THC_SET_RTC_FROM_BUILD_ON_NEW_FIRMWARE && THC_HAS_EEPROM
  uint32_t thisBuildHash = fnv1a32(THC_BUILD_STAMP);
  uint32_t storedBuildHash = eepromReadU32(eeBuildStampAddr);
  if (storedBuildHash != thisBuildHash) {
    Serial.println(F("New firmware build detected; setting RTC to build time once."));
    shouldSetToBuildTime = true;
    eepromWriteU32(eeBuildStampAddr, thisBuildHash);
  }
#endif

  if (shouldSetToBuildTime) {
    setRtcToBuildTime();
  }

  tellRtcTime(Serial);
}
#else
static bool handleTimeCommand(const char *cmd) {
  if (cmd[0] != 't') {
    return false;
  }
  Serial.println(F("RTC support is not compiled. Install RTClib or set THC_ENABLE_RTC=1 with RTClib available."));
  return true;
}

static void tellRtcTime(Stream &out) {
  out.println(F("RTC support is not compiled."));
}

static void initRtc() {
  rtcReady = false;
  Serial.println(F("RTC support is not compiled."));
}
#endif

static bool pinIsThermocoupleCs(uint8_t pin) {
  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    if (CSs[k] == pin) {
      return true;
    }
  }
  return false;
}

static void checkPinConfiguration() {
  if (pinIsThermocoupleCs(ACTIVE_PWM)) {
    Serial.println(F("WARNING: ACTIVE_PWM conflicts with a thermocouple CS pin."));
  }
  if (ENABLE_HEATER_PIN_B && pinIsThermocoupleCs(HEATER_PIN_B)) {
    Serial.println(F("WARNING: HEATER_PIN_B conflicts with a thermocouple CS pin."));
  }
  if (ENABLE_HEATER_PIN_C && pinIsThermocoupleCs(HEATER_PIN_C)) {
    Serial.println(F("WARNING: HEATER_PIN_C conflicts with a thermocouple CS pin."));
  }
}

static void deselectAllSpiDevices() {
  digitalWrite(SDCS, HIGH);
  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    digitalWrite(CSs[k], HIGH);
  }
}

// ------------------------------ EEPROM helpers ---------------------------------

static void initPersistentStorage() {
#if THC_HAS_EEPROM
  #if defined(ARDUINO_ARCH_ESP32)
    EEPROM.begin(EEPROM_SIZE_BYTES);
  #endif
#endif
}

static bool loadIntervalIndex(uint8_t &idx) {
#if THC_HAS_EEPROM
  uint8_t stored = EEPROM.read(eeAcqRateAddr);
  if (stored < NUM_INTERVAL_OPTIONS) {
    idx = stored;
    return true;
  }
#endif
  return false;
}

static void saveIntervalIndex(uint8_t idx) {
#if THC_HAS_EEPROM
  if (EEPROM.read(eeAcqRateAddr) != idx) {
    EEPROM.write(eeAcqRateAddr, idx);
    #if defined(ARDUINO_ARCH_ESP32)
      EEPROM.commit();
    #endif
  }
#else
  (void)idx;
#endif
}

// ------------------------------- PWM helpers -----------------------------------

#if defined(ARDUINO_ARCH_ESP32)
static int8_t esp32PwmChannelForPin(uint8_t pin) {
  if (pin == HEATER_PIN_A) { return 0; }
  if (pin == HEATER_PIN_B) { return 1; }
  if (pin == HEATER_PIN_C) { return 2; }
  return 0;
}
#endif

static void initPwmPin(uint8_t pin) {
  pinMode(pin, OUTPUT);

#if defined(ARDUINO_ARCH_ESP32)
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    // Arduino-ESP32 3.x LEDC API: pin-based attach/write.
    ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);
    ledcWrite(pin, 0);
  #else
    // Arduino-ESP32 2.x LEDC API: channel-based setup/attach/write.
    int8_t channel = esp32PwmChannelForPin(pin);
    ledcSetup(channel, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
  #endif
#elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || defined(TEENSYDUINO)
  // Due, SAMD, and Teensy-style cores generally support a global resolution.
  analogWriteResolution(PWM_BITS);
  analogWrite(pin, 0);
#else
  // Classic AVR analogWrite is 8-bit unless timer registers are modified.
  analogWrite(pin, 0);
#endif
}

static void initHeaters() {
  initPwmPin(ACTIVE_PWM);
  if (ENABLE_HEATER_PIN_B && HEATER_PIN_B != ACTIVE_PWM) {
    initPwmPin(HEATER_PIN_B);
  }
  if (ENABLE_HEATER_PIN_C && HEATER_PIN_C != ACTIVE_PWM) {
    initPwmPin(HEATER_PIN_C);
  }
}

static void writeHeaterDuty(uint8_t pin, uint16_t duty) {
  if (duty > PWM_MAX) {
    duty = PWM_MAX;
  }

#if defined(ARDUINO_ARCH_ESP32)
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);
  #else
    ledcWrite(esp32PwmChannelForPin(pin), duty);
  #endif
#elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || defined(TEENSYDUINO)
  analogWrite(pin, duty);
#else
  // Classic AVR analogWrite is 8-bit unless timer registers are modified.
  analogWrite(pin, (uint8_t)((uint32_t)duty * 255UL / PWM_MAX));
#endif
}

static void allHeatersOff() {
  writeHeaterDuty(ACTIVE_PWM, 0);
  if (ENABLE_HEATER_PIN_B && HEATER_PIN_B != ACTIVE_PWM) {
    writeHeaterDuty(HEATER_PIN_B, 0);
  }
  if (ENABLE_HEATER_PIN_C && HEATER_PIN_C != ACTIVE_PWM) {
    writeHeaterDuty(HEATER_PIN_C, 0);
  }
  currentDuty = 0;
}

// ----------------------------- MAX31856 helpers --------------------------------

static uint8_t readSingleRegister(uint8_t csPin, uint8_t reg) {
  deselectAllSpiDevices();
  SPI.beginTransaction(max31856SpiSettings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  SPI.transfer(reg & 0x7F);   // bit 7 low => read
  uint8_t data = SPI.transfer(0x00);
  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return data;
}

static uint32_t readMultipleRegisters(uint8_t csPin, uint8_t startReg, uint8_t count) {
  uint32_t data = 0;
  if (count > 4) {
    count = 4;
  }

  deselectAllSpiDevices();
  SPI.beginTransaction(max31856SpiSettings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  SPI.transfer(startReg & 0x7F);   // bit 7 low => read

  for (uint8_t k = 0; k < count; ++k) {
    data = (data << 8) | SPI.transfer(0x00);
  }

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return data;
}

static void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
  deselectAllSpiDevices();
  SPI.beginTransaction(max31856SpiSettings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  SPI.transfer(reg | 0x80);   // bit 7 high => write
  SPI.transfer(value);
  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

static float readTemperatureC(uint8_t csPin) {
  // MAX31856 linearized thermocouple temperature is a signed 19-bit value in
  // registers 0x0C..0x0E, with 0.0078125 C/LSB.
  uint32_t raw = readMultipleRegisters(csPin, 0x0C, 3);
  raw >>= 5;

  if (raw & 0x40000UL) {          // sign bit of 19-bit value
    raw |= 0xFFF80000UL;          // sign-extend to 32 bits
  }

  return (float)((int32_t)raw) * 0.0078125f;
}

static void initializeMAX31856Pins() {
  Serial.println(F("Initializing SPI chip-select pins"));

  pinMode(SDCS, OUTPUT);
  digitalWrite(SDCS, HIGH);

  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    pinMode(CSs[k], OUTPUT);
    digitalWrite(CSs[k], HIGH);
    Serial.print(CSs[k]);
    Serial.print(F(" "));
  }

  Serial.println(F("\nDone"));
}

static void initializeChannel(uint8_t csPin) {
  Serial.print(F("Initializing MAX31856 channel on CS pin "));
  Serial.println(csPin);

  for (uint8_t k = 0; k < NUM31856REGS; ++k) {
    writeRegister(csPin, RegisterAddresses[k], RegisterValues[k]);
  }
}

static bool verifyChannel(uint8_t csPin) {
  bool ok = true;

  for (uint8_t k = 0; k < NUM31856REGS; ++k) {
    uint8_t regVal = readSingleRegister(csPin, RegisterAddresses[k]);
    if (regVal != RegisterValues[k]) {
      ok = false;
      Serial.print(RegisterNames[k]);
      Serial.print(F(" has 0x"));
      Serial.print(regVal, HEX);
      Serial.print(F(" and should have 0x"));
      Serial.println(RegisterValues[k], HEX);
    }
  }

  if (ok) {
    Serial.println(F("No discrepancies found"));
  }

  return ok;
}

// ------------------------------ SD card helpers --------------------------------

static bool initSdCard() {
  Serial.print(F("Initializing SD card on CS pin "));
  Serial.print(SDCS);
  Serial.print(F("... "));

  pinMode(SDCS, OUTPUT);
  digitalWrite(SDCS, HIGH);

  if (!SD.begin(SDCS)) {
    Serial.println(F("FAILED."));
    Serial.println(F("SD diagnostics: check card insertion, FAT/FAT32 formatting, wiring, chip-select pin, and shared SPI wiring."));
    return false;
  }

  Serial.println(F("ready."));

  File root = SD.open("/");
  if (root) {
    Serial.println(F("SD root directory opened successfully."));
    root.close();
  } else {
    Serial.println(F("WARNING: SD.begin succeeded, but SD root directory could not be opened."));
  }

  // Establish the next log-file number from the card contents. This prevents
  // overwriting or reusing lower numbers after a reset/reprogram cycle.
  // Uses the common SD File::openNextFile() directory iteration API instead of
  // library-specific ls() calls.
  scanLogFileNumbers();

  return true;
}

static const char *baseNameOnly(const char *name) {
  if (name == NULL) {
    return "";
  }

  const char *slash1 = strrchr(name, '/');
  const char *slash2 = strrchr(name, '\\');
  const char *base = name;

  if (slash1 && slash1 + 1 > base) {
    base = slash1 + 1;
  }
  if (slash2 && slash2 + 1 > base) {
    base = slash2 + 1;
  }

  return base;
}

static bool parseLogFilenameIndex(const char *name, uint32_t &indexOut) {
  const char *base = baseNameOnly(name);

  // Expected 8.3 name: LOG00000.CSV. Match case-insensitively because some
  // SD libraries report lowercase names.
  if (strlen(base) != 12) {
    return false;
  }

  if (toupper((unsigned char)base[0]) != 'L' ||
      toupper((unsigned char)base[1]) != 'O' ||
      toupper((unsigned char)base[2]) != 'G') {
    return false;
  }

  uint32_t value = 0;
  for (uint8_t i = 3; i < 8; ++i) {
    if (!isdigit((unsigned char)base[i])) {
      return false;
    }
    value = value * 10UL + (uint32_t)(base[i] - '0');
  }

  if (base[8] != '.' ||
      toupper((unsigned char)base[9])  != 'C' ||
      toupper((unsigned char)base[10]) != 'S' ||
      toupper((unsigned char)base[11]) != 'V') {
    return false;
  }

  if (value >= MAX_LOG_FILES) {
    return false;
  }

  indexOut = value;
  return true;
}

static bool scanLogFileNumbers() {
  nextLogFileNumber = 0;
  logFileScanDone = false;

  if (!sdReady) {
    Serial.println(F("Cannot scan SD log files: SD card is not ready."));
    return false;
  }

  File root = SD.open("/");
  if (!root) {
    Serial.println(F("Cannot scan SD log files: root directory could not be opened."));
    return false;
  }

  uint32_t highest = 0;
  bool foundAny = false;
  uint16_t entryCount = 0;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      break;
    }

    ++entryCount;
    if (!entry.isDirectory()) {
      uint32_t idx = 0;
      if (parseLogFilenameIndex(entry.name(), idx)) {
        if (!foundAny || idx > highest) {
          highest = idx;
        }
        foundAny = true;
      }
    }

    entry.close();
  }

  root.close();

  if (foundAny) {
    if (highest + 1UL < MAX_LOG_FILES) {
      nextLogFileNumber = highest + 1UL;
      logFileScanDone = true;
      Serial.print(F("Highest existing log file index: "));
      Serial.print(highest);
      Serial.print(F("; next log file will be "));
      char tmp[13];
      snprintf(tmp, sizeof(tmp), "LOG%05lu.CSV", (unsigned long)nextLogFileNumber);
      Serial.println(tmp);
      return true;
    }

    Serial.println(F("All LOG#####.CSV names appear to be used; cannot choose a new log filename."));
    nextLogFileNumber = MAX_LOG_FILES;
    logFileScanDone = true;
    return false;
  }

  nextLogFileNumber = 0;
  logFileScanDone = true;
  Serial.print(F("No existing LOG#####.CSV files found among "));
  Serial.print(entryCount);
  Serial.println(F(" root-directory entries; next log file will be LOG00000.CSV."));
  return true;
}

static void printNextLogFilename(Stream &out) {
  if (!sdReady) {
    out.println(F("Next log filename unknown: SD card is not ready."));
    return;
  }

  if (!logFileScanDone) {
    scanLogFileNumbers();
  }

  if (nextLogFileNumber >= MAX_LOG_FILES) {
    out.println(F("Next log filename unavailable: LOG#####.CSV file-number limit reached."));
    return;
  }

  char filename[13];
  snprintf(filename, sizeof(filename), "LOG%05lu.CSV", (unsigned long)nextLogFileNumber);
  out.print(F("Next log filename: "));
  out.println(filename);
}

static bool writeCsvHeader() {
  if (!logfile) {
    Serial.println(F("Cannot write CSV header: log file is not open."));
    csvHeaderWritten = false;
    return false;
  }

  logfile.print(F("# Program,"));
  logfile.println(F(THC_PROGRAM_NAME));
  logfile.print(F("# Version,"));
  logfile.println(F(THC_VERSION_STRING));
  logfile.print(F("# Build date/time,"));
  logfile.print(F(THC_BUILD_DATE));
  logfile.print(' ');
  logfile.println(F(THC_BUILD_TIME));
  logfile.print(F("# Log start date/time,"));
#if THC_HAS_RTC
  if (rtcReady) {
    DateTime now = rtc.now();
    char stamp[24];
    formatDateTime(stamp, sizeof(stamp), now);
    logfile.println(stamp);
  } else {
    logfile.println(F("RTC unavailable"));
  }
#else
  logfile.println(F("RTC support not compiled"));
#endif
  logfile.print(F("# Acquisition interval index,"));
  logfile.println(intervalIndex);
  logfile.print(F("# Acquisition period ms,"));
  logfile.println(samplePeriodMs);
  logfile.print(F("# PWM command range,0.."));
  logfile.println(PWM_MAX);
  logfile.print(F("# Requested PWM frequency Hz,"));
  logfile.println(PWM_FREQ_HZ);

  // Data columns only. The date/time is recorded once above in the metadata
  // header as "# Log start date/time"; it is not repeated on every data row.
  logfile.print(F("elapsed_time_s"));
  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    logfile.print(F(",TC"));
    logfile.print(k + 1);
    logfile.print(F("_C"));
  }
  logfile.println(F(",power_command"));
  logfile.flush();

  csvHeaderWritten = true;
  Serial.print(F("CSV header written to "));
  Serial.print(currentLogFilename[0] ? currentLogFilename : "<unnamed>");
  Serial.print(F("; file size is now "));
  Serial.print(logfile.size());
  Serial.println(F(" bytes."));
  return true;
}

static bool openNewLogFile() {
  if (!sdReady) {
    Serial.println(F("Cannot open log file: SD card is not ready."));
    return false;
  }

  if (logFileOpen) {
    return true;
  }

  if (!logFileScanDone) {
    scanLogFileNumbers();
  }

  char filename[13];  // 8.3 name: LOG00000.CSV plus NUL.

  for (uint32_t n = nextLogFileNumber; n < MAX_LOG_FILES; ++n) {
    snprintf(filename, sizeof(filename), "LOG%05lu.CSV", (unsigned long)n);

    if (!SD.exists(filename)) {
      logfile = SD.open(filename, FILE_WRITE);
      if (!logfile) {
        Serial.print(F("Could not create "));
        Serial.println(filename);
        return false;
      }

      strncpy(currentLogFilename, filename, sizeof(currentLogFilename));
      currentLogFilename[sizeof(currentLogFilename) - 1] = '\0';
      csvHeaderWritten = false;

      Serial.print(F("Logging to "));
      Serial.println(currentLogFilename);

      if (!writeCsvHeader()) {
        Serial.println(F("Failed to write CSV header; closing log file."));
        logfile.close();
        currentLogFilename[0] = '\0';
        logFileOpen = false;
        return false;
      }

      logFileOpen = true;
      nextLogFileNumber = n + 1UL;
      logFileScanDone = true;
      return true;
    }
  }

  Serial.println(F("Could not create a log file: file-number limit reached."));
  currentLogFilename[0] = '\0';
  csvHeaderWritten = false;
  return false;
}

static void writeDataToSD() {
  if (!logFileOpen) {
    Serial.println(F("writeDataToSD called, but no log file is open."));
    return;
  }

  if (!csvHeaderWritten) {
    Serial.println(F("CSV header was not marked as written; attempting to write it now."));
    if (!writeCsvHeader()) {
      Serial.println(F("Cannot write data because the CSV header failed."));
      return;
    }
  }

  logfile.print(elapsedLogTimeS, 3);
  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    logfile.print(',');
    logfile.print(tempC[k], 4);
  }
  logfile.print(',');
  logfile.println(currentDuty);

  // Flush periodically to limit data loss without imposing huge write overhead.
  if ((numDataPoints % 10UL) == 0UL) {
    logfile.flush();
  }
}
static void listDirectory(File dir, uint8_t levels) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }

    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(F("/"));
      if (levels > 0) {
        listDirectory(entry, levels - 1);
      }
    } else {
      Serial.print(F("\t"));
      Serial.println(entry.size());
    }

    entry.close();
  }
}

static void listSdFiles() {
  if (!sdReady) {
    Serial.println(F("SD card is not ready."));
    return;
  }

  File root = SD.open("/");
  if (!root) {
    Serial.println(F("Could not open SD root directory."));
    return;
  }

  Serial.println(F("Files found on the card:"));
  listDirectory(root, 1);
  root.close();

  // Re-scan after listing in case the card was modified outside the sketch.
  scanLogFileNumbers();
  printNextLogFilename(Serial);
}

// ----------------------------- Command processing -----------------------------

static void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
#if ECHO_INPUT
      Serial.println();
#endif
      if (inputOverflow) {
        Serial.println(F("Input command too long; discarded."));
        inputOverflow = false;
        inputIndex = 0;
        inputBuffer[0] = '\0';
        commandReady = false;
      } else if (inputIndex > 0) {
        inputBuffer[inputIndex] = '\0';
        commandReady = true;
      }
      return;
    }

    if (c == 8 || c == 127) {  // Backspace or DEL.
      if (inputIndex > 0) {
        --inputIndex;
        inputBuffer[inputIndex] = '\0';
#if ECHO_INPUT
        Serial.print('\b');
        Serial.print(' ');
        Serial.print('\b');
#endif
      }
      continue;
    }

    if (inputIndex < INPUT_BUFFER_LEN - 1) {
      inputBuffer[inputIndex++] = c;
#if ECHO_INPUT
      Serial.print(c);
#endif
    } else {
      inputOverflow = true;
    }
  }
}

static void scheduleNextSampleFromNow() {
  nextSampleMs = millis() + (uint32_t)samplePeriodMs;
}

static void applyAcquisitionInterval(uint8_t newIndex, bool persist) {
  intervalIndex = newIndex;
  samplePeriodMs = updateIntervalsMs[intervalIndex];
  samplePeriodS = (float)samplePeriodMs / 1000.0f;
  scheduleNextSampleFromNow();

  if (persist) {
    saveIntervalIndex(intervalIndex);
#if THC_HAS_EEPROM
    Serial.print(F("Saved acquisition interval index to EEPROM: "));
    Serial.println(intervalIndex);
#else
    Serial.println(F("WARNING: EEPROM support is not compiled; acquisition interval will not persist after reset."));
#endif
  }
}

static void printAcquisitionInterval(Stream &out) {
  out.print(F("Acquisition interval index = "));
  out.print(intervalIndex);
  out.print(F(", period = "));
  out.print(samplePeriodMs);
  out.println(F(" ms"));
}

static void startLogging() {
  Serial.println(F("Logging requested."));
  if (!sdReady) {
    Serial.println(F("WARNING: SD card is not ready, so no file can be created yet."));
  }
  if (!logFileOpen) {
    createFile = true;
  } else {
    Serial.print(F("Continuing existing log file: "));
    Serial.println(currentLogFilename);
  }
  saveData = true;
  numDataPoints = 0;
  elapsedLogTimeS = 0.0f;
  readTempNow = true;
}

static void parseSerialInput() {
  char *cmd = trimCommand(inputBuffer);

  if (*cmd == '\0') {
    return;
  }

  if (strcmp(cmd, "h") == 0) {
    printHelp(Serial);
    return;
  }

  if (strcmp(cmd, "v") == 0) {
    printVersionInfo(Serial);
    return;
  }

  if (strcmp(cmd, "d") == 0) {
    printStartupDiagnostics();
    return;
  }

  if (cmd[0] == 't') {
    handleTimeCommand(cmd);
    return;
  }

  if (strcmp(cmd, "a") == 0) {
    stopAllFlag = true;
    return;
  }

  if (strcmp(cmd, "l") == 0) {
    listFilesFlag = true;
    return;
  }

  if (strcmp(cmd, "L") == 0) {
    startLogging();
    return;
  }

  if (strcmp(cmd, "ss") == 0) {
    printAcqRateFlag = true;
    return;
  }

  if (cmd[0] == 's') {
    uint16_t idx = 0;
    if (parseUnsignedInRange(cmd + 1, 0, NUM_INTERVAL_OPTIONS - 1, idx)) {
      intervalIndex = (uint8_t)idx;
      setAcqRateFlag = true;
    } else {
      Serial.println(F("Invalid acquisition interval. Use s0 through s10, or ss to print the current rate."));
    }
    return;
  }

  if (cmd[0] == 'W') {
    uint16_t duty = 0;
    if (parseUnsignedInRange(cmd + 1, 0, PWM_MAX, duty)) {
      requestedDuty = duty;
      setDutyFlag = true;
      startLogging();
    } else {
      Serial.println(F("Invalid duty cycle. Use W0 through W1023, e.g. W512."));
    }
    return;
  }

  if (cmd[0] == 'w') {
    uint16_t duty = 0;
    if (parseUnsignedInRange(cmd + 1, 0, PWM_MAX, duty)) {
      requestedDuty = duty;
      setDutyFlag = true;
      if (saveData && logFileOpen) {
        Serial.println(F("Duty change requested; continuing current log timeline."));
      } else {
        Serial.println(F("Duty change requested; logging is not active, so no timeline was changed."));
        Serial.println(F("Use W#### to set duty and start/restart logging, or L to start a new log."));
      }
    } else {
      Serial.println(F("Invalid duty cycle. Use w0 through w1023, e.g. w512."));
    }
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(cmd);
  Serial.println(F("Type h for help."));
}

static void clearInputBuffer() {
  memset(inputBuffer, 0, sizeof(inputBuffer));
  inputIndex = 0;
  commandReady = false;
  inputOverflow = false;
}

// ------------------------------- Main actions ----------------------------------

static void sampleThermocouples() {
  deselectAllSpiDevices();

  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    tempC[k] = readTemperatureC(CSs[k]);
  }

  if (monitorData) {
    Serial.print(elapsedLogTimeS, 3);
    for (uint8_t k = 0; k < NUM_TCS; ++k) {
      Serial.print('\t');
      Serial.print(tempC[k], 4);
    }
    Serial.print('\t');
    Serial.println(currentDuty);
  }

  ++numDataPoints;

  if (saveData) {
    writeDataToSD();
  }

  elapsedLogTimeS += samplePeriodS;
}

static void stopAllAndHalt() {
  allHeatersOff();
  deselectAllSpiDevices();

  if (logFileOpen) {
    logfile.flush();
    logfile.close();
    logFileOpen = false;
    csvHeaderWritten = false;
    currentLogFilename[0] = '\0';
  }

  Serial.println(F("Stopped. Heaters are off and the log file is closed. Reset the board to restart."));

  while (true) {
    delay(1000);
  }
}


static void clearPendingSerialInput() {
  while (Serial.available() > 0) {
    (void)Serial.read();
  }
}

static void printStartupDiagnostics() {
#if THC_PRINT_STARTUP_DIAGNOSTICS
  Serial.println(F("\n--- Startup/runtime diagnostics ---"));
  Serial.print(F("Serial baud: "));
  Serial.println(SERIAL_BAUD);
  Serial.print(F("Serial startup delay: "));
  Serial.print((uint32_t)THC_SERIAL_STARTUP_DELAY_MS);
  Serial.println(F(" ms"));
  Serial.print(F("Program: "));
  Serial.println(F(THC_PROGRAM_NAME));
  Serial.print(F("Version: "));
  Serial.println(F(THC_VERSION_STRING));
  Serial.print(F("Build: "));
  Serial.print(F(THC_BUILD_DATE));
  Serial.print(' ');
  Serial.println(F(THC_BUILD_TIME));
  Serial.print(F("RTC compiled: "));
  Serial.println(THC_HAS_RTC ? F("yes") : F("no"));
  Serial.print(F("RTC type: "));
  Serial.println(rtcTypeName());
  Serial.print(F("RTC detected: "));
  Serial.println(rtcReady ? F("yes") : F("no"));
  Serial.print(F("EEPROM compiled: "));
  Serial.println(THC_HAS_EEPROM ? F("yes") : F("no"));
#if THC_HAS_RTC
  if (rtcReady) {
    tellRtcTime(Serial);
  }
#endif
  Serial.print(F("SD ready: "));
  Serial.println(sdReady ? F("yes") : F("no"));
  Serial.print(F("Log file open: "));
  Serial.println(logFileOpen ? F("yes") : F("no"));
  if (logFileOpen) {
    Serial.print(F("Log filename: "));
    Serial.println(currentLogFilename);
    Serial.print(F("CSV header written: "));
    Serial.println(csvHeaderWritten ? F("yes") : F("no"));
    Serial.print(F("Current file size: "));
    Serial.print(logfile.size());
    Serial.println(F(" bytes"));
  }
  Serial.print(F("Save data enabled: "));
  Serial.println(saveData ? F("yes") : F("no"));
  Serial.print(F("Monitor data enabled: "));
  Serial.println(monitorData ? F("yes") : F("no"));
  Serial.print(F("Acquisition interval index: "));
  Serial.println(intervalIndex);
  Serial.print(F("Acquisition period ms: "));
  Serial.println(samplePeriodMs);
  Serial.print(F("Active PWM pin: "));
  Serial.println(ACTIVE_PWM);
  Serial.print(F("PWM command range: 0.."));
  Serial.println(PWM_MAX);
  Serial.print(F("Current duty command: "));
  Serial.println(currentDuty);
  Serial.println(F("--- End diagnostics ---\n"));
#endif
}

// ---------------------------------- setup/loop ---------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Avoid hanging forever on boards without native USB Serial. On boards such
  // as the Mega, Serial is ready immediately, but giving the USB/serial bridge
  // and Serial Monitor a short settling interval greatly reduces garbled first
  // characters after auto-reset.
  uint32_t serialStart = millis();
  while (!Serial && (millis() - serialStart < (uint32_t)THC_SERIAL_READY_TIMEOUT_MS)) {
    delay(10);
  }
  delay((uint32_t)THC_SERIAL_STARTUP_DELAY_MS);
  clearPendingSerialInput();

  Serial.println();
  Serial.println(F("============================================================"));
  Serial.print(F(THC_PROGRAM_NAME));
  Serial.print(F(" v"));
  Serial.println(F(THC_VERSION_STRING));
  Serial.print(F("Build: "));
  Serial.print(F(THC_BUILD_DATE));
  Serial.print(' ');
  Serial.println(F(THC_BUILD_TIME));
  Serial.println(F("Booting..."));
  serialStartupPace();

  initPersistentStorage();
  initRtc();
  serialStartupPace();
  checkPinConfiguration();
  serialStartupPace();

#if THC_HAS_EEPROM
  uint8_t storedInterval = intervalIndex;
  if (loadIntervalIndex(storedInterval)) {
    intervalIndex = storedInterval;
    Serial.print(F("Loaded acquisition interval index from EEPROM: "));
    Serial.println(intervalIndex);
  } else {
    Serial.print(F("No valid EEPROM acquisition interval found; using default index: "));
    Serial.println(intervalIndex);
  }
#else
  Serial.println(F("EEPROM support is not compiled; using default acquisition interval."));
#endif
  applyAcquisitionInterval(intervalIndex, false);

  initHeaters();
  allHeatersOff();
  serialStartupPace();

  SPI.begin();
  initializeMAX31856Pins();
  serialStartupPace();

  sdReady = initSdCard();
  serialStartupPace();

  for (uint8_t k = 0; k < NUM_TCS; ++k) {
    initializeChannel(CSs[k]);
    delay(10);
    Serial.print(F("Verifying CS pin "));
    Serial.println(CSs[k]);
    verifyChannel(CSs[k]);
    serialStartupPace();
  }

  printAcquisitionInterval(Serial);
  Serial.print(F("Active PWM pin: "));
  Serial.println(ACTIVE_PWM);
  printStartupDiagnostics();
  Serial.println(F("READY"));
  Serial.println(F("Type h for help."));
  Serial.println(F("If the Arduino IDE Serial Monitor does not follow new lines, enable its Autoscroll toggle."));
}

void loop() {
  handleSerialInput();

  if (commandReady) {
    parseSerialInput();
    clearInputBuffer();
  }

  if (printAcqRateFlag) {
    printAcquisitionInterval(Serial);
    printAcqRateFlag = false;
  }

  if (setAcqRateFlag) {
    applyAcquisitionInterval(intervalIndex, true);
    printAcquisitionInterval(Serial);
    setAcqRateFlag = false;
  }

  if (listFilesFlag) {
    listSdFiles();
    listFilesFlag = false;
  }

  if (createFile) {
    if (!openNewLogFile()) {
      Serial.println(F("Logging was requested, but no log file could be opened. Heaters will be turned off."));
      stopAllFlag = true;
    }
    createFile = false;
  }

  if (setDutyFlag) {
    currentDuty = requestedDuty;
    writeHeaterDuty(ACTIVE_PWM, currentDuty);
    Serial.print(F("Duty cycle = "));
    Serial.println(currentDuty);
    setDutyFlag = false;
  }

  uint32_t now = millis();
  if ((int32_t)(now - nextSampleMs) >= 0) {
    readTempNow = true;
    nextSampleMs += (uint32_t)samplePeriodMs;

    // If the loop was blocked for more than one period, resynchronize rather
    // than trying to catch up with a burst of stale samples.
    if ((int32_t)(now - nextSampleMs) >= 0) {
      scheduleNextSampleFromNow();
    }
  }

  if (readTempNow) {
    sampleThermocouples();
    readTempNow = false;
  }

  if (stopAllFlag) {
    stopAllAndHalt();
  }
}
