//-------------------------------------------------------------------------------------------------
//
// Clock OS Operating System by Terry Clarkson      10-28-2011
//
// Thank you to Maurice Ribble from ClockOS for the 1307 RTC code used in this project
// 1307 RTC code by Maurice Ribble 2008-04-17
// http://www.glacialwanderer.com/hobbyrobotics
// The DS1307 works in Binary Coded Decimal.
//
// History
// 1.0   Basic clock routine PIN_PB1 set time in CCW mode PIN_PB3 advance time
// 1.1   Add seconds clock routine
// 1.2   Add ability for user to set colors with 3 buttons, saves setup to eeprom
// 1.3   Added servo routines and sound routine
// 1.4   Added ability to save 10 clock faces
// 1.5   Atmel now controls all clock functions, PIC is slave processor for multiplexing LEDS
// 1.6   Removed servo and sound routines, cleanup and document all code
//
//-------------------------------------------------------------------------------------------------
//
// Additional improvements and additions by Hazze Molin
//
// * Implemented support for 7-segment display board connecting a HT16K33 to I2C for
//   displaying time/date and easier configuration.
// * The code for the pendulum functionality was removed (sorry guys).
// * Simple menu system to set date and time, and program clock faces.
// * You can mix and match "dot", "trace", and small "hands" in every clock face.
// * You can select markers for every "hour", "quarter", or "twelth" position only.
// * You can select if the time colons should flash or be static.
// * You can choose to display time only, date only, or alternating time and date.
// * The alternation speed can be selected in steps of every 1, 2, 5, 10, 15, 30, 60 seconds.
// * Using the same code for config and time display.
//
// History - Hazze Molin forking for adding 7-segment display board
// 2.0 2020-04-12   Created 7-Digit display board in KiCad.
// 2.1 2020-04-30   Refactoring most of the code to make it more efficient.
//                  Adding HT16K33 to control 6 pcs of 7 segment LED displays for showing
//                  time, date and configuration.
//                  Removed pendulum display functionality.
// 2.2 2023-11-05   Converted project to PlatformIO.
//
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// NOTE WHEN PROGRAMMING ClockOS ARDUINO:
// The ClockOS board looks like an Arduino PRO Mini (5V 16mhz AT168)
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//  Note 512 bytes of Eeprom available!
//       1k of RAM
//       14k of Flash
//
//  Command list from Atmel chip to PIC always consist of 5 bytes
//  Start code is 0xF#, then three command bytes and then Ends with 0x03
//  Set [unused] data as 0x00.
//
//  *** Turn on or off selected circle position LED(s) (Turn off using color = 0)
//  0xF1, [hours,min,sec], [position], [color 0-7], 0x03
//        4, 2, 1 => 1-7   1-60
//
//  *** Move forward clockwise the LED pattern ***
//  0xF2, [hours,min,sec], [number of postions to move forward], [unused], 0x03
//        4, 2, 1 => 1-7
//
//  *** Move reverse counter-clockwise the LED pattern ***
//  0xF3, [hours,min,sec], [number of postions to move back], [unused], 0x03
//        4, 2, 1 => 1-7
//
//  *** Meter mode ***    TODO IS REALLY 6 BYTES ???
//  0xF4, [hours,min,sec], [position start number], [position end number], [color], 0x03 
//        4, 2, 1 => 1-7
//
//  *** Turn off all LEDs in selected circle ***    TODO IS REALLY 6 BYTES ???
//  0xF5, [hours,min,sec], [unused], [unused], [unused], 0x03
//        4, 2, 1 => 1-7
//
//  *** Turn off all LEDs in all circles ***
//  0xF6, [unused], [unused], [unused], 0x03
//
//-------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <EEPROM.h>
#include "Wire.h"

//  Define I2C addresses
#define HT16K33_I2C_ADDRESS   0x70
#define DS1307_I2C_ADDRESS    0x68

//  Define delays (in milliseconds)
#define ANIMATION_SHORT_DELAY         10
#define ANIMATION_KEY_DELAY           50
#define BUTTON_DEBOUNCE_SHORT_DELAY   100
#define BUTTON_PAUSE_SHORT_DELAY      20
#define BUTTON_PAUSE_LONG_DELAY       450
#define EDIT_POSITION_FLASH_DELAY     500

//  Define key press combinations
#define KEY_PRESSED_NONE  0x00
#define KEY_PRESSED_1     0x01
#define KEY_PRESSED_2     0x02
#define KEY_PRESSED_3     0x04
#define KEY_PRESSED_1_2   0x03
#define KEY_PRESSED_1_3   0x05
#define KEY_PRESSED_2_3   0x06
#define KEY_PRESSED_1_2_3 0x07

//  Define positions when configuring the clock
#define SET_POSITION_NONE         0x00
#define SET_POSITION_HOURS        0x01
#define SET_POSITION_MINUTES      0x02
#define SET_POSITION_SECONDS      0x03
#define SET_POSITION_YEAR         0x04
#define SET_POSITION_MONTH        0x05
#define SET_POSITION_DAY          0x06
#define SET_POSITION_MARKERS      0x08
#define SET_POSITION_CLOCK_FACE   0x10
#define SET_POSITION_TIME_DATE    0x11
#define SET_POSITION_ALT_TIMER    0x12
#define SET_POSITION_FLASH_COLON  0x13

//  Define mode LED settings
#define MODE_LED_NONE           0x00
#define MODE_LED_SET_UNUSED     0x01
#define MODE_LED_SET_TIME_DATE  0x02
#define MODE_LED_SET_SETTINGS   0x04
#define MODE_LED_SET_STYLING    0x08
#define MODE_LED_RESET          0x0f

//  Define Eeprom memory positions
#define EEPROM_CLOCK_FACE_NUMBER    0
#define EEPROM_DATE_TIME_AND_COLON  1
#define EEPROM_ALTERNATE_COUNTER    2
#define EEPROM_CLOCK_FACE_SETTINGS  10

//  Define Eeprom memory size for each clock face
#define DEFAULT_CLOCK_FACE_LENGTH 10

//  Define number of factory clock faces
#define DEFAULT_FACTORY_CLOCK_FACES 10

#define MARKER_HOUR_EVERY         0x10
#define MARKER_HOUR_QUARTERS      0x20
#define MARKER_HOUR_TWELTH        0x40
#define MARKER_BIT_HOUR_EVERY     4
#define MARKER_BIT_HOUR_QUARTERS  5
#define MARKER_BIT_HOUR_TWELTH    6

#define COLOR_HANDS       0x10
#define COLOR_DOT         0x20
#define COLOR_TRACE       0x40
#define COLOR_BIT_HANDS   4
#define COLOR_BIT_DOT     5
#define COLOR_BIT_TRACE   6

//  Define PIC commands
#define RING_CMD_UNUSED       0x00
#define RING_CMD_ON_OFF_LEDS  0xF1
#define RING_CMD_MOVE_FORWARD 0xF2
#define RING_CMD_MOVE_REVERSE 0xF3
#define RING_CMD_METER_LEDS   0xF4
#define RING_CMD_OFF_LEDS     0xF5
#define RING_CMD_OFF_ALL_LEDS 0xF6
#define RING_CMD_END          0x03

//  Define LED rings commands
#define RING_NONE                   0x00
#define RING_SECONDS                0x01
#define RING_MINUTES                0x02
#define RING_MINUTES_SECONDS        0x03
#define RING_HOURS                  0x04
#define RING_HOURS_SECONDS          0x05
#define RING_HOURS_MINUTES          0x06
#define RING_HOURS_MINUTES_SECONDS  0x07

//  Define color start and end indexes
#define COLORS_START  0
#define COLORS_END    7

//  Define LED colors for matching PIC colors
#define COLOR_BLANK   0x00
#define COLOR_RED     0x01    
#define COLOR_GREEN   0x02
#define COLOR_ORANGE  0x03    // Yellow in PIC code
#define COLOR_BLUE    0x04
#define COLOR_PURPLE  0x05
#define COLOR_CYAN    0x06
#define COLOR_WHITE   0x07

//  Define 7-segments display
#define DISPLAY_COLONS_OFF                0x00
#define DISPLAY_COLONS_ON                 0x01
#define DISPLAY_COLONS_FLASH_EVERY_SECOND 0x02
#define DISPLAY_COLONS_BOTTOM_TWO         0x04
#define DISPLAY_COLONS_TOP_TWO            0x08

#define DISPLAY_NONE          0x00
#define DISPLAY_TIME          0x10
#define DISPLAY_DATE          0x20
#define DISPLAY_TIME_AND_DATE 0x30
#define DISPLAY_CONFIG        0x40
#define DISPLAY_SETTINGS      0x80
#define DISPLAY_RESET         0xf0

#define LED_SEGMENT_ZERO_BYTE  0x00

//  Define button pins
#define PIN_BUTTON1   8
#define PIN_BUTTON2   9
#define PIN_BUTTON3   10

//  Define modes
#define MODE_NORMAL             0
#define MODE_SET_STYLING        1
#define MODE_SET_SETTINGS       2
#define MODE_SET_TIME_AND_DATE  3


byte mode = MODE_NORMAL;
byte pressedKeys = KEY_PRESSED_NONE;

//  Clock face variables
byte clockFace = 0;

//  Date and Time variables
byte hours, minutes, seconds, years, months, dayOfMonth, dayOfWeek;
byte hoursHand = 0;
byte previousHoursHand = 0;
byte previousHours = 0;
byte previousMinutes = 0;
byte previousSeconds = 0;
byte previousYears = 0;
byte previousMonths = 0;
byte previousDayOfMonth = 0;

// Clock display variables
byte markers = RING_NONE;
byte loopMarker = 0;

#define DISP_CHAR_BLANK     ' '
#define DISP_CHAR_SELECTED  ' '
const char DISP_HELLO[] PROGMEM = "HELLO ";
const char DISP_RESET[] PROGMEM = "rESEt ";
const char DISP_SELECT[] PROGMEM = "SELECt";
const char DISP_FACE[] PROGMEM = "FACE  ";
const char DISP_MENU_FACE[] PROGMEM = "FACE  ";
const char DISP_MENU_CLOCK[] PROGMEM = "CLOC  ";
const char DISP_MENU_DISPLAY[] PROGMEM = "dISP  ";

const byte valueTimeDateMin[] = {0, 0, 0, 0, 1, 1};
const byte valueTimeDateMax[] = {23, 59, 59, 99, 12, 31};

const byte valueAltTimes[] = {1, 2, 5, 10, 15, 30, 60};

//  7-segments display board variables
byte ledSegmentsBrightness = 9;
byte ledSegmentsStatus = MODE_LED_NONE;
byte ledSegmentsColons = DISPLAY_COLONS_OFF;
byte ledSegmentsDisplay = DISPLAY_TIME;
byte ledSegmentsSettings = DISPLAY_TIME_AND_DATE | DISPLAY_COLONS_FLASH_EVERY_SECOND;
byte ledSegmentsCounter = 0;
byte ledSegmentsToggleSeconds = 10;
char segmentsDisplayChars[7];

//  Common configuration variables
bool exitFlag = false;
bool isButtonPressed = false;
byte position = 0;
byte settingsChangedFlag = 0;
byte blinkUpdate = 0;
bool blinkActive = false;
unsigned long blinkTimer = 0;

//  Setup default initial colors
byte hoursMarkerColor = COLOR_PURPLE | MARKER_HOUR_EVERY;
byte hoursColor = COLOR_RED;
byte minutesColor = COLOR_RED;
byte secondsColor = COLOR_GREEN;

//  DEFAULT_FACTORY_COLORS(hoursMarkers, hours, minutes, seconds)
//
//  hoursMarkers:
//  bit 4 = every hours
//  bit 5 = quarter hours
//  bit 6 = 12th hours only
//  bit 4, 5, 6 = 0 no hours markers
//
//  hours/minutes/seconds:
//  bit 4 = hands mode active
//  bit 5 = dot mode active
//  bit 6 = trace mode active
//
const byte DEFAULT_FACTORY_COLORS[DEFAULT_FACTORY_CLOCK_FACES][4] =
{
  // Hands examples
  {COLOR_BLUE|MARKER_HOUR_EVERY, COLOR_CYAN|COLOR_HANDS, COLOR_GREEN|COLOR_HANDS, COLOR_RED|COLOR_HANDS},
  {COLOR_PURPLE|MARKER_HOUR_QUARTERS, COLOR_CYAN|COLOR_TRACE, COLOR_GREEN|COLOR_HANDS, COLOR_RED|COLOR_DOT},

  // Trace examples
  {COLOR_BLUE|MARKER_HOUR_EVERY, COLOR_BLANK|COLOR_DOT, COLOR_BLANK|COLOR_DOT, COLOR_RED|COLOR_TRACE},
  {COLOR_RED|MARKER_HOUR_QUARTERS, COLOR_BLANK|COLOR_DOT, COLOR_BLANK|COLOR_DOT, COLOR_BLUE|COLOR_TRACE},
  {COLOR_ORANGE|MARKER_HOUR_TWELTH, COLOR_BLANK|COLOR_DOT, COLOR_GREEN|COLOR_TRACE, COLOR_BLUE|COLOR_TRACE},

  // Simple dot examples
  {COLOR_BLANK, COLOR_BLANK|COLOR_DOT, COLOR_BLANK|COLOR_DOT, COLOR_RED|COLOR_DOT},
  {COLOR_BLANK, COLOR_BLUE|COLOR_DOT, COLOR_GREEN|COLOR_DOT, COLOR_RED|COLOR_DOT},
  {COLOR_BLANK, COLOR_BLANK|COLOR_DOT, COLOR_BLANK|COLOR_DOT, COLOR_RED|COLOR_TRACE},

  // Only traces examples
  {COLOR_BLUE|MARKER_HOUR_EVERY,COLOR_CYAN|COLOR_TRACE, COLOR_GREEN|COLOR_TRACE, COLOR_RED|COLOR_TRACE},
  {COLOR_BLANK, COLOR_BLANK|COLOR_TRACE, COLOR_GREEN|COLOR_TRACE, COLOR_RED|COLOR_TRACE}
};

//  ====================================================================================

// Convert normal decimal numbers to binary coded decimal
//
byte decToBcd(byte val) {
  return (val/10*16) + (val%10);
}
 
// Convert binary coded decimal to normal decimal numbers
//
byte bcdToDec(byte val) {
  return (val/16*10) + (val%16);
}

//  ====================================================================================

// Stops the DS1307, but it has the side effect of setting seconds to 0
// Probably only want to use this for testing
/*void stopDs1307() {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  byte i = 0;
  Wire.write(i);
  Wire.write(0x80);
  Wire.endTransmission();
}*/

// 1) Sets the date and time on the DS1307
// 2) Starts the clock
// 3) Sets hours mode to 24 hours clock
// Note! Assumes you're passing in valid numbers
//
void setDateDs1307(byte seconds,        // 0-59
                   byte minutes,        // 0-59
                   byte hours,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte months,         // 1-12
                   byte years) {        // 0-99

   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   Wire.write(0); // Register address 0x00
   Wire.write(decToBcd(seconds));      // Clear bit 7 starts the clock
   Wire.write(decToBcd(minutes));
   Wire.write(decToBcd(hours));        // If you want 12 hours am/pm you need to set bit 6.
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(months));
   Wire.write(decToBcd(years));
   Wire.endTransmission();
}

// Gets the date and time from the DS1307
//
void getDateDs1307(byte *seconds,
          byte *minutes,
          byte *hours,
          byte *dayOfWeek,
          byte *dayOfMonth,
          byte *months,
          byte *years) {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0); // Register address
  Wire.endTransmission();

  // A few of these need masks because certain bits are control bits
  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
  *seconds     = bcdToDec(Wire.read() & 0x7f);
  *minutes     = bcdToDec(Wire.read());
  *hours       = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek   = bcdToDec(Wire.read());
  *dayOfMonth  = bcdToDec(Wire.read());
  *months      = bcdToDec(Wire.read());
  *years       = bcdToDec(Wire.read());
}

//  ====================================================================================

void initUserSelect() {
  exitFlag = false;
  isButtonPressed = false;
  
  blinkTimer = 0;
  blinkActive = false;
  blinkUpdate = 0;
}

void updateBlinkTimer() {
  if (millis() - blinkTimer > EDIT_POSITION_FLASH_DELAY) {
    blinkTimer = millis();
    if (blinkUpdate == 0) {
      blinkUpdate = 1;
      blinkActive = !blinkActive;
    }
  }
}

//  ====================================================================================

byte readKeys() {
  byte result = KEY_PRESSED_NONE;
  if (digitalRead(PIN_BUTTON1) == LOW) {
    result = (result | KEY_PRESSED_1);
  }
  if (digitalRead(PIN_BUTTON2) == LOW) {
    result = (result | KEY_PRESSED_2);
  }
  if (digitalRead(PIN_BUTTON3) == LOW) {
    result = (result | KEY_PRESSED_3);
  }
  return result;
}

byte readPressedKeys() {
  byte result1 = readKeys();

  // Debouncing
  delay(BUTTON_DEBOUNCE_SHORT_DELAY);

  byte result2 = readKeys();

  return (result1 == result2 ? result1 : KEY_PRESSED_NONE);
}

void waitForReleaseAllButtons() {
  byte pressed = readKeys();
  while (pressed != KEY_PRESSED_NONE) {
    delay(BUTTON_PAUSE_SHORT_DELAY);
    pressed = readKeys();
  }

  // Simple debouncing delay
  delay(BUTTON_DEBOUNCE_SHORT_DELAY);

  pressed = readKeys();
  while (pressed != KEY_PRESSED_NONE) {
    delay(BUTTON_PAUSE_SHORT_DELAY);
    pressed = readKeys();
  }
}

//  ====================================================================================

/**
 * NOTE! Only translsates values of 0-15 to 0-9, a-f
 */
char translateValueToHex(byte value) {
  char result = '?';
  if (value >= 0 && value < 10) {
    result = value + '0';
  } else if (value > 9 && value < 16) {
    result = (value-10) + 'a';
  }
  return result;
}

//  ====================================================================================

byte translateCharTo7SegDigit(char value, boolean hideZeros) {
  switch(value) {
    case ' ':
      return B00000000;
    case '-':
      return B01000000;
    case '_':
      return B00001000;
    case '=':
      return B01001000;
    case '0':
      if (hideZeros) {
        return B00000000;
      } else {
        return B00111111;
      }        
    case '1':
    case 'i':
    case 'I':
      return B00000110;
    case '2':
      return B01011011;
    case '3':
      return B01001111;
    case '4':
      return B01100110;
    case '5':
    case 's':
    case 'S':
      return B01101101;
    case '6':
      return B01111101;
    case '7':
      return B00000111;
    case '8':
      return B01111111;
    case '9':
      return B01101111;
    case 'A':
      return B01110111;
    case 'b':
      return B01111100;
    case 'C':
      return B00111001;
    case 'd':
      return B01011110;
    case 'E':
      return B01111001;
    case 'F':
      return B01110001;
    case 'G':
      return B00111101;
    case 'h':
      return B01110100;
    case 'H':
      return B01110110;
    case 'J':
      return B00011110;
    case 'L':
      return B00111000;
    case 'n':
      return B01010100;
    case 'o':
      return B01011100;
    case 'O':
      return B00111111;
    case 'P':
      return B01110011;
    case 'Q':
      return B01100111;
    case 'r':
      return B01010000;
    case 't':
      return B01111000;
    case 'U':
      return B00111110;
    default:  // ?
      return B01010011;
  }
}

//  ====================================================================================

//  Write circle LED data
//
void ledWrite(byte ring, byte number, byte color) {
  Serial.write(RING_CMD_ON_OFF_LEDS);
  Serial.write(ring);
  Serial.write(number); //  LED to light (0-59)
  Serial.write(color);
  Serial.write(RING_CMD_END);

  Serial.read();
}

void ledWriteMeter(byte ring, byte startPos, byte endPos, byte color) {
  // TODO - Does not seem to work with current PIC version?
  Serial.write(RING_CMD_METER_LEDS);
  Serial.write(ring);
  Serial.write(startPos); //  LED to start light (0-59)
  Serial.write(endPos);   //  LED to end light (0-59)
  Serial.write(color);
//  Serial.write(RING_CMD_END);

  Serial.read();
}

void ledWriteAllInRingOff(byte ring) {
  Serial.write(RING_CMD_OFF_LEDS);
  Serial.write(ring);
  Serial.write(RING_CMD_UNUSED);
  Serial.write(RING_CMD_UNUSED);
  Serial.write(RING_CMD_END);

  Serial.read();
}

void ledWriteAllOff() {
  ledWriteAllInRingOff(RING_HOURS_MINUTES_SECONDS);
}

void ledWriteAllSecondsOff() {
  ledWriteAllInRingOff(RING_SECONDS);
}

void ledWriteAllMinutesOff() {
  ledWriteAllInRingOff(RING_MINUTES);
}

void ledWriteAllHoursOff() {
  ledWriteAllInRingOff(RING_HOURS);
}

//  ====================================================================================

void ledSegmentsStatusWriteByte() {

  byte byteToWrite = ledSegmentsStatus << 4;

  if (ledSegmentsColons == DISPLAY_COLONS_ON) {
    byteToWrite = byteToWrite | 0x0f;
  }
  else if (ledSegmentsColons == DISPLAY_COLONS_BOTTOM_TWO) {
    byteToWrite = byteToWrite | 0x0a;
  }
  else if (ledSegmentsColons == DISPLAY_COLONS_TOP_TWO) {
    byteToWrite = byteToWrite | 0x05;
  }

  Wire.write(byteToWrite);  
}

void ledSegmentsDisplayChars() {
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);

  Wire.write(0x00); // Start at address
  
  Wire.write(translateCharTo7SegDigit(segmentsDisplayChars[5], false));
  Wire.write(LED_SEGMENT_ZERO_BYTE);

  Wire.write(translateCharTo7SegDigit(segmentsDisplayChars[4], false));
  Wire.write(LED_SEGMENT_ZERO_BYTE);

  Wire.write(translateCharTo7SegDigit(segmentsDisplayChars[3], false));
  Wire.write(LED_SEGMENT_ZERO_BYTE);

  Wire.write(translateCharTo7SegDigit(segmentsDisplayChars[2], false));
  Wire.write(LED_SEGMENT_ZERO_BYTE);

  Wire.write(translateCharTo7SegDigit(segmentsDisplayChars[1], false));
  Wire.write(LED_SEGMENT_ZERO_BYTE);

  Wire.write(translateCharTo7SegDigit(segmentsDisplayChars[0], false));
  Wire.write(LED_SEGMENT_ZERO_BYTE);

  Wire.write(LED_SEGMENT_ZERO_BYTE);
  ledSegmentsStatusWriteByte();
  
  Wire.endTransmission();  
}

void ledSegmentsClearAll() {
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0x00); // Start at address

  for (byte r = 0; r < 16; r++) {
    Wire.write(LED_SEGMENT_ZERO_BYTE);
  }

  Wire.endTransmission();
}

void ledSegmentsDisplayStatus() {
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0x0D); // Start at address
  ledSegmentsStatusWriteByte();
  Wire.endTransmission();
}

void setLedSegmentsBrightness(byte b) {
  if(b > 15) {
    return;
  }
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0xE0 | b); // Dimming command
  Wire.endTransmission();
}

void ledSegmentsShow() {
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0x80 | true); // Blanking / blinking command
  Wire.endTransmission();
}

void ledSegmentsBlank() {
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0x80 | false); // Blanking / blinking command
  Wire.endTransmission();
}

void setLedSegmentsBlink(byte b) {
  if (b > 3) {
    return;
  }
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0x80 | b << 1 | 1); // Blinking / blanking command
  Wire.endTransmission();
}

void ledSegmentsSetup() {
  Wire.beginTransmission(HT16K33_I2C_ADDRESS);
  Wire.write(0x20 | 1); // Turn on oscillator
  Wire.endTransmission();

  setLedSegmentsBrightness(ledSegmentsBrightness);
  setLedSegmentsBlink(0);
}

//  ====================================================================================

void ledSegmentsDisplayTime(byte positionAlternate) {

  // Seconds
  if (positionAlternate != SET_POSITION_SECONDS) {
    segmentsDisplayChars[5] = seconds % 10 + '0';
  } else {
    segmentsDisplayChars[5] = DISP_CHAR_SELECTED;
  }
  
  if (positionAlternate != SET_POSITION_SECONDS) {
    segmentsDisplayChars[4] = seconds / 10 + '0';
  } else {
    segmentsDisplayChars[4] = DISP_CHAR_SELECTED;
  }

  // Minutes
  if (positionAlternate != SET_POSITION_MINUTES) {
    segmentsDisplayChars[3] = minutes % 10 + '0';
  } else {
    segmentsDisplayChars[3] = DISP_CHAR_SELECTED;
  }

  if (positionAlternate != SET_POSITION_MINUTES) {
    segmentsDisplayChars[2] = minutes / 10 + '0';
  } else {
    segmentsDisplayChars[2] = DISP_CHAR_SELECTED;
  }

  // Hours
  if (positionAlternate != SET_POSITION_HOURS) {
    segmentsDisplayChars[1] = hours % 10 + '0';
  } else {
    segmentsDisplayChars[1] = DISP_CHAR_SELECTED;
  }

  if (positionAlternate != SET_POSITION_HOURS) {
    segmentsDisplayChars[0] = hours / 10 + '0';
  } else {
    segmentsDisplayChars[0] = DISP_CHAR_SELECTED;
  }

  ledSegmentsDisplayChars();
}

void ledSegmentsDisplayDate(byte positionAlternate) {

  // Day of month
  if (positionAlternate != SET_POSITION_DAY) {
    segmentsDisplayChars[5] = dayOfMonth % 10 + '0';
  } else {
    segmentsDisplayChars[5] = DISP_CHAR_SELECTED;
  }
  
  if (positionAlternate != SET_POSITION_DAY) {
    segmentsDisplayChars[4] = dayOfMonth / 10 + '0';
  } else {
    segmentsDisplayChars[4] = DISP_CHAR_SELECTED;
  }

  // Months
  if (positionAlternate != SET_POSITION_MONTH) {
    segmentsDisplayChars[3] = months % 10 + '0';
  } else {
    segmentsDisplayChars[3] = DISP_CHAR_SELECTED;
  }

  if (positionAlternate != SET_POSITION_MONTH) {
    segmentsDisplayChars[2] = months / 10 + '0';
  } else {
    segmentsDisplayChars[2] = DISP_CHAR_SELECTED;
  }

  // Years
  if (positionAlternate != SET_POSITION_YEAR) {
    segmentsDisplayChars[1] = years % 10 + '0';
  } else {
    segmentsDisplayChars[1] = DISP_CHAR_SELECTED;
  }

  if (positionAlternate != SET_POSITION_YEAR) {
    segmentsDisplayChars[0] = years / 10 + '0';
  } else {
    segmentsDisplayChars[0] = DISP_CHAR_SELECTED;
  }

  ledSegmentsDisplayChars();
}

void ledSegmentsDisplayConfig(byte positionAlternate) {

  if (position == SET_POSITION_MARKERS) {
    if (positionAlternate == SET_POSITION_MARKERS) {
      segmentsDisplayChars[0] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[1] = DISP_CHAR_SELECTED;
    } else {
      byte value = (hoursMarkerColor & 0xf0);
      if (value == MARKER_HOUR_EVERY) {
        segmentsDisplayChars[0] = 'h';
      } else if (value == MARKER_HOUR_QUARTERS) {
        segmentsDisplayChars[0] = 'Q';
      } else if (value == MARKER_HOUR_TWELTH) {
        segmentsDisplayChars[0] = 't';
      } else {
        segmentsDisplayChars[0] = '?';
      }
      segmentsDisplayChars[1] = translateValueToHex(hoursMarkerColor & 0x0f);
    }

    segmentsDisplayChars[2] = DISP_CHAR_BLANK;
    segmentsDisplayChars[3] = DISP_CHAR_BLANK;
    segmentsDisplayChars[4] = DISP_CHAR_BLANK;
    segmentsDisplayChars[5] = DISP_CHAR_BLANK;
    
  } else {

    if (positionAlternate == SET_POSITION_HOURS) {
      segmentsDisplayChars[0] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[1] = DISP_CHAR_SELECTED;
    } else {
      byte value = (hoursColor & 0xf0);
      if (value == COLOR_TRACE) {
        segmentsDisplayChars[0] = 't';
      } else if (value == COLOR_DOT) {
        segmentsDisplayChars[0] = 'd';
      } else if (value == COLOR_HANDS) {
        segmentsDisplayChars[0] = 'h';
      } else {
        segmentsDisplayChars[0] = '?';
      }
      segmentsDisplayChars[1] = translateValueToHex(hoursColor & 0x0f);
    }

    if (positionAlternate == SET_POSITION_MINUTES) {
      segmentsDisplayChars[2] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[3] = DISP_CHAR_SELECTED;
    } else {
      byte value = (minutesColor & 0xf0);
      if (value == COLOR_TRACE) {
        segmentsDisplayChars[2] = 't';
      } else if (value == COLOR_DOT) {
        segmentsDisplayChars[2] = 'd';
      } else if (value == COLOR_HANDS) {
        segmentsDisplayChars[2] = 'h';
      } else {
        segmentsDisplayChars[2] = '?';
      }    
      segmentsDisplayChars[3] = translateValueToHex(minutesColor & 0x0f);
    }

    if (positionAlternate == SET_POSITION_SECONDS) {
      segmentsDisplayChars[4] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[5] = DISP_CHAR_SELECTED;
    } else {
      byte value = (secondsColor & 0xf0);
      if (value == COLOR_TRACE) {
        segmentsDisplayChars[4] = 't';
      } else if (value == COLOR_DOT) {
        segmentsDisplayChars[4] = 'd';
      } else if (value == COLOR_HANDS) {
        segmentsDisplayChars[4] = 'h';
      } else {
        segmentsDisplayChars[4] = '?';
      }
      segmentsDisplayChars[5] = translateValueToHex(secondsColor & 0x0f);
    }
  }

  ledSegmentsDisplayChars();
}

void ledSegmentsDisplaySettings(byte positionAlternate) {

  if (position == SET_POSITION_CLOCK_FACE) {
    strncpy_P(segmentsDisplayChars, DISP_FACE, 6);
    if (positionAlternate != SET_POSITION_CLOCK_FACE) {
      segmentsDisplayChars[5] = clockFace + '0';
    }
    
  } else {
    
    if (positionAlternate == SET_POSITION_TIME_DATE) {
      segmentsDisplayChars[0] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[1] = DISP_CHAR_SELECTED;
    } else {
      byte value = (ledSegmentsSettings & 0xf0);
      if (value == DISPLAY_TIME_AND_DATE) {
        segmentsDisplayChars[0] = 't';
        segmentsDisplayChars[1] = 'd';
      } else if (value == DISPLAY_TIME) {
        segmentsDisplayChars[0] = 't';
        segmentsDisplayChars[1] = DISP_CHAR_BLANK;
      } else if (value == DISPLAY_DATE) {
        segmentsDisplayChars[0] = DISP_CHAR_BLANK;
        segmentsDisplayChars[1] = 'd';
      } else {
        segmentsDisplayChars[0] = 'n';
        segmentsDisplayChars[1] = 'o';
      }    
    }

    if (positionAlternate == SET_POSITION_ALT_TIMER) {
      segmentsDisplayChars[2] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[3] = DISP_CHAR_SELECTED;
    } else {
      if (ledSegmentsToggleSeconds < 10) {
        segmentsDisplayChars[2] = DISP_CHAR_BLANK;
      } else {
        segmentsDisplayChars[2] = ledSegmentsToggleSeconds / 10 + '0';
      }
      segmentsDisplayChars[3] = ledSegmentsToggleSeconds % 10 + '0';
    }

    if (positionAlternate == SET_POSITION_FLASH_COLON) {
      segmentsDisplayChars[4] = DISP_CHAR_SELECTED;
      segmentsDisplayChars[5] = DISP_CHAR_SELECTED;
    } else {
      byte value = (ledSegmentsSettings & 0x0f);
      if (value == DISPLAY_COLONS_FLASH_EVERY_SECOND) {
        segmentsDisplayChars[4] = 'F';
        segmentsDisplayChars[5] = 'L';
      } else {
        segmentsDisplayChars[4] = 'o';
        segmentsDisplayChars[5] = 'n';
      }
    }    
  }

  ledSegmentsDisplayChars();
}

void displayHexAndPause(byte hexValue) {
  segmentsDisplayChars[0] = '-';
  segmentsDisplayChars[1] = '-';
  segmentsDisplayChars[2] = translateValueToHex(hexValue >> 4);
  segmentsDisplayChars[3] = translateValueToHex(hexValue & 0x0f);
  segmentsDisplayChars[4] = '-';
  segmentsDisplayChars[5] = '-';
  ledSegmentsDisplayChars();
  
  while(readPressedKeys() == 0) {
    // Wait for any key to be pressed.
  }
}

//  ====================================================================================

void drawHourMarkers(byte steps, byte drawColor) {
  for (loopMarker = 0; loopMarker < 60; loopMarker = loopMarker + steps) {
    markers = RING_SECONDS;
    
    if (loopMarker == 0) {
      markers = RING_HOURS_MINUTES_SECONDS;
    } else if (loopMarker == 15 || loopMarker == 30 || loopMarker == 45) {
      markers = RING_MINUTES_SECONDS;
    }

    if ((secondsColor & 0x0f) != COLOR_BLANK) {
      if (seconds == loopMarker) {
        if (bitRead(secondsColor, COLOR_BIT_TRACE) == 1) {
          if (seconds > 0) {
            // Do not display marker marker if seconds trace is here, except at twelve position
            markers = markers & RING_HOURS_MINUTES;
          }
        } else if (bitRead(secondsColor, COLOR_BIT_DOT) == 1) {
          // Do not display marker marker if seconds dot is here
          markers = markers & RING_HOURS_MINUTES;
        } else if (bitRead(secondsColor, COLOR_BIT_HANDS) == 1) {
          // Covers whole marker when seconds hand is here
          markers = RING_NONE;
        }
      }
    }
  
    if ((minutesColor & 0x0f) != COLOR_BLANK) {
      if (minutes == loopMarker) {
        if (bitRead(minutesColor, COLOR_BIT_TRACE) == 1) {
          if (minutes > 0) {
            // Do not display marker if minutes trace is here, except at twelve position
            markers = markers & RING_HOURS_SECONDS;
          }
        } else if (bitRead(minutesColor, COLOR_BIT_DOT) == 1) {
          // Do not display marker if minutes dot is here
          markers = markers & RING_HOURS_SECONDS;
        } else if (bitRead(minutesColor, COLOR_BIT_HANDS) == 1) {
          // Covers whole marker when minutes hand is here
          markers = RING_NONE;
        }
      }
    }

    if ((hoursColor & 0x0f) != COLOR_BLANK) {
      if (hoursHand == loopMarker) {
        if (bitRead(hoursColor, COLOR_BIT_TRACE) == 1) {
          if (hoursHand > 0) {
            // Do not display marker if hours trace is here, except at twelve position
            markers = markers & RING_MINUTES_SECONDS;
          }
        } else if (bitRead(hoursColor, COLOR_BIT_DOT) == 1) {
          // Do not display marker if hours dot is here
          markers = markers & RING_MINUTES_SECONDS;
        } else if (bitRead(hoursColor, COLOR_BIT_HANDS) == 1) {
          // Covers marker except seconds when hours hand is here
          markers = markers & RING_SECONDS;
        }
      }
    }

    if (markers != RING_NONE) {
      ledWrite(markers, loopMarker, drawColor);
    }
  }
}

//  Draw markers where no hands are displayed
//
void drawMarkers() {
  if ((hoursMarkerColor & 0x0f) != COLOR_BLANK) {
    if (bitRead(hoursMarkerColor, MARKER_BIT_HOUR_EVERY) == 1) {
      drawHourMarkers(5, hoursMarkerColor & 0x0f);
    } else if (bitRead(hoursMarkerColor, MARKER_BIT_HOUR_QUARTERS) == 1) {
      drawHourMarkers(15, hoursMarkerColor & 0x0f);
    } else if (bitRead(hoursMarkerColor, MARKER_BIT_HOUR_TWELTH) == 1) {
      drawHourMarkers(60, hoursMarkerColor & 0x0f);
    }
  }
}

//  ====================================================================================

//  Clear hands (if needed)
//
void clearHands() {
  byte r;

  //  Check if hours should be cleared
  if ((hoursColor & 0x0f) != COLOR_BLANK) {
    if (hoursHand != previousHoursHand) {
      if (bitRead(hoursColor, COLOR_BIT_TRACE) == 1) {
        if (hoursHand == 0) {
          //  Clear the ring if moved to zero position.
          ledWriteAllInRingOff(RING_HOURS);
        } else {
          //  Clear hours down to current time.
          for (r = previousHoursHand; r > hoursHand; r--) {
            ledWrite(RING_HOURS, r, COLOR_BLANK);
          }
        }
      } else if (bitRead(hoursColor, COLOR_BIT_DOT) == 1) {
          //  Clear the previous hours
          ledWrite(RING_HOURS, previousHoursHand, COLOR_BLANK);
      } else if (bitRead(hoursColor, COLOR_BIT_HANDS) == 1) {
          //  Clear the previous hours
          ledWrite(RING_HOURS_MINUTES, previousHoursHand, COLOR_BLANK);
      }
    }
  }

  //  Check if minutes should be cleared
  if ((minutesColor & 0x0f) != COLOR_BLANK) {
    if (minutes != previousMinutes) {
      if (bitRead(minutesColor, COLOR_BIT_TRACE) == 1) {
        if (minutes == 0) {
          //  Clear the ring if moved to zero position.
          ledWriteAllInRingOff(RING_MINUTES);
        } else {
          //  Clear minutes down to current time.
          for (r = previousMinutes; r > minutes; r--) {
            ledWrite(RING_MINUTES, r, COLOR_BLANK);
          }
        }
      } else if (bitRead(minutesColor, COLOR_BIT_DOT) == 1) {
          //  Clear the previous minutes
          ledWrite(RING_MINUTES, previousMinutes, COLOR_BLANK);
      } else if (bitRead(minutesColor, COLOR_BIT_HANDS) == 1) {
          //  Clear the previous minutes
          ledWrite(RING_HOURS_MINUTES_SECONDS, previousMinutes, COLOR_BLANK);
      }
    }
  }

  //  Check if seconds should be cleared
  if ((secondsColor & 0x0f) != COLOR_BLANK) {
    if (seconds != previousSeconds) {
      if (bitRead(secondsColor, COLOR_BIT_TRACE) == 1) {
        if (seconds == 0) {
          //  Clear the ring if moved to zero position.
          ledWriteAllInRingOff(RING_SECONDS);
        } else {
          //  Clear seconds down to current time.
          for (r = previousSeconds; r > seconds; r--) {
            ledWrite(RING_SECONDS, r, COLOR_BLANK);
          }
        }
      } else if (bitRead(secondsColor, COLOR_BIT_DOT) == 1) {
          //  Clear the previous seconds when not in trace mode
          ledWrite(RING_SECONDS, previousSeconds, COLOR_BLANK);
      } else if (bitRead(secondsColor, COLOR_BIT_HANDS) == 1) {
          //  Clear the previous seconds when not in trace mode
          ledWrite(RING_HOURS_MINUTES_SECONDS, previousSeconds, COLOR_BLANK);
      }
    }
  }
}

//  Draw the current hands (if needed)
//
void drawHands() {
  byte r;

  //  Check if minutes needs to be drawn
  if ((minutesColor & 0x0f) != COLOR_BLANK) {
    if (bitRead(minutesColor, COLOR_BIT_TRACE) == 1) {

      if (minutes != previousMinutes && (minutes > 0 || (hoursMarkerColor & 0x0f) == COLOR_BLANK)) {
        //  Fill minutes up to current time, skip 0 for trace.
        for (r = (minutes <= 1 ? minutes : previousMinutes); r <= minutes; r++) {
          ledWrite(RING_MINUTES, r, minutesColor & 0x0f);
        }
      }

      // Also redraw minutes if second hand has been there previously.
      if ((secondsColor & 0x0f) != COLOR_BLANK) {
        if (bitRead(secondsColor, COLOR_BIT_HANDS) == 1) {
          if (seconds != previousSeconds && minutes >= previousSeconds && previousSeconds > 0) {
            ledWrite(RING_MINUTES, previousSeconds, minutesColor & 0x0f);
          }
        }
      }

      // Also redraw minutes if hour hand has been there previously.
      if ((hoursColor & 0x0f) != COLOR_BLANK) {
        if (bitRead(hoursColor, COLOR_BIT_HANDS) == 1) {
          if (hoursHand != previousHoursHand && minutes >= previousHoursHand && previousHoursHand > 0) {
            ledWrite(RING_MINUTES, previousHoursHand, minutesColor & 0x0f);
          }
        }
      }
    } else if (bitRead(minutesColor, COLOR_BIT_DOT) == 1) {
      if (minutes != previousMinutes || minutes == previousSeconds || minutes == previousHoursHand) {
        ledWrite(RING_MINUTES, minutes, minutesColor & 0x0f);
      }
    } else if (bitRead(minutesColor, COLOR_BIT_HANDS) == 1) {
      if (minutes != previousMinutes || minutes == previousSeconds || minutes == previousHoursHand) {
        ledWrite(RING_HOURS_MINUTES_SECONDS, minutes, minutesColor & 0x0f);
      }
    }
  }

  //  Check if hours needs to be drawn
  if ((hoursColor & 0x0f) != COLOR_BLANK) {
    if (bitRead(hoursColor, COLOR_BIT_TRACE) == 1) {
      if (hoursHand != previousHoursHand && (hoursHand > 0 || (hoursMarkerColor & 0x0f) == COLOR_BLANK)) {
        //  Fill hours up to current time, skip 0 for trace.
        for (r = (hoursHand <= 1 ? hoursHand : previousHoursHand); r <= hoursHand; r++) {
          ledWrite(RING_HOURS, r, hoursColor & 0x0f);
        }
      }

      // Also redraw hours if minutes hand has been there previously.
      if ((minutesColor & 0x0f) != COLOR_BLANK) {
        if (bitRead(minutesColor, COLOR_BIT_HANDS) == 1) {
          if (minutes != previousMinutes && hoursHand >= previousMinutes && previousMinutes > 0) {
            ledWrite(RING_HOURS, previousMinutes, hoursColor & 0x0f);
          }
        }
      }

      // Also redraw hours if seconds hand has been there previously.
      if ((secondsColor & 0x0f) != COLOR_BLANK) {
        if (bitRead(secondsColor, COLOR_BIT_HANDS) == 1) {
          if (seconds != previousSeconds && hoursHand >= previousSeconds && previousSeconds > 0) {
            ledWrite(RING_HOURS, previousSeconds, hoursColor & 0x0f);
          }
        }
      }
    } else if (bitRead(hoursColor, COLOR_BIT_DOT) == 1) {
      if (hoursHand != previousHoursHand || hoursHand == previousMinutes || hoursHand == previousSeconds) {
        ledWrite(RING_HOURS, hoursHand, hoursColor & 0x0f);
      }
    } else if (bitRead(hoursColor, COLOR_BIT_HANDS) == 1) {
      if (hoursHand != previousHoursHand || hoursHand == previousMinutes || hoursHand == previousSeconds) {
        ledWrite(RING_HOURS_MINUTES, hoursHand, hoursColor & 0x0f);
      }
    }
  }

  //  Check if seconds needs to be drawn
  if ((secondsColor & 0x0f) != COLOR_BLANK) {
    if (bitRead(secondsColor, COLOR_BIT_TRACE) == 1) {
      if (seconds != previousSeconds && (seconds > 0 || (hoursMarkerColor & 0x0f) == COLOR_BLANK)) {
        //  Fill seconds up to current time, skip 0 for trace.
        for (r = (seconds <= 1 ? seconds : previousSeconds); r <= seconds; r++) {
          ledWrite(RING_SECONDS, r, secondsColor & 0x0f);
        }
      }

      // Also redraw seconds if minutes hand has been there previously.
      if ((minutesColor & 0x0f) != COLOR_BLANK) {
        if (bitRead(minutesColor, COLOR_BIT_HANDS) == 1) {
          if (minutes != previousMinutes && seconds >= previousMinutes && previousMinutes > 0) {
            ledWrite(RING_SECONDS, previousMinutes, secondsColor & 0x0f);
          }
        }
      }
    } else if (bitRead(secondsColor, COLOR_BIT_DOT) == 1) {
      if (seconds != previousSeconds || seconds == previousMinutes) {
        ledWrite(RING_SECONDS, seconds, secondsColor & 0x0f);
      }
    } else if (bitRead(secondsColor, COLOR_BIT_HANDS) == 1) {
      if (seconds != previousSeconds || seconds == previousMinutes) {
        ledWrite(RING_HOURS_MINUTES_SECONDS, seconds, secondsColor & 0x0f);
      }
    }
  }  
}   

void drawClockFace() {
    // Calculate position for hours hand (depends on both current hours and minutes)
    hoursHand = (hours%12)*5 + minutes/12;
    
    clearHands();
    drawHands();
    drawMarkers();

    previousHoursHand = hoursHand;
    previousHours = hours;
    previousMinutes = minutes;
    previousSeconds = seconds;
}

//  Forces redrawing the clock face.
void resetPreviousValues() {
  previousHoursHand = 0;
  previousHours = 0;
  previousMinutes = 0;
  previousSeconds = 0;
}

//  ====================================================================================

//  Draws the setup for appearance configuration
//
void drawDisplayConfiguration()
{
  //  Setup static positions for demo configuration mode.
  hours = 22;
  minutes = 10;
  seconds = 23;

  //  Forces redrawing the clock face.
  resetPreviousValues();
  drawClockFace();
}

//  ====================================================================================

void ringAnimation(byte color) {
  //  Clear clock face with wipe of LEDs
  ledWrite(RING_HOURS_MINUTES_SECONDS, 0, color);
  delay(ANIMATION_SHORT_DELAY);
  
  for (byte loopCtr=1; loopCtr < 30; loopCtr++) {
    ledWrite(RING_HOURS_MINUTES_SECONDS, 60-loopCtr, color);
    ledWrite(RING_HOURS_MINUTES_SECONDS, loopCtr, color);
    delay(ANIMATION_SHORT_DELAY);
  }
  
  ledWrite(RING_HOURS_MINUTES_SECONDS, 30, color);
  delay(ANIMATION_SHORT_DELAY);
}

void ringAnimationUntilNotKeyCombination(byte color, byte keyCombination) {

  //  Clear clock face with wipe of LEDs
  ledWrite(RING_HOURS_MINUTES_SECONDS, 0, color);
  delay(ANIMATION_KEY_DELAY);
  pressedKeys = readPressedKeys();
  if (pressedKeys != keyCombination) {
    return;
  }
  
  for (byte loopCtr=1; loopCtr < 30; loopCtr++) {
    ledWrite(RING_HOURS_MINUTES_SECONDS, 60-loopCtr, color);
    ledWrite(RING_HOURS_MINUTES_SECONDS, loopCtr, color);
    delay(ANIMATION_KEY_DELAY);
    pressedKeys = readPressedKeys();
    if (pressedKeys != keyCombination) {
      return;
    }
  }
  
  ledWrite(RING_HOURS_MINUTES_SECONDS, 30, color);
  delay(ANIMATION_KEY_DELAY);
  pressedKeys = readPressedKeys();
}

//  ====================================================================================

void drawNormalLedSegments() {
  if ((ledSegmentsSettings & DISPLAY_TIME_AND_DATE) == DISPLAY_TIME_AND_DATE) {
    ledSegmentsCounter = seconds % ledSegmentsToggleSeconds;
    if ((ledSegmentsCounter == 0) || ((ledSegmentsDisplay & DISPLAY_TIME_AND_DATE) == DISPLAY_NONE)) {
      if (((seconds / ledSegmentsToggleSeconds) % 2) == 0) {
        ledSegmentsDisplay = DISPLAY_TIME;
        ledSegmentsColons = DISPLAY_COLONS_OFF;
      } else {
        ledSegmentsDisplay = DISPLAY_DATE;
      }
    }
  } else {
    ledSegmentsDisplay = (ledSegmentsSettings & DISPLAY_TIME_AND_DATE);
  }

  if ((ledSegmentsDisplay & DISPLAY_TIME) == DISPLAY_TIME) {
    // Flash colons
    if ((ledSegmentsSettings & DISPLAY_COLONS_FLASH_EVERY_SECOND) == DISPLAY_COLONS_FLASH_EVERY_SECOND) {
      if (ledSegmentsColons == DISPLAY_COLONS_ON) {
        ledSegmentsColons = DISPLAY_COLONS_OFF;
      } else {
        ledSegmentsColons = DISPLAY_COLONS_ON;
      }
    } else {
      ledSegmentsColons = DISPLAY_COLONS_ON;
    }

    // Display time
    ledSegmentsDisplayTime(0);
  } else if ((ledSegmentsDisplay & DISPLAY_DATE) == DISPLAY_DATE) {
    ledSegmentsColons = DISPLAY_COLONS_BOTTOM_TWO;
    
    // Display date
    ledSegmentsDisplayDate(0);
  } else {
    // Clear display
    ledSegmentsClearAll();
  }
}

//  ====================================================================================

void drawConfigurationLedSegments(byte positionAlternate) {
  if ((ledSegmentsDisplay & DISPLAY_TIME) == DISPLAY_TIME) {
    ledSegmentsColons = DISPLAY_COLONS_ON;

    // Display time
    ledSegmentsDisplayTime(positionAlternate);
  } else if ((ledSegmentsDisplay & DISPLAY_DATE) == DISPLAY_DATE) {
    ledSegmentsColons = DISPLAY_COLONS_BOTTOM_TWO;
    
    // Display date
    ledSegmentsDisplayDate(positionAlternate);
  } else if ((ledSegmentsDisplay & DISPLAY_CONFIG) == DISPLAY_CONFIG) {
    if (position == SET_POSITION_MARKERS) {
      ledSegmentsColons = DISPLAY_COLONS_OFF;
    } else {
      ledSegmentsColons = DISPLAY_COLONS_ON;
    }

    // Display config
    ledSegmentsDisplayConfig(positionAlternate);
  } else if ((ledSegmentsDisplay & DISPLAY_SETTINGS) == DISPLAY_SETTINGS) {
    if (position == SET_POSITION_CLOCK_FACE) {
      ledSegmentsColons = DISPLAY_COLONS_OFF;
    } else {
      ledSegmentsColons = DISPLAY_COLONS_TOP_TWO;
    }    

    // Display settings
    ledSegmentsDisplaySettings(positionAlternate);
  } else {
    // Clear display
    ledSegmentsClearAll();
  }
}

//  ====================================================================================

void loadSettingsOrFactoryDefaults() {
  //  Load in the default clock face saved in Eeprom
  clockFace = EEPROM.read(EEPROM_CLOCK_FACE_NUMBER);
  //  If not a valid number then assign number
  if (clockFace > DEFAULT_FACTORY_CLOCK_FACES) {
    clockFace = 0;
  }

  //  Load in the default date/time appearance saved in Eeprom
  ledSegmentsSettings = EEPROM.read(EEPROM_DATE_TIME_AND_COLON);

  //  Load in the default date/time appearance saved in Eeprom
  ledSegmentsToggleSeconds = EEPROM.read(EEPROM_ALTERNATE_COUNTER);
  //  If not a valid number then assign number
  if (ledSegmentsToggleSeconds == 0) {
    ledSegmentsToggleSeconds = 5;
  }
}

void loadFaceSettingsOrFactoryDefaults() {
  //  Load in colors saved in Eeprom for the selected clock face
  hoursMarkerColor = EEPROM.read(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 0);
  hoursColor =       EEPROM.read(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 1);
  minutesColor =     EEPROM.read(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 2);
  secondsColor =     EEPROM.read(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 3);

  // Get factory settings if no marker or color was previously set in Eeprom memory.
  if (hoursMarkerColor == 0 && hoursColor == 0 && minutesColor == 0 && secondsColor == 0) {
    hoursMarkerColor = DEFAULT_FACTORY_COLORS[clockFace][0];
    hoursColor = DEFAULT_FACTORY_COLORS[clockFace][1]; 
    minutesColor = DEFAULT_FACTORY_COLORS[clockFace][2];
    secondsColor = DEFAULT_FACTORY_COLORS[clockFace][3];
  }
}

void writeFactorySettingsToEeprom() {
  // Write default values to Eeprom.
  EEPROM.write(EEPROM_CLOCK_FACE_NUMBER, 0);
  EEPROM.write(EEPROM_DATE_TIME_AND_COLON, DISPLAY_TIME_AND_DATE | DISPLAY_COLONS_FLASH_EVERY_SECOND);
  EEPROM.write(EEPROM_ALTERNATE_COUNTER, 5);

  // Write default clock faces to Eeprom.
  for (byte r = 0; r < DEFAULT_FACTORY_CLOCK_FACES; r++) {
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + r*DEFAULT_CLOCK_FACE_LENGTH + 0, DEFAULT_FACTORY_COLORS[r][0]);
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + r*DEFAULT_CLOCK_FACE_LENGTH + 1, DEFAULT_FACTORY_COLORS[r][1]);
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + r*DEFAULT_CLOCK_FACE_LENGTH + 2, DEFAULT_FACTORY_COLORS[r][2]);
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + r*DEFAULT_CLOCK_FACE_LENGTH + 3, DEFAULT_FACTORY_COLORS[r][3]);
  }
}

void setup() {
  pinMode(PIN_BUTTON1, INPUT);    //  Setup pin8 as input
  pinMode(PIN_BUTTON2, INPUT);    //  Setup pin9 as input
  pinMode(PIN_BUTTON3, INPUT);    //  Setup pin10 as input

  //  I2C interface for the 1307 RTC chip
  Wire.begin();

  //  Enable uart port at desired baud rate 
  Serial.begin(9600);

  //  Setup led segements board HT16K33.
  ledSegmentsSetup();

  // Display greeting
  setLedSegmentsBrightness(0);
  strncpy_P(segmentsDisplayChars, DISP_HELLO, 6);
  ledSegmentsStatus = MODE_LED_NONE;
  ledSegmentsDisplayChars();
  ledSegmentsShow();

  // Fade up the HELLO display
  for (byte br = 0; br < ledSegmentsBrightness; br++) {
    setLedSegmentsBrightness(br);
    delay(225 - br*15);
  }

  delay(500);

  //  Clear led memory buffers in PIC processor
  ledWriteAllOff();

  //  Clear 7-segments display
  ledSegmentsClearAll();

  //  On cold start get time and then set it to start clock up
  //  getDateDs1307(&seconds, &minutes, &hours, &dayOfWeek, &dayOfMonth, &months, &years);
  
  loadSettingsOrFactoryDefaults();
  loadFaceSettingsOrFactoryDefaults();

  delay(500);
}

//  ====================================================================================

void initLedSegmentsStatusByMode(byte value) {
  if (value == MODE_SET_TIME_AND_DATE) {
    ledSegmentsStatus = MODE_LED_SET_TIME_DATE;
  } else if (value == MODE_SET_STYLING) {
    ledSegmentsStatus = MODE_LED_SET_STYLING;
  } else if (value == MODE_SET_SETTINGS) {
    ledSegmentsStatus = MODE_LED_SET_SETTINGS;
  } else {
    ledSegmentsStatus = MODE_LED_NONE;
  }
}

void userSelectMode() {
  initUserSelect();

  int8_t value = MODE_NORMAL;
  ledSegmentsColons = DISPLAY_COLONS_OFF;
  initLedSegmentsStatusByMode(value);

  strncpy_P(segmentsDisplayChars, DISP_SELECT, 6);
  ledSegmentsDisplayChars();

  waitForReleaseAllButtons();

  while(!exitFlag) {
    
    pressedKeys = readPressedKeys();

    if (pressedKeys == KEY_PRESSED_1) {
      value++;
      if (value > MODE_SET_TIME_AND_DATE) {
        value = MODE_NORMAL;
      }
      blinkUpdate = 2;
    }

    if (pressedKeys == KEY_PRESSED_3) {
      value--;
      if (value < MODE_NORMAL) {
        value = MODE_SET_TIME_AND_DATE;
      }
      blinkUpdate = 2;
    }
    
    if (pressedKeys == KEY_PRESSED_2) {
      mode = value;
      blinkUpdate = 3;
      exitFlag = true;
    }

    updateBlinkTimer();

    if (blinkUpdate > 0) {
      if (blinkUpdate < 2 && blinkActive) {
        ledSegmentsStatus = MODE_LED_NONE;
      } else {
        blinkActive = false;
        initLedSegmentsStatusByMode(value);
        if (blinkUpdate >= 2) {

          switch(value) {
            case MODE_SET_TIME_AND_DATE:
                strncpy_P(segmentsDisplayChars, DISP_MENU_CLOCK, 6);
                break;
            case MODE_SET_STYLING:
                strncpy_P(segmentsDisplayChars, DISP_MENU_FACE, 6);
                break;
            case MODE_SET_SETTINGS:
                strncpy_P(segmentsDisplayChars, DISP_MENU_DISPLAY, 6);
                break;
            default:
                strncpy_P(segmentsDisplayChars, DISP_SELECT, 6);
                break;
          }

          ledSegmentsDisplayChars();

          waitForReleaseAllButtons();          
          blinkTimer = millis();
        }
      }
      ledSegmentsDisplayStatus();
      blinkUpdate = 0;
    }
  }
}

//  ====================================================================================

void userSelectedStyle() {
  // Write selected face on display
  strncpy_P(segmentsDisplayChars, DISP_FACE, 6);
  segmentsDisplayChars[5] = clockFace + '0';
  ledSegmentsColons = DISPLAY_COLONS_OFF;
  ledSegmentsDisplayChars();

  ringAnimation(COLOR_WHITE);
  ringAnimation(COLOR_BLANK);  

  loadFaceSettingsOrFactoryDefaults();

  // Force redrawing clock face.
  resetPreviousValues();
  
  //  Clear 7-segments display
  ledSegmentsClearAll();

  waitForReleaseAllButtons();
}

//  ====================================================================================

byte getOptionsByPosition(byte position) {
  if (position == SET_POSITION_HOURS) {
    return (hoursColor & 0xf0);
  }
  else if (position == SET_POSITION_MINUTES) {
    return (minutesColor & 0xf0);
  }
  else if (position == SET_POSITION_SECONDS) {
    return (secondsColor & 0xf0);
  }
  else if (position == SET_POSITION_MARKERS) {
    return (hoursMarkerColor & 0xf0);
  }
  else {
    return 0;
  }
}

void setOptionsByPosition(byte position, byte value) {
  if (position == SET_POSITION_HOURS) {
    hoursColor = (hoursColor & 0x0f) | (value & 0xf0);
  }
  else if (position == SET_POSITION_MINUTES) {
    minutesColor = (minutesColor & 0x0f) | (value & 0xf0);
  }
  else if (position == SET_POSITION_SECONDS) {
    secondsColor = (secondsColor & 0x0f) | (value & 0xf0);
  }
  else if (position == SET_POSITION_MARKERS) {
    hoursMarkerColor = (hoursMarkerColor & 0x0f) | (value & 0xf0);
  }
}

byte getColorByPosition(byte position) {
  if (position == SET_POSITION_HOURS) {
    return (hoursColor & 0x0f);
  }
  else if (position == SET_POSITION_MINUTES) {
    return (minutesColor & 0x0f);
  }
  else if (position == SET_POSITION_SECONDS) {
    return (secondsColor & 0x0f);
  }
  else if (position == SET_POSITION_MARKERS) {
    return (hoursMarkerColor & 0x0f);
  }
  else {
    return 0;
  }
}

void setColorByPosition(byte position, byte value) {
  if (position == SET_POSITION_HOURS) {
    hoursColor = (hoursColor & 0xf0) | (value & 0x0f);
  }
  else if (position == SET_POSITION_MINUTES) {
    minutesColor = (minutesColor & 0xf0) | (value & 0x0f);
  }
  else if (position == SET_POSITION_SECONDS) {
    secondsColor = (secondsColor & 0xf0) | (value & 0x0f);
  }
  else if (position == SET_POSITION_MARKERS) {
    hoursMarkerColor = (hoursMarkerColor & 0xf0) | (value & 0x0f);
  }
}

void userSetFaceColorAndStyle() {
  initUserSelect();
  
  settingsChangedFlag = 0;
  position = SET_POSITION_HOURS;

  ledSegmentsStatus = MODE_LED_SET_STYLING;
  ledSegmentsDisplay = DISPLAY_CONFIG;
  drawConfigurationLedSegments(position);
  ledWriteAllOff();
  drawDisplayConfiguration();
  waitForReleaseAllButtons();

  while(!exitFlag) {
    
    pressedKeys = readPressedKeys();

    if (pressedKeys == KEY_PRESSED_1) {
      byte value = getOptionsByPosition(position);
      if (position == SET_POSITION_MARKERS) {
        if (value == MARKER_HOUR_TWELTH) {
          value = MARKER_HOUR_QUARTERS;
        } else if (value == MARKER_HOUR_QUARTERS) {
          value = MARKER_HOUR_EVERY;
        } else {
          value = MARKER_HOUR_TWELTH;
        }        
      } else {
        if (value == COLOR_HANDS) {
          value = COLOR_TRACE;
        } else if (value == COLOR_TRACE) {
          value = COLOR_DOT;
        } else {
          value = COLOR_HANDS;
        }        
      }
      
      setOptionsByPosition(position, value);

      ledWriteAllOff();
      settingsChangedFlag = 1;
      blinkUpdate = 2;
    }

    if (pressedKeys == KEY_PRESSED_3) {
      byte value = getColorByPosition(position);
      value++;
      if (value > COLORS_END) {
        value = COLORS_START;
      }
      
      setColorByPosition(position, value);

      if (value == COLOR_BLANK) {
        ledWriteAllOff();
      }
      settingsChangedFlag = 1;
      blinkUpdate = 2;
    }

    if (pressedKeys == KEY_PRESSED_2) {
      blinkUpdate = 3;
      position++;

      if (position == SET_POSITION_YEAR) {
        position = SET_POSITION_MARKERS;
        ledSegmentsColons = DISPLAY_COLONS_OFF;
      }
      else if (position > SET_POSITION_MARKERS) {
        position = SET_POSITION_NONE;        
        ledSegmentsStatus = MODE_LED_NONE;
        exitFlag = true;
      }
    }

    updateBlinkTimer();
    
    if (blinkUpdate == 2) {
      drawDisplayConfiguration();
    }    

    if (blinkUpdate > 0) {
      if (blinkUpdate < 2 && blinkActive) {
        drawConfigurationLedSegments(position);
      } else {
        blinkActive = false;
        drawConfigurationLedSegments(0);
        if (blinkUpdate >= 2) {
          waitForReleaseAllButtons();
          blinkTimer = millis();
        }
      }
      blinkUpdate = 0;
    }
  }

  if (settingsChangedFlag > 0) {
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 0, hoursMarkerColor);
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 1, hoursColor);
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 2, minutesColor);
    EEPROM.write(EEPROM_CLOCK_FACE_SETTINGS + clockFace*DEFAULT_CLOCK_FACE_LENGTH + 3, secondsColor);
    ringAnimation(COLOR_GREEN);
  } else {
    ringAnimation(COLOR_BLUE);
  }
  ringAnimation(COLOR_BLANK);

  // Force redrawing clock face.
  resetPreviousValues();

  //  Clear 7-segments display
  ledSegmentsClearAll();
}

//  ====================================================================================

void normalMode() {
  // Get current time and date
  getDateDs1307(&seconds, &minutes, &hours, &dayOfWeek, &dayOfMonth, &months, &years);

  // Update the clock face every second
  if (seconds != previousSeconds) {
    drawClockFace();
    ledSegmentsStatus = MODE_LED_NONE;
    drawNormalLedSegments();
  }
}

//  ====================================================================================


byte getValueByPosition(byte position) {
  if (position == SET_POSITION_HOURS) {
    return hours;
  }
  else if (position == SET_POSITION_MINUTES) {
    return minutes;
  }
  else if (position == SET_POSITION_SECONDS) {
    return seconds;
  }
  else if (position == SET_POSITION_YEAR) {
    return years;
  }
  else if (position == SET_POSITION_MONTH) {
    return months;
  }
  else if (position == SET_POSITION_DAY) {
    return dayOfMonth;
  }
  else {
    return 0;
  }
}

void setValueByPosition(byte position, byte value) {
  if (position == SET_POSITION_HOURS) {
    hours = value;
  }
  else if (position == SET_POSITION_MINUTES) {
    minutes = value;
  }
  else if (position == SET_POSITION_SECONDS) {
    seconds = value;
  }
  else if (position == SET_POSITION_YEAR) {
    years = value;
  }
  else if (position == SET_POSITION_MONTH) {
    months = value;
  }
  else if (position == SET_POSITION_DAY) {
    dayOfMonth = value;
  }
}

int8_t getDaysMaxBasedOnMonthAndLeapYear() {
  byte days = 31;
  if (months == 4 || months == 6 || months == 9 || months == 11) {
    days = 30;
  } else if (months == 2) {
    days = 28;
    if ((years % 4) == 0) {
      days = 29;
    }
  }
  return days;
}

void userSetTimeAndDate() {
  initUserSelect();
  
  settingsChangedFlag = 0;
  position = SET_POSITION_HOURS;

  ledSegmentsStatus = MODE_LED_SET_TIME_DATE;
  ledSegmentsColons = DISPLAY_COLONS_ON;
  ledSegmentsDisplay = DISPLAY_TIME;
  drawConfigurationLedSegments(position);
  waitForReleaseAllButtons();

  while(!exitFlag) {
    
    pressedKeys = readPressedKeys();
    if (isButtonPressed && pressedKeys == KEY_PRESSED_NONE) {
      isButtonPressed = false;
    }

    if (pressedKeys == KEY_PRESSED_1) {
      int8_t value = getValueByPosition(position);
      value--;
      if (value < valueTimeDateMin[position-1]) {
        if (position == SET_POSITION_DAY) {
          value = getDaysMaxBasedOnMonthAndLeapYear();
        } else {
          value = valueTimeDateMax[position-1];
        }
      }
      setValueByPosition(position, value);

      settingsChangedFlag = 1;
      blinkUpdate = 2;
    }

    if (pressedKeys == KEY_PRESSED_3) {
      int8_t value = getValueByPosition(position);
      value++;
      if (position == SET_POSITION_DAY) {
        if (value > getDaysMaxBasedOnMonthAndLeapYear()) {
          value = valueTimeDateMin[position-1];
        }
      } else {
        if (value > valueTimeDateMax[position-1]) {
          value = valueTimeDateMin[position-1];
        }
      }
      setValueByPosition(position, value);

      settingsChangedFlag = 1;
      blinkUpdate = 2;
    }
    
    if (pressedKeys == KEY_PRESSED_2) {
      blinkUpdate = 3;
      position++;

      if (position == SET_POSITION_YEAR) {
        ledSegmentsColons = DISPLAY_COLONS_BOTTOM_TWO;
        ledSegmentsDisplay = DISPLAY_DATE;
      }
      
      if (position > SET_POSITION_DAY) {
        position = SET_POSITION_NONE;
        ledSegmentsStatus = MODE_LED_NONE;
        exitFlag = true;
      }
    }

    updateBlinkTimer();

    if (seconds != previousSeconds || minutes != previousMinutes || hours != previousHours) {
      drawClockFace();
    }
    
    if (dayOfMonth != previousDayOfMonth || months != previousMonths || years != previousYears) {
      previousYears = years;
      previousMonths = months;
      previousDayOfMonth = dayOfMonth;
    }

    if (blinkUpdate > 0) {
      if (blinkUpdate < 2 && blinkActive) {
        drawConfigurationLedSegments(position);
      } else {
        blinkActive = false;
        drawConfigurationLedSegments(0);
        if (blinkUpdate == 2) {
          blinkTimer = millis();
        }
        else if (blinkUpdate == 3) {
          waitForReleaseAllButtons();
          blinkTimer = millis();
        }
      }
      blinkUpdate = 0;
    }

    if (pressedKeys != KEY_PRESSED_NONE && !isButtonPressed) {
      //  If in time set mode and first increment increase delay for the first pulse  
      delay(BUTTON_PAUSE_LONG_DELAY);
  
      //  Set flag after first increment so rapid advance will occur
      isButtonPressed = true;
    }    
  }

  if (settingsChangedFlag > 0) {
    setDateDs1307(0, minutes, hours, dayOfWeek, dayOfMonth, months, years);
    ringAnimation(COLOR_GREEN);
  } else {
    ringAnimation(COLOR_BLUE);
  }
  ringAnimation(COLOR_BLANK);

  // Force redrawing clock face.
  resetPreviousValues();

  //  Clear 7-segments display
  ledSegmentsClearAll();

  waitForReleaseAllButtons();
}

//  ====================================================================================

byte getSettingByPosition(byte position) {
  if (position == SET_POSITION_CLOCK_FACE) {
    return clockFace;
  }
  else if (position == SET_POSITION_TIME_DATE) {
    return (ledSegmentsSettings & 0xf0);
  }
  else if (position == SET_POSITION_ALT_TIMER) {
    return ledSegmentsToggleSeconds;
  }
  else if (position == SET_POSITION_FLASH_COLON) {
    return (ledSegmentsSettings & 0x0f);
  }
  else {
    return 0;
  }
}

void setSettingByPosition(byte position, byte value) {
  if (position == SET_POSITION_CLOCK_FACE) {
    clockFace = value;
  }
  else if (position == SET_POSITION_TIME_DATE) {
    ledSegmentsSettings = (minutesColor & 0x0f) | (value & 0xf0);
  }
  else if (position == SET_POSITION_ALT_TIMER) {
    ledSegmentsToggleSeconds = value;
  }
  else if (position == SET_POSITION_FLASH_COLON) {
    ledSegmentsSettings = (ledSegmentsSettings & 0xf0) | (value & 0x0f);
  }
}

byte findPreviousAltTime(byte value) {
  for (byte r = sizeof(valueAltTimes)-1; r > 0; r--) {
    if (valueAltTimes[r] == value) {
      return valueAltTimes[r-1];
    }
  }
  return valueAltTimes[sizeof(valueAltTimes)-1];
}

byte findNextAltTime(byte value) {
  for (byte r = 0; r < sizeof(valueAltTimes)-1; r++) {
    if (valueAltTimes[r] == value) {
      return valueAltTimes[r+1];
    }
  }
  return valueAltTimes[0];
}

void userSettings() {
  initUserSelect();
  
  settingsChangedFlag = 0;
  position = SET_POSITION_CLOCK_FACE;

  ledSegmentsStatus = MODE_LED_SET_SETTINGS;
  ledSegmentsDisplay = DISPLAY_SETTINGS;
  drawConfigurationLedSegments(position);
  ledWriteAllOff();
  waitForReleaseAllButtons();

  while(!exitFlag) {
    
    pressedKeys = readPressedKeys();

    if (pressedKeys == KEY_PRESSED_1) {
      byte value = getSettingByPosition(position);
      
      if (position == SET_POSITION_CLOCK_FACE) {
        value--;
        if (value >= DEFAULT_FACTORY_CLOCK_FACES) {
          value = 0;
        }        
      } else if (position == SET_POSITION_TIME_DATE) {
        if (value == DISPLAY_TIME_AND_DATE) {
          value = DISPLAY_TIME;
        } else if (value == DISPLAY_TIME) {
          value = DISPLAY_DATE;
        } else if (value == DISPLAY_DATE) {
          value = DISPLAY_NONE;
        } else {
          value = DISPLAY_TIME_AND_DATE;
        }
      } else if (position == SET_POSITION_ALT_TIMER) {
        value = findPreviousAltTime(value);
      } else if (position == SET_POSITION_FLASH_COLON) {
        if (value == DISPLAY_COLONS_FLASH_EVERY_SECOND) {
          value = DISPLAY_COLONS_ON;
        } else {
          value = DISPLAY_COLONS_FLASH_EVERY_SECOND;
        }
      }
      
      setSettingByPosition(position, value);

      settingsChangedFlag = 1;
      blinkUpdate = 2;
    }

    if (pressedKeys == KEY_PRESSED_3) {
      byte value = getSettingByPosition(position);
      
      if (position == SET_POSITION_CLOCK_FACE) {
        value++;
        if (value >= DEFAULT_FACTORY_CLOCK_FACES) {
          value = DEFAULT_FACTORY_CLOCK_FACES-1;
        }        
      } else if (position == SET_POSITION_TIME_DATE) {
        if (value == DISPLAY_TIME_AND_DATE) {
          value = DISPLAY_NONE;
        } else if (value == DISPLAY_NONE) {
          value = DISPLAY_DATE;
        } else if (value == DISPLAY_DATE) {
          value = DISPLAY_TIME;
        } else {
          value = DISPLAY_TIME_AND_DATE;
        }
      } else if (position == SET_POSITION_ALT_TIMER) {
        value = findNextAltTime(value);
      } else if (position == SET_POSITION_FLASH_COLON) {
        if (value == DISPLAY_COLONS_FLASH_EVERY_SECOND) {
          value = DISPLAY_COLONS_ON;
        } else {
          value = DISPLAY_COLONS_FLASH_EVERY_SECOND;
        }
      }

      setSettingByPosition(position, value);

      settingsChangedFlag = 1;
      blinkUpdate = 2;
    }

    if (pressedKeys == KEY_PRESSED_2) {
      blinkUpdate = 3;
      position++;
      if (position > SET_POSITION_FLASH_COLON) {
        position = SET_POSITION_NONE;        
        ledSegmentsStatus = MODE_LED_NONE;
        exitFlag = true;
      }
    }

    updateBlinkTimer();
    
    if (blinkUpdate > 0) {
      if (blinkUpdate < 2 && blinkActive) {
        drawConfigurationLedSegments(position);
      } else {
        blinkActive = false;
        drawConfigurationLedSegments(0);
        if (blinkUpdate >= 2) {
          waitForReleaseAllButtons();
          blinkTimer = millis();
        }
      }
      blinkUpdate = 0;
    }
  }

  if (settingsChangedFlag > 0) {
    EEPROM.write(EEPROM_CLOCK_FACE_NUMBER, clockFace);
    EEPROM.write(EEPROM_DATE_TIME_AND_COLON, ledSegmentsSettings);
    EEPROM.write(EEPROM_ALTERNATE_COUNTER, ledSegmentsToggleSeconds);
    ringAnimation(COLOR_GREEN);
  } else {
    ringAnimation(COLOR_BLUE);
  }
  ringAnimation(COLOR_BLANK);

  // Force redrawing clock face.
  resetPreviousValues();

  //  Clear 7-segments display
  ledSegmentsClearAll();
}

void userResetFactoryDefaults() {
  ledSegmentsStatus = MODE_LED_RESET;
  ledSegmentsDisplay = DISPLAY_RESET;
  ledSegmentsColons = DISPLAY_COLONS_OFF;
  ledWriteAllOff();

  strncpy_P(segmentsDisplayChars, DISP_RESET, 6);
  ledSegmentsDisplayChars();

  // Antimate circle and check keys are pressed continiuously.
  ringAnimationUntilNotKeyCombination(COLOR_RED, KEY_PRESSED_1_2);
  
  // If reset keys are still pressed, then factory reset settings.
  if (pressedKeys == KEY_PRESSED_1_2) {
    waitForReleaseAllButtons();
    ringAnimation(COLOR_BLANK);
    writeFactorySettingsToEeprom();
    loadSettingsOrFactoryDefaults();
    loadFaceSettingsOrFactoryDefaults();
    ringAnimation(COLOR_GREEN);
  }

  ringAnimation(COLOR_BLANK);

  // Force redrawing clock face.
  resetPreviousValues();

  //  Clear 7-segments display
  ledSegmentsClearAll();
}

//  ====================================================================================

  /**
   * BUTTON FUNCTIONALITY LEGEND
   * ---------------------------
   * Clock mode
   * Button 3 - Next clock style (0-9)
   * Button 2 - Enter menu
   * Button 1 - Previous clock style (0-9)
   * 
   * Menu
   * Button 3 - Next menu (1-3)
   * Button 2 - Enter menu
   * Button 1 - Previous menu (1-3)
   * 
   * Menu 1   - Set Time and Date
   *              Set Hour, Minutes, Seconds
   *                  Button 3 - Up
   *                  Button 2 - Enter
   *                  Button 1 - Down
   *             Set Year, Month, Day
   *                  Button 3 - Up
   *                  Button 2 - Enter
   *                  Button 1 - Down
   * Menu 2   - Config display
   *              Set Startup Face - Clock style (0-9)
   *              Set Display      - None, Time, Date, Time & Date alternating
   *              Set Speed        - Choose alternating speed in seconds
   *              Set Colons       - On or Flashing
   * Menu 3   - Config current clock style
   *              Set Hours
   *                  Button 3 - Change colors  (0-disable)
   *                  Button 1 - Hand, Dot, Trace
   *              Set Minutes
   *                  Button 3 - Change colors  (0-disable)
   *                  Button 1 - Hand, Dot, Trace
   *              Set Seconds
   *                  Button 3 - Change colors  (0-disable)
   *                  Button 1 - Hand, Dot, Trace
   *              Set Markers
   *                  Button 3 - Change colors  (0-disable)
   *                  Button 1 - Quarterly, Hourly, Twelve only
   * 
   * Reset factory settings
   * Button 1 & 2     - Hold down until full red circle is completed for reset to factory settings
   * 
   * Programming mode    
   * Button 1 & 2 & 3 - PIC standby for programming Arduino (PIC is also reading the buttons)
   *                    Button 1 - Exit PIC standby
   */

void loop() {
  pressedKeys = readPressedKeys();

  if (pressedKeys == KEY_PRESSED_1) {
    clockFace--;
    if (clockFace >= DEFAULT_FACTORY_CLOCK_FACES) {
      clockFace = DEFAULT_FACTORY_CLOCK_FACES-1;
    }
    userSelectedStyle();
  }
  
  if (pressedKeys == KEY_PRESSED_3) {
    clockFace++;
    if (clockFace >= DEFAULT_FACTORY_CLOCK_FACES) {
      clockFace = 0;
    }
    userSelectedStyle();
  }

  if (pressedKeys == KEY_PRESSED_2) {
    userSelectMode();
  }

  // Check if PushButton 1 and 2 are pressed simoultaneously.
  // Reset to factory defaults.
  //
  if (pressedKeys == KEY_PRESSED_1_2) {
    userResetFactoryDefaults();
  }
  
  if (mode == MODE_SET_TIME_AND_DATE) {
    userSetTimeAndDate();
  } else if (mode == MODE_SET_STYLING) {
    userSetFaceColorAndStyle();
  } else if (mode == MODE_SET_SETTINGS) {
    userSettings();
  }
  mode = MODE_NORMAL;
  normalMode();
}
