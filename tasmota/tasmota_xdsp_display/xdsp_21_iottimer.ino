/*
  xdsp_21_iottimer.ino - TM1640B LED display controller support for Tasmota
  (specifically for its use in the IOTTIMER WiFi clock)

  Copyright (C) 2024  Stefan Oskamp

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  The WiFi LED clock called IOTTIMER has the following characteristics:
   - Controlled by an ESP-12F
   - Display with four 35 mm (1 12/32 in), two 21 mm (26/32 in), and three 12 mm (~1/2 in),
     seven-segment LED digits, plus special symbols (alarm, AM/PM)
   - TM1640B LED controller
   - R8010 RTC with CR1220 battery
   - Temperature sensor M1601
   - Ambient light sensor (analog voltage)
   - Buzzer
   - Three buttons on the backside
   - USB C port for power supply (only)

  The TM1640B chip is a controller for a sixteen-digit seven-segment (plus dot) LED display.
  It is also sometimes used to control a 16 x 8 LED matrix. The controller is controlled 
  through a proprietary two-wire serial interface bearing some similarities with I2C. The 
  two wires are called CLK and DIN. We use two GPIO pins and one-microsecond sleeps to 
  implement the required timing.

  The wiring of the LEDs in the IOTTIMER clock has been optimized for a simple routing of the
  traces on the display board. The enumeration of the digit segments is non-standard, but
  consistent across all digits. The bigger digits have two LEDs per segment, controlled by
  separate digit lines of the LED controller. From the software perspective, they appear as
  two layers of four digits each.

  The brightness of the LEDs can be controlled in seven steps (plus off). In theory, the
  brightness of the segments with two LEDs could be set in fifteen levels (plus off).
  To keep things simple and to avoid brightness gradients within segments, both LEDs of a
  segment will always be set to the same level.

  The intention of this display driver (together with the drivers for the other components)
  is to be able to use the IOTTIMER as an alarm clock that can be fully integrated in your
  home automation using Tasmota and rules.

  This driver is not a generic TM1640B driver as use cases of the TM1640B in different 
  devices will differ significantly.
*/

#ifdef USE_DISPLAY
#ifdef USE_DISPLAY_IOTTIMER


/*********************************************************************************************\
  This driver enables the display of the current time, numbers (both integers and floats) and 
  basic text on the IOTTIMER clock.

  In addition, it is possible to set brightness (seven levels plus off), clear the display,
  scroll text, display a rudimentary bar graph.

  To use, compile Tasmota with USE_DISPLAY and USE_DISPLAY_IOTTIMER, or build the tasmota-display
  firmware.

  For the IOTTIMER clock assign the pins as follows from Tasmota's GUI:

  GPIO12 --> "IOTTIMER DIN"
  GPIO13 --> "IOTTIMER CLK"

  Once the GPIO configuration is saved and the ESP8266/ESP32 module restarts, set the Display
  Model to 21 and Display Mode to 0 using the command "Backlog DisplayModel 20 ; DisplayMode 0;"
  Before using it set the Display Type to 0 (for IOTTIMER) using the "DisplayType 0" command.

  After the ESP8266 restarts again, turn ON the display with the command "Power 1"

  Now, the following "Display" commands can be used:


  DisplayClear

                               Clears the display, command: "DisplayClear"


  DisplayNumber         num [,position {0-(Settings->display_width-1))} [,leading_zeros {0|1} [,length {1 to Settings->display_width}]]]

                               Clears and then displays number without decimal. command e.g., "DisplayNumber 1234"
                               Control 'leading zeros', 'length' and 'position' with  "DisplayNumber 1234, <position>, <leadingZeros>, <length>"
                               'leading zeros' can be 1 or 0 (default), 'length' can be 1 to Settings->display_width, 'position' can be 0 (left-most) to Settings->display_width (right-most).
                               See function description below for more details.

  DisplayNumberNC       num [,position {0-(Settings->display_width-1))} [,leading_zeros {0|1} [,length {1 to Settings->display_width}]]]

                               Display integer number as above, but without clearing first. e.g., "DisplayNumberNC 1234". Usage is same as above.



  DisplayFloat          num [,position {0-(Settings->display_width-1)} [,precision {0-Settings->display_width} [,length {1 to Settings->display_width}]]]

                               Clears and then displays float (with decimal point)  command e.g., "DisplayFloat 12.34"
                               See function description below for more details.



  DisplayFloatNC        num [,position {0-(Settings->display_width-1)} [,precision {0-Settings->display_width} [,length {1 to Settings->display_width}]]]

                               Displays float (with decimal point) as above, but without clearing first. command e.g., "DisplayFloatNC 12.34"
                               See function description below for more details.



  DisplayRaw            position {0-(Settings->display_width-1)},length {1 to Settings->display_width}, num1 [, num2[, num3[, num4[, ...upto Settings->display_width numbers]]]]]

                               Takes upto Settings->display_width comma-separated integers (0-255) and displays raw segments. Each number represents a
                               7-segment digit. Each 8-bit number represents individual segments of a digit.
                               For example, the command "DisplayRaw 0, 4, 255, 255, 255, 255" would display "[8.8.8.8.]"



  DisplayText           text [, position {0-(Settings->display_width-1)} [,length {1 to Settings->display_width}]]

                               Clears and then displays basic text.  command e.g., "DisplayText ajith vasudevan"
                               Control 'length' and 'position' with  "DisplayText <text>, <position>, <length>"
                               'length' can be 1 to Settings->display_width, 'position' can be 0 (left-most) to Settings->display_width-1 (right-most)
                               A caret(^) or backtick(`) symbol in the text input is dispayed as the degrees(°) symbol. This is useful for 
			       displaying Temperature! For example, the command "DisplayText 22.5^" will display "22.5°".


  DisplayTextNC         text [, position {0-Settings->display_width-1} [,length {1 to Settings->display_width}]]

                               Clears first, then displays text. Usage is same as above.



  DisplayScrollText     text [, num_loops]

                              Displays scrolling text indefinitely, until another Display- command (other than DisplayScrollText 
                              or DisplayScrollDelay is issued). Optionally, stop scrolling after num_loops iterations.



  DisplayScrollDelay delay {0-15}   // default = 4

                               Sets the speed of text scroll. Smaller delay = faster scrolling.



  DisplayLevel          num {0-100}

                               Display a horizontal bar graph (0-100) command e.g., "DisplayLevel 50" will display [||||    ]



  DisplayClock  1|2|0

                               Displays a clock.
                               Commands "DisplayClock 1"     // 12 hr format
                                        "DisplayClock 2"     // 24 hr format
                                        "DisplayClock 0"     // turn off clock


In addition, if you compile using USE_DISPLAY_MODES1TO5, setting DisplayMode to 1 shows the time,
setting it to 2 shows the date and setting it to 3 alternates between time and date (using "DisplayRefresh [1..7]" 
for the time and seconds you want to show the time before displaying the date)

\*********************************************************************************************/


#define XDSP_21                   21

#define CMD_MAX_LEN               55
#define LEVEL_MIN                 0
#define LEVEL_MAX                 100
#define SCROLL_MAX_LEN            50

#define IOTTIMER_DIGITS           9   // 4 (left) + 2 (lower right) + 3 (upper right).

// IOTTIMER display bits:
#define IOTTIMER_DISPLAY_DOT       7

// IOTTIMER Control bits:
#define IOTTIMER_CONTROL_ON          0
#define IOTTIMER_CONTROL_RESERVED1   1
#define IOTTIMER_CONTROL_RESERVED2   2
#define IOTTIMER_CONTROL_DOT         3   // Dots can be switched on/off with both control and display.
#define IOTTIMER_CONTROL_BRIGHTNESS  4   // Bits 4...6
#define IOTTIMER_CONTROL_RESERVED3   7

#include <Wire.h>

static unsigned char IOTTIMERControl[IOTTIMER_DIGITS] = {0, 0, 0, 0};
static unsigned char IOTTIMERDisplay[IOTTIMER_DIGITS] = {0, 0, 0, 0};

static const byte IOTTIMERFont[128] {
//0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x00
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x10
//      !     "                             '     (     )                       -
  0x00, 0x82, 0x21, 0x00, 0x00, 0x00, 0x00, 0x02, 0x39, 0x0F, 0x00, 0x00, 0x00, 0x40, 0x80, 0x00, // 0x20
//0     1     2     3     4     5     6     7     8     9                       =           ?
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7f, 0x6f, 0x00, 0x00, 0x00, 0x48, 0x00, 0x53, // 0x30
//      A     B     C     D     E     F     G     H     I     J           L           N     O
  0x00, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0x6F, 0x76, 0x06, 0x1E, 0x00, 0x38, 0x00, 0x54, 0x3F, // 0x40
//P     Q     R     S     T     U                       Y           [           ]     ^=°   _
  0x73, 0x67, 0x50, 0x6D, 0x78, 0x3E, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x39, 0x00, 0x0F, 0x63, 0x08, // 0x50 
//`=°   a     b     c     d     e     f     g     h     i     j           l           n     o
  0x63, 0x5F, 0x7C, 0x58, 0x5E, 0x7B, 0x71, 0x6F, 0x74, 0x02, 0x1E, 0x00, 0x06, 0x00, 0x54, 0x5C, // 0x60
//p     q     r     s     t     u                       y           {     |     }
  0x73, 0x67, 0x50, 0x6D, 0x78, 0x1C, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x39, 0x30, 0x0F, 0x00, 0x00  // 0x70
};

enum display_options_types
{
  IOTTIMER, // IOTTIMER WiFi clock
  SOMETHINGELSE
};

struct
{
  uint8_t clk
  char scroll_text[CMD_MAX_LEN];
  char msg[60];
  char model_name[8];
  uint8_t scroll_delay = 4;
  uint8_t scroll_index = 0;
  uint8_t iteration = 0;
  uint8_t scroll_counter = 0;
  uint8_t scroll_counter_max = 3;
  uint8_t display_type = XDSP_21; 

  bool init_driver_done = false;
  bool scroll = false;
  bool show_clock = false;
  bool clock_24 = false;
  bool clock_blynk_dots = true;
} IOTTIMERData;

/*********************************************************************************************\
* Init function
\*********************************************************************************************/
void IOTTIMERInit(uint8_t mode)
{
  switch(mode) {
    case DISPLAY_INIT_MODE:
      IOTTIMERDim();
      IOTTIMERClearDisplay();
      break;
    case DISPLAY_INIT_PARTIAL:
    case DISPLAY_INIT_FULL:
      IOTTIMERDim();
      IOTTIMERClearDisplay();
      break;
  }
}

/*********************************************************************************************\
* Init Drive function
\*********************************************************************************************/
void IOTTIMERInitDriver(void)
{
  if (!TasmotaGlobal.i2c_enabled) {
    return;
  }

  if (!Settings->display_model) {
    if (I2cSetDevice(IOTTIMER_CONTROL_BASE)) {
      Settings->display_address[0] = IOTTIMER_CONTROL_BASE;
      Settings->display_model = XDSP_20;
    }
  }

  if (XDSP_20 == Settings->display_model) {
    I2cSetActiveFound(Settings->display_address[0], "IOTTIMER");

    Settings->display_cols[0] = 4;
    Settings->display_rows = 1;
    Settings->display_width = Settings->display_cols[0];
    Settings->display_height = Settings->display_rows;

    if (T_XY_CLOCK == Settings->display_options.type)
    {
      strcpy_P(IOTTIMERData.model_name, PSTR("XY-Clock"));
    }
    else if (NOTUSED == Settings->display_options.type)
    {
      strcpy_P(IOTTIMERData.model_name, PSTR("NOTUSED"));
    }
    else if (T_303WIFILC01 == Settings->display_options.type)
    {
      strcpy_P(IOTTIMERData.model_name, PSTR("303WiFiLC01"));
    }

    IOTTIMERDim();
    IOTTIMERClearDisplay();

    AddLog(LOG_LEVEL_INFO, PSTR("DSP: IOTTIMER \"%s\" with %d digits (type %d)"), IOTTIMERData.model_name, Settings->display_width, Settings->display_options.type);
    IOTTIMERData.init_driver_done = true;
  }
}

void IOTTIMERDisplayOn ()
{
  for (int i = 0; i < IOTTIMER_DIGITS; i++) {
  	IOTTIMERControl[i] |= _BV(IOTTIMER_CONTROL_ON);
    Wire.beginTransmission(IOTTIMER_CONTROL_BASE + i);
    Wire.write(IOTTIMERControl[i]);
    Wire.endTransmission();
  }
}

void IOTTIMERDisplayOff ()
{
  for (int i = 0; i < IOTTIMER_DIGITS; i++) {
	  IOTTIMERControl[i] &= ~_BV(IOTTIMER_CONTROL_ON);
    Wire.beginTransmission(IOTTIMER_CONTROL_BASE + i);
    Wire.write(IOTTIMERControl[i]);
    Wire.endTransmission();
  }
}

void IOTTIMERDisplayOnOff()
{
  if (disp_power) {
    IOTTIMERDisplayOn();
  }
  else {
    IOTTIMERDisplayOff();
  }
}

void IOTTIMERSetBrightness (unsigned int level)
{
  if (level > 0b111) level = 0b111;

  for (int i = 0; i < IOTTIMER_DIGITS; i++) {
	  IOTTIMERControl[i] = IOTTIMERControl[i] & ~(0b111 << IOTTIMER_CONTROL_BRIGHTNESS) | level << IOTTIMER_CONTROL_BRIGHTNESS;
    Wire.beginTransmission(IOTTIMER_CONTROL_BASE + i);
    Wire.write(IOTTIMERControl[i]);
    Wire.endTransmission();
  }
}


void IOTTIMERDisplayText (char *text)  // Text shall match regex (([^.]?\.?){0,4}\0), e.g., 123.4 or 8.8.8.8 for full digit
{
  //AddLog(LOG_LEVEL_DEBUG, PSTR("IOTTIMERDisplayText(\"%s\")"), text);
  for (int i = 0; i < IOTTIMER_DIGITS; i++) {
    
    if (*text != 0) {
      if (*text == '.') { 
        IOTTIMERDisplay[i] = 0; // Blank this digit, set the dot below.
      }
      else {  // Something to display.
        char c = *text++;
        IOTTIMERDisplay[i] = IOTTIMERFont[c & ~_BV(IOTTIMER_DISPLAY_DOT)];
      }

      if (*text == '.') {
        char c = *text++;
        IOTTIMERDisplay[i] |= _BV(IOTTIMER_DISPLAY_DOT);
      }

      if (T_303WIFILC01 == Settings->display_options.type) {
        IOTTIMERDisplay[i] = swapbits(IOTTIMERDisplay[i]); // 303WIFILC01 board has special wiring
      }
    }
    else {
      IOTTIMERDisplay[i] = 0;  // Clear digits after the text.
    }
    Wire.beginTransmission(IOTTIMER_DISPLAY_BASE + i);
    Wire.write(IOTTIMERDisplay[i]);
    Wire.endTransmission();
  }
}

uint8_t swapbits(uint8_t n)
{
    // For model 303WiFiLC01 board has different segments wiring  
    // PGFEDCBA -> BFAEDCGP
 
            //seg EDC    //seg A             //seg B
     return (n & 0x1C) | ((n & 0x01) << 5) | ((n & 0x02) << 6)
     | ((n & 0x20) << 1) | ((n & 0x40) >> 5) | ((n & 0x80) >> 7); 
       //seg F             //seg G             //seg P
}  

void IOTTIMERDisplayRaw (char *text)  // Text shall match regex (([^.]?\.?){0,4}\0), e.g., 123.4
{
  for (int i = 0; i < IOTTIMER_DIGITS; i++) {
    char c = *text++;
    IOTTIMERDisplay[i] = c;
    //AddLog(LOG_LEVEL_DEBUG, PSTR("Raw Digit(%d - %d - %c)"), i, IOTTIMERDisplay[i], IOTTIMERDisplay[i]);
    Wire.beginTransmission(IOTTIMER_DISPLAY_BASE + i);
    Wire.write(IOTTIMERDisplay[i]);
    Wire.endTransmission();
  }
}


/*********************************************************************************************\
* Displays number without decimal, with/without leading zeros, specifying start-position
* and length, optionally skipping clearing display before displaying the number.
* commands: DisplayNumber   num [,position {0-(Settings->display_width-1)} [,leading_zeros {0|1} [,length {1 to Settings->display_width}]]]
*           DisplayNumberNC num [,position {0-(Settings->display_width-1)} [,leading_zeros {0|1} [,length {1 to Settings->display_width}]]]    // "NC" --> "No Clear"
\*********************************************************************************************/

bool CmndIOTTIMERNumber(bool clear)
{
  char sNum[CMD_MAX_LEN];
  char sLeadingzeros[CMD_MAX_LEN];
  char sPosition[CMD_MAX_LEN];
  char sLength[CMD_MAX_LEN];
  uint8_t length = 0;
  bool leadingzeros = false;
  uint8_t position = 0;

  uint32_t num = 0;

  switch (ArgC())
  {
  case 4:
    subStr(sLength, XdrvMailbox.data, ",", 4);
    length = atoi(sLength);
  case 3:
    subStr(sLeadingzeros, XdrvMailbox.data, ",", 3);
    leadingzeros = atoi(sLeadingzeros);
  case 2:
    subStr(sPosition, XdrvMailbox.data, ",", 2);
    position = atoi(sPosition);
  case 1:
    subStr(sNum, XdrvMailbox.data, ",", 1);
    num = TextToInt(sNum);
  }

  if ((position < 0) || (position > (Settings->display_width - 1)))
    position = 0;

  AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: num %d, pos %d, lead %d, len %d"), num, position, leadingzeros, length);

  if (clear)
    IOTTIMERClearDisplay();

  char txt[30];
  snprintf_P(txt, sizeof(txt), PSTR("%d"), num);
  if (!length)
    length = strlen(txt);
  if ((length < 0) || (length > Settings->display_width))
    length = Settings->display_width;

  char pad = (leadingzeros ? '0' : ' ');
  uint32_t i = position;

  char text[IOTTIMER_DIGITS + 1];

  //write empty pos
  for (uint32_t j=0; j < position; j++)
  {
    if (j >= Settings->display_width)
      break;
    text[j] = ' ';
  }

  //write pads
  for (; i < position + (length - strlen(txt)); i++)
  {
    if (i >= Settings->display_width)
      break;
    text[i] = pad;
  }

  //write digits
  for (uint32_t j = 0; i < position + length; i++, j++)
  {
    if (i >= Settings->display_width)
      break;
    if (txt[j] == 0)
      break;
    if (txt[j] == '.') {
      i--;
      continue;
    }
    text[i] = txt[j];
  }

  text[i] = 0; //string termination

  IOTTIMERDisplayText(text);
  return true;
}


/*********************************************************************************************\
* Displays number with decimal, specifying position, precision and length,
* optionally skipping clearing display before displaying the number.
* commands: DisplayFloat   num [,position {0-(Settings->display_width-1)} [,precision {0-Settings->display_width} [,length {1 to Settings->display_width}]]]
*           DisplayFloatNC num [,position {0-(Settings->display_width-1)} [,precision {0-Settings->display_width} [,length {1 to Settings->display_width}]]]    // "NC" --> "No Clear"
\*********************************************************************************************/

bool CmndIOTTIMERFloat(bool clear)
{

  char sNum[CMD_MAX_LEN];
  char sPrecision[CMD_MAX_LEN];
  char sPosition[CMD_MAX_LEN];
  char sLength[CMD_MAX_LEN];
  uint8_t length = 0;
  uint8_t precision = Settings->display_width-1;
  uint8_t position = 0;

  float fnum = 0.0f;

  switch (ArgC())
  {
  case 4:
    subStr(sLength, XdrvMailbox.data, ",", 4);
    length = atoi(sLength);
  case 3:
    subStr(sPrecision, XdrvMailbox.data, ",", 3);
    precision = atoi(sPrecision);
  case 2:
    subStr(sPosition, XdrvMailbox.data, ",", 2);
    position = atoi(sPosition);
  case 1:
    subStr(sNum, XdrvMailbox.data, ",", 1);
    fnum = CharToFloat(sNum);
  }

  if ((position < 0) || (position > (Settings->display_width)))
    position = 0;
  if ((precision < 0) || (precision >= Settings->display_width))
    precision = Settings->display_width-1;

  if (clear)
    IOTTIMERClearDisplay();

  char txt[30];
  ext_snprintf_P(txt, sizeof(txt), PSTR("%*_f"), precision, &fnum);

  if (!length)
    length = strlen(txt);
  if ((length <= 0) || (length > Settings->display_width))
    length = Settings->display_width;

  AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: num %4_f, pos %d, prec %d, len %d, txt %s"), &fnum, position, precision, length, txt);

  uint32_t i = position;
  uint32_t dots = 0;
  char text[IOTTIMER_DIGITS + 1 + 1];

  //write empty pos
  for (uint32_t j=0; j < position; j++)
  {
    if (j >= Settings->display_width)
      break;
    text[j] = ' ';
  }

  if (T_XY_CLOCK == Settings->display_options.type) {

    for (uint32_t j = 0; i < position + length + dots; i++, j++)
    {
      if (txt[j] == '.') dots++;
      if (i >= Settings->display_width + dots)
        break;
      if (txt[j] == 0)
        break;
      if (txt[j] == '.') {
        if(i==2) {
          //2nd dot ok
        }
        else if(i==3){
          //3rd dot but move do 1st 
          text[i] = text[i-1];
          text[i-1] = text[i-2];
          text[i-2] = txt[j];
          continue;
        }
        else {
          //dot on 1st or 4th
          AddLog(LOG_LEVEL_INFO, PSTR("TM5: Can't display this float"));
          return false;
        }
      }
      text[i] = txt[j];
    }

  }
  else if (T_303WIFILC01 == Settings->display_options.type) {
     //todo
     AddLog(LOG_LEVEL_INFO, PSTR("TM5: DisplayFloat not implemented yet for display type: 303WifiClock"));
     return false;
  }

  text[i] = 0; //string termination

  IOTTIMERDisplayText(text);
  return true;
}

// /*********************************************************************************************\
// * Clears the display
// * Command:  DisplayClear
// \*********************************************************************************************/
bool CmndIOTTIMERClear(void)
{
  IOTTIMERClearDisplay();
  sprintf(IOTTIMERData.msg, PSTR("Cleared"));
  XdrvMailbox.data = IOTTIMERData.msg;
  return true;
}

// /*********************************************************************************************\
// * Clears the display
// \*********************************************************************************************/
void IOTTIMERClearDisplay (void)
{
  for (int i = 0; i < IOTTIMER_DIGITS; i++) {
	  IOTTIMERDisplay[i] = 0;
    Wire.beginTransmission(IOTTIMER_DISPLAY_BASE + i);
    Wire.write(IOTTIMERDisplay[i]);
    Wire.endTransmission();
  }
}

/*********************************************************************************************\
* Display scrolling text
* Command:   DisplayScrollText text
\*********************************************************************************************/
bool CmndIOTTIMERScrollText(void)
{

  char sString[SCROLL_MAX_LEN + 1];
  char sMaxLoopCount[CMD_MAX_LEN];
  uint8_t maxLoopCount = 0;

  switch (ArgC())
  {
  case 2:
    subStr(sMaxLoopCount, XdrvMailbox.data, ",", 2);
    maxLoopCount = atoi(sMaxLoopCount);
  case 1:
    subStr(sString, XdrvMailbox.data, ",", 1);
  }

  if (maxLoopCount < 0)
    maxLoopCount = 0;

  //AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: sString %s, maxLoopCount %d"), sString, maxLoopCount);

  IOTTIMERData.scroll_counter_max = maxLoopCount;

  if (strlen(sString) > SCROLL_MAX_LEN)
  {
    snprintf(IOTTIMERData.msg, sizeof(IOTTIMERData.msg), PSTR("Text too long. Length should be less than %d"), SCROLL_MAX_LEN);
    XdrvMailbox.data = IOTTIMERData.msg;
    return false;
  }
  else
  {
    snprintf(IOTTIMERData.scroll_text, sizeof(IOTTIMERData.scroll_text), PSTR("                                                               "));
    snprintf(IOTTIMERData.scroll_text, Settings->display_width + sizeof(IOTTIMERData.scroll_text), PSTR("    %s"), &sString);
    IOTTIMERData.scroll_text[strlen(sString) + Settings->display_width] = 0;
    IOTTIMERData.scroll_index = 0;
    IOTTIMERData.scroll = true;
    IOTTIMERData.scroll_counter = 0;
    return true;
  }
}

/*********************************************************************************************\
* Sets the scroll delay for scrolling text.
* Command:  DisplayScrollDelay delay {0-15}    // default = 4
\*********************************************************************************************/
bool CmndIOTTIMERScrollDelay(void)
{
  if (ArgC() == 0)
  {
    XdrvMailbox.payload = IOTTIMERData.scroll_delay;
    return true;
  }
  if (IOTTIMERData.scroll_delay < 0)
    IOTTIMERData.scroll_delay = 0;
  IOTTIMERData.scroll_delay = XdrvMailbox.payload;
  return true;
}

/*********************************************************************************************\
* Scrolls a given string. Called every 50ms
\*********************************************************************************************/
void IOTTIMERScrollText(void)
{
  if(!IOTTIMERData.scroll) return;
  IOTTIMERData.iteration++;
  if (IOTTIMERData.scroll_delay)
    IOTTIMERData.iteration = IOTTIMERData.iteration % IOTTIMERData.scroll_delay;
  else
    IOTTIMERData.iteration = 0;
  if (IOTTIMERData.iteration)
    return;

  if (IOTTIMERData.scroll_index > strlen(IOTTIMERData.scroll_text))
  {
    IOTTIMERData.scroll_index = 0;
    IOTTIMERData.scroll_counter++;
    if(IOTTIMERData.scroll_counter_max != 0 && (IOTTIMERData.scroll_counter >= IOTTIMERData.scroll_counter_max)) {
      IOTTIMERData.scroll = false;
      return;
    }    
  }
 
  char text[CMD_MAX_LEN + 2 + 1];
  uint32_t i;
  uint32_t j;
  for (i = 0, j = IOTTIMERData.scroll_index; i < 1 + strlen(IOTTIMERData.scroll_text); i++, j++)
  {
    if (i > (Settings->display_width - 1))
    {
      break;
    }
     text[i] = IOTTIMERData.scroll_text[j];
  }

  text[i] = 0; //string termination

  IOTTIMERDisplayText(text);
  IOTTIMERData.scroll_index++;
}

/*********************************************************************************************\
* Displays a horizontal bar graph. Takes a percentage number (0-100) as input
* Command:   DisplayLevel level {0-100}
\*********************************************************************************************/
bool CmndIOTTIMERLevel(void)
{
  uint16_t val = XdrvMailbox.payload;
  if ((val < LEVEL_MIN) || (val > LEVEL_MAX))
  {
    Response_P(PSTR("{\"Error\":\"Level should be a number in the range [%d, %d]\"}"), LEVEL_MIN, LEVEL_MAX);
    return false;
  }

  char text[IOTTIMER_DIGITS + 1];
  for (uint32_t i = 0; i < IOTTIMER_DIGITS; i++)
  {
     text[i]=0;
  }

  uint8_t totalBars = 2 * Settings->display_width;
  AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: CmndIOTTIMERLevel totalBars=%d"), totalBars);
  float barsToDisplay = totalBars * val / 100.0f;
  char txt[5];
  ext_snprintf_P(txt, sizeof(txt), PSTR("%*_f"), 1, &barsToDisplay);
  AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: CmndIOTTIMERLevel barsToDisplay=%s"), txt);
  char s[4];
  ext_snprintf_P(s, sizeof(s), PSTR("%0_f"), &barsToDisplay);
  uint8_t numBars = atoi(s);
  AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: CmndIOTTIMERLevel numBars %d"), numBars);

  IOTTIMERClearDisplay();

  uint8_t digit = numBars / 2;
  uint8_t remainder = numBars % 2;

  for (uint32_t i = 0; i < IOTTIMER_DIGITS; i++)
  {
    if (i >= Settings->display_width)
      break;
    if(i<digit) {
      text[i] = 0x36;
    }
    else if(i == digit && remainder == 1) {
       text[i] = 0x30;
    }
    else {
      text[i] = 0;
    }
  }

  IOTTIMERDisplayRaw(text);

  return true;
}

/*********************************************************************************************\
* Display arbitrary data on the display module
* Command:   DisplayRaw position {0-(Settings->display_width-1)},length {1 to Settings->display_width}, a [, b[, c[, d[...upto Settings->display_width]]]]
* where a,b,c,d... are upto Settings->display_width numbers in the range 0-255, each number (byte)
* corresponding to a single 7-segment digit. Within each byte, bit 0 is segment A,
* bit 1 is segment B etc. The function may either set the entire display
* or any desired part using the length and position parameters.
\*********************************************************************************************/
bool CmndIOTTIMERRaw(void)
{
  char text[IOTTIMER_DIGITS + 1];
  for (uint32_t i = 0; i < IOTTIMER_DIGITS; i++)
  {
     text[i]=0;
  }

  uint8_t DATA[4] = {0, 0, 0, 0};

  char as[CMD_MAX_LEN];
  char bs[CMD_MAX_LEN];
  char cs[CMD_MAX_LEN];
  char ds[CMD_MAX_LEN];


  char sLength[CMD_MAX_LEN];
  char sPos[CMD_MAX_LEN];

  uint32_t position = 0;
  uint32_t length = 0;

  switch (ArgC())
  {
  case 6:
    subStr(ds, XdrvMailbox.data, ",", 6);
    DATA[3] = atoi(ds);
  case 5:
    subStr(cs, XdrvMailbox.data, ",", 5);
    DATA[2] = atoi(cs);
  case 4:
    subStr(bs, XdrvMailbox.data, ",", 4);
    DATA[1] = atoi(bs);
  case 3:
    subStr(as, XdrvMailbox.data, ",", 3);
    DATA[0] = atoi(as);
  case 2:
    subStr(sLength, XdrvMailbox.data, ",", 2);
    length = atoi(sLength);
  case 1:
    subStr(sPos, XdrvMailbox.data, ",", 1);
    position = atoi(sPos);
  }

  if (!length)
    length = ArgC() - 2;
  if (length < 0 || length > Settings->display_width)
    length = Settings->display_width;
  if (position < 0 || position > (Settings->display_width - 1))
    position = 0;

  AddLog(LOG_LEVEL_DEBUG, PSTR("TM5: a %d, b %d, c %d, d %d, len %d, pos %d"),
         DATA[0], DATA[1], DATA[2], DATA[3], length, position);

  for (uint32_t i = position; i < position + length; i++)
  {
    if (i >= Settings->display_width)
      break;
    text[i] = DATA[i - position];
  }

  IOTTIMERDisplayRaw(text);

  return true;
}

/*********************************************************************************************\
* Display a given string.
* Text can be placed at arbitrary location on the display using the length and
* position parameters without affecting the rest of the display.
* Command:   DisplayText text [, position {0-(Settings->display_width-1)} [,length {1 to Settings->display_width}]]
\*********************************************************************************************/
bool CmndIOTTIMERText(bool clear)
{
  char text[CMD_MAX_LEN + 2 + 1];
  char sString[CMD_MAX_LEN + 2 + 1];
  char sPosition[CMD_MAX_LEN];
  char sLength[CMD_MAX_LEN];
  uint8_t length = 0;
  uint8_t position = 0;
  uint8_t dots = 0;
  uint8_t strLen = 0;

  switch (ArgC())
  {
  case 3:
    subStr(sLength, XdrvMailbox.data, ",", 3);
    length = atoi(sLength);
  case 2:
    subStr(sPosition, XdrvMailbox.data, ",", 2);
    position = atoi(sPosition);
  case 1:
    subStr(sString, XdrvMailbox.data, ",", 1);
  }

  if ((position < 0) || (position > (Settings->display_width - 1)))
    position = 0;
  
  strLen = strlen(sString);

  if (!length)
    length = Settings->display_width;
  if ((length < 0) || (length > Settings->display_width))
    length = Settings->display_width;

  if (clear)
    IOTTIMERClearDisplay();

  uint32_t s = 0;
  uint32_t i = 0;
  
  for (i = 0; i < (strLen + position); i++)
  {
    if ((i >= (length + dots + position)) || dots > 4) {
	    break;
    }
	  
	  if(i<position) {
      text[i] = ' '; 
      continue;
	  }
	  
	  if (sString[s] == '.' && dots <= 4) {
      dots++;
    }
	
    text[i] = sString[s];
    s++;
  }

  if(i<strLen && sString[s] == '.') {
      text[i] = '.';
      i++;
  }

  text[i] = 0; //terminate string

  IOTTIMERDisplayText(text);
  return true;
}


/*********************************************************************************************\
* Displays a clock.
* Command: DisplayClock 1   // 12-hour format
*          DisplayClock 2   // 24-hour format
*          DisplayClock 0   // turn off clock and clear
\*********************************************************************************************/

bool CmndIOTTIMERClock(void)
{
  uint16_t val = XdrvMailbox.payload;

  if (ArgC() == 0)
    val = 0;

  if ((val < 0) || (val > 2))
    return false;

  if (val == 1) {
    IOTTIMERData.show_clock = true;
    IOTTIMERData.clock_24 = false;
  } 
  else if (val == 2) {
    IOTTIMERData.show_clock = true;
    IOTTIMERData.clock_24 = true;
  } else {
    IOTTIMERData.show_clock = false;
    IOTTIMERData.clock_24 = false;
  }

  IOTTIMERClearDisplay();
  return true;
}


/*********************************************************************************************\
* refreshes the time if clock is displayed
\*********************************************************************************************/
void IOTTIMERShowTime()
{
  IOTTIMERData.iteration++;
  if (20 != IOTTIMERData.iteration) {
    // every 20*50ms = 1000 ms should be enough
    return;
  }
  IOTTIMERData.iteration = 0;

  char text[IOTTIMER_DIGITS + 2 + 1];
  int i = 0;
  uint8_t hour = RtcTime.hour;
  uint8_t min = RtcTime.minute;
  
  if (!IOTTIMERData.clock_24)
  {
    if (hour > 12)
      hour -= 12;
    if (hour == 0)
      hour = 12;
  }

  text[i++] = '0' + hour / 10;
  text[i++] = '0' + hour % 10;

  if (T_XY_CLOCK == Settings->display_options.type) {
    text[i++] = '0' + min / 10;
    if(IOTTIMERData.clock_blynk_dots)  text[i++] = '.';  // Lower half of the colon, depending on how the LEDs are connected to the IOTTIMER in the XY-Clock.
    text[i++] = '0' + min % 10;
    if(IOTTIMERData.clock_blynk_dots)  text[i++] = '.';  // Upper half of the colon.
  }
  else if (T_303WIFILC01 == Settings->display_options.type) {
    if(IOTTIMERData.clock_blynk_dots) text[i++] = '.';  // Colon for 303WIFILC01
    text[i++] = '0' + min / 10;
    text[i++] = '0' + min % 10;
  }

  text[i++] = 0;

  if (!IOTTIMERData.clock_24 && text[0] == '0')
  {
    text[0] = ' ';
  }
  
  IOTTIMERData.clock_blynk_dots = !IOTTIMERData.clock_blynk_dots;
  IOTTIMERDisplayText(text);
}

/*********************************************************************************************\
* This function is called for all Display functions.
\*********************************************************************************************/
bool IOTTIMERMainFunc(uint8_t fn)
{
  bool result = false;
  if(fn != FUNC_DISPLAY_SCROLLDELAY) IOTTIMERData.scroll = false;
  if (XdrvMailbox.data_len > CMD_MAX_LEN)
  {
    Response_P(PSTR("{\"Error\":\"Command text too long. Please limit it to %d characters\"}"), CMD_MAX_LEN);
    return false;
  }

  switch (fn)
  {
  case FUNC_DISPLAY_CLEAR:
    result = CmndIOTTIMERClear();
    break;
  case FUNC_DISPLAY_NUMBER:
    result = CmndIOTTIMERNumber(true);
    break;
  case FUNC_DISPLAY_NUMBERNC:
    result = CmndIOTTIMERNumber(false);
    break;
  case FUNC_DISPLAY_FLOAT:
    result = CmndIOTTIMERFloat(true);
    break;
  case FUNC_DISPLAY_FLOATNC:
    result = CmndIOTTIMERFloat(false);
    break;
  case FUNC_DISPLAY_RAW:
    result = CmndIOTTIMERRaw();
    break;
  case FUNC_DISPLAY_SEVENSEG_TEXT:
    result = CmndIOTTIMERText(true);
    break;
  case FUNC_DISPLAY_SEVENSEG_TEXTNC:
    result = CmndIOTTIMERText(false);
    break;
  case FUNC_DISPLAY_LEVEL:
    result = CmndIOTTIMERLevel();
    break;
  case FUNC_DISPLAY_SCROLLTEXT:
    result = CmndIOTTIMERScrollText();
    break;
  case FUNC_DISPLAY_SCROLLDELAY:
    result = CmndIOTTIMERScrollDelay();
    break;
  case FUNC_DISPLAY_CLOCK:
    result = CmndIOTTIMERClock();
    break;
  }

  return result;
}


void IOTTIMERDim(void)
{
  int brightness = GetDisplayDimmer16();
  if (brightness < 2) {
    IOTTIMERDisplayOff();
  }
  else if (brightness > 14) {
    IOTTIMERDisplayOn();
    IOTTIMERSetBrightness(0); // In IOTTIMER, brightness 0 means max brightness (level 8).
    IOTTIMERDisplayOn();
  }
  else {
    // Map 2...14 to 1...7:
    IOTTIMERSetBrightness(brightness >> 1);
    IOTTIMERDisplayOn();
  }
}


/*********************************************************************************************/

#ifdef USE_DISPLAY_MODES1TO5

void IOTTIMERTime(void)
{
  char text[IOTTIMER_DIGITS + 2 + 1];
  int i = 0;

  text[i++] = '0' + RtcTime.hour / 10;
  text[i++] = '0' + RtcTime.hour % 10;

  if (T_XY_CLOCK == Settings->display_options.type) {
    text[i++] = '0' + RtcTime.minute / 10;
    if(IOTTIMERData.clock_blynk_dots)  text[i++] = '.';  // Lower half of the colon, depending on how the LEDs are connected to the IOTTIMER in the XY-Clock.
    text[i++] = '0' + RtcTime.minute % 10;
    if(IOTTIMERData.clock_blynk_dots)  text[i++] = '.';  // Upper half of the colon.
  }
  else if (T_303WIFILC01 == Settings->display_options.type) {
    if(IOTTIMERData.clock_blynk_dots) text[i++] = '.';  // Colon for 303WIFILC01
    text[i++] = '0' + RtcTime.minute / 10;
    text[i++] = '0' + RtcTime.minute % 10;
  }

  text[i++] = 0;
  
  IOTTIMERData.clock_blynk_dots = !IOTTIMERData.clock_blynk_dots;
  IOTTIMERDisplayText(text);
}

void IOTTIMERDate(void)
{
  char text[IOTTIMER_DIGITS + 2 + 1];
  int i = 0;

  text[i++] = '0' + RtcTime.day_of_month / 10;
  text[i++] = '0' + RtcTime.day_of_month % 10;
  
  if (T_XY_CLOCK == Settings->display_options.type) {
    text[i++] = '0' + RtcTime.month / 10;
    text[i++] = '0' + RtcTime.month % 10;
    text[i++] = '.';  // Lower half of the colon, depending on how the LEDs are connected to the IOTTIMER in the XY-Clock.
  }
  else if (T_303WIFILC01 == Settings->display_options.type ) {
    text[i++] = '.';  // Colon for 303WIFILC01
    text[i++] = '0' + RtcTime.month / 10;
    text[i++] = '0' + RtcTime.month % 10;
  }

  text[i++] = 0;

  IOTTIMERDisplayText(text);
}

void IOTTIMERRefresh(void)  // Every second
{
  if (Settings->display_mode) {  // Mode 0 is User text
    switch (Settings->display_mode) {
      case 1:  // Time
        IOTTIMERTime();
        break;
      case 2:  // Date
        IOTTIMERDate();
        break;
      case 3: // Time/Date
        if (TasmotaGlobal.uptime % Settings->display_refresh)
        {
          IOTTIMERTime();
        }
        else
        {
          IOTTIMERDate();
        }
        break;
      case 4: 
      case 5:
        // not in use
        break;
    }
  }
}


#endif // USE_DISPLAY_MODES1TO5

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdsp20(uint32_t function)
{
  if (!I2cEnabled(XI2C_74)) { return false; }

  bool result = false;

  if (FUNC_DISPLAY_INIT_DRIVER == function)
  {
    IOTTIMERInitDriver();
  }
  else if ((IOTTIMERData.init_driver_done || FUNC_DISPLAY_MODEL == function) 
           && (XDSP_20 == Settings->display_model)) {
    switch (function) {
      case FUNC_DISPLAY_EVERY_50_MSECOND:
      if (disp_power && !Settings->display_mode)
      {
        if (IOTTIMERData.scroll)
        {
          IOTTIMERScrollText();
        }
        if (IOTTIMERData.show_clock)
        {
          IOTTIMERShowTime();
        }
      }
      break;
      case FUNC_DISPLAY_INIT:
        IOTTIMERInit(dsp_init);
        break;
#ifdef USE_DISPLAY_MODES1TO5
      case FUNC_DISPLAY_EVERY_SECOND:
        IOTTIMERRefresh();
        break;
#endif  // USE_DISPLAY_MODES1TO5
      case FUNC_DISPLAY_MODEL:
        result = true;
        break;
      case FUNC_DISPLAY_SEVENSEG_TEXT:
      case FUNC_DISPLAY_CLEAR:
      case FUNC_DISPLAY_NUMBER:
      case FUNC_DISPLAY_FLOAT:
      case FUNC_DISPLAY_NUMBERNC:
      case FUNC_DISPLAY_FLOATNC:
      case FUNC_DISPLAY_RAW:
      case FUNC_DISPLAY_LEVEL:
      case FUNC_DISPLAY_SEVENSEG_TEXTNC:
      case FUNC_DISPLAY_SCROLLTEXT:
      case FUNC_DISPLAY_SCROLLDELAY:
      case FUNC_DISPLAY_CLOCK:
          result = IOTTIMERMainFunc(function);
        break;
      case FUNC_DISPLAY_DIM:
        IOTTIMERDim();
	break;
      case FUNC_DISPLAY_POWER:
        IOTTIMERDisplayOnOff();
        break;  

    }
  }
  return result;
}

#endif  // USE_DISPLAY_IOTTIMER
#endif  // USE_DISPLAY
