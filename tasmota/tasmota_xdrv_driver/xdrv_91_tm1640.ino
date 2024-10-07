/*
  xdrv_91_tm1640.ino - TM1640 16-digit 7-segment LED controller support for Tasmota

  Copyright (C) 2024  Theo Arends, Stefan Oskamp

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

  Based on xdrv_66_tm1638.ino as example. 
  Using code snippets from Maxint R&D's TM16xx/TM1640 classes.
*/

#ifdef USE_TM1640
/*********************************************************************************************\
 * TM1640 16-digit 7-segment LED controller
 *
 * Uses GPIO TM1640 CLK and TM1640 DIN
\*********************************************************************************************/

#define XDRV_91               91

#define TM1640_CMD_DATA_AUTO 0x40
#define TM1640_CMD_DATA_FIXED 0x44
#define TM1640_CMD_DISPLAY 0x80
#define TM1640_CMD_ADDRESS 0xC0

#ifndef TM1640_MAX_LEDS
#define TM1640_MAX_LEDS       16
#endif

#define TM1640_CLOCK_DELAY    1    // uSec

struct TM1640 {
  int8_t clock_pin = 0;
  int8_t data_pin = 0;
  int8_t led_offset;
  bool detected = false;
} Tm1640;


void TM1640Start (void) {
  digitalWrite(Tm1640.data_pin, LOW);
  digitalWrite(Tm1640.clock_pin, LOW);
  delayMicroseconds(TM1640_CLOCK_DELAY);
} // End TM1640Start()

void TM1640Stop (void) {
  digitalWrite(Tm1640.clock_pin, HIGH);
  digitalWrite(Tm1640.data_pin, HIGH);
  delayMicroseconds(TM1640_CLOCK_DELAY);
} // End TM1640Stop()

void TM1640Send(uint8_t data) {
	for (uint32_t i = 0; i < 8; i++) {          // 8 bits
    digitalWrite(Tm1640.data_pin, data & 1 ? HIGH : LOW);
    delayMicroseconds(TM1640_CLOCK_DELAY);
    data >>= 1;
    digitalWrite(Tm1640.clock_pin, HIGH);
    delayMicroseconds(TM1640_CLOCK_DELAY);
    digitalWrite(Tm1640.clock_pin, LOW);
    delayMicroseconds(TM1640_CLOCK_DELAY);
  }
  digitalWrite(Tm1640.data_pin, LOW);
  delayMicroseconds(TM1640_CLOCK_DELAY);
} // End TM1640Send()

void TM1640SendData(uint8_t address, uint8_t data) {
  // First, send data commaand using FIXED addressing:
  TM1640Start();
  TM1640Send(TM1640_CMD_DATA_FIXED);
  TM1640Stop();
  // Then, send address and one data byte:
	TM1640Start();
  TM1640Send(TM1640_CMD_ADDRESS | address);
  TM1640Send(data);
  TM1640Stop();
} // End TM1640SendData()

void TM1640SendDataArray(uint8_t address, uint8_t *data, uint8_t count) {
  // First, send data commaand using AUTO addressing:
  TM1640Start();
  TM1640Send(TM1640_CMD_DATA_AUTO);
  TM1640Stop();
  // Then, send address and all data bytes:
	TM1640Start();
  TM1640Send(TM1640_CMD_ADDRESS | address);
  while (count-- > 0) {
    TM1640Send(*data++);
  }
  TM1640Stop();
} // End TM1640SendDataArray()

void TM1640SetBrightness(uint8_t level) {
  // level can be 0 to 8. 
  // 0 means off
  // 
  // Other levels are mapped to TM1640 levels 0 ... 7
  // The mapping to the PWM level is non-linear, according to the data sheet
  // level | TM1640 | PWM
  //     1 |      0 | 1/16
  //     2 |      1 | 2/16
  //     3 |      2 | 4/16
  //     4 |      3 | 10/16
  //     5 |      4 | 11/16
  //     6 |      5 | 12/16
  //     7 |      6 | 13/16
  //     8 |      7 | 14/16
  uint8_t cmd = TM1640_CMD_DISPLAY | (level > 0 ? 0x8 : 0) | ((level - 1) % 8);
  TM1640Start();
  TM1640Send (cmd);  
  TM1640Stop();
} // End TM1640SetBrightness


void TM1640Init(void) {
  if (PinUsed(GPIO_TM1640CLK) && PinUsed(GPIO_TM1640DIN)) {
    Tm1640.clock_pin = Pin(GPIO_TM1640CLK);
    Tm1640.data_pin = Pin(GPIO_TM1640DIN);

    pinMode(Tm1640.data_pin, OUTPUT);
    pinMode(Tm1640.clock_pin, OUTPUT);

    digitalWrite(Tm1640.clock_pin, HIGH);
    digitalWrite(Tm1640.data_pin, HIGH);

    Tm1640.led_offset = TasmotaGlobal.devices_present;
    UpdateDevicesPresent(TM1640_MAX_LEDS);
    Tm1640.detected = true;
  }
}

void TM1640Power(void) {
  /* From TM1638:
  power_t rpower = XdrvMailbox.index >> Tm1640.led_offset;
  for (uint32_t i = 0; i < TM1640_MAX_LEDS; i++) {
    uint32_t state = rpower & 1;
    uint8_t color = (state) ? TM1640_COLOR_RED : TM1640_COLOR_NONE;
    Tm1640SetLED(color, i);
    rpower >>= 1;                             // Select next power
  }
  */

  uint8_t leds[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };

  TM1640SetBrightness(1);
  TM1640SendDataArray (0, leds, sizeof(leds));
}

void TM1640Loop (void) {
  uint8_t leds[TM1640_MAX_LEDS];
  time_t t = time(0);
  for (uint32_t i = 0; i < TM1640_MAX_LEDS; i++) {
    leds[i] = (uint8_t) ((t + i) % 256);
  }
  TM1640SetBrightness(t % 8 + 1);
  TM1640SendDataArray(0, leds, sizeof(leds));
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv91(uint32_t function) {
  bool result = false;

  if (FUNC_SETUP_RING2 == function) {
    TM1640Init();
  } else if (Tm1640.detected) {
    switch (function) {
      case FUNC_EVERY_50_MSECOND:
        TM1640Loop();
        break;
      case FUNC_SET_POWER:
        TM1640Power();
        break;
      case FUNC_ACTIVE:
        result = true;
        break;
    }
  }
  return result;
}

#endif  // USE_TM1640