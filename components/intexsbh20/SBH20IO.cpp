/*
 * project:  Intex PureSpa SB-H20 WiFi Controller
 *
 * file:     SBH20IO.cpp
 *
 * encoding: UTF-8
 * created:  14th March 2021
 *
 * Copyright (C) 2021 Jens B.
 *
 *
 * Receive data handling based on code from:
 *
 * DIYSCIP <https://github.com/yorffoeg/diyscip> (c) by Geoffroy HUBERT - yorffoeg@gmail.com
 *
 * DIYSCIP is licensed under a
 * Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * You should have received a copy of the license along with this
 * work. If not, see <https://creativecommons.org/licenses/by-nc-sa/4.0/>.
 *
 * DIYSCIP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY.
 *
 * SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *
 */

#include "SBH20IO.h"
#include <Arduino.h>

// bit mask for LEDs
namespace FRAME_LED
{
  const uint16_t POWER = 0x0001;
  const uint16_t HEATER_ON = 0x0080; // max. 72 h, will start filter, will not stop filter
  const uint16_t NO_BEEP = 0x0100;
  const uint16_t HEATER_STANDBY = 0x0200;
  const uint16_t BUBBLE = 0x0400; // max. 30 min
  const uint16_t FILTER = 0x1000; // max. 24 h
}

namespace FRAME_DIGIT
{
  // bit mask of 7-segment display selector
  const uint16_t POS_1 = 0x0040;
  const uint16_t POS_2 = 0x0020;
  const uint16_t POS_3 = 0x0800;
  const uint16_t POS_4 = 0x0004;
  const uint16_t POS_ALL = POS_1 | POS_2 | POS_3 | POS_4;

  // bit mask of 7-segment display element
  const uint16_t SEGMENT_A = 0x2000;
  const uint16_t SEGMENT_B = 0x1000;
  const uint16_t SEGMENT_C = 0x0200;
  const uint16_t SEGMENT_D = 0x0400;
  const uint16_t SEGMENT_E = 0x0080;
  const uint16_t SEGMENT_F = 0x0008;
  const uint16_t SEGMENT_G = 0x0010;
  const uint16_t SEGMENT_DP = 0x8000;
  const uint16_t SEGMENTS = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G;

  // bit mask of human readable value on 7-segment display
  const uint16_t OFF = 0x0000;
  const uint16_t NUM_0 = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F;
  const uint16_t NUM_1 = SEGMENT_B | SEGMENT_C;
  const uint16_t NUM_2 = SEGMENT_A | SEGMENT_B | SEGMENT_G | SEGMENT_E | SEGMENT_D;
  const uint16_t NUM_3 = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G;
  const uint16_t NUM_4 = SEGMENT_F | SEGMENT_G | SEGMENT_B | SEGMENT_C;
  const uint16_t NUM_5 = SEGMENT_A | SEGMENT_F | SEGMENT_G | SEGMENT_C | SEGMENT_D;
  const uint16_t NUM_6 = SEGMENT_A | SEGMENT_F | SEGMENT_E | SEGMENT_D | SEGMENT_C | SEGMENT_G;
  const uint16_t NUM_7 = SEGMENT_A | SEGMENT_B | SEGMENT_C;
  const uint16_t NUM_8 = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G;
  const uint16_t NUM_9 = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G;
  const uint16_t LET_A = SEGMENT_E | SEGMENT_F | SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_G;
  const uint16_t LET_C = SEGMENT_A | SEGMENT_F | SEGMENT_E | SEGMENT_D;
  const uint16_t LET_D = SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G;
  const uint16_t LET_E = SEGMENT_A | SEGMENT_F | SEGMENT_E | SEGMENT_D | SEGMENT_G;
  const uint16_t LET_F = SEGMENT_E | SEGMENT_F | SEGMENT_A | SEGMENT_G;
  const uint16_t LET_H = SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_G;
  const uint16_t LET_N = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F;
}

// bit mask of button
namespace FRAME_BUTTON
{
  const uint16_t POWER = 0x0400;
  const uint16_t FILTER = 0x0002;
  const uint16_t HEATER = 0x8000;
  const uint16_t BUBBLE = 0x0008;
  const uint16_t TEMP_UP = 0x1000;
  const uint16_t TEMP_DOWN = 0x0080;
  const uint16_t TEMP_UNIT = 0x2000;
  const uint16_t ALL = POWER | FILTER | HEATER | BUBBLE | TEMP_UP | TEMP_DOWN | TEMP_UNIT;
}

// frame type markers
namespace FRAME_TYPE
{
  const uint16_t CUE = 0x0100;
  const uint16_t LED = 0x4000;
  const uint16_t DIGIT = FRAME_DIGIT::POS_ALL;
  const uint16_t BUTTON = CUE | FRAME_BUTTON::ALL;
}

namespace DIGIT
{
  // 7-segment display update control
  const uint8_t POS_1 = 0x8;
  const uint8_t POS_2 = 0x4;
  const uint8_t POS_3 = 0x2;
  const uint8_t POS_4 = 0x1;
  const uint8_t POS_1_2 = POS_1 | POS_2;
  const uint8_t POS_1_2_3 = POS_1 | POS_2 | POS_3;
  const uint8_t POS_ALL = POS_1 | POS_2 | POS_3 | POS_4;

  // nibble value used to map a subset of non-numeric states of the 7-segment display
  const uint8_t LET_C = 0xC;
  const uint8_t LET_D = 0xD;
  const uint8_t LET_E = 0xE;
  const uint8_t LET_F = 0xF;

  const uint8_t LET_N = 0xA;
  const uint8_t OFF = 0xB;
};

namespace ERROR
{
  // internal binary value of error display (3 letters)
  const uint16_t NONE = 0;
  const uint16_t NO_WATER_FLOW = 0xE90;
  const uint16_t WATER_TEMP_LOW = 0xE94;
  const uint16_t WATER_TEMP_HIGH = 0xE95;
  const uint16_t SYSTEM = 0xE96;
  const uint16_t DRY_FIRE_PROTECT = 0xE97;
  const uint16_t TEMP_SENSOR = 0xE99;
  const uint16_t HEATING_ABORTED = 0xEAD; // A => DIGIT::LET_N ie: "END" => 0xEAD

  const uint16_t VALUES[] = {NO_WATER_FLOW, WATER_TEMP_LOW, WATER_TEMP_HIGH, WATER_TEMP_HIGH, SYSTEM, DRY_FIRE_PROTECT, TEMP_SENSOR, HEATING_ABORTED};
  const unsigned int COUNT = sizeof(VALUES) / sizeof(uint16_t);

  // human readable error on display
  const char CODE_90[] PROGMEM = "E90";
  const char CODE_94[] PROGMEM = "E94";
  const char CODE_95[] PROGMEM = "E95";
  const char CODE_96[] PROGMEM = "E96";
  const char CODE_97[] PROGMEM = "E97";
  const char CODE_99[] PROGMEM = "E99";
  const char CODE_END[] PROGMEM = "END";
  const char CODE_OTHER[] PROGMEM = "EXX";

  // English error messages
  const char EN_90[] PROGMEM = "no water flow";
  const char EN_94[] PROGMEM = "water temp too low";
  const char EN_95[] PROGMEM = "water temp too high";
  const char EN_96[] PROGMEM = "system error";
  const char EN_97[] PROGMEM = "dry fire protection";
  const char EN_99[] PROGMEM = "water temp sensor error";
  const char EN_END[] PROGMEM = "heating aborted after 72h";
  const char EN_OTHER[] PROGMEM = "error";

  // German error messages
  const char DE_90[] PROGMEM = "kein Wasserdurchfluss";
  const char DE_94[] PROGMEM = "Wassertemperatur zu niedrig";
  const char DE_95[] PROGMEM = "Wassertemperatur zu hoch";
  const char DE_96[] PROGMEM = "Systemfehler";
  const char DE_97[] PROGMEM = "Trocken-Brandschutz";
  const char DE_99[] PROGMEM = "Wassertemperatursensor defekt";
  const char DE_END[] PROGMEM = "Heizbetrieb nach 72 h deaktiviert";
  const char DE_OTHER[] PROGMEM = "Störung";

  const char *const TEXT[3][COUNT + 1] PROGMEM = {
      {CODE_90, CODE_94, CODE_95, CODE_96, CODE_97, CODE_99, CODE_END, CODE_OTHER},
      {EN_90, EN_94, EN_95, EN_96, EN_97, EN_99, EN_END, EN_OTHER},
      {DE_90, DE_94, DE_95, DE_96, DE_97, DE_99, DE_END, DE_OTHER}};
}

// special display values
inline uint16_t display2Num(uint16_t v) { return (((v >> 12) & 0x000F) * 100) + (((v >> 8) & 0x000F) * 10) + ((v >> 4) & 0x000F); }
inline uint16_t display2Error(uint16_t v) { return (v >> 4) & 0x0FFF; }
inline bool displayIsTemp(uint16_t v) { return (v & 0x000F) == DIGIT::LET_C || (v & 0x000F) == DIGIT::LET_F; }
inline bool displayIsError(uint16_t v) { return (v & 0xF000) == 0xE000; }
inline bool displayIsBlank(uint16_t v) { return (v & 0xFFF0) == ((DIGIT::OFF << 12) + (DIGIT::OFF << 8) + (DIGIT::OFF << 4)); }

volatile SBH20IO::State SBH20IO::state;
volatile SBH20IO::Buttons SBH20IO::buttons;

// @TODO detect when latch signal stays low
// @TODO detect act temp change during error
// @TODO improve reliability of water temp change (counter auto repeat and too short press)
void SBH20IO::setup(LANG language)
{
  this->language = language;

  pinMode(PIN::CLOCK, INPUT);
  pinMode(PIN::DATA, INPUT);
  pinMode(PIN::LATCH, INPUT);

  attachInterruptArg(digitalPinToInterrupt(PIN::LATCH), SBH20IO::latchFallingISR, this, FALLING);
  attachInterruptArg(digitalPinToInterrupt(PIN::CLOCK), SBH20IO::clockRisingISR, this, RISING);
}

void SBH20IO::loop()
{
  // device online check
  unsigned long now = millis();
  if (state.stateUpdated)
  {
    lastStateUpdateTime = now;
    state.online = true;
    state.stateUpdated = false;
  }
  else if (timeDiff(now, lastStateUpdateTime) > CYCLE::RECEIVE_TIMEOUT)
  {
    state.online = false;
  }
}

bool SBH20IO::isOnline() const
{
  return state.online;
}

unsigned int SBH20IO::getTotalFrames() const
{
  return state.frameCounter;
}

unsigned int SBH20IO::getDroppedFrames() const
{
  return state.frameDropped;
}

int SBH20IO::getActWaterTempCelsius() const
{
  return (state.waterTemp != UNDEF::USHORT) ? convertDisplayToCelsius(state.waterTemp) : UNDEF::USHORT;
}

int SBH20IO::getDesiredWaterTempCelsius() const
{
  return (state.desiredTemp != UNDEF::USHORT) ? convertDisplayToCelsius(state.desiredTemp) : UNDEF::USHORT;
}

void SBH20IO::forceGetDesiredWaterTempCelsius()
{
  changeWaterTemp(-1);
}

unsigned int SBH20IO::getErrorValue() const
{
  return state.error;
}

String SBH20IO::getErrorMessage(unsigned int errorValue) const
{
  if (errorValue)
  {
    // get error text index of error value
    unsigned int i;
    for (i = 0; i < ERROR::COUNT; i++)
    {
      if (ERROR::VALUES[i] == errorValue)
      {
        break;
      }
    }

    // load error text from PROGMEM
    return FPSTR(ERROR::TEXT[(unsigned int)language][i]);
  }
  else
  {
    // no error
    return "";
  }
}

unsigned int SBH20IO::getRawLedValue() const
{
  return (state.ledStatus != UNDEF::USHORT) ? state.ledStatus : UNDEF::USHORT;
}

uint8_t SBH20IO::isPowerOn() const
{
  return (state.ledStatus != UNDEF::USHORT) ? ((state.ledStatus & FRAME_LED::POWER) != 0) : UNDEF::BOOL;
}

uint8_t SBH20IO::isFilterOn() const
{
  return (state.ledStatus != UNDEF::USHORT) ? ((state.ledStatus & FRAME_LED::FILTER) != 0) : UNDEF::BOOL;
}

uint8_t SBH20IO::isBubbleOn() const
{
  return (state.ledStatus != UNDEF::USHORT) ? ((state.ledStatus & FRAME_LED::BUBBLE) != 0) : UNDEF::BOOL;
}

uint8_t SBH20IO::isHeaterOn() const
{
  return (state.ledStatus != UNDEF::USHORT) ? ((state.ledStatus & (FRAME_LED::HEATER_ON | FRAME_LED::HEATER_STANDBY)) != 0) : UNDEF::BOOL;
}

uint8_t SBH20IO::isHeaterStandby() const
{
  return (state.ledStatus != UNDEF::USHORT) ? ((state.ledStatus & FRAME_LED::HEATER_STANDBY) != 0) : UNDEF::BOOL;
}

uint8_t SBH20IO::isBuzzerOn() const
{
  return (state.ledStatus != UNDEF::USHORT) ? ((state.ledStatus & FRAME_LED::NO_BEEP) == 0) : UNDEF::BOOL;
}

/**
 * set desired water temperature by performing button up or down actions
 * repeatedly depending on temperature delta
 *
 * notes:
 * - method will block until setting is completed
 * - WiFi is temporarily put to sleep to improve receive decoding reliability
 * - actual setpoint is not checked for verification because this
 *   would slow down the setpoint modification significantly
 *
 * @param temp water temperature setpoint [°C]
 */
void SBH20IO::setDesiredWaterTempCelsius(int temp)
{
  if (temp >= WATER_TEMP::SET_MIN && temp <= WATER_TEMP::SET_MAX)
  {
    if (isPowerOn() == true && state.error == ERROR::NONE)
    {
      // try to get initial temp
      int setTemp = getDesiredWaterTempCelsius();
      bool modifying = false;
      if (setTemp == UNDEF::USHORT)
      {
        // trigger temp modification
        changeWaterTemp(-1);
        modifying = true;

        // wait for temp readback (will take 2-3 blink durations)
        int sleep = 20; // ms
        int tries = 4 * BLINK::PERIOD / sleep;
        do
        {
          delay(sleep);
          setTemp = getDesiredWaterTempCelsius();
          tries--;
        } while (setTemp == UNDEF::USHORT && tries);

        // check success
        if (setTemp == UNDEF::USHORT)
        {
          // error, abort
          DEBUG_MSG("\naborted\n");
          delay(1);
          return;
        }
      }

      // modify desired temp
      int deltaTemp = temp - setTemp;
      // DEBUG_MSG("\nBdelta %d", deltaTemp);
      while (deltaTemp)
      {
        if (deltaTemp > 0)
        {
          // DEBUG_MSG("\nBU");
          changeWaterTemp(1);
          if (modifying)
          {
            deltaTemp--;
            setTemp++;
          }
        }
        else
        {
          // DEBUG_MSG("\nBD");
          changeWaterTemp(-1);
          if (modifying)
          {
            deltaTemp++;
            setTemp--;
          }
        }
        modifying = true;
      }
      delay(1);
    }
  }
}

/**
 * press specific button and wait for confirmation (blocking)
 *
 * notes:
 * - WiFi is temporarily put to sleep to improve receive decoding reliability
 *
 * @param buttonPressCount
 * @return true if beep was received, false if no beep was received until timeout
 */
bool SBH20IO::pressButton(volatile unsigned int &buttonPressCount)
{
  waitBuzzerOff();
  unsigned int tries = BUTTON::ACK_TIMEOUT / BUTTON::ACK_CHECK_PERIOD;
  buttonPressCount = BUTTON::PRESS_COUNT;
  while (buttonPressCount && tries)
  {
    delay(BUTTON::ACK_CHECK_PERIOD);
    tries--;
  }
  // delay(1);

  return tries;
}

void SBH20IO::setBubbleOn(bool on)
{
  if (on ^ (isBubbleOn() == true))
  {
    pressButton(buttons.toggleBubble);
  }
}

void SBH20IO::setFilterOn(bool on)
{
  if (on ^ (isFilterOn() == true))
  {
    pressButton(buttons.toggleFilter);
  }
}

void SBH20IO::setHeaterOn(bool on)
{
  if (on ^ (isHeaterOn() == true || isHeaterStandby() == true))
  {
    pressButton(buttons.toggleHeater);
  }
}

void SBH20IO::setPowerOn(bool on)
{
  bool active = isPowerOn() == true;
  if (on ^ active)
  {
    pressButton(buttons.togglePower);
  }
}

/**
 * wait for buzzer to go off or timeout
 * and delay for a cycle period
 *
 * @return true if buzzer is off, false if buzzer is still on after timeout
 */
bool SBH20IO::waitBuzzerOff() const
{
  int tries = BUTTON::ACK_TIMEOUT / BUTTON::ACK_CHECK_PERIOD;
  while (state.buzzer && tries)
  {
    delay(BUTTON::ACK_CHECK_PERIOD);
    tries--;
  }

  // extra delay reduces chance to trigger auto repeat
  if (tries)
  {
    delay(2 * CYCLE::PERIOD);
    return true;
  }
  else
  {
    DEBUG_MSG("\nwBO fail");
    return false;
  }
}

/**
 * change water temperature setpoint by 1 degree and wait for confirmation (blocking)
 *
 * @param up press up (> 0) or down (< 0) button
 * @return true if beep was received, false if no beep was received until timeout
 */
bool SBH20IO::changeWaterTemp(int up)
{
  if (isPowerOn() == true && state.error == ERROR::NONE)
  {
    // perform button action
    waitBuzzerOff();
    // DEBUG_MSG("\nP ");
    int tries = BUTTON::ACK_TIMEOUT / BUTTON::ACK_CHECK_PERIOD;
    if (up > 0)
    {
      buttons.toggleTempUp = BUTTON::PRESS_COUNT;
      while (buttons.toggleTempUp && tries)
      {
        delay(BUTTON::ACK_CHECK_PERIOD);
        tries--;
      }
    }
    else if (up < 0)
    {
      buttons.toggleTempDown = BUTTON::PRESS_COUNT;
      while (buttons.toggleTempDown && tries)
      {
        delay(BUTTON::ACK_CHECK_PERIOD);
        tries--;
      }
    }

    if (tries && state.buzzer)
    {
      return true;
    }
    else
    {
      DEBUG_MSG("\ncWT fail");
      return false;
    }
  }
  return false;
}

uint16_t SBH20IO::convertDisplayToCelsius(uint16_t value) const
{
  uint16_t celsiusValue = display2Num(value);
  uint16_t tempUint = value & 0x000F;
  if (tempUint == DIGIT::LET_F)
  {
    // convert °F to °C
    float fValue = (float)celsiusValue;
    celsiusValue = (uint16_t)round(((fValue - 32) * 5) / 9);
  }
  else if (tempUint != DIGIT::LET_C)
  {
    celsiusValue = UNDEF::USHORT;
  }

  return (celsiusValue >= 0) && (celsiusValue <= 60) ? celsiusValue : UNDEF::USHORT;
}

IRAM_ATTR void SBH20IO::latchFallingISR(void *arg)
{
  pinMode(PIN::DATA, INPUT);
}

IRAM_ATTR void SBH20IO::clockRisingISR(void *arg)
{
  static uint16_t frame=0x0000;
  static uint16_t receivedBits=0x0000;
  bool data = !digitalRead(PIN::DATA);
  bool enable = digitalRead(PIN::LATCH) == LOW;

  if (enable || receivedBits == (FRAME::BITS - 1))
  {
    frame = (frame << 1) + data;
    receivedBits++;

    if (receivedBits == FRAME::BITS)
    {
      state.frameCounter++;

      if (frame == FRAME_TYPE::CUE)
      {
        // cue frame, ignore
        // DEBUG_MSG("\nC");
      }
      else if (frame & FRAME_TYPE::DIGIT)
      {
        // display frame
        // DEBUG_MSG("\nD");
        decodeDisplay(frame);
      }
      else if (frame & FRAME_TYPE::LED)
      {
        // LED frame
        // DEBUG_MSG("\nL");
        decodeLED(frame);
      }
      else if (frame & FRAME_TYPE::BUTTON)
      {
        // button frame
        // DEBUG_MSG("\nB");
        decodeButton(frame);
      }
      else if (frame != 0)
      {
        // unsupported frame
        // DEBUG_MSG("\nU");
      }

      receivedBits = 0;
    }
  }
  else
  {
    // DEBUG_MSG(" %d ", receivedBits);
    frame = 0;
    receivedBits = 0;
  }
}

IRAM_ATTR inline uint8_t SBH20IO::BCD(uint16_t value)
{
  uint8_t digit;
  switch (value & FRAME_DIGIT::SEGMENTS)
  {
  case FRAME_DIGIT::OFF:
    digit = DIGIT::OFF;
    break;
  case FRAME_DIGIT::NUM_0:
    digit = 0x0;
    break;
  case FRAME_DIGIT::NUM_1:
    digit = 0x1;
    break;
  case FRAME_DIGIT::NUM_2:
    digit = 0x2;
    break;
  case FRAME_DIGIT::NUM_3:
    digit = 0x3;
    break;
  case FRAME_DIGIT::NUM_4:
    digit = 0x4;
    break;
  case FRAME_DIGIT::NUM_5:
    digit = 0x5;
    break;
  case FRAME_DIGIT::NUM_6:
    digit = 0x6;
    break;
  case FRAME_DIGIT::NUM_7:
    digit = 0x7;
    break;
  case FRAME_DIGIT::NUM_8:
    digit = 0x8;
    break;
  case FRAME_DIGIT::NUM_9:
    digit = 0x9;
    break;
  case FRAME_DIGIT::LET_C:
    digit = DIGIT::LET_C; // for °C
    break;
  case FRAME_DIGIT::LET_D:
    digit = DIGIT::LET_D; // for error code "END"
    break;
  case FRAME_DIGIT::LET_E:
    digit = DIGIT::LET_E; // for error code
    break;
  case FRAME_DIGIT::LET_F:
    digit = DIGIT::LET_F; // for °F
    break;
  case FRAME_DIGIT::LET_N:
  default:
    digit = DIGIT::LET_N; // for error code "END"
    break;
  }
  return digit;
}

IRAM_ATTR inline void SBH20IO::decodeDisplay(uint16_t frame)
{
  static uint16_t value = 0;  // current display
  static uint16_t pValue = 0; // previous display
  static uint16_t stableValue = 0;  
  static uint debounce = 0;   // quick debounce

  static uint8_t largeDebounce =0; // larger than blank frames count
  static uint16_t stableTemp=0x0000; // stable temperature

  uint8_t digit = BCD(frame);

  if (frame & FRAME_DIGIT::POS_1)
  {
    value = (value & 0x0FFF) | (digit << 12);
  }
  else if (frame & FRAME_DIGIT::POS_2)
  {
    value = (value & 0xF0FF) | (digit << 8);
  }
  else if (frame & FRAME_DIGIT::POS_3)
  {
    value = (value & 0xFF0F) | (digit << 4);
  }
  else if (frame & FRAME_DIGIT::POS_4)
  {
    value = (value & 0xFFF0) | digit;
    if (value != pValue)
    {
      pValue = value;
      debounce = 3;
    }
    else
    {
      if (debounce)
        debounce--;
      else if (value != stableValue)
      {
        largeDebounce = 250;

        stableValue = value;
        if (displayIsBlank(value))
        {
          if (state.desiredTemp != stableTemp)
          {
            state.desiredTemp = stableTemp;
            state.stateUpdated = true;
          }
        }
        else if (displayIsError(value))
        {
          state.error = display2Error(value);
        }
        else
        {
          stableTemp = stableValue;
        }
      }
      if (largeDebounce)
      {
        largeDebounce--;
      }
      else
      {
        if (state.waterTemp != stableTemp)
        {
          state.waterTemp = stableTemp;
          state.stateUpdated = true;
        }
      }
    }
  }
}

IRAM_ATTR inline void SBH20IO::decodeLED(uint16_t frame)
{
  static uint16_t pFrame=0x000;
  static int count=0;

  if (frame == pFrame)
  {
    // wait for confirmation
    count--;
    if (count == 0)
    {
      if (state.ledStatus != frame)
      {
        state.ledStatus = frame;
        state.buzzer = !(state.ledStatus & FRAME_LED::NO_BEEP);
        state.stateUpdated = true;

        // clear buttons if buzzer is on
        if (state.buzzer)
        {
          buttons.toggleBubble = 0;
          buttons.toggleFilter = 0;
          buttons.toggleHeater = 0;
          buttons.togglePower = 0;
          buttons.toggleTempUp = 0;
          buttons.toggleTempDown = 0;
        }
      }
    }
  }
  else
  {
    // LED status changed
    pFrame = frame;
    count = 3;
  }
}

IRAM_ATTR inline void SBH20IO::decodeButton(uint16_t frame)
{
  bool reply =false;
  if (frame & FRAME_BUTTON::FILTER)
  {
    // DEBUG_MSG("F");
    if (buttons.toggleFilter)
    {
      reply = true;
      buttons.toggleFilter--;
    }
  }
  else if (frame & FRAME_BUTTON::HEATER)
  {
    // DEBUG_MSG("H");
    if (buttons.toggleHeater)
    {
      reply = true;
      buttons.toggleHeater--;
    }
  }
  else if (frame & FRAME_BUTTON::BUBBLE)
  {
    // DEBUG_MSG("B");
    if (buttons.toggleBubble)
    {
      reply = true;
      buttons.toggleBubble--;
    }
  }
  else if (frame & FRAME_BUTTON::POWER)
  {
    // DEBUG_MSG(" P");
    if (buttons.togglePower)
    {
      reply = true;
      buttons.togglePower--;
    }
  }
  else if (frame & FRAME_BUTTON::TEMP_UP)
  {
    // DEBUG_MSG("U");
    if (buttons.toggleTempUp)
    {
      reply = true;
      buttons.toggleTempUp--;
    }
  }
  else if (frame & FRAME_BUTTON::TEMP_DOWN)
  {
    // DEBUG_MSG("D");
    if (buttons.toggleTempDown)
    {
      reply = true;
      buttons.toggleTempDown--;
    }
  }

  if (reply)
  {
    pinMode(PIN::DATA, OUTPUT);
  }
}