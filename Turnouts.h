/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef Turnouts_h
#define Turnouts_h

#include <Arduino.h>
#include "DCC.h"
#include "LCN.h"

const byte STATUS_ACTIVE=0x80; // Flag as activated
const byte STATUS_PWM=0x40; // Flag as a PWM turnout
const byte STATUS_PWMPIN=0x3F; // PWM  pin 0-63
const int  LCN_TURNOUT_ADDRESS=-1;  // spoof dcc address -1 indicates a LCN turnout
struct TurnoutData {
  int id;
  uint8_t tStatus; // has STATUS_ACTIVE, STATUS_PWM, STATUS_PWMPIN  
  union {
    struct {
      uint8_t subAddress;  // DCC  sub addrerss
      int address;         // DCC address
    };
    struct {
      uint16_t activePosition:12; // PWM Servo position
      uint16_t inactivePosition:12; // PWM servo position
    };
  };
};

class Turnout {
  public:
  static Turnout *firstTurnout;
  static int turnoutlistHash;
  TurnoutData data;
  Turnout *nextTurnout;
  static  bool activate(int n, bool state);
  static Turnout* get(int);
  static bool remove(int);
  static bool isActive(int);
  static void load();
  static void store();
  static Turnout *create(int id , int address , int subAddress);
  static Turnout *create(int id , byte pin , int activePosition, int inactivePosition);
  static Turnout *create(int id);
  void activate(bool state);
  static void printAll(Print *);
#ifdef EESTOREDEBUG
  void print(Turnout *tt);
#endif
private:
  int num;  // EEPROM address of tStatus in TurnoutData struct, or zero if not stored.
}; // Turnout
  
#endif
