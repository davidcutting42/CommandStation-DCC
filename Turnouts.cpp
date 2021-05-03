/*
 *  © 2013-2016 Gregg E. Berman
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
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

#include "Turnouts.h"
#include "EEStore.h"
#include "StringFormatter.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

// Keywords used for turnout configuration.
const int16_t HASH_KEYWORD_SERVO=27709;
const int16_t HASH_KEYWORD_DCC=6436;
const int16_t HASH_KEYWORD_VPIN=-415;

///////////////////////////////////////////////////////////////////////////////
// Static function to print all Turnout states to stream in form "<H id state>"

void Turnout::printAll(Print *stream){
  for (Turnout *tt = Turnout::firstTurnout; tt != NULL; tt = tt->nextTurnout)
    StringFormatter::send(stream, F("<H %d %d>\n"), tt->data.id, (tt->data.tStatus & STATUS_ACTIVE)!=0);
} // Turnout::printAll

///////////////////////////////////////////////////////////////////////////////
// Object method to print configuration of one Turnout to stream, in one of the following forms:
//  <H id SERVO vpin activePos inactivePos profile state>
//  <H id LCN state>
//  <H id VPIN vpin state>
//  <H id DCC address subAddress active state>

void Turnout::print(Print *stream){
  int state = ((data.tStatus & STATUS_ACTIVE) != 0);
  if (data.tStatus & STATUS_PWM) {
    // Servo Turnout
    int inactivePosition = (data.positionWord) & 0x1ff;
    int activePosition = ((data.positionWord & 0x200) >> 1) | data.positionByte;
    int profile = (data.positionWord >> 10) & 0x7;
    int pin = (data.tStatus & STATUS_PWMPIN);
    int vpin = pin+IODevice::firstServoVpin;
    StringFormatter::send(stream, F("<H %d SERVO %d %d %d %d %d>\n"), data.id, vpin, 
      activePosition, inactivePosition, profile, state);
  } else if (data.address == LCN_TURNOUT_ADDRESS) {
    // LCN Turnout
    StringFormatter::send(stream, F("<H %d LCN %d>\n"), data.id, state);
  } else if (data.subAddress == VPIN_TURNOUT_SUBADDRESS) {
    // VPIN Digital output
    StringFormatter::send(stream, F("<H %d VPIN %d %d>\n"), data.id, data.address, state);
  } else {
    // DCC Turnout
    StringFormatter::send(stream, F("<H %d DCC %d %d %d>\n"), data.id, data.address, 
        (int)data.subAddress, state);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to activate/deactivate Turnout with ID 'n'.
//   Returns false if turnout not found.

bool Turnout::activate(int n,bool state){
#ifdef EESTOREDEBUG
  DIAG(F("Turnout::activate(%d,%d)"),n,state);
#endif
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  tt->activate(state);
  turnoutlistHash++;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Static function to check if the Turnout with ID 'n' is activated or not.
// Returns false if turnout not found.

bool Turnout::isActive(int n){
  Turnout * tt=get(n);
  if (tt==NULL) return false;
  return tt->data.tStatus & STATUS_ACTIVE;
}

///////////////////////////////////////////////////////////////////////////////
// Object method to activate or deactivate the Turnout.  

// activate is virtual here so that it can be overridden by a non-DCC turnout mechanism
void Turnout::activate(bool state) {
#ifdef EESTOREDEBUG
  DIAG(F("Turnout::activate(%d)"),state);
#endif
  if (data.address==LCN_TURNOUT_ADDRESS) {
     // A LCN turnout is transmitted to the LCN master.
     LCN::send('T',data.id,state);
     return;   // The tStatus will be updated by a message from the LCN master, later.    
  }
  if (state)
    data.tStatus|=STATUS_ACTIVE;
  else
    data.tStatus &= ~STATUS_ACTIVE;

  if (data.tStatus & STATUS_PWM) {
    int pin = (data.tStatus & STATUS_PWMPIN);
    IODevice::write(pin+IODevice::firstServoVpin, state);  // Servo turnout
  } else if (data.subAddress == VPIN_TURNOUT_SUBADDRESS) {
    int pin = data.address;
    IODevice::write(pin, state);  // VPIN turnout
  } else
    DCC::setAccessory(data.address, data.subAddress, state);
  // Save state if stored in EEPROM
  if (EEStore::eeStore->data.nTurnouts > 0 && num > 0) 
    EEPROM.put(num, data.tStatus);
}

///////////////////////////////////////////////////////////////////////////////
// Static function to find Turnout object specified by ID 'n'.  Return NULL if not found.

Turnout* Turnout::get(int n){
  Turnout *tt;
  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;tt=tt->nextTurnout);
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function to delete Turnout object specified by ID 'n'.  Return false if not found.

bool Turnout::remove(int n){
  Turnout *tt,*pp=NULL;

  for(tt=firstTurnout;tt!=NULL && tt->data.id!=n;pp=tt,tt=tt->nextTurnout);

  if(tt==NULL) return false;
  
  if(tt==firstTurnout)
    firstTurnout=tt->nextTurnout;
  else
    pp->nextTurnout=tt->nextTurnout;

  free(tt);
  turnoutlistHash++;
  return true; 
}

///////////////////////////////////////////////////////////////////////////////
// Static function to load all Turnout definitions from EEPROM
// TODO: Consider transmitting the initial state of the DCC/LCN turnout here.
//  (already done for servo turnouts and VPIN turnouts).

void Turnout::load(){
  struct TurnoutData data;
  Turnout *tt;

  for(int i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    EEPROM.get(EEStore::pointer(),data);
    int initialState = (data.tStatus & STATUS_ACTIVE) ? 1 : 0;
    if (data.tStatus & STATUS_PWM) {
      // Unpack PWM values
      int inactivePosition = (data.positionWord) & 0x1ff;
      int activePosition = ((data.positionWord & 0x200) >> 1) | data.positionByte;
      int profile = (data.positionWord >> 10) & 0x7;
      int pin = (data.tStatus & STATUS_PWMPIN);
      int vpin = pin+IODevice::firstServoVpin;
      tt=createServo(data.id,vpin,activePosition, inactivePosition, profile, initialState);
    } else if (data.subAddress==VPIN_TURNOUT_SUBADDRESS) {
      int vpin = data.address;
      tt=createVpin(data.id, vpin, initialState);  // VPIN-based turnout
    } else
      tt=createDCC(data.id,data.address,data.subAddress); // DCC/LCN-based turnout
    tt->data.tStatus=data.tStatus;
    tt->num=EEStore::pointer()+offsetof(TurnoutData,tStatus); // Save pointer to status byte within EEPROM
    EEStore::advance(sizeof(tt->data));
#ifdef EESTOREDEBUG
    print(tt);
#endif
  }
}

///////////////////////////////////////////////////////////////////////////////
// Static function to store all Turnout definitions to EEPROM

void Turnout::store(){
  Turnout *tt;

  tt=firstTurnout;
  EEStore::eeStore->data.nTurnouts=0;

  while(tt!=NULL){
#ifdef EESTOREDEBUG
    print(tt);
#endif
    tt->num=EEStore::pointer()+offsetof(TurnoutData,tStatus); // Save pointer to tstatus byte within EEPROM
    EEPROM.put(EEStore::pointer(),tt->data);
    EEStore::advance(sizeof(tt->data));
    tt=tt->nextTurnout;
    EEStore::eeStore->data.nTurnouts++;
  }

}
///////////////////////////////////////////////////////////////////////////////
// Static function for associating a Turnout id with a virtual pin in IODevice space.
// The actual creation and configuration of the pin must be done elsewhere,
// e.g. in mySetup.h during startup of the CS.

Turnout *Turnout::createVpin(int id, VPIN vpin, int state){
  if (vpin > VPIN_MAX) return NULL;
  Turnout *tt=create(id);
  tt->data.subAddress = VPIN_TURNOUT_SUBADDRESS;
  tt->data.tStatus=0;
  tt->data.address = vpin;
  if (state) tt->data.tStatus |= STATUS_ACTIVE;
  IODevice::write(vpin, state);   // Set initial state of output.
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function for creating a DCC/LCN-controlled Turnout.

Turnout *Turnout::createDCC(int id, int add, int subAdd){
  Turnout *tt=create(id);
  tt->data.address=add;
  tt->data.subAddress=subAdd;
  tt->data.tStatus=0;
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Method for creating a PCA9685 PWM Turnout.  Vpins are numbered from IODevice::firstServoVPIN
// The pin used internally by the turnout is the number within this range.  So if firstServoVpin is 100,
// then VPIN 100 is pin 0 on the first PCA9685, VPIN 101 is pin 1 etc. up to VPIN 163 is pin 15 on the
// fourth PCA9685 (if present and configured).  Servos generally operate 
// over the range of 200-400 so the activePosition and inactivePosition are limited to 0-511 in range.
// Ideally, the VPIN wouldn't be limited and probably the position wouldn't be so limited, but the 
// problem is that there is limited space within the structure.

Turnout *Turnout::createServo(int id, VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile, uint8_t initialState){
  int pin = vpin - IODevice::firstServoVpin;
  if (pin < 0 || pin >=64) return NULL; // Check valid range of servo pins
  if (activePosition > 511 || inactivePosition > 511 || profile > 4) return NULL;
  // Configure PWM interface device
  int deviceParams[] = {(int)activePosition, (int)inactivePosition, profile, initialState};
  if (!IODevice::configure(vpin, sizeof(deviceParams)/sizeof(deviceParams[0]), deviceParams)) return NULL;

  Turnout *tt=create(id);
  tt->data.tStatus= STATUS_PWM | (pin &  STATUS_PWMPIN);
  // Pack active/inactive positions into available space.
  tt->data.positionWord = (profile << 10) | ((activePosition & 0x100) << 1) | inactivePosition; 
  tt->data.positionByte = activePosition & 0xff;  // low 8 bits of activeAngle.
  return(tt);
}


///////////////////////////////////////////////////////////////////////////////
// Support for <T id SERVO pin activepos inactive pos profile>
// and <T id DCC address subaddress>
// and <T id VPIN pin>

Turnout *Turnout::create(int id, int params, int16_t p[]) {
  if (params == 5 && p[0] == HASH_KEYWORD_SERVO) { // <T id SERVO n n n n>
    return createServo(id, (VPIN)p[1], (uint16_t)p[2], (uint16_t)p[3], (uint8_t)p[4]);
  } else if (params == 3 && p[0] == HASH_KEYWORD_DCC) { // <T id DCC n n>
    return createDCC(id, p[1], p[2]);
  } else if (params == 2 && p[0] == HASH_KEYWORD_VPIN) { // <T id VPIN n>
    return createVpin(id, p[1]);
  } else if (params == 2) { // <T id n n> for DCC or LCN
    return createDCC(id, p[0], p[1]);
  } else if (params == 3) { // legacy <T id n n n> for Servo
    return createServo(id, (VPIN)p[0], (uint16_t)p[1], (uint16_t)p[2]);
  }
  return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// Create basic Turnout object.  The details of what sort of object it is 
//  controlling are not set here.

Turnout *Turnout::create(int id){
  Turnout *tt=get(id);
  if (tt==NULL) { 
     tt=(Turnout *)calloc(1,sizeof(Turnout));
     tt->nextTurnout=firstTurnout;
     firstTurnout=tt;
     tt->data.id=id;
  }
  tt->num = 0; // Make sure turnout doesn't get written to EEPROM until store() command.
  turnoutlistHash++;
  return tt;
}

///////////////////////////////////////////////////////////////////////////////
//
// Object method to print debug info about the state of a Turnout object
//
#ifdef EESTOREDEBUG
void Turnout::print(Turnout *tt) {
  tt->print(StringFormatter::diagSerial);
  if (tt->data.tStatus & STATUS_PWM) {
    int inactivePosition = (tt->data.positionWord) & 0x1ff;
    int activePosition = ((tt->data.positionWord & 0x200) >> 1) | tt->data.positionByte;
    int profile = (tt->data.positionByte >> 10) & 0x7;
    int pin = (tt->data.tStatus & STATUS_PWMPIN);
    int vpin = pin+IODevice::firstServoVpin;
    DIAG(F("<H %d SERVO %d %d %d %d %d>\n"), tt->data.id, vpin, 
        activePosition, inactivePosition, profile, (tt->data.tStatus & STATUS_ACTIVE)!=0);
  } else
    DIAG(F("<H %d DCC %d %d %d>\n"), tt->data.id, tt->data.address, 
        tt->data.subAddress, (tt->data.tStatus & STATUS_ACTIVE)!=0);
}
#endif

///////////////////////////////////////////////////////////////////////////////
Turnout *Turnout::firstTurnout=NULL;
int Turnout::turnoutlistHash=0; //bump on every change so clients know when to refresh their lists
