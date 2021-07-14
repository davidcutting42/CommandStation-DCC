/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth
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
#include "DIAG.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "EEStore.h"
#include "GITHUB_SHA.h"
#include "version.h"
#include "FSH.h"

// This module is responsible for converting API calls into
// messages to be sent to the waveform generator.
// It has no visibility of the hardware, timers, interrupts
// nor of the waveform issues such as preambles, start bits checksums or cutouts.
//
// Nor should it have to deal with JMRI responsess other than the OK/FAIL
// or cv value returned. I will move that back to the JMRI interface later
//
// The interface to the waveform generator is narrowed down to merely:
//   Scheduling a message on the prog or main track using a function
//   Obtaining ACKs from the prog track using a function
//   There are no volatiles here.

const byte FN_GROUP_1=0x01;         
const byte FN_GROUP_2=0x02;         
const byte FN_GROUP_3=0x04;         
const byte FN_GROUP_4=0x08;         
const byte FN_GROUP_5=0x10;         

FSH* DCC::shieldName=NULL;
byte DCC::joinRelay=UNUSED_PIN;
byte DCC::globalSpeedsteps=128;

void DCC::begin(const FSH * motorShieldName, MotorDriver * mainDriver, MotorDriver* progDriver) {
  shieldName=(FSH *)motorShieldName;
  StringFormatter::send(Serial,F("<iDCC-EX V-%S / %S / %S G-%S>\n"), F(VERSION), F(ARDUINO_TYPE), shieldName, F(GITHUB_SHA));

  // Load stuff from EEprom
  (void)EEPROM; // tell compiler not to warn this is unused
  EEStore::init();

  DCCWaveform::begin(mainDriver,progDriver); 
}

void DCC::setJoinRelayPin(byte joinRelayPin) {
  joinRelay=joinRelayPin;
  if (joinRelay!=UNUSED_PIN) {
    pinMode(joinRelay,OUTPUT);
    digitalWrite(joinRelay,LOW);  // LOW is relay disengaged
  }
}

void DCC::setThrottle( uint16_t cab, uint8_t tSpeed, bool tDirection)  {
  byte speedCode = (tSpeed & 0x7F)  + tDirection * 128; 
  setThrottle2(cab, speedCode);
  // retain speed for loco reminders
  updateLocoReminder(cab, speedCode );
}

void DCC::setThrottle2( uint16_t cab, byte speedCode)  {

  uint8_t b[4];
  uint8_t nB = 0;
  // DIAG(F("setSpeedInternal %d %x"),cab,speedCode);
  
  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);

  if (globalSpeedsteps <= 28) {

    uint8_t speed128 = speedCode & 0x7F;
    uint8_t speed28;
    uint8_t code28;

    if (speed128 == 0 || speed128 == 1) { // stop or emergency stop
      code28 = speed128;
    } else {
      speed28= (speed128*10+36)/46;                 // convert 2-127 to 1-28
/*
      if (globalSpeedsteps <= 14)                   // Don't want to do 14 steps, to get F0 there is ugly
        code28 = (speed28+3)/2 | (Value of F0);     // convert 1-28 to DCC 14 step speed code
      else
*/
      code28 = (speed28+3)/2 | ( (speed28 & 1) ? 0 : 0b00010000 ); // convert 1-28 to DCC 28 step speed code
    }
    //        Construct command byte from:
    //        command      speed    direction
    b[nB++] = 0b01000000 | code28 | ((speedCode & 0x80) ? 0b00100000 : 0);

  } else { // 128 speedsteps

    b[nB++] = SET_SPEED;                      // 128-step speed control byte
    b[nB++] = speedCode; // for encoding see setThrottle

  }

  DCCWaveform::mainTrack.schedulePacket(b, nB, 0);
}

void DCC::setFunctionInternal(int cab, byte byte1, byte byte2) {
  // DIAG(F("setFunctionInternal %d %x %x"),cab,byte1,byte2);
  byte b[4];
  byte nB = 0;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
  b[nB++] = lowByte(cab);
  if (byte1!=0) b[nB++] = byte1;
  b[nB++] = byte2;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 0);
}

uint8_t DCC::getThrottleSpeed(int cab) {
  int reg=lookupSpeedTable(cab);
  if (reg<0) return -1;
  return speedTable[reg].speedCode & 0x7F;
}

bool DCC::getThrottleDirection(int cab) {
  int reg=lookupSpeedTable(cab);
  if (reg<0) return false ;
  return (speedTable[reg].speedCode & 0x80) !=0;
}

// Set function to value on or off
void DCC::setFn( int cab, int16_t functionNumber, bool on) {
  if (cab<=0 ) return;
  
  if (functionNumber>28) { 
    //non reminding advanced binary bit set 
    byte b[5];
    byte nB = 0;
    if (cab > 127)
      b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address
    b[nB++] = lowByte(cab);
    if (functionNumber <= 127) {
       b[nB++] = 0b11011101;   // Binary State Control Instruction short form  
       b[nB++] = functionNumber | (on ? 0x80 : 0);
    }
    else  {
       b[nB++] = 0b11000000;   // Binary State Control Instruction long form  
       b[nB++] = (functionNumber & 0x7F) | (on ? 0x80 : 0);  // low order bits and state flag
       b[nB++] = functionNumber >>7 ;  // high order bits
    }
    DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
    return;
  }
  
  int reg = lookupSpeedTable(cab);
  if (reg<0) return;  

  // Take care of functions:
  // Set state of function
  unsigned long funcmask = (1UL<<functionNumber);
  if (on) {
      speedTable[reg].functions |= funcmask;
  } else {
      speedTable[reg].functions &= ~funcmask;
  }
  updateGroupflags(speedTable[reg].groupFlags, functionNumber);
  return;
}

// Change function according to how button was pressed,
// typically in WiThrottle.
// Returns new state or -1 if nothing was changed.
int DCC::changeFn( int cab, int16_t functionNumber, bool pressed) {
  int funcstate = -1;
  if (cab<=0 || functionNumber>28) return funcstate;
  int reg = lookupSpeedTable(cab);
  if (reg<0) return funcstate;  

  // Take care of functions:
  // Imitate how many command stations do it: Button press is
  // toggle but for F2 where it is momentary
  unsigned long funcmask = (1UL<<functionNumber);
  if (functionNumber == 2) {
      // turn on F2 on press and off again at release of button
      if (pressed) {
	  speedTable[reg].functions |= funcmask;
	  funcstate = 1;
      } else {
	  speedTable[reg].functions &= ~funcmask;
	  funcstate = 0;
      }
  } else {
      // toggle function on press, ignore release
      if (pressed) {
        speedTable[reg].functions ^= funcmask;
      }
      funcstate = (speedTable[reg].functions & funcmask)? 1 : 0;
  }
  updateGroupflags(speedTable[reg].groupFlags, functionNumber);
  return funcstate;
}

int DCC::getFn( int cab, int16_t functionNumber) {
  if (cab<=0 || functionNumber>28) return -1;  // unknown
  int reg = lookupSpeedTable(cab);
  if (reg<0) return -1;  

  unsigned long funcmask = (1UL<<functionNumber);
  return  (speedTable[reg].functions & funcmask)? 1 : 0;
}

// Set the group flag to say we have touched the particular group.
// A group will be reminded only if it has been touched.  
void DCC::updateGroupflags(byte & flags, int16_t functionNumber) {
  byte groupMask;
  if (functionNumber<=4)       groupMask=FN_GROUP_1;
  else if (functionNumber<=8)  groupMask=FN_GROUP_2;
  else if (functionNumber<=12) groupMask=FN_GROUP_3;
  else if (functionNumber<=20) groupMask=FN_GROUP_4;
  else                         groupMask=FN_GROUP_5;
  flags |= groupMask; 
}

void DCC::setAccessory(int address, byte number, bool activate) {
  // use masks to detect wrong values and do nothing
  if(address != (address & 511))
    return;
  if(number != (number & 3))
    return;
  byte b[2];

  b[0] = address % 64 + 128;                                     // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least signifcant bits of accessory address
  b[1] = ((((address / 64) % 8) << 4) + (number % 4 << 1) + activate % 2) ^ 0xF8; // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

  DCCWaveform::mainTrack.schedulePacket(b, 2, 4);      // Repeat the packet four times
}

//
// writeCVByteMain: Write a byte with PoM on main. This writes
// the 5 byte sized packet to implement this DCC function
//
void DCC::writeCVByteMain(int cab, int cv, byte bValue)  {
  byte b[5];
  byte nB = 0;
  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BYTE_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = bValue;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

//
// writeCVBitMain: Write a bit of a byte with PoM on main. This writes
// the 5 byte sized packet to implement this DCC function
//
void DCC::writeCVBitMain(int cab, int cv, byte bNum, bool bValue)  {
  byte b[5];
  byte nB = 0;
  bValue = bValue % 2;
  bNum = bNum % 8;

  if (cab > 127)
    b[nB++] = highByte(cab) | 0xC0;    // convert train number into a two-byte address

  b[nB++] = lowByte(cab);
  b[nB++] = cv1(WRITE_BIT_MAIN, cv); // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
  b[nB++] = cv2(cv);
  b[nB++] = WRITE_BIT | (bValue ? BIT_ON : BIT_OFF) | bNum;

  DCCWaveform::mainTrack.schedulePacket(b, nB, 4);
}

void DCC::setProgTrackSyncMain(bool on) {
  if (joinRelay!=UNUSED_PIN) digitalWrite(joinRelay,on?HIGH:LOW);
  DCCWaveform::progTrackSyncMain=on;
}
void DCC::setProgTrackBoost(bool on) {
  DCCWaveform::progTrackBoosted=on;
}

FSH* DCC::getMotorShieldName() {
  return shieldName;
}
  
const ackOp FLASH WRITE_BIT0_PROG[] = {
     BASELINE,
     W0,WACK,
     V0, WACK,  // validate bit is 0 
     ITC1,      // if acked, callback(1)
     FAIL  // callback (-1)
};
const ackOp FLASH WRITE_BIT1_PROG[] = {
     BASELINE,
     W1,WACK,
     V1, WACK,  // validate bit is 1 
     ITC1,      // if acked, callback(1)
     FAIL  // callback (-1)
};

const ackOp FLASH VERIFY_BIT0_PROG[] = {
     BASELINE,
     V0, WACK,  // validate bit is 0 
     ITC0,      // if acked, callback(0)
     V1, WACK,  // validate bit is 1
     ITC1,       
     FAIL  // callback (-1)
};
const ackOp FLASH VERIFY_BIT1_PROG[] = {
     BASELINE,
     V1, WACK,  // validate bit is 1 
     ITC1,      // if acked, callback(1)
     V0, WACK, 
     ITC0,
     FAIL  // callback (-1)
};

const ackOp FLASH READ_BIT_PROG[] = {
     BASELINE,
     V1, WACK,  // validate bit is 1 
     ITC1,      // if acked, callback(1)
     V0, WACK,  // validate bit is zero
     ITC0,      // if acked callback 0
     FAIL       // bit not readable 
     };
     
const ackOp FLASH WRITE_BYTE_PROG[] = {
      BASELINE,
      WB,WACK,ITC1,    // Write and callback(1) if ACK 
      // handle decoders that dont ack a write 
      VB,WACK,ITC1,    // validate byte and callback(1) if correct 
      FAIL        // callback (-1)
      };
      
const ackOp FLASH VERIFY_BYTE_PROG[] = {
      BASELINE,
      VB,WACK,     // validate byte 
      ITCB,       // if ok callback value
      STARTMERGE,    //clear bit and byte values ready for merge pass
      // each bit is validated against 0 and the result inverted in MERGE
      // this is because there tend to be more zeros in cv values than ones.  
      // There is no need for one validation as entire byte is validated at the end
      V0, WACK, MERGE,        // read and merge first tested bit (7)
      ITSKIP,                 // do small excursion if there was no ack
        SETBIT,(ackOp)7,
        V1, WACK, NAKFAIL,    // test if there is an ack on the inverse of this bit (7)
        SETBIT,(ackOp)6,      // and abort whole test if not else continue with bit (6)
      SKIPTARGET,
      V0, WACK, MERGE,        // read and merge second tested bit (6)
      V0, WACK, MERGE,        // read and merge third  tested bit (5) ...
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB,  // verify merged byte and return it if acked ok 
      FAIL };
      
      
const ackOp FLASH READ_CV_PROG[] = {
      BASELINE,
      STARTMERGE,    //clear bit and byte values ready for merge pass
      // each bit is validated against 0 and the result inverted in MERGE
      // this is because there tend to be more zeros in cv values than ones.  
      // There is no need for one validation as entire byte is validated at the end
      V0, WACK, MERGE,        // read and merge first tested bit (7)
      ITSKIP,                 // do small excursion if there was no ack
        SETBIT,(ackOp)7,
        V1, WACK, NAKFAIL,    // test if there is an ack on the inverse of this bit (7)
        SETBIT,(ackOp)6,      // and abort whole test if not else continue with bit (6)
      SKIPTARGET,
      V0, WACK, MERGE,        // read and merge second tested bit (6)
      V0, WACK, MERGE,        // read and merge third  tested bit (5) ...
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB,  // verify merged byte and return it if acked ok 
      FAIL };          // verification failed


const ackOp FLASH LOCO_ID_PROG[] = {
      BASELINE,
      SETCV, (ackOp)19,     // CV 19 is consist setting
      SETBYTE, (ackOp)0,    
      VB, WACK, ITSKIP,     // ignore consist if cv19 is zero (no consist)
      SETBYTE, (ackOp)128,
      VB, WACK, ITSKIP,     // ignore consist if cv19 is 128 (no consist, direction bit set)
      STARTMERGE,           // Setup to read cv 19
      V0, WACK, MERGE,  
      V0, WACK, MERGE,  
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB7,  // return 7 bits only, No_ACK means CV19 not supported so ignore it
      
      SKIPTARGET,     // continue here if CV 19 is zero or fails all validation      
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      V0, WACK, ITSKIP,  // Skip to SKIPTARGET if bit 5 of CV29 is zero
      
      // Long locoid  
      SETCV, (ackOp)17,       // CV 17 is part of locoid
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // verify merged byte and return -1 it if not acked ok
      STASHLOCOID,         // keep stashed cv 17 for later 
      // Read 2nd part from CV 18 
      SETCV, (ackOp)18,
      STARTMERGE,
      V0, WACK, MERGE,  // read and merge bit 1 etc
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, NAKFAIL,  // verify merged byte and return -1 it if not acked ok
      COMBINELOCOID,        // Combile byte with stash to make long locoid and callback
      
      // ITSKIP Skips to here if CV 29 bit 5 was zero. so read CV 1 and return that  
      SKIPTARGET,
      SETCV, (ackOp)1,
      STARTMERGE,
      SETBIT, (ackOp)6,  // skip over first bit as we know its a zero
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      V0, WACK, MERGE,
      VB, WACK, ITCB,  // verify merged byte and callback
      FAIL
      };    

const ackOp FLASH SHORT_LOCO_ID_PROG[] = {
      BASELINE,
      SETCV,(ackOp)19,
      SETBYTE, (ackOp)0,
      WB,WACK,     // ignore dedcoder without cv19 support
      // Turn off long address flag
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      W0,WACK,
      V0,WACK,NAKFAIL,
      SETCV, (ackOp)1,   
      SETBYTEL,   // low byte of word 
      WB,WACK,    // some decoders don't ACK writes
      VB,WACK,ITCB,
      FAIL
};    

const ackOp FLASH LONG_LOCO_ID_PROG[] = {
      BASELINE,
      // Clear consist CV 19
      SETCV,(ackOp)19,
      SETBYTE, (ackOp)0,
      WB,WACK,     // ignore decoder without cv19 support
      // Turn on long address flag cv29 bit 5
      SETCV,(ackOp)29,
      SETBIT,(ackOp)5,
      W1,WACK,
      V1,WACK,NAKFAIL,
      // Store high byte of address in cv 17
      SETCV, (ackOp)17,
      SETBYTEH,   // high byte of word 
      WB,WACK,
      VB,WACK,NAKFAIL,
      // store 
      SETCV, (ackOp)18,
      SETBYTEL,   // low byte of word 
      WB,WACK,
      VB,WACK,ITC1,   // callback(1) means Ok
      FAIL
};    

void  DCC::writeCVByte(int16_t cv, byte byteValue, ACK_CALLBACK callback)  {
  ackManagerSetup(cv, byteValue,  WRITE_BYTE_PROG, callback);
}

void DCC::writeCVBit(int16_t cv, byte bitNum, bool bitValue, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else ackManagerSetup(cv, bitNum, bitValue?WRITE_BIT1_PROG:WRITE_BIT0_PROG, callback);
}

void  DCC::verifyCVByte(int16_t cv, byte byteValue, ACK_CALLBACK callback)  {
  ackManagerSetup(cv, byteValue,  VERIFY_BYTE_PROG, callback);
}

void DCC::verifyCVBit(int16_t cv, byte bitNum, bool bitValue, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else ackManagerSetup(cv, bitNum, bitValue?VERIFY_BIT1_PROG:VERIFY_BIT0_PROG, callback);
}


void DCC::readCVBit(int16_t cv, byte bitNum, ACK_CALLBACK callback)  {
  if (bitNum >= 8) callback(-1);
  else ackManagerSetup(cv, bitNum,READ_BIT_PROG, callback);
}

void DCC::readCV(int16_t cv, ACK_CALLBACK callback)  {
  ackManagerSetup(cv, 0,READ_CV_PROG, callback);
}

void DCC::getLocoId(ACK_CALLBACK callback) {
  ackManagerSetup(0,0, LOCO_ID_PROG, callback);
}

void DCC::setLocoId(int id,ACK_CALLBACK callback) {
  if (id<1 || id>10239) { //0x27FF according to standard
    callback(-1);
    return;
  }
  if (id<=127)
      ackManagerSetup(id, SHORT_LOCO_ID_PROG, callback);
  else
      ackManagerSetup(id | 0xc000,LONG_LOCO_ID_PROG, callback);
}

void DCC::forgetLoco(int cab) {  // removes any speed reminders for this loco
  setThrottle2(cab,1); // ESTOP this loco if still on track  
  int reg=lookupSpeedTable(cab);
  if (reg>=0) speedTable[reg].loco=0;
  setThrottle2(cab,1); // ESTOP if this loco still on track
}
void DCC::forgetAllLocos() {  // removes all speed reminders
  setThrottle2(0,1); // ESTOP all locos still on track      
  for (int i=0;i<MAX_LOCOS;i++) speedTable[i].loco=0;
}

byte DCC::loopStatus=0;  

void DCC::loop()  {
  DCCWaveform::loop(ackManagerProg!=NULL); // power overload checks
  ackManagerLoop();    // maintain prog track ack manager
  issueReminders();
}

void DCC::issueReminders() {
  // if the main track transmitter still has a pending packet, skip this time around.
  if ( DCCWaveform::mainTrack.packetPending) return;

  // This loop searches for a loco in the speed table starting at nextLoco and cycling back around
  for (int reg=0;reg<MAX_LOCOS;reg++) {
       int slot=reg+nextLoco;
       if (slot>=MAX_LOCOS) slot-=MAX_LOCOS; 
       if (speedTable[slot].loco > 0) {
          // have found the next loco to remind 
          // issueReminder will return true if this loco is completed (ie speed and functions)
          if (issueReminder(slot)) nextLoco=slot+1; 
          return;
        }
  }
}
 
bool DCC::issueReminder(int reg) {
  unsigned long functions=speedTable[reg].functions;
  int loco=speedTable[reg].loco;
  byte flags=speedTable[reg].groupFlags;
  
  switch (loopStatus) {
        case 0:
      //   DIAG(F("Reminder %d speed %d"),loco,speedTable[reg].speedCode);
         setThrottle2(loco, speedTable[reg].speedCode);
         break;
       case 1: // remind function group 1 (F0-F4)
          if (flags & FN_GROUP_1) 
              setFunctionInternal(loco,0, 128 | ((functions>>1)& 0x0F) | ((functions & 0x01)<<4)); // 100D DDDD
          break;     
       case 2: // remind function group 2 F5-F8
          if (flags & FN_GROUP_2) 
              setFunctionInternal(loco,0, 176 | ((functions>>5)& 0x0F));                           // 1011 DDDD
          break;     
       case 3: // remind function group 3 F9-F12
          if (flags & FN_GROUP_3) 
              setFunctionInternal(loco,0, 160 | ((functions>>9)& 0x0F));                           // 1010 DDDD
          break;   
       case 4: // remind function group 4 F13-F20
          if (flags & FN_GROUP_4) 
              setFunctionInternal(loco,222, ((functions>>13)& 0xFF)); 
          flags&= ~FN_GROUP_4;  // dont send them again
          break;  
       case 5: // remind function group 5 F21-F28
          if (flags & FN_GROUP_5)
              setFunctionInternal(loco,223, ((functions>>21)& 0xFF)); 
          flags&= ~FN_GROUP_5;  // dont send them again
          break; 
      }
      loopStatus++;
      // if we reach status 6 then this loco is done so
      // reset status to 0 for next loco and return true so caller 
      // moves on to next loco. 
      if (loopStatus>5) loopStatus=0;
      return loopStatus==0;
    }
 
 


///// Private helper functions below here /////////////////////

byte DCC::cv1(byte opcode, int cv)  {
  cv--;
  return (highByte(cv) & (byte)0x03) | opcode;
}
byte DCC::cv2(int cv)  {
  cv--;
  return lowByte(cv);
}

int DCC::lookupSpeedTable(int locoId) {
  // determine speed reg for this loco
  int firstEmpty = MAX_LOCOS;
  int reg;
  for (reg = 0; reg < MAX_LOCOS; reg++) {
    if (speedTable[reg].loco == locoId) break;
    if (speedTable[reg].loco == 0 && firstEmpty == MAX_LOCOS) firstEmpty = reg;
  }
  if (reg == MAX_LOCOS) reg = firstEmpty;
  if (reg >= MAX_LOCOS) {
    DIAG(F("Too many locos"));
    return -1;
  }
  if (reg==firstEmpty){
        speedTable[reg].loco = locoId;
        speedTable[reg].speedCode=128;  // default direction forward
        speedTable[reg].groupFlags=0;
        speedTable[reg].functions=0;
  }
  return reg;
}
  
void  DCC::updateLocoReminder(int loco, byte speedCode) {
 
  if (loco==0) {
     // broadcast stop/estop but dont change direction
     for (int reg = 0; reg < MAX_LOCOS; reg++) {
       speedTable[reg].speedCode = (speedTable[reg].speedCode & 0x80) |  (speedCode & 0x7f);
     }
     return; 
  }
  
  // determine speed reg for this loco
  int reg=lookupSpeedTable(loco);       
  if (reg>=0) speedTable[reg].speedCode = speedCode;
}

DCC::LOCO DCC::speedTable[MAX_LOCOS];
int DCC::nextLoco = 0;

//ACK MANAGER
ackOp  const *  DCC::ackManagerProg;
byte   DCC::ackManagerByte;
byte   DCC::ackManagerStash;
int    DCC::ackManagerWord;
int    DCC::ackManagerCv;
byte   DCC::ackManagerBitNum;
bool   DCC::ackReceived;
bool   DCC::ackManagerRejoin;

CALLBACK_STATE DCC::callbackState=READY;

ACK_CALLBACK DCC::ackManagerCallback;

void  DCC::ackManagerSetup(int cv, byte byteValueOrBitnum, ackOp const program[], ACK_CALLBACK callback) {
  if (!DCCWaveform::progTrack.canMeasureCurrent()) {
    callback(-2);
    return;
  }

  ackManagerRejoin=DCCWaveform::progTrackSyncMain;     
  if (ackManagerRejoin ) {
        // Change from JOIN must zero resets packet.
        setProgTrackSyncMain(false);
        DCCWaveform::progTrack.sentResetsSincePacket = 0;      
      }
      
   DCCWaveform::progTrack.autoPowerOff=false;           
   if (DCCWaveform::progTrack.getPowerMode() == POWERMODE::OFF) {
        DCCWaveform::progTrack.autoPowerOff=true;  // power off afterwards           
        if (Diag::ACK) DIAG(F("Auto Prog power on"));
        DCCWaveform::progTrack.setPowerMode(POWERMODE::ON);
        DCCWaveform::progTrack.sentResetsSincePacket = 0;      
    }

  ackManagerCv = cv;
  ackManagerProg = program;
  ackManagerByte = byteValueOrBitnum;
  ackManagerBitNum=byteValueOrBitnum;
  ackManagerCallback = callback;
}

void  DCC::ackManagerSetup(int wordval, ackOp const program[], ACK_CALLBACK callback) {
  ackManagerWord=wordval;
  ackManagerSetup(0, 0, program, callback);
  }

const byte RESET_MIN=8;  // tuning of reset counter before sending message

// checkRessets return true if the caller should yield back to loop and try later.
bool DCC::checkResets(uint8_t numResets) {
  return DCCWaveform::progTrack.sentResetsSincePacket < numResets;
}

void DCC::ackManagerLoop() {
  while (ackManagerProg) {
    byte opcode=GETFLASH(ackManagerProg);
    
    // breaks from this switch will step to next prog entry
    // returns from this switch will stay on same entry
    // (typically waiting for a reset counter or ACK waiting, or when all finished.)
    switch (opcode) {
      case BASELINE:
      	  if (checkResets(DCCWaveform::progTrack.autoPowerOff || ackManagerRejoin  ? 20 : 3)) return;
          DCCWaveform::progTrack.setAckBaseline();
          callbackState=READY;
          break;   
      case W0:    // write 0 bit 
      case W1:    // write 1 bit 
            {
	      if (checkResets(RESET_MIN)) return;
              if (Diag::ACK) DIAG(F("W%d cv=%d bit=%d"),opcode==W1, ackManagerCv,ackManagerBitNum); 
              byte instruction = WRITE_BIT | (opcode==W1 ? BIT_ON : BIT_OFF) | ackManagerBitNum;
              byte message[] = {cv1(BIT_MANIPULATE, ackManagerCv), cv2(ackManagerCv), instruction };
              DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
              DCCWaveform::progTrack.setAckPending(); 
              callbackState=AFTER_WRITE;
            }
            break; 
      
      case WB:   // write byte 
            {
	      if (checkResets( RESET_MIN)) return;
              if (Diag::ACK) DIAG(F("WB cv=%d value=%d"),ackManagerCv,ackManagerByte);
              byte message[] = {cv1(WRITE_BYTE, ackManagerCv), cv2(ackManagerCv), ackManagerByte};
              DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
              DCCWaveform::progTrack.setAckPending(); 
              callbackState=AFTER_WRITE;
            }
            break;
      
      case   VB:     // Issue validate Byte packet
        {
	  if (checkResets( RESET_MIN)) return; 
          if (Diag::ACK) DIAG(F("VB cv=%d value=%d"),ackManagerCv,ackManagerByte);
          byte message[] = { cv1(VERIFY_BYTE, ackManagerCv), cv2(ackManagerCv), ackManagerByte};
          DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
          DCCWaveform::progTrack.setAckPending(); 
        }
        break;
      
      case V0:
      case V1:      // Issue validate bit=0 or bit=1  packet
        {
	  if (checkResets(RESET_MIN)) return; 
          if (Diag::ACK) DIAG(F("V%d cv=%d bit=%d"),opcode==V1, ackManagerCv,ackManagerBitNum); 
          byte instruction = VERIFY_BIT | (opcode==V0?BIT_OFF:BIT_ON) | ackManagerBitNum;
          byte message[] = {cv1(BIT_MANIPULATE, ackManagerCv), cv2(ackManagerCv), instruction };
          DCCWaveform::progTrack.schedulePacket(message, sizeof(message), PROG_REPEATS);
          DCCWaveform::progTrack.setAckPending(); 
        }
        break;
      
      case WACK:   // wait for ack (or absence of ack)
         {
          byte ackState=2; // keep polling
      
          ackState=DCCWaveform::progTrack.getAck();
          if (ackState==2) return; // keep polling
          ackReceived=ackState==1;
          break;  // we have a genuine ACK result
         }
     case ITC0:
     case ITC1:   // If True Callback(0 or 1)  (if prevous WACK got an ACK)
        if (ackReceived) {
            callback(opcode==ITC0?0:1);
            return;
          }
        break;
        
      case ITCB:   // If True callback(byte)
          if (ackReceived) {
            callback(ackManagerByte);
            return;
          }
        break;

      case ITCB7:   // If True callback(byte & 0x7F)
          if (ackReceived) {
            callback(ackManagerByte & 0x7F);
            return;
          }
        break;
        
      case NAKFAIL:   // If nack callback(-1)
          if (!ackReceived) {
            callback(-1);
            return;
          }
        break;
        
      case FAIL:  // callback(-1)
           callback(-1);
           return;
           
      case STARTMERGE:
           ackManagerBitNum=7;
           ackManagerByte=0;     
          break;
          
      case MERGE:  // Merge previous Validate zero wack response with byte value and update bit number (use for reading CV bytes)
          ackManagerByte <<= 1;
          // ackReceived means bit is zero. 
          if (!ackReceived) ackManagerByte |= 1;
          ackManagerBitNum--;
          break;

      case SETBIT:
          ackManagerProg++; 
          ackManagerBitNum=GETFLASH(ackManagerProg);
          break;

     case SETCV:
          ackManagerProg++; 
          ackManagerCv=GETFLASH(ackManagerProg);
          break;

     case SETBYTE:
          ackManagerProg++; 
          ackManagerByte=GETFLASH(ackManagerProg);
          break;

    case SETBYTEH:
          ackManagerByte=highByte(ackManagerWord);
          break;
          
    case SETBYTEL:
          ackManagerByte=lowByte(ackManagerWord);
          break;

     case STASHLOCOID:
          ackManagerStash=ackManagerByte;  // stash value from CV17 
          break;
          
     case COMBINELOCOID: 
          // ackManagerStash is  cv17, ackManagerByte is CV 18
         {
          int locoid=ackManagerByte + ((ackManagerStash - 192) << 8);
          if (locoid<128) locoid=-3;
          callback(locoid);
         }
         return;            

     case ITSKIP:
          if (!ackReceived) break; 
          // SKIP opcodes until SKIPTARGET found
          while (opcode!=SKIPTARGET) {
            ackManagerProg++; 
            opcode=GETFLASH(ackManagerProg);
          }
          break;
     case SKIPTARGET: 
          break;     
     default: 
          DIAG(F("!! ackOp %d FAULT!!"),opcode);
          callback( -1);
          return;        
    
      }  // end of switch
    ackManagerProg++;
  }
}

void DCC::callback(int value) {
    static unsigned long callbackStart;
    // We are about to leave programming mode
    // Rule 1: If we have written to a decoder we must maintain power for 100mS
    // Rule 2: If we are re-joining the main track we must power off for 30mS

    switch (callbackState) {    
       case AFTER_WRITE:  // first attempt to callback after a write operation
	    if (!ackManagerRejoin && !DCCWaveform::progTrack.autoPowerOff) {
               callbackState=READY;
               break;
            }                              // lines 906-910 added. avoid wait after write. use 1 PROG
            callbackStart=millis();
            callbackState=WAITING_100;
            if (Diag::ACK) DIAG(F("Stable 100mS"));
            break;
            
       case WAITING_100:  // waiting for 100mS
            if (millis()-callbackStart < 100) break;
            // stable after power maintained for 100mS

            // If we are going to power off anyway, it doesnt matter
            // but if we will keep the power on, we must off it for 30mS
            if (DCCWaveform::progTrack.autoPowerOff) callbackState=READY;
            else { // Need to cycle power off and on
                DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF); 
                callbackStart=millis();
                callbackState=WAITING_30;
                if (Diag::ACK) DIAG(F("OFF 30mS"));
            }
            break;
 
        case WAITING_30:  // waiting for 30mS with power off
            if (millis()-callbackStart < 30) break;
            //power has been off for 30mS
            DCCWaveform::progTrack.setPowerMode(POWERMODE::ON); 
            callbackState=READY;
            break;
     
       case READY:  // ready after read, or write after power delay and off period.
            // power off if we powered it on
           if (DCCWaveform::progTrack.autoPowerOff) {
              if (Diag::ACK) DIAG(F("Auto Prog power off"));
              DCCWaveform::progTrack.doAutoPowerOff();
           }
          // Restore <1 JOIN> to state before BASELINE
          if (ackManagerRejoin) {
              setProgTrackSyncMain(true);
              if (Diag::ACK) DIAG(F("Auto JOIN"));
          }  
    
          ackManagerProg=NULL;  // no more steps to execute
          if (Diag::ACK) DIAG(F("Callback(%d)"),value);
          (ackManagerCallback)( value);
    }
}

void DCC::displayCabList(Print * stream) {

    int used=0;
    for (int reg = 0; reg < MAX_LOCOS; reg++) {
       if (speedTable[reg].loco>0) {
        used ++;
        StringFormatter::send(stream,F("cab=%d, speed=%d, dir=%c \n"),       
           speedTable[reg].loco,  speedTable[reg].speedCode & 0x7f,(speedTable[reg].speedCode & 0x80) ? 'F':'R');
       }
     }
     StringFormatter::send(stream,F("Used=%d, max=%d\n"),used,MAX_LOCOS);
     
}
