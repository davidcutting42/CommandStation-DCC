/*
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
 #pragma GCC optimize ("-O3")
#include <Arduino.h>

#include "DCCWaveform.h"
#include "DCCTimer.h"
#include "DIAG.h"
#include "freeMemory.h"

DCCWaveform  DCCWaveform::mainTrack(PREAMBLE_BITS_MAIN, true);
DCCWaveform  DCCWaveform::progTrack(PREAMBLE_BITS_PROG, false);

bool DCCWaveform::useRailcom=false;
bool DCCWaveform::supportsRailcom=false;
bool DCCWaveform::progTrackSyncMain=false; 
bool DCCWaveform::progTrackBoosted=false; 
int  DCCWaveform::progTripValue=0;
volatile uint8_t DCCWaveform::numAckGaps=0;
volatile uint8_t DCCWaveform::numAckSamples=0;
uint8_t DCCWaveform::trailingEdgeCounter=0;

void DCCWaveform::begin(MotorDriver * mainDriver, MotorDriver * progDriver) {
  mainTrack.motorDriver=mainDriver;
  progTrack.motorDriver=progDriver;
  progTripValue = progDriver->mA2raw(TRIP_CURRENT_PROG); // need only calculate once hence static
  mainTrack.setPowerMode(POWERMODE::OFF);      
  progTrack.setPowerMode(POWERMODE::OFF);
  // Fault pin config for odd motor boards (example pololu)
  MotorDriver::commonFaultPin = ((mainDriver->getFaultPin() == progDriver->getFaultPin())
				 && (mainDriver->getFaultPin() != UNUSED_PIN));
  // Only use PWM if both pins are PWM capable. Otherwise JOIN does not work
  MotorDriver::usePWM= mainDriver->isPWMCapable() && progDriver->isPWMCapable();
  #ifndef TEENSYDUINO
    supportsRailcom= MotorDriver::usePWM && mainDriver->isRailcomCapable() && progDriver->isRailcomCapable();
  #else
    supportsRailcom=mainDriver->isRailcomCapable() && progDriver->isRailcomCapable();
  #endif  // supportsRailcom depends on hardware caopability
  
  // useRailcom is user switchable at run time. 
  useRailcom=supportsRailcom;
  
  if (MotorDriver::usePWM){
    DIAG(F("Signal pin config: high accuracy waveform"));
    if (supportsRailcom) DIAG(F("Railcom cutout enabled"));
  }
  else
    DIAG(F("Signal pin config: normal accuracy waveform"));
    if (supportsRailcom) DIAG(F("Railcom cutout enabled"));
    DCCTimer::begin(DCCWaveform::interruptHandler);     
}

void DCCWaveform::loop(bool ackManagerActive) {
  mainTrack.checkPowerOverload(false);
  progTrack.checkPowerOverload(ackManagerActive);
}

void DCCWaveform::interruptHandler() {
  // call the timer edge sensitive actions for progtrack and maintrack
  // member functions would be cleaner but have more overhead
  byte sigMain=signalTransform[mainTrack.state];
  byte sigProg=progTrackSyncMain? sigMain : signalTransform[progTrack.state];
  
  // Set the signal state for both tracks
  mainTrack.motorDriver->setSignal(sigMain);
  progTrack.motorDriver->setSignal(sigProg);
  
  // Move on in the state engine
  mainTrack.state=stateTransform[mainTrack.state];    
  progTrack.state=stateTransform[progTrack.state];    


  // WAVE_PENDING means we dont yet know what the next bit is
  if (mainTrack.state==WAVE_PENDING) mainTrack.interrupt2();  
  if (progTrack.state==WAVE_PENDING) progTrack.interrupt2();
  else if (progTrack.ackPending) progTrack.checkAck();

}


// An instance of this class handles the DCC transmissions for one track. (main or prog)
// Interrupts are marshalled via the statics.
// A track has a current transmit buffer, and a pending buffer.
// When the current buffer is exhausted, either the pending buffer (if there is one waiting) or an idle buffer.


// This bitmask has 9 entries as each byte is trasmitted as a zero + 8 bits.
const byte bitMask[] = {0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};


DCCWaveform::DCCWaveform( byte preambleBits, bool isMain) {
  isMainTrack = isMain;
  packetPending = false;
  memcpy(transmitPacket, idlePacket, sizeof(idlePacket));
  state = WAVE_START;
  // The +1 below is to allow the preamble generator to create the stop bit
  // for the previous packet. 
  requiredPreambles = preambleBits+1;  
  bytes_sent = 0;
  bits_sent = 0;
  sampleDelay = 0;
  lastSampleTaken = millis();
  ackPending=false;
}

POWERMODE DCCWaveform::getPowerMode() {
  return powerMode;
}

void DCCWaveform::setPowerMode(POWERMODE mode) {
  powerMode = mode;
  bool ison = (mode == POWERMODE::ON);
  motorDriver->setPower( ison);
}

bool DCCWaveform::setUseRailcom(bool on) {
  if (!supportsRailcom) return false; 
  useRailcom=on;
  if (!on) {
    // turn off any existing cutout 
    mainTrack.motorDriver->setRailcomCutout(false);
    progTrack.motorDriver->setRailcomCutout(false);
  }
  return true;
}

void DCCWaveform::checkPowerOverload(bool ackManagerActive) {
  if (millis() - lastSampleTaken  < sampleDelay) return;
  lastSampleTaken = millis();
  int tripValue= motorDriver->getRawCurrentTripValue();
  if (!isMainTrack && !ackManagerActive && !progTrackSyncMain && !progTrackBoosted)
    tripValue=progTripValue;
  
  switch (powerMode) {
    case POWERMODE::OFF:
      sampleDelay = POWER_SAMPLE_OFF_WAIT;
      break;
    case POWERMODE::ON:
      // Check current
      lastCurrent=motorDriver->getCurrentRaw();
      if (lastCurrent < 0) {
	  // We have a fault pin condition to take care of
	  lastCurrent = -lastCurrent;
	  setPowerMode(POWERMODE::OVERLOAD); // Turn off, decide later how fast to turn on again
	  if (MotorDriver::commonFaultPin) {
	      if (lastCurrent <= tripValue) {
		setPowerMode(POWERMODE::ON); // maybe other track
	      }
	      // Write this after the fact as we want to turn on as fast as possible
	      // because we don't know which output actually triggered the fault pin
	      DIAG(F("*** COMMON FAULT PIN ACTIVE - TOGGLED POWER on %S ***"), isMainTrack ? F("MAIN") : F("PROG"));
	  } else {
	      DIAG(F("*** %S FAULT PIN ACTIVE - OVERLOAD ***"), isMainTrack ? F("MAIN") : F("PROG"));
	      if (lastCurrent < tripValue) {
		  lastCurrent = tripValue; // exaggerate
	      }
	  }
      }
      if (lastCurrent < tripValue) {
        sampleDelay = POWER_SAMPLE_ON_WAIT;
	if(power_good_counter<100)
	  power_good_counter++;
	else
	  if (power_sample_overload_wait>POWER_SAMPLE_OVERLOAD_WAIT) power_sample_overload_wait=POWER_SAMPLE_OVERLOAD_WAIT;
      } else {
        setPowerMode(POWERMODE::OVERLOAD);
        unsigned int mA=motorDriver->raw2mA(lastCurrent);
        unsigned int maxmA=motorDriver->raw2mA(tripValue);
	power_good_counter=0;
        sampleDelay = power_sample_overload_wait;
        DIAG(F("*** %S TRACK POWER OVERLOAD current=%d max=%d  offtime=%d ***"), isMainTrack ? F("MAIN") : F("PROG"), mA, maxmA, sampleDelay);
	if (power_sample_overload_wait >= 10000)
	    power_sample_overload_wait = 10000;
	else
	    power_sample_overload_wait *= 2;
      }
      break;
    case POWERMODE::OVERLOAD:
      // Try setting it back on after the OVERLOAD_WAIT
      setPowerMode(POWERMODE::ON);
      sampleDelay = POWER_SAMPLE_ON_WAIT;
      // Debug code....
      DIAG(F("*** %S TRACK POWER RESET delay=%d ***"), isMainTrack ? F("MAIN") : F("PROG"), sampleDelay);
      break;
    default:
      sampleDelay = 999; // cant get here..meaningless statement to avoid compiler warning.
  }
}
// For each state of the wave  nextState=stateTransform[currentState] 
const WAVE_STATE DCCWaveform::stateTransform[]={
   /* WAVE_START   -> */ WAVE_PENDING,
   /* WAVE_MID_1   -> */ WAVE_START,
   /* WAVE_HIGH_0  -> */ WAVE_MID_0,
   /* WAVE_MID_0   -> */ WAVE_LOW_0,
   /* WAVE_LOW_0   -> */ WAVE_START,
   /* WAVE_PENDING (should not happen) -> */ WAVE_PENDING};

// For each state of the wave, signal pin is HIGH or LOW   
const bool DCCWaveform::signalTransform[]={
   /* WAVE_START   -> */ HIGH,
   /* WAVE_MID_1   -> */ LOW,
   /* WAVE_HIGH_0  -> */ HIGH,
   /* WAVE_MID_0   -> */ LOW,
   /* WAVE_LOW_0   -> */ LOW,
   /* WAVE_PENDING (should not happen) -> */ LOW};
        
void DCCWaveform::interrupt2() {
  // calculate the next bit to be sent:
  // set state WAVE_MID_1  for a 1=bit
  //        or WAVE_HIGH_0 for a 0 bit.

  if (remainingPreambles > 0 ) {
    state=WAVE_MID_1;  // switch state to trigger LOW on next interrupt
    remainingPreambles--;
    
    // Railcom cutout processing but not on prog if synced with main
    if (useRailcom && (isMainTrack || !progTrackSyncMain)) do {
        bool cutout;
        if (remainingPreambles==(requiredPreambles-2)) cutout=true;
        else if (remainingPreambles==(requiredPreambles-6)) cutout=false;    
        else break;
        motorDriver->setRailcomCutout(cutout);
        if (progTrackSyncMain) progTrack.motorDriver->setRailcomCutout(cutout);   
        } while(false);  // this is to allow break out of do {...} above 
     
    // Update free memory diagnostic as we don't have anything else to do this time.
    // Allow for checkAck and its called functions using 22 bytes more.
    updateMinimumFreeMemory(22); 
    return;
  }

  // Wave has gone HIGH but what happens next depends on the bit to be transmitted
  // beware OF 9-BIT MASK  generating a zero to start each byte
  state=(transmitPacket[bytes_sent] & bitMask[bits_sent])? WAVE_MID_1 : WAVE_HIGH_0; 
  bits_sent++;

  // If this is the last bit of a byte, prepare for the next byte

  if (bits_sent == 9) { // zero followed by 8 bits of a byte
    //end of Byte
    bits_sent = 0;
    bytes_sent++;
    // if this is the last byte, prepere for next packet
    if (bytes_sent >= transmitLength) {
      // end of transmission buffer... repeat or switch to next message
      bytes_sent = 0;
      remainingPreambles = requiredPreambles;

      if (transmitRepeats > 0) {
        transmitRepeats--;
      }
      else if (packetPending) {
        // Copy pending packet to transmit packet
        // a fixed length memcpy is faster than a variable length loop for these small lengths
        // for (int b = 0; b < pendingLength; b++) transmitPacket[b] = pendingPacket[b];
        memcpy( transmitPacket, pendingPacket, sizeof(pendingPacket));
        
        transmitLength = pendingLength;
        transmitRepeats = pendingRepeats;
        packetPending = false;
        sentResetsSincePacket=0;
      }
      else {
        // Fortunately reset and idle packets are the same length
        memcpy( transmitPacket, isMainTrack ? idlePacket : resetPacket, sizeof(idlePacket));
        transmitLength = sizeof(idlePacket);
        transmitRepeats = 0;
        if (sentResetsSincePacket<250) sentResetsSincePacket++;
      }
    }
  }  
}



// Wait until there is no packet pending, then make this pending
void DCCWaveform::schedulePacket(const byte buffer[], byte byteCount, byte repeats) {
  if (byteCount > MAX_PACKET_SIZE) return; // allow for chksum
  while (packetPending);

  byte checksum = 0;
  for (byte b = 0; b < byteCount; b++) {
    checksum ^= buffer[b];
    pendingPacket[b] = buffer[b];
  }
  // buffer is MAX_PACKET_SIZE but pendingPacket is one bigger
  pendingPacket[byteCount] = checksum;
  pendingLength = byteCount + 1;
  pendingRepeats = repeats;
  packetPending = true;
  sentResetsSincePacket=0;
}

// Operations applicable to PROG track ONLY.
// (yes I know I could have subclassed the main track but...) 

void DCCWaveform::setAckBaseline() {
      if (isMainTrack) return;
      int baseline=motorDriver->getCurrentRaw();
      ackThreshold= baseline + motorDriver->mA2raw(ackLimitmA);
      if (Diag::ACK) DIAG(F("ACK baseline=%d/%dmA Threshold=%d/%dmA Duration between %dus and %dus"),
			  baseline,motorDriver->raw2mA(baseline),
			  ackThreshold,motorDriver->raw2mA(ackThreshold),
                          minAckPulseDuration, maxAckPulseDuration);
}

void DCCWaveform::setAckPending() {
      if (isMainTrack) return; 
      ackMaxCurrent=0;
      ackPulseStart=0;
      ackPulseDuration=0;
      ackDetected=false;
      ackCheckStart=millis();
      numAckSamples=0;
      numAckGaps=0;
      ackPending=true;  // interrupt routines will now take note
}

byte DCCWaveform::getAck() {
      if (ackPending) return (2);  // still waiting
      if (Diag::ACK) DIAG(F("%S after %dmS max=%d/%dmA pulse=%duS samples=%d gaps=%d"),ackDetected?F("ACK"):F("NO-ACK"), ackCheckDuration,
			  ackMaxCurrent,motorDriver->raw2mA(ackMaxCurrent), ackPulseDuration, numAckSamples, numAckGaps);
      if (ackDetected) return (1); // Yes we had an ack
      return(0);  // pending set off but not detected means no ACK.   
}

void DCCWaveform::checkAck() {
    // This function operates in interrupt() time so must be fast and can't DIAG 
    if (sentResetsSincePacket > 6) {  //ACK timeout
        ackCheckDuration=millis()-ackCheckStart;
        ackPending = false;
        return; 
    }
      
    int current=motorDriver->getCurrentRaw();
    numAckSamples++;
    if (current > ackMaxCurrent) ackMaxCurrent=current;
    // An ACK is a pulse lasting between minAckPulseDuration and maxAckPulseDuration uSecs (refer @haba)
        
    if (current>ackThreshold) {
       if (trailingEdgeCounter > 0) {
	 numAckGaps++;
	 trailingEdgeCounter = 0;
       }
       if (ackPulseStart==0) ackPulseStart=micros();    // leading edge of pulse detected
       return;
    }
    
    // not in pulse
    if (ackPulseStart==0) return; // keep waiting for leading edge 
    
    // if we reach to this point, we have
    // detected trailing edge of pulse
    if (trailingEdgeCounter == 0) {
      ackPulseDuration=micros()-ackPulseStart;
    }

    // but we do not trust it yet and return (which will force another
    // measurement) and first the third time around with low current
    // the ack detection will be finalized. 
    if (trailingEdgeCounter < 2) {
      trailingEdgeCounter++;
      return;
    }
    trailingEdgeCounter = 0;

    if (ackPulseDuration>=minAckPulseDuration && ackPulseDuration<=maxAckPulseDuration) {
        ackCheckDuration=millis()-ackCheckStart;
        ackDetected=true;
        ackPending=false;
        transmitRepeats=0;  // shortcut remaining repeat packets 
        return;  // we have a genuine ACK result
    }      
    ackPulseStart=0;  // We have detected a too-short or too-long pulse so ignore and wait for next leading edge 
}