/*
 *  © 2021, Chris Harlow & David Cutting. All rights reserved.
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


/* This timer class is used to manage the single timer required to handle the DCC waveform.
 *  All timer access comes through this class so that it can be compiled for 
 *  various hardware CPU types. 
 *  
 *  DCCEX works on a single timer interrupt at a regular 58uS interval.
 *  The DCCWaveform class generates the signals to the motor shield  
 *  based on this timer. 
 *  
 *  If the motor drivers are BOTH configured to use the correct 2 pins for the architecture,
 *  (see isPWMPin() function. )
 *  then this allows us to use a hardware driven pin switching arrangement which is
 *  achieved by setting the duty cycle of the NEXT clock interrupt to 0% or 100% depending on 
 *  the required pin state. (see setPWM())  
 *  This is more accurate than the software interrupt but at the expense of 
 *  limiting the choice of available pins. 
 *  Fortunately, a standard motor shield on a Mega uses pins that qualify for PWM... 
 *  Other shields may be jumpered to PWM pins or run directly using the software interrupt.
 *  
 *  Because the PWM-based waveform is effectively set half a cycle after the software version,
 *  it is not acceptable to drive the two tracks on different methiods or it would cause
 *  problems for <1 JOIN> etc.
 *  
 */

#include "DCCTimer.h"
const int DCC_SIGNAL_TIME=58;  // this is the 58uS DCC 1-bit waveform half-cycle 
const long CLOCK_CYCLES=(F_CPU / 1000000 * DCC_SIGNAL_TIME) >>1;

INTERRUPT_CALLBACK interruptHandler=0;

#ifdef ARDUINO_ARCH_MEGAAVR
  // Arduino unoWifi Rev2 and nanoEvery architectire 
  
  void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
    interruptHandler=callback;
    noInterrupts(); 
    ADC0.CTRLC = (ADC0.CTRLC & 0b00110000) | 0b01000011;  // speed up analogRead sample time   
    TCB0.CTRLB = TCB_CNTMODE_INT_gc & ~TCB_CCMPEN_bm; // timer compare mode with output disabled
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc; //   8 MHz ~ 0.125 us      
    TCB0.CCMP =  CLOCK_CYCLES -1;  // 1 tick less for timer reset
    TCB0.INTFLAGS = TCB_CAPT_bm; // clear interrupt request flag
    TCB0.INTCTRL = TCB_CAPT_bm;  // Enable the interrupt
    TCB0.CNT = 0;
    TCB0.CTRLA |= TCB_ENABLE_bm;  // start
    interrupts();
  }

  // ISR called by timer interrupt every 58uS
  ISR(TCB0_INT_vect){
    TCB0.INTFLAGS = TCB_CAPT_bm;
    interruptHandler();
  }

  bool DCCTimer::isPWMPin(byte pin) {
       (void) pin; 
       return false;  // TODO what are the relevant pins? 
  }
  
  bool DCCTimer::isPWMPin(byte pin) {
       (void) pin; 
       return false;  // TODO what are the relevant pins? 
  }

 void DCCTimer::setPWM(byte pin, bool high) {
    (void) pin;
    (void) high;
    // TODO what are the relevant pins?
 }

  void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
    memcpy(mac,(void *) &SIGROW.SERNUM0,6);  // serial number
    mac[0] &= 0xFE;
    mac[0] |= 0x02;
  }

#elif defined(TEENSYDUINO)
    //railcom timer facility
    #define TIMER4_A_PIN   6
    #define TIMER4_B_PIN   7
    #define TIMER4_C_PIN   8


  IntervalTimer myDCCTimer;

  void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
    interruptHandler=callback;

  myDCCTimer.begin(interruptHandler, DCC_SIGNAL_TIME);

  }

  bool DCCTimer::isPWMPin(byte pin) {
       //Teensy: digitalPinHasPWM, todo
      (void) pin;
       return false;  // TODO what are the relevant pins? 
  }

  bool DCCTimer::isRailcomPin(byte pin) {
	  return
       #ifdef TIMER4_A_PIN 
              pin==TIMER4_A_PIN ||
              pin==TIMER4_B_PIN ||
              pin==TIMER4_C_PIN ||
       #endif       
              false;
  }

 void DCCTimer::setPWM(byte pin, bool high) {
    // TODO what are the relevant pins?
    (void) pin;
    (void) high;
}

  void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
#if defined(__IMXRT1062__)  //Teensy 4.0 and Teensy 4.1
    uint32_t m1 = HW_OCOTP_MAC1;
    uint32_t m2 = HW_OCOTP_MAC0;
    mac[0] = m1 >> 8;
    mac[1] = m1 >> 0;
    mac[2] = m2 >> 24;
    mac[3] = m2 >> 16;
    mac[4] = m2 >> 8;
    mac[5] = m2 >> 0;
#else
    read_mac(mac);
#endif
  }

#if !defined(__IMXRT1062__)
  void DCCTimer::read_mac(byte mac[6]) {
    read(0xe,mac,0);
    read(0xf,mac,3);
  }

// http://forum.pjrc.com/threads/91-teensy-3-MAC-address
void DCCTimer::read(uint8_t word, uint8_t *mac, uint8_t offset) {
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF));

  *(mac+offset) =   FTFL_FCCOB5;       // collect only the top three bytes,
  *(mac+offset+1) = FTFL_FCCOB6;       // in the right orientation (big endian).
  *(mac+offset+2) = FTFL_FCCOB7;       // Skip FTFL_FCCOB4 as it's always 0.
}
#endif

#else 
  // Arduino nano, uno, mega etc
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define TIMER1_A_PIN   11
    #define TIMER1_B_PIN   12
    #define TIMER1_C_PIN   13
    //railcom timer facility
    #define TIMER4_A_PIN   6
    #define TIMER4_B_PIN   7
    #define TIMER4_C_PIN   8
 #else
   #define TIMER1_A_PIN   9
   #define TIMER1_B_PIN   10
#endif

  void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
    interruptHandler=callback;
    noInterrupts();          
    ADCSRA = (ADCSRA & 0b11111000) | 0b00000100;   // speed up analogRead sample time 
    TCCR1A = 0;
    ICR1 = CLOCK_CYCLES;
    TCCR1B = _BV(WGM13) | _BV(CS10);     // Mode 8, clock select 1
    TIMSK1 = _BV(TOIE1); // Enable Software interrupt
    TCNT1 = 0;   
    
    #if defined(TIMER4_A_PIN)
      //railcom timer facility
      TCCR4A = 0;
      ICR4 = CLOCK_CYCLES;
      TCCR4B = _BV(WGM43) | _BV(CS40);     // Mode 8, clock select 1
      TIMSK4 = 0; // Disable Software interrupt
      delayMicroseconds(DCC_SIGNAL_TIME/2);  
      TCNT4 = 0;  // this timer fires half cycle after Timer 1 (no idea why /4 !)     
      #endif
    
    interrupts();
  }

// ISR called by timer interrupt every 58uS
  ISR(TIMER1_OVF_vect){ interruptHandler(); }

// Alternative pin manipulation via PWM control.
  bool DCCTimer::isPWMPin(byte pin) {
       return pin==TIMER1_A_PIN 
           || pin==TIMER1_B_PIN
       #ifdef TIMER1_C_PIN 
           || pin==TIMER1_C_PIN
       #endif       
       ;
  }
// Alternative pin manipulation via PWM control.
  bool DCCTimer::isRailcomPin(byte pin) {
       return  
       #ifdef TIMER4_A_PIN 
              pin==TIMER4_A_PIN ||
              pin==TIMER4_B_PIN ||
              pin==TIMER4_C_PIN ||
       #endif       
              false;
  }

 void DCCTimer::setPWM(byte pin, bool high) {
    uint16_t val=high?1024:0;
    if (pin==TIMER1_A_PIN) {
      TCCR1A |= _BV(COM1A1);
      OCR1A= val;
    }
    else if (pin==TIMER1_B_PIN) { 
      TCCR1A |= _BV(COM1B1);
      OCR1B= val;
    }
 #ifdef TIMER1_C_PIN 
    else if (pin==TIMER1_C_PIN) { 
      TCCR1A |= _BV(COM1C1);
      OCR1C= val;
    }
 #endif       
 #ifdef TIMER4_A_PIN 
    else if (pin==TIMER4_A_PIN) { 
      TCCR4A |= _BV(COM4A1); 
      OCR4A= val;
    }
    else if (pin==TIMER4_B_PIN) { 
      TCCR4A |= _BV(COM4B1); 
      OCR4B= val;
    }
    else if (pin==TIMER4_C_PIN) { 
      TCCR4A |= _BV(COM4C1); 
      OCR4C= val;
    }
 #endif       
 }


  #include <avr/boot.h> 
  void DCCTimer::getSimulatedMacAddress(byte mac[6]) {
    for (byte i=0; i<6; i++) {
      mac[i]=boot_signature_byte_get(0x0E + i);
    }
    mac[0] &= 0xFE;
    mac[0] |= 0x02;

  }

#endif
