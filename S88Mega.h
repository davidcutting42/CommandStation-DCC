/*
*	S88Mega.h
*	Günther Simon
*	29.05.2021
* 
*	S88Mega.cpp and S88Mega.h are the source files for supporting the common S88 bus sensors.
* 
*	You can choose to use 
		* the standard-loop, so you do need an extra timer / int and can use it for other things but the loop may run with variying speed (inacurrate)
		* the timer based solution. Just comment in the line
			#define S88_USE_TIMER 
		  in the config.h file.
	You can connect the pins of the parallel port of the S88 sensors to the pins on the Arduino. You can change the pins by changing
	the defines S88_PORTIN and S88_PORTOUT. 

	Here are the connection details:
	ParallelPortPin  Meaning  Arduino PORTA PORTC PORTL
	2                Clock             26    33    45
	3                Load              27    32    44
	4                Reset             28    31    43
	10               Bus0              22    37    49
	11               Bus1              23    36    48
	12               Bus2              24    35    47
	13               Bus3              25    34    46
	14               5V                5V -> 
*/

//Define from config.h
#ifdef S88_MEGA

//duplicate include guard
#ifndef __S88_Mega_h 
#define __S88_Mega_h

#include <Arduino.h>
#pragma once

#ifdef S88_USE_TIMER
// We use the Arduino Timer 5
ISR(TIMER5_COMPA_vect);

#define TIMER5_25us			OCR5A = 399;TCCR5B |= ((1 << CS50) | (1 << WGM52))					// compare 399, prescaler 1		25us
#define TIMER5_50us			OCR5A = 799;TCCR5B |= ((1 << CS50) | (1 << WGM52))					// compare 799, Prescaler 1		50us
#define TIMER5_100us		OCR5A = 1599;TCCR5B |= ((1 << CS50) | (1 << WGM52))					// compare 1599, Prescaler 1	100us
#define TIMER5_200us		OCR5A = 3199;TCCR5B |= ((1 << CS50) | (1 << WGM52))					// compare 3199, Prescaler 1	200us
#define TIMER5_400us		OCR5A = 6399;TCCR5B |= ((1 << CS50) | (1 << WGM52))					// compare 6399, Prescaler 1	400us
#define TIMER5_6400us		OCR5A = 15999;TCCR5B |= ((1 << CS50) | (1 << WGM52))				// compare 15999, Prescaler 1	1ms
#endif

// Port input bits 0 = Bus0, 1 =  Bus1, 2 = Bus2, 3 = Bus3
// Port output bits 4 = Clock, 5 = Load, 6 = Reset, 7 = Contoll LED (LED ON = L, LED OFF = H)
#define S88_PORTIN    PINA    //  (Data Input)
#define S88_PORTOUT   PORTA   //  (Data Output)
#define S88_PORTDIR   DDRA    //  (Data Direction)
#define S88AdrBase    100     //  Addressbase Bus0 = 100 - ..., Bus1 = 200 - ..., Bus2 = 300 - ..., Bus3 = 400 - ...
#define S88_CHAIN_MAX 16	  // max number of sensors on a bus

#define S88_RESET             S88_Set_RM(B01000000)
#define S88_LOAD              S88_Set_RM(B00100000)
#define S88_LOAD_CLOCK        S88_Set_RM(B00110000)
#define S88_NOSIGNAL          S88_Set_RM(B00000000)
#define S88_CLOCK             S88_Set_RM(B00010000)
#define S88_LED_ON            portreg &= B01111111;
#define S88_LED_OFF           portreg |= B10000000;
#define S88_LED_TOGGLE        bitWrite (portreg, 7, !bitRead (portreg, 7))


typedef struct
{
	byte bus[4];
	byte buslen;
} S88_RM;

//The loop is doing one step and than continue with the next. Using enum instead of just defines
enum S88NextLoopStep {
	S88_SET_CLOCK,
	S88_SET_CLEAR_CLOCK,
	S88_SET_LOAD,
	S88_SET_LOAD_CLOCK,
	S88_SET_LOAD_NOCLOCK,
	S88_SET_AFTERLOAD,
	S88_SET_RESET,
	S88_SET_AFTER_RESET
};

class S88Mega {
public:
	//Get the only instance. Created at the first call
	static S88Mega* getInstance()
	{
		if (instance == NULL)
		{
			instance = new S88Mega();			
		}
		return instance;
	}	
	void S88Mega::Timer5_Init(void);
	void S88Mega::Timer5_Off(void);
	void S88Mega::S88_Set_RM(byte value);
	void S88Mega::S88_Init(byte bus0len, byte bus1len, byte bus2len, byte bus3len);
	void S88Mega::S88_Status(void);
	void S88Mega::S88_CheckChanges(Print* stream);
	boolean S88Mega::is_rm(byte rmindex, byte busindex);
	void S88Mega::S88_Read(void);	
	void S88Mega::loop();

private:
	static S88Mega* instance;
	byte portreg;
	//The lower 4 bytes are data of the bux 0-3, the upper 4 bytes remember if the value has changed
	byte RmBytes[S88_CHAIN_MAX];	
	//Index of the next sensor which will be processed
	byte InIndex;
	S88_RM rm;	
	byte ledcounter;
	S88NextLoopStep eNextLoopStep = S88_SET_CLOCK;
};

#endif // __S88_Mega_h

#endif S88_MEGA
