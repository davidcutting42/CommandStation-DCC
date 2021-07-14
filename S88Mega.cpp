//Define from config.h
#if __has_include ( "config.h")
#include "config.h"
#else
#warning config.h not found.Using defaults from config.example.h
#include "config.example.h"
#endif

#ifdef S88_MEGA

#include "S88Mega.h"
#include "DIAG.h"

S88Mega* S88Mega::instance = NULL;
void S88Mega::S88_Init(byte bus0len, byte bus1len, byte bus2len, byte bus3len)
{
	byte i;
	Timer5_Off();
	portreg = 0;
	ledcounter = 0;
	rm.bus[0] = bus0len;
	rm.bus[1] = bus1len;
	rm.bus[2] = bus2len;
	rm.bus[3] = bus3len;
	rm.buslen = 0;
	for (i = 0; i < 4; i++) {
		if (rm.bus[i] > S88_CHAIN_MAX) rm.bus[i] = S88_CHAIN_MAX;
		if ((rm.bus[i] > 0) && (rm.bus[i] < 8)) rm.bus[i] = 8;
		if (rm.bus[i] > rm.buslen) rm.buslen = rm.bus[i];		
	}
	memset(RmBytes, 0x00, rm.buslen);	
	eNextLoopStep = S88_SET_LOAD;
	InIndex = 0;
	S88_PORTDIR = B11110000;    // bit 0 - 3 = input , bit 4 - 7 = output
	S88_PORTOUT = B10000000;    // LED aus, Reset aus, Load aus, Clock aus 
	DIAG(F("S88 Initalized"));	
	for (i = 0; i < 4; i++) {
		DIAG(F("Bus length %d = %d"), i, rm.bus[i]);
	}
	DIAG(F("Bus length max = %d"), rm.buslen);
	Timer5_Init();
}


void S88Mega::Timer5_Init(void)
{
#ifdef S88_USE_TIMER
	noInterrupts();
	Timer5_Off();
	TIMER5_100us;	// compare 1599, Prescaler 1	100us
	//the following lines are alternative timers if your sensors need special timings
	//	TIMER5_25us;	// compare 399, prescaler 1		25us
	//	TIMER5_50us;	// compare 799, Prescaler 1		50us
	//	TIMER5_200us;	// compare 3199, Prescaler 1	200us
	//	TIMER5_400us;	// compare 6399, Prescaler 1	400us
	//	TIMER5_6400us;	// compare 15999, Prescaler 1	1ms
	TIMSK5 |= (1 << OCIE5A);
	DIAG(F("Timer5 initialized 100us for S88 scan"));
	//	Serial.println("Timer1 initialized 100us");
	interrupts();
#endif 
}
#ifdef S88_USE_TIMER

ISR(TIMER5_COMPA_vect)
{
	S88Mega* instance = S88Mega::getInstance();
	if (instance != NULL) {
		instance->loop();
	}
}
#endif

void S88Mega::Timer5_Off(void)
{
#ifdef S88_USE_TIMER
	TCCR5A = 0;
	TCCR5B = 0;
	TCNT5 = 0;
	TIMSK5 = 0;
#endif
}

void S88Mega::S88_Set_RM(byte value)
{
	portreg &= B10001111;
	portreg |= value;
	S88_PORTOUT = portreg;
}

boolean S88Mega::is_rm(byte rmindex, byte busindex)
{
	if (!rm.bus[busindex]) return (false);
	if (rm.bus[busindex] < rmindex) return (false);
	return (true);
}

void S88Mega::S88_Read(void)
{
	byte indata = S88_PORTIN & 0x0F;
	for (byte i = 0; i < 4; i++)
	{
		if (is_rm(InIndex, i))
		{
			if (bitRead(indata, i) != bitRead(RmBytes[InIndex], i))
			{
				bitWrite(RmBytes[InIndex], i, bitRead(indata, i));
				bitSet(RmBytes[InIndex], i + 4);
			}
		}
	}
}
void S88Mega::S88_CheckChanges(Print* stream)
{
	boolean rd;
	word addr;
	for (byte i = 0; i < rm.buslen; i++)
	{
		for (byte x = 0; x < 4; x++)
		{
			if (is_rm(i, x))
			{
				if (bitRead(RmBytes[i], x + 4))
				{
					addr = i + (S88AdrBase * x) + S88AdrBase;
					bitClear(RmBytes[i], x + 4);
					rd = bitRead(RmBytes[i], x);
					if (stream != NULL) {
						StringFormatter::send(stream, F("<%c %d>"), rd ? 'Q' : 'q', addr);
					}
				}
			}
		}
	}
}
void S88Mega::S88_Status(void)
{
	for (byte i = 0; i < rm.buslen; i++)
	{
		for (byte x = 0; x < 4; x++)
		{
			if (is_rm(i, x)) bitSet(RmBytes[i], x + 4);
		}
	}
}
//overloading the ++ operator, so you can increment an enum
inline S88NextLoopStep& operator++(S88NextLoopStep& currentValue, int)
{
	if (currentValue == S88_SET_AFTER_RESET) {
		currentValue = static_cast<S88NextLoopStep>(0);
		return currentValue;
	}
	const int i = static_cast<int>(currentValue);
	currentValue = static_cast<S88NextLoopStep>(i + 1);
	return currentValue;
}

void S88Mega::loop() {
	switch (eNextLoopStep)
	{
	case S88_SET_CLOCK:
		S88_CLOCK;
		eNextLoopStep++;
		break;
	case S88_SET_CLEAR_CLOCK:
		S88_NOSIGNAL;
		S88_Read();
		InIndex++;
		if (InIndex >= rm.buslen)
		{
			eNextLoopStep = S88_SET_LOAD;
			return;
		}
		eNextLoopStep = S88_SET_CLOCK;
		break;
	case S88_SET_LOAD:
		ledcounter++;
		if (ledcounter > 200)
		{
			ledcounter = 0;
			S88_LED_TOGGLE;
		}
		InIndex = 0;
		S88_LOAD;
		eNextLoopStep++;
		break;
	case S88_SET_LOAD_CLOCK:
		S88_LOAD_CLOCK;
		eNextLoopStep++;
		break;
	case S88_SET_LOAD_NOCLOCK:
		S88_LOAD;
		eNextLoopStep++;
		break;
	case S88_SET_AFTERLOAD:
		S88_NOSIGNAL;
		eNextLoopStep++;
		break;
	case S88_SET_RESET:
		InIndex = 0;		
		S88_Read();
		InIndex++;
		S88_RESET;
		eNextLoopStep++;
		break;
	case S88_SET_AFTER_RESET:
		S88_NOSIGNAL;
		eNextLoopStep = S88_SET_CLOCK;
		break;
	default:
		S88_NOSIGNAL;
		eNextLoopStep = S88_SET_LOAD;
		break;
	}
}


#endif