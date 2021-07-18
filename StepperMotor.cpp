
#include "Arduino.h"
#include "StepperMotor.h"
#include <avr/wdt.h>
#include "wiring_private.h"

#define STEPPER_MOTOR_DEBUG 1


extern bool g_bautoStopCheck;

extern HardwareSerial *cmd_port;


StepperMotor::StepperMotor(int pulsepin, int dirpin)
{
	// Arduino pins for the motor control pulse
	
	this->PulsePin = pulsepin;
	this->DirPin = dirpin;
	this->Timer = digitalPinToTimer(PulsePin);
	this->Position = 0;

	setFrequence(this->Frequence);

	digitalWrite(this->PulsePin, MOTOR_OUTPUT_LOW);
	setDirection(MOTOR_CW);
	pinMode(this->PulsePin, OUTPUT);
	pinMode(this->DirPin, OUTPUT);

	for(int i=0; i<SENSOR_PIN_TOTAL; i++)
	{
		Sensor[i][MOTOR_CW].Pin = -1;
		Sensor[i][MOTOR_CCW].Pin = -1;
	}
	Sensor[SENSOR_PIN_STOP][MOTOR_CW].Pin = -1;
	Sensor[SENSOR_PIN_STOP][MOTOR_CCW].Pin = -1;
	Sensor[SENSOR_PIN_TOGGLE][MOTOR_CW].Pin = -1;
	Sensor[SENSOR_PIN_TOGGLE][MOTOR_CCW].Pin = -1;
	
	AccelerateInfo.stepscnt = 0;
	AccelerateInfo.startms = millis();
	AccelerateInfo.endms = millis();
	DecelerateInfo.stepscnt = 0;
	DecelerateInfo.startms = millis();
	DecelerateInfo.endms = millis();
#if STEPPER_MOTOR_DEBUG
	cmd_port->println("PlusePin: " + String(this->PulsePin) + ", DirPin: " + String(this->DirPin));
#endif
}
StepperMotor::StepperMotor(int pulsepin, int dirpin, unsigned long resolution)
{
	// Arduino pins for the motor control pulse
	this->PulsePin = pulsepin;
	this->DirPin = dirpin;
	this->Timer = digitalPinToTimer(PulsePin);
	this->Position = 0;

	setFrequence(this->Frequence);

	digitalWrite(this->PulsePin, MOTOR_OUTPUT_LOW);
	setDirection(MOTOR_CW);
	pinMode(this->PulsePin, OUTPUT);
	pinMode(this->DirPin, OUTPUT);

	for(int i=0; i<SENSOR_PIN_TOTAL; i++)
	{
		Sensor[i][MOTOR_CW].Pin = -1;
		Sensor[i][MOTOR_CCW].Pin = -1;
	}
	
	AccelerateInfo.stepscnt = 0;
	AccelerateInfo.startms = millis();
	AccelerateInfo.endms = millis();
	DecelerateInfo.stepscnt = 0;
	DecelerateInfo.startms = millis();
	DecelerateInfo.endms = millis();
	this->Resolution = resolution;
#if STEPPER_MOTOR_DEBUG
	cmd_port->println("PlusePin: " + String(this->PulsePin) + ", DirPin: " + String(this->DirPin));
	cmd_port->println("Resolution: " + String(this->Resolution));
#endif
}
StepperMotor::StepperMotor(int pulsepin, int dirpin, unsigned long resolution, unsigned long frequence)
{
	// Arduino pins for the motor control pulse
	this->PulsePin = pulsepin;
	this->DirPin = dirpin;
	this->Timer = digitalPinToTimer(PulsePin);

	setFrequence(this->Frequence);

	digitalWrite(this->PulsePin, MOTOR_OUTPUT_LOW);
	setDirection(MOTOR_CW);
	pinMode(this->PulsePin, OUTPUT);
	pinMode(this->DirPin, OUTPUT);

	for(int i=0; i<SENSOR_PIN_TOTAL; i++)
	{
		Sensor[i][MOTOR_CW].Pin = -1;
		Sensor[i][MOTOR_CCW].Pin = -1;
	}
	
	AccelerateInfo.stepscnt = 0;
	AccelerateInfo.startms = millis();
	AccelerateInfo.endms = millis();
	DecelerateInfo.stepscnt = 0;
	DecelerateInfo.startms = millis();
	DecelerateInfo.endms = millis();
	this->Resolution = resolution;
	setFrequence(frequence);
#if STEPPER_MOTOR_DEBUG
	cmd_port->println("PlusePin: " + String(this->PulsePin) + ", DirPin: " + String(this->DirPin));
	cmd_port->println("Resolution: " + String(this->Resolution) + ", Frequence: " + String(this->Frequence));
#endif
}
void StepperMotor::setFrequence(long freq)
{
	freq = abs(freq);
	if(freq < OCRAXA_TABLE_FREQ_BASE)
		freq = OCRAXA_TABLE_FREQ_BASE;
	else if(freq > OCRAXA_TABLE_FREQ_MAX)
		freq = OCRAXA_TABLE_FREQ_MAX;

	
	this->Frequence = freq;

#if 0
	if(this->Resolution!=0)
	{
		this->RPM = (float)(this->Frequence*60)/this->Resolution + 0.5;	//四捨五入取整數
	}
#endif
}
void StepperMotor::setFrequenceStartup(long freq)
{
	freq = abs(freq);
	if(freq < ACCELERATE_TABLE_FREQ_STARTUP_MIN)
		freq = ACCELERATE_TABLE_FREQ_STARTUP_MIN;
	else if(freq > OCRAXA_TABLE_FREQ_MAX)
		freq = OCRAXA_TABLE_FREQ_MAX;
	
	this->FrequenceStartup = freq;
}

unsigned long StepperMotor::getFrequenceNow(void)
{
  return FrequenceNow;
}

void StepperMotor::setFrequenceNow(long freq)
{
	freq = abs(freq);
	if(freq < OCRAXA_TABLE_FREQ_BASE)
		freq = OCRAXA_TABLE_FREQ_BASE;
	else if(freq > OCRAXA_TABLE_FREQ_MAX)
		freq = OCRAXA_TABLE_FREQ_MAX;
	
	this->FrequenceNow = freq;
}

unsigned long StepperMotor::getFrequence(void)
{
  return Frequence;
}
unsigned long StepperMotor::getFrequenceStartup(void)
{
  return FrequenceStartup;
}

void StepperMotor::setRotateMode(int mode)
{
	switch(mode)
	{
		case ROTATE_MODE_KEEP_SPEED:
		case ROTATE_MODE_ACCELEARTION:
		case ROTATE_MODE_DECELEARTION:
			this->RotateMode = mode;
			break;
	}

}
int StepperMotor::getRotateMode(void)
{
  return RotateMode;
}

void StepperMotor::setAccelerateTime(int accms)
{
	
	this->AccelerateTime = abs(accms);

}
int StepperMotor::getAccelerateTime(void)
{
  return AccelerateTime;
}

void StepperMotor::setRPM(unsigned long rpm)
{
	if(this->Resolution!=0)
	{
		this->RPM = rpm;
		this->Frequence = (unsigned long)((this->Resolution * rpm) / 60);
	}
}

unsigned long StepperMotor::getRPM(void)
{
  return RPM;
}

int StepperMotor::getPulsePin()
{
	return PulsePin;
}
int StepperMotor::getDirPin()
{
	return DirPin;
}
unsigned long StepperMotor::getResolution()
{
	return Resolution;
}

void StepperMotor::setPosition(long position)
{
	Position = position;
}

long StepperMotor::getPosition()
{
	return Position;
}

int StepperMotor::getDirection()
{
	return Direction;
}

int StepperMotor::getTimer()
{
	return Timer;
}
int StepperMotor::getState()
{
	return State;
}
int StepperMotor::getVectNum()
{
	return Vect_Num;
}

void StepperMotor::setStopPin(int pin, int hl, uint8_t toggletimes)
{
	setSensorPin(SENSOR_PIN_TOGGLE, pin, hl, pin, hl, SENSOR_PIN_MODE_STOP | SENSOR_PIN_MODE_ONCE, toggletimes);
}

void StepperMotor::setStopPin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes)
{
	setSensorPin(SENSOR_PIN_TOGGLE, pin, hl, ccwpin, ccwhl, SENSOR_PIN_MODE_STOP | SENSOR_PIN_MODE_ONCE, toggletimes);
}

void StepperMotor::setStopRangePin(int pin, int hl, uint8_t toggletimes, int range)
{
	setSensorPin(SENSOR_PIN_STOP_RANGE, pin, hl, pin, hl, SENSOR_PIN_MODE_STOP | SENSOR_PIN_MODE_RANGE | SENSOR_PIN_MODE_ONCE, toggletimes, range);
}

void StepperMotor::setStopRangePin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes, int range)
{
	setSensorPin(SENSOR_PIN_STOP_RANGE, pin, hl, ccwpin, ccwhl, SENSOR_PIN_MODE_STOP | SENSOR_PIN_MODE_RANGE | SENSOR_PIN_MODE_ONCE, toggletimes, range);
}

void StepperMotor::setSlowdownPin(int pin, int hl, uint8_t toggletimes)
{
	setSensorPin(SENSOR_PIN_SLOWDOWN, pin, hl, pin, hl, SENSOR_PIN_MODE_SLOWDOWN | SENSOR_PIN_MODE_ONCE, toggletimes);
}

void StepperMotor::setSlowdownPin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes)
{
	setSensorPin(SENSOR_PIN_SLOWDOWN, pin, hl, ccwpin, ccwhl, SENSOR_PIN_MODE_SLOWDOWN | SENSOR_PIN_MODE_ONCE, toggletimes);
}

void StepperMotor::setSlowdownStopPin(int pin, int hl, uint8_t toggletimes)
{
	setSensorPin(SENSOR_PIN_SLOWDOWN_STOP, pin, hl, pin, hl, SENSOR_PIN_MODE_STOP | SENSOR_PIN_MODE_SLOWDOWN | SENSOR_PIN_MODE_ONCE, toggletimes);
}

void StepperMotor::setSlowdownStopPin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes)
{
	setSensorPin(SENSOR_PIN_SLOWDOWN_STOP, pin, hl, ccwpin, ccwhl, SENSOR_PIN_MODE_STOP | SENSOR_PIN_MODE_SLOWDOWN | SENSOR_PIN_MODE_ONCE, toggletimes);
}

void StepperMotor::setLimitPin(int pin, int hl)
{
	setSensorPin(SENSOR_PIN_LIMIT, pin, hl, pin, hl, SENSOR_PIN_MODE_STOP, 0);
}

void StepperMotor::setLimitPin(int pin, int hl, int ccwpin, int ccwhl)
{
	setSensorPin(SENSOR_PIN_LIMIT, pin, hl, ccwpin, ccwhl, SENSOR_PIN_MODE_STOP, 0);
}


void StepperMotor::setSensorPin(int index, int pin, int hl, int ccwpin, int ccwhl, uint8_t mode, uint8_t toggletimes, int range)
{
	if(index >= SENSOR_PIN_TOTAL)
		return;
	Sensor[index][MOTOR_CW].Pin = pin;
	Sensor[index][MOTOR_CW].State = hl;
	Sensor[index][MOTOR_CW].Mode = mode;
	Sensor[index][MOTOR_CW].ToggleTimes = toggletimes;
	Sensor[index][MOTOR_CW].Range = range;
	
	Sensor[index][MOTOR_CCW].Pin = ccwpin;
	Sensor[index][MOTOR_CCW].State = ccwhl;
	Sensor[index][MOTOR_CCW].Mode = mode;
	Sensor[index][MOTOR_CCW].ToggleTimes = toggletimes;
	Sensor[index][MOTOR_CCW].Range = range;
	
}

void StepperMotor::setDirection(int direction)
{
	Direction = direction;
	if(direction == MOTOR_CW)
	{
		//level trigger for Motor CW(���ɰw)
		digitalWrite(DirPin, MOTOR_CCW);
		digitalWrite(DirPin, MOTOR_CW);
	}
	else
	{
		//level trigger for Motor CCW(�f�ɰw)
		digitalWrite(DirPin, MOTOR_CW);
		digitalWrite(DirPin, MOTOR_CCW);
	}
	Direction = direction;
}

unsigned int StepperMotor::Steps(long numberofsteps)
{
	unsigned int result = 0;
	if(numberofsteps == 0)
		return result;
	TargetPosition = Position + numberofsteps;
	this->State = MOTOR_STATE_MOVE_TO_POSITION;
	if(TargetPosition < Position)
		result = Speed(-this->Frequence, this->FrequenceStartup, this->AccelerateTime);
	else
		result = Speed(this->Frequence, this->FrequenceStartup, this->AccelerateTime);


 return result;
}

unsigned int StepperMotor::Steps(long numberofsteps, long frequence)
{
	unsigned int result = 0;
	if(numberofsteps == 0)
		return result;
	TargetPosition = Position + numberofsteps;
	setFrequence(frequence);
	this->State = MOTOR_STATE_MOVE_TO_POSITION;
	if(TargetPosition < Position)
		result = Speed(-this->Frequence, this->FrequenceStartup, this->AccelerateTime);
	else
		result = Speed(this->Frequence, this->FrequenceStartup, this->AccelerateTime);

 return result;
}

unsigned int StepperMotor::MoveTo(long targetpositon)
{
	return MoveTo(targetpositon, this->Frequence);
}

unsigned int StepperMotor::MoveTo(long targetpositon, long frequence)
{
#if STEPPER_MOTOR_DEBUG
	cmd_port->println("MoveTo: " + String(targetpositon));
#endif
	unsigned int result = 0;
	TargetPosition = targetpositon;
	setFrequence(abs(frequence));

	if(this->State == MOTOR_STATE_STOP)
	{
		AccelerateInfo.stepscnt = 0;
		AccelerateInfo.startms = millis();
		AccelerateInfo.endms = millis();
	}
	
	DecelerateInfo.stepscnt = 0;
	DecelerateInfo.startms = millis();
	DecelerateInfo.endms = millis();
	
	this->State = MOTOR_STATE_MOVE_TO_POSITION;
	if(TargetPosition < Position)
	{
		result = Speed(-this->Frequence, this->FrequenceStartup, this->AccelerateTime);
	}
	else if(TargetPosition > Position)
	{
		result = Speed(this->Frequence, this->FrequenceStartup, this->AccelerateTime);
	}
	else
		this->State = MOTOR_STATE_STOP;


	return result;
}

unsigned int StepperMotor::MoveToStopPin(long targetpositon)
{
	return MoveToStopPin(targetpositon, this->Frequence);
}

unsigned int StepperMotor::MoveToStopPin(long targetpositon, long frequence)
{
	unsigned int result = MoveTo(targetpositon, frequence);

	if(this->State == MOTOR_STATE_MOVE_TO_POSITION)
		this->State = MOTOR_STATE_MOVE_TO_STOP_PIN;

	return result;
}


void StepperMotor::Stop(void)
{
	TargetPosition = Position;
	this->State = MOTOR_STATE_STOP;
}



long StepperMotor::getTargetPosition(void)
{
	return TargetPosition;
}


TimerReg StepperMotor::ComputeTimerCCR(int timer, long freq)
{
	TimerReg reg;
	long tableindex;
	int interpolation = 0;
	
	tableindex = freq;
	tableindex -= OCRAXA_TABLE_FREQ_BASE;
	if(OCRAXA_TABLE_INC_EXP > 0)
		interpolation = (tableindex >> (OCRAXA_TABLE_INC_EXP - 1)) & 0x01;
	tableindex >>= OCRAXA_TABLE_INC_EXP;
	reg.freq = tableindex << OCRAXA_TABLE_INC_EXP;
	reg.freq += OCRAXA_TABLE_FREQ_BASE;
	//cmd_port->println("Freq shit " + String(OCRAXA_TABLE_INC_EXP, DEC) + " ,Table " + String(tableindex, DEC)+ " =" + String(OCRxA_Table[tableindex], DEC));

	reg.OCRxA = OCRxA_Table[tableindex];

	if(interpolation == 1)	//只內插1/2
	{
	}
	
	reg.Prescale = OCRAXA_TABLE_PRESCALER;

	return reg;
}	


bool StepperMotor::SetPrescale(int timer, int prescale) 
{
	switch(timer)
	{
		// XXX fix needed for atmega8
	#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		case TIMER0A:

	#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A:
	#endif
	#if defined(TCCR0A) && defined(COM0B1)
		case TIMER0B:
	#endif
			switch(prescale)
			{
				case 1:
				{
					TCCR0A |=  _BV(WGM01)| _BV(WGM00); 
					TCCR0B = _BV(WGM02) 

| _BV(CS00); 
					break;
				}
				case 8:
				{
					TCCR0A |=  

_BV(WGM01)| _BV(WGM00); 
					TCCR0B = _BV(WGM02) 

| _BV(CS01); 
					break;
				}
				case 64:
				{
					TCCR0A |=  

_BV(WGM01)| _BV(WGM00); 
					TCCR0B = _BV(WGM02) 

| _BV(CS01) 

| _BV(CS00); 
					break;
				}
				case 256:
				{
					TCCR0A |=  

_BV(WGM01)| _BV(WGM00); 
					TCCR0B = _BV(WGM02) 

| _BV(CS02);
					break;
				}
				case 1024:
				{
					TCCR0A |=  

_BV(WGM01)| _BV(WGM00); 
					TCCR0B = _BV(WGM02) 

| _BV(CS02) 

| _BV(CS00); 
					break;
				}
			}
			break;
	#endif
	
	#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:

	#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:
	#endif

	#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:
	#endif
			switch(prescale)
			{
				case 1: 
				{
					TCCR1A |= _BV(WGM11) | _BV(WGM10); 
					TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); 
					break;
				}
				case 8: 
				{
					TCCR1A |= _BV(WGM11) | _BV(WGM10); 
					TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
					break;
				}
				case 64:
				{
					TCCR1A |= _BV(WGM11) | _BV(WGM10); 
					TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10); 
					break;
				}
				case 256:
				{
					TCCR1A |= _BV(WGM11) | _BV(WGM10); 
					TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
					break;
				}
				case 1024: 
				{
					TCCR1A |= _BV(WGM11) | _BV(WGM10); 
					TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS10);
					break;
				}
			}
			break;
	
	#endif

	#if defined(TCCR2) && defined(COM21)
		case TIMER2:

	#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2A:
	#endif

	#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2B:
	#endif
			switch(prescale)
			{
				case 1:
				{
					TCCR2A |=  _BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS20); 
					break;
				}
				case 8:
				{
					TCCR2A |=  

_BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS21); 
					break;
				}
				case 32:
				{
					TCCR2A |=  

_BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS21) | _BV(CS20);
					break;
				}
				case 64:
				{
					TCCR2A |=  

_BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS22); 
					break;
				}
				case 128:
				{
					TCCR2A |=  

_BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS22) 

| _BV(CS20); 
					break;
				}
				case 256:
				{
					TCCR2A |=  

_BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS22) | _BV(CS21);
					break;
				}
				case 1024:
				{
					TCCR2A |=  

_BV(WGM21)| _BV(WGM20); 
					TCCR2B = _BV(WGM22) 

| _BV(CS22) 

| _BV(CS21) | _BV(CS20); 
					break;
				}
			}
			break;
	
	#endif

	

	#if defined(TCCR3A) && defined(COM3A1)
		case TIMER3A:

	#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3B:
	#endif
	#if defined(TCCR3A) && defined(COM3C1)
		case TIMER3C:
	#endif
			switch(prescale)
			{
				case 1: 
				{
					TCCR3A |= _BV(WGM31) | _BV(WGM30); 
					TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30); 
					break;
				}
				case 8: 
				{
					TCCR3A |= _BV(WGM31) | _BV(WGM30); 
					TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS41);
					break;
				}
				case 64:
				{
					TCCR3A |= _BV(WGM31) | _BV(WGM30); 
					TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31) | _BV(CS30); 
					break;
				}
				case 256:
				{
					TCCR3A |= _BV(WGM31) | _BV(WGM30); 
					TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS32);
					break;
				}
				case 1024: 
				{
					TCCR3A |= _BV(WGM31) | _BV(WGM30); 
					TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS32) | _BV(CS30);
					break;
				}
			}
			break;
	#endif

	#if defined(TCCR4A)
		case TIMER4A:
		
	#if defined(TCCR4A) && defined(COM4B1)
		case TIMER4B:
	#endif
	#if defined(TCCR4A) && defined(COM4C1)
		case TIMER4C:
	#endif
	#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:				
	#endif
			switch(prescale)
			{
				case 1: 
				{
					TCCR4A |= _BV(WGM41) | _BV(WGM40); 
					TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40); 
					break;
				}
				case 8: 
				{
					TCCR4A |= _BV(WGM41) | _BV(WGM40); 
					TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);
					break;
				}
				case 64:
				{
					TCCR4A |= _BV(WGM41) | _BV(WGM40); 
					TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41) | _BV(CS40); 
					break;
				}
				case 256:
				{
					TCCR4A |= _BV(WGM41) | _BV(WGM40); 
					TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS42);
					break;
				}
				case 1024: 
				{
					TCCR4A |= _BV(WGM41) | _BV(WGM40); 
					TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS42) | _BV(CS40);
					break;
				}
			}
			break;
	#endif

						
	#if defined(TCCR5A) && defined(COM5A1)
		case TIMER5A:
	#if defined(TCCR5A) && defined(COM5B1)
		case TIMER5B:
	#endif
	#if defined(TCCR5A) && defined(COM5C1)
		case TIMER5C:
	#endif
			switch(prescale)
			{
				case 1: 
				{
					TCCR5A |= _BV(WGM51) | _BV(WGM50); 
					TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50); 
					break;
				}
				case 8: 
				{
					TCCR5A |= _BV(WGM51) | _BV(WGM50); 
					TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51);
					break;
				}
				case 64:
				{
					TCCR5A |= _BV(WGM51) | _BV(WGM50); 
					TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(CS50); 
					break;
				}
				case 256:
				{
					TCCR5A |= _BV(WGM51) | _BV(WGM50); 
					TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS52);
					break;
				}
				case 1024: 
				{
					TCCR5A |= _BV(WGM51) | _BV(WGM50); 
					TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS52) | _BV(CS50);
					break;
				}
			}
			break;
	#endif

		case NOT_ON_TIMER:
		default:
			return false;
	}
	return true;
}


bool StepperMotor::SetPWMCompareReg(int timer, unsigned short ocrxa) 
{
	switch(timer)
	{
		// XXX fix needed for atmega8
	#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		case TIMER0A:

	#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A:
	#endif
	#if defined(TCCR0A) && defined(COM0B1)
		case TIMER0B:
	#endif
			OCR0A = ocrxa;
			break;
	#endif
	
	#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:

	#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:
	#endif

	#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:
	#endif
			OCR1A = ocrxa;
			break;
	
	#endif

	#if defined(TCCR2) && defined(COM21)
		case TIMER2:

	#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2A:
	#endif

	#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2B:
	#endif
			OCR2A = ocrxa;
			break;
	
	#endif

	

	#if defined(TCCR3A) && defined(COM3A1)
		case TIMER3A:

	#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3B:
	#endif
	#if defined(TCCR3A) && defined(COM3C1)
		case TIMER3C:
	#endif
			OCR3A = ocrxa;
			break;
	#endif

	#if defined(TCCR4A)
		case TIMER4A:
		
	#if defined(TCCR4A) && defined(COM4B1)
		case TIMER4B:
	#endif
	#if defined(TCCR4A) && defined(COM4C1)
		case TIMER4C:
	#endif
	#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:				
	#endif
			OCR4A = ocrxa;
			break;
	#endif

						
	#if defined(TCCR5A) && defined(COM5A1)
		case TIMER5A:
	#if defined(TCCR5A) && defined(COM5B1)
		case TIMER5B:
	#endif
	#if defined(TCCR5A) && defined(COM5C1)
		case TIMER5C:
	#endif
			OCR5A = ocrxa;
			break;
	#endif

		case NOT_ON_TIMER:
		default:
			return false;
	}
	return true;
}

bool StepperMotor::ClearTimerCnt(int timer)
{
	switch(timer)
	{
		// XXX fix needed for atmega8
		#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
			case TIMER0A:

			#if defined(TCCR0A) && defined(COM0A1)
			case TIMER0A:
			#endif
			#if defined(TCCR0A) && defined(COM0B1)
			case TIMER0B:
			#endif
				TCNT0 = 0;
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1A1)
			case TIMER1A:
			#if defined(TCCR1A) && defined(COM1B1)
			case TIMER1B:
			#endif

			#if defined(TCCR1A) && defined(COM1C1)
			case TIMER1C:
				TCNT1 = 0;
				break;
			#endif
		#endif

		#if defined(TCCR2) && defined(COM21)
			case TIMER2:
			#if defined(TCCR2A) && defined(COM2A1)
			case TIMER2A:
			#endif

			#if defined(TCCR2A) && defined(COM2B1)
			case TIMER2B:
				TCNT2 = 0;
				break;
			#endif
		#endif

		#if defined(TCCR3A) && defined(COM3A1)
			case TIMER3A:
			#if defined(TCCR3A) && defined(COM3B1)
			case TIMER3B:
			#endif

			#if defined(TCCR3A) && defined(COM3C1)
			case TIMER3C:
				TCNT3 = 0;
				break;
			#endif
		#endif

		#if defined(TCCR4A)
			case TIMER4A:
			#if defined(TCCR4A) && defined(COM4B1)
			case TIMER4B:
			#endif

			#if defined(TCCR4A) && defined(COM4C1)
			case TIMER4C:
			#endif
				
			#if defined(TCCR4C) && defined(COM4D1)
			case TIMER4D:				
				TCNT4 = 0;
				break;
			#endif
		#endif

						
		#if defined(TCCR5A) && defined(COM5A1)
			case TIMER5A:

			#if defined(TCCR5A) && defined(COM5B1)
			case TIMER5B:
			#endif

			#if defined(TCCR5A) && defined(COM5C1)
			case TIMER5C:
				TCNT5 = 0;
				break;
			#endif
		#endif
		default:
			Vect_Num = -1;
			return false;

	}
	PWMInitCnt = 0;
	return true;
}

bool StepperMotor::turnOnTimer(int timer, int duty)
{
	switch(timer)
	{
		// XXX fix needed for atmega8
		#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
		case TIMER0A:
			// connect pwm to pin on timer 0
			sbi(TCCR0, COM00);
			OCR0 = duty; // set pwm duty
			Vect_Num = TIMER0_COMPA_vect_num;
			//TCNT0 = 0;
			bitWrite(TIMSK0, OCIE0A, 1);
			break;
		#endif

		#if defined(TCCR0A) && defined(COM0A1)
		case TIMER0A:
			// connect pwm to pin on timer 0, channel A
			sbi(TCCR0A, COM0A1);
			sbi(TCCR1A, COM1A0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR0A = duty; // set pwm duty
			Vect_Num = TIMER0_COMPA_vect_num;
			//TCNT0 = 0;
			bitWrite(TIMSK0, OCIE0A, 1);
			break;
		#endif

		#if defined(TCCR0A) && defined(COM0B1)
		case TIMER0B:
			// connect pwm to pin on timer 0, channel B
			sbi(TCCR0A, COM0B1);
			sbi(TCCR1A, COM1B0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR0B = duty; // set pwm duty
			Vect_Num = TIMER0_COMPB_vect_num;
			//TCNT0 = 0;
			bitWrite(TIMSK0, OCIE0B, 1);
			break;
		#endif

		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:
			// connect pwm to pin on timer 1, channel A
			sbi(TCCR1A, COM1A1);
			sbi(TCCR1A, COM1A0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR1A = duty; // set pwm duty
			Vect_Num = TIMER1_COMPA_vect_num;
			//TCNT1 = 0;
			bitWrite(TIMSK1, OCIE1A, 1);
			break;
		#endif

		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:
			// connect pwm to pin on timer 1, channel B
			sbi(TCCR1A, COM1B1);
			sbi(TCCR1A, COM1B0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR1B = duty; // set pwm duty
			Vect_Num = TIMER1_COMPB_vect_num;
			//TCNT1 = 0;
			bitWrite(TIMSK1, OCIE1B, 1);
			break;
		#endif

		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:
			// connect pwm to pin on timer 1, channel B
			sbi(TCCR1A, COM1C1);
			sbi(TCCR1A, COM1C0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR1C = duty; // set pwm duty
			Vect_Num = TIMER1_COMPC_vect_num;
			//TCNT1 = 0;
			bitWrite(TIMSK1, OCIE1C, 1);
			break;
		#endif

		#if defined(TCCR2) && defined(COM21)
		case TIMER2:
			// connect pwm to pin on timer 2
			sbi(TCCR2, COM21);
			sbi(TCCR2, COM20); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR2 = duty; // set pwm duty
			Vect_Num = TIMER2_COMPA_vect_num;
			//TCNT2 = 0;
			bitWrite(TIMSK2, OCIE2A, 1);
			break;
		#endif

		#if defined(TCCR2A) && defined(COM2A1)
		case TIMER2A:
			// connect pwm to pin on timer 2, channel A
			sbi(TCCR2A, COM2A1);
			sbi(TCCR2A, COM2A0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR2A = duty; // set pwm duty
			Vect_Num = TIMER2_COMPA_vect_num;
			//TCNT2 = 0;
			bitWrite(TIMSK2, OCIE2A, 1);
			break;
		#endif

		#if defined(TCCR2A) && defined(COM2B1)
		case TIMER2B:
			// connect pwm to pin on timer 2, channel B
			sbi(TCCR2A, COM2B1);
			sbi(TCCR2A, COM2B0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR2B = duty; // set pwm duty
			Vect_Num = TIMER2_COMPB_vect_num;
			//TCNT2 = 0;
			bitWrite(TIMSK2, OCIE2B, 1);
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3A1)
		case TIMER3A:
			// connect pwm to pin on timer 3, channel A
			sbi(TCCR3A, COM3A1);
			sbi(TCCR3A, COM3A0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR3A = duty; // set pwm duty
			Vect_Num = TIMER3_COMPA_vect_num;
			//TCNT3 = 0;
			bitWrite(TIMSK3, OCIE3A, 1);
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3B1)
		case TIMER3B:
			// connect pwm to pin on timer 3, channel B
			sbi(TCCR3A, COM3B1);
			sbi(TCCR3A, COM3B0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR3B = duty; // set pwm duty
			Vect_Num = TIMER3_COMPB_vect_num;
			//TCNT3 = 0;
			bitWrite(TIMSK3, OCIE3B, 1);
			break;
		#endif

		#if defined(TCCR3A) && defined(COM3C1)
		case TIMER3C:
			// connect pwm to pin on timer 3, channel C
			sbi(TCCR3A, COM3C1);
			sbi(TCCR3A, COM3C0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR3C = duty; // set pwm duty
			Vect_Num = TIMER3_COMPC_vect_num;
			//TCNT3 = 0;
			bitWrite(TIMSK3, OCIE3C, 1);
			break;
		#endif

		#if defined(TCCR4A)
		case TIMER4A:
			//connect pwm to pin on timer 4, channel A
			sbi(TCCR4A, COM4A1);
			#if defined(COM4A0)		// only used on 32U4
			cbi(TCCR4A, COM4A0);
			#endif
			OCR4A = duty;	// set pwm duty
			Vect_Num = TIMER4_COMPA_vect_num;
			//TCNT4 = 0;
			bitWrite(TIMSK4, OCIE4A, 1);
			break;
		#endif
		
		#if defined(TCCR4A) && defined(COM4B1)
		case TIMER4B:
			// connect pwm to pin on timer 4, channel B
			sbi(TCCR4A, COM4B1);
			sbi(TCCR4A, COM4B0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR4B = duty; // set pwm duty
			Vect_Num = TIMER4_COMPB_vect_num;
			//TCNT4 = 0;
			bitWrite(TIMSK4, OCIE4B, 1);
			break;
		#endif

		#if defined(TCCR4A) && defined(COM4C1)
		case TIMER4C:
			// connect pwm to pin on timer 4, channel C
			sbi(TCCR4A, COM4C1);
			sbi(TCCR4A, COM4C0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR4C = duty; // set pwm duty
			Vect_Num = TIMER4_COMPC_vect_num;
			//TCNT4 = 0;
			bitWrite(TIMSK4, OCIE4C, 1);
			break;
		#endif
			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:				
			// connect pwm to pin on timer 4, channel D
			sbi(TCCR4C, COM4D1);
			#if defined(COM4D0)		// only used on 32U4
			cbi(TCCR4C, COM4D0);
			#endif
			OCR4D = duty;	// set pwm duty
			Vect_Num = TIMER4_COMPD_vect_num;
			//TCNT4 = 0;
			bitWrite(TIMSK4, OCIE4D, 1);
			break;
		#endif

						
		#if defined(TCCR5A) && defined(COM5A1)
		case TIMER5A:
			// connect pwm to pin on timer 5, channel A
			sbi(TCCR5A, COM5A1);
			sbi(TCCR5A, COM5A0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR5A = duty; // set pwm duty
			Vect_Num = TIMER5_COMPA_vect_num;
			//TCNT5 = 0;
			bitWrite(TIMSK5, OCIE5A, 1);
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5B1)
		case TIMER5B:
			// connect pwm to pin on timer 5, channel B
			sbi(TCCR5A, COM5B1);
			sbi(TCCR5A, COM5B0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR5B = duty; // set pwm duty
			Vect_Num = TIMER5_COMPB_vect_num;
			//TCNT5 = 0;
			bitWrite(TIMSK5, OCIE5B, 1);
			break;
		#endif

		#if defined(TCCR5A) && defined(COM5C1)
		case TIMER5C:
			// connect pwm to pin on timer 5, channel C
			sbi(TCCR5A, COM5C1);
			sbi(TCCR5A, COM5C0); //Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode)
			OCR5C = duty; // set pwm duty
			Vect_Num = TIMER5_COMPC_vect_num;
			//TCNT5 = 0;
			bitWrite(TIMSK5, OCIE5C, 1);
			break;
		#endif
		default:
			Vect_Num = -1;
			return false;

	}
	return true;
}

bool StepperMotor::turnOffTimer(int timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1); bitWrite(TIMSK1, OCIE1A, 0);   break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1); bitWrite(TIMSK1, OCIE1B, 0);   break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1); bitWrite(TIMSK1, OCIE1C, 0);   break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21); bitWrite(TIMSK2, OCIE2, 0);     break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1); bitWrite(TIMSK0, OCIE0A, 0);   break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1); bitWrite(TIMSK0, OCIE0B, 0);   break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1); bitWrite(TIMSK2, OCIE2A, 0);   break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1); bitWrite(TIMSK2, OCIE2B, 0);   break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1); bitWrite(TIMSK3, OCIE3A, 0);   break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1); bitWrite(TIMSK3, OCIE3B, 0);   break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1); bitWrite(TIMSK3, OCIE3C, 0);   break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1); bitWrite(TIMSK4, OCIE4A, 0);   break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1); bitWrite(TIMSK4, OCIE4B, 0);   break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1); bitWrite(TIMSK4, OCIE4C, 0);   break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1); bitWrite(TIMSK4, OCIE4D, 0);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1); bitWrite(TIMSK5, OCIE5A, 0);   break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1); bitWrite(TIMSK5, OCIE5B, 0);   break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1); bitWrite(TIMSK5, OCIE5C, 0);   break;
		#endif
		default: return false;
	}
	
	return true;
}

bool StepperMotor::setPWMOnOff(int timer, int onoff)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   bitWrite(TIMSK1, OCIE1A, onoff);   break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   bitWrite(TIMSK1, OCIE1B, onoff);   break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   bitWrite(TIMSK1, OCIE1C, onoff);   break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   bitWrite(TIMSK2, OCIE2, onoff);     break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  bitWrite(TIMSK0, OCIE0A, onoff);   break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  bitWrite(TIMSK0, OCIE0B, onoff);   break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  bitWrite(TIMSK2, OCIE2A, onoff);   break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  bitWrite(TIMSK2, OCIE2B, onoff);   break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  bitWrite(TIMSK3, OCIE3A, onoff);   break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  bitWrite(TIMSK3, OCIE3B, onoff);   break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  bitWrite(TIMSK3, OCIE3C, onoff);   break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  bitWrite(TIMSK4, OCIE4A, onoff);   break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  bitWrite(TIMSK4, OCIE4B, onoff);   break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  bitWrite(TIMSK4, OCIE4C, onoff);   break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	bitWrite(TIMSK4, OCIE4D, onoff);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  bitWrite(TIMSK5, OCIE5A, onoff);   break;
		case  TIMER5B:  bitWrite(TIMSK5, OCIE5B, onoff);   break;
		case  TIMER5C:  bitWrite(TIMSK5, OCIE5C, onoff);   break;
		#endif
		default: return false;
	}
	
	return true;
}

int StepperMotor::getPWMOnOff(int timer)
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   return bitRead(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   return bitRead(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   return bitRead(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   return bitRead(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  return bitRead(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  return bitRead(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  return bitRead(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  return bitRead(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  return bitRead(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  return bitRead(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  return bitRead(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  return bitRead(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  return bitRead(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	return bitRead(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  return bitRead(TCCR5A, COM5A1);    break;
		case  TIMER5B:  return bitRead(TCCR5A, COM5B1);    break;
		case  TIMER5C:  return bitRead(TCCR5A, COM5C1);    break;
		#endif
		default: return 0;
	}
	
}

int StepperMotor::getLimitPinState()
{
	int hl = 0;
	if(Sensor[SENSOR_PIN_LIMIT][this->Direction].Pin != -1)
	{
		hl = digitalRead(Sensor[SENSOR_PIN_LIMIT][this->Direction].Pin);
	}
	return hl;
}

TimerReg StepperMotor::updateTimer(long freq)
{
	TimerReg reg = ComputeTimerCCR(Timer, freq);

	//cmd_port->println("Freq:" + String(reg.freq, DEC) + " OCRxA:" + String(reg.OCRxA, DEC) + " Div:" + String(reg.Prescale, DEC) + " Timer:" + String(Timer, DEC));
	if(SetPrescale(Timer, reg.Prescale))
	{
		if(SetPWMCompareReg(Timer, reg.OCRxA))
		{
			//this->State = MOTOR_STATE_ROTATE;
			if(turnOnTimer(Timer, reg.OCRxA >> 1))
			{
				//TIMSK4 |= (1<<OCIE4C);	//enable timer compare interrupt
			}
			//else
				//cmd_port->println("turnOnPWM fail.");
		}
		else
		{
			//cmd_port->println("SetPWMCompareReg fail.");
		}
	}
	else
	{
		;//cmd_port->println("SetPrescale fail.");
	}

	return reg;
}

unsigned int StepperMotor::Speed(long freq, long freqstartup, int accmsec)
{
	if(freq < 0)
	{
		freq = -freq;
		setDirection(MOTOR_CCW);
	}
	else
		setDirection(MOTOR_CW);
	setFrequenceStartup(freqstartup);

	setFrequence(freq);

	setAccelerateTime(accmsec);
	if(AccelerateTime == 0)
		FrequenceNow = Frequence;
	else
	{
		long freqdiff = Frequence - FrequenceStartup;
		if(FrequenceNow < FrequenceStartup)
			FrequenceNow = FrequenceStartup;

		if(FrequenceNow < Frequence)
			RotateMode = ROTATE_MODE_ACCELEARTION;
		else if(FrequenceNow > Frequence)
			RotateMode = ROTATE_MODE_DECELEARTION;
		else
			RotateMode = ROTATE_MODE_KEEP_SPEED;
			
		FrequenceIncIndex = (int)((Frequence - FrequenceStartup) / 1000);
		FrequenceIncValue = AccelerateTable[FrequenceIncIndex];
		FrequenceIncValue *= 1000;
		FrequenceIncValue /= AccelerateTime;
	}
	if(State == MOTOR_STATE_STOP)
		State = MOTOR_STATE_ROTATE;
	ClearTimerCnt(Timer);
	TimerReg reg = updateTimer(FrequenceNow);

	AccelerateInfo.stepscnt = 0;
	AccelerateInfo.startms = millis();
	AccelerateInfo.endms = millis();
	
	
	return reg.freq;
}

void StepperMotor::Slowdown()
{
	RotateMode = ROTATE_MODE_DECELEARTION;
	DecelerateInfo.stepscnt = 0;
	DecelerateInfo.startms = millis();
	DecelerateInfo.endms = millis();
	
}

void StepperMotor::SlowdownStop()
{
	RotateMode = ROTATE_MODE_DECELEARTION;
	State = MOTOR_STATE_SLOWDOWN_STOP;
	
	DecelerateInfo.stepscnt = 0;
	DecelerateInfo.startms = millis();
	DecelerateInfo.endms = millis();
	
}
void StepperMotor::Accelerate()
{
	RotateMode = ROTATE_MODE_ACCELEARTION;
}


//20200521, 約24us, //20200607, 約22.5us, 極限約22KHZ, 加減速約48us, 所以不要超過20KHz, 不然會計步錯誤
//在中斷中不要用for loop去執行其它沒事的==>17.5us
//20KHz, interrupt間隔為50us
//15KHz, interrupt間隔為66.6us
//10KHz, interrupt間隔為100us
//5KHz, interrupt間隔為200us

void StepperMotor::TimerProcess(int vect_num)
{
	int incvalue = 0, i;
	if(Vect_Num != vect_num)
		return;
	if(PWMInitCnt < 2)	//前2個中斷不會有PWM輸出
	{
		PWMInitCnt ++;
		for(i=0; i<SENSOR_PIN_TOTAL; i++)
		{
			Sensor[i][this->Direction].ToggleCnt = 0;
			Sensor[i][this->Direction].PreHL = -1;
		}
	}
	else
	{
		if(getPWMOnOff(this->Timer))
		{
			//if(digitalRead(this->DirPin))	//24us
			if(Direction == MOTOR_CW)		//21.8us
				this->Position ++;
			else
				this->Position --;
			
			if((this->State == MOTOR_STATE_MOVE_TO_POSITION) || (this->State == MOTOR_STATE_MOVE_TO_STOP_PIN))
			{
				if(this->Position == this->TargetPosition)
				{
					if(this->State == MOTOR_STATE_MOVE_TO_POSITION)
						this->State = MOTOR_STATE_STOP;
					else
					{
						if((Sensor[SENSOR_PIN_STOP][this->Direction].Pin == -1) 
							&& (Sensor[SENSOR_PIN_TOGGLE][this->Direction].Pin == -1) 
							&& (Sensor[SENSOR_PIN_LIMIT][this->Direction].Pin == -1))
							this->State = MOTOR_STATE_STOP;
					}
				}
				else if((RotateMode == ROTATE_MODE_ACCELEARTION) || (RotateMode == ROTATE_MODE_KEEP_SPEED))
					if(abs(this->Position - this->TargetPosition) <= AccelerateInfo.stepscnt)
					{
						RotateMode = ROTATE_MODE_DECELEARTION;
						#if STEPPER_MOTOR_DEBUG		
							DecelerateInfo.startms = millis();
						#endif
						
					}
			}
		}
		
		//3us
		if(RotateMode == ROTATE_MODE_ACCELEARTION) 
		{
			if(FrequenceNow < Frequence)
			{
				AccelerateInfo.stepscnt ++;
				#if STEPPER_MOTOR_DEBUG		
					AccelerateInfo.endms = millis();
				#endif

				FrequenceIncCnt += FrequenceIncValue;
				incvalue = FrequenceIncCnt >> ACCELERATE_TABLE_EXP;
				FrequenceIncCnt -= incvalue << ACCELERATE_TABLE_EXP;

				setFrequenceNow(FrequenceNow + incvalue);
				updateTimer(FrequenceNow);
			}
		}
		else if(RotateMode == ROTATE_MODE_DECELEARTION)
		{
			if(FrequenceNow > FrequenceStartup)
			{
				DecelerateInfo.stepscnt ++;
				#if STEPPER_MOTOR_DEBUG		
					DecelerateInfo.endms = millis();
				#endif
				FrequenceIncCnt += FrequenceIncValue;
				incvalue = FrequenceIncCnt >> ACCELERATE_TABLE_EXP;
				FrequenceIncCnt -= incvalue << ACCELERATE_TABLE_EXP;

				setFrequenceNow(FrequenceNow - incvalue);
				updateTimer(FrequenceNow);

				if(FrequenceNow <= FrequenceStartup)
					if(State == MOTOR_STATE_SLOWDOWN_STOP)
						this->State = MOTOR_STATE_STOP;
			}
		}
		else
			RotateMode = ROTATE_MODE_KEEP_SPEED;
	}

	
	//1us
	for(i=0; i<SENSOR_PIN_TOTAL; i++)
		if(Sensor[i][this->Direction].Pin != -1)
		{
			int hl = digitalRead(Sensor[i][this->Direction].Pin);
			bool checkpin = false;
			if(hl == Sensor[i][this->Direction].State)
			{
				if(Sensor[i][this->Direction].Mode & SENSOR_PIN_MODE_RANGE)
				{
					if((this->Direction == MOTOR_CW)
						&& (Position > (TargetPosition - Sensor[i][this->Direction].Range)))
						checkpin = true;
					else if((this->Direction == MOTOR_CCW)
						&& (Position < (TargetPosition + Sensor[i][this->Direction].Range)))
						checkpin = true;
				}
				else
					checkpin = true;

				if(checkpin)
				{
					if(Sensor[i][this->Direction].PreHL != hl)
					{
						Sensor[i][this->Direction].ToggleCnt ++;
#if 0//STEPPER_MOTOR_DEBUG
						cmd_port->print("#");
#endif
					}

					if(Sensor[i][this->Direction].ToggleCnt >= Sensor[i][this->Direction].ToggleTimes)
					{
						if(Sensor[i][this->Direction].Mode & SENSOR_PIN_MODE_SLOWDOWN)
						{
							RotateMode = ROTATE_MODE_DECELEARTION;
							if(Sensor[i][this->Direction].Mode & SENSOR_PIN_MODE_STOP)
								State = MOTOR_STATE_SLOWDOWN_STOP;
						}
						else 
							this->State = MOTOR_STATE_STOP;
						
						if(Sensor[i][this->Direction].Mode & SENSOR_PIN_MODE_ONCE)
						{
							Sensor[i][MOTOR_CW].Pin = -1;
							Sensor[i][MOTOR_CCW].Pin = -1;
						}
					}
				}
			}
			Sensor[i][this->Direction].PreHL = hl;
		}
	
	
	if(this->State == MOTOR_STATE_STOP)  
	{
		turnOffTimer(this->Timer);
		FrequenceNow = 0;
	}
}


#if 0
//for PWM 2
ISR(TIMER3_COMPB_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER3_COMPB_vect_num);
}

//for PWM 3
ISR(TIMER3_COMPC_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER3_COMPC_vect_num);
}

#if 0
//for PWM 4, 勿用
ISR(TIMER0_COMPB_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER0_COMPB_vect_num);
}
#endif

//for PWM 5, 沒反應
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER3_COMPA_vect_num);
}

//for PWM 6//只LOW一次就沒反應
ISR(TIMER4_COMPA_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER4_COMPA_vect_num);
}

//for PWM 7
ISR(TIMER4_COMPB_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER4_COMPB_vect_num);
}


long isrcnt = 0;
//for PWM8
ISR(TIMER4_COMPC_vect)          // timer compare interrupt service routine
{
	isrcnt ++;
	if(isrcnt >= targetcnt)
	{
		isrcnt = 0;
		digitalWrite(LedPin, digitalRead(LedPin) ^ 1);	 // toggle LED pin
	}
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER4_COMPC_vect_num);
}

//for PWM 11	//有問題, Timer1底層已使用，需避開, 會造成mills()及micros()不準確
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
	if(isrcnt >= targetcnt)
	{
		isrcnt = 0;
		digitalWrite(LedPin, digitalRead(LedPin) ^ 1);	 // toggle LED pin
	}
	//for(int i=0; i<3; i++)
	//	Motor[i]->TimerProcess(TIMER1_COMPA_vect_num);
	//Motor[0]->TimerProcess(TIMER1_COMPA_vect_num);
}

//for PWM 12
ISR(TIMER1_COMPB_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER1_COMPB_vect_num);
}

//for PWM 44	
ISR(TIMER5_COMPC_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER5_COMPC_vect_num);
}

//for PWM 45
ISR(TIMER5_COMPB_vect)          // timer compare interrupt service routine
{
	for(int i=0; i<3; i++)
		Motor[i]->TimerProcess(TIMER5_COMPB_vect_num);
}


#endif 
