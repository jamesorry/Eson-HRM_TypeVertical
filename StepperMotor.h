#ifndef _STEPPER_MOTOR_H_
#define _STEPPER_MOTOR_H_

#include "Arduino.h"
#include "OCRxA_Table.h"

#define MOTOR_VELOCITY_MIN		512
#define MOTOR_VELOCITY_MAX		20000
#define MOTOR_VELOCITY_NORMAL	1500

#define MOTOR_STATE_STOP				0x00
#define MOTOR_STATE_ROTATE				0x01
#define MOTOR_STATE_ACCELERATION		0x02
#define MOTOR_STATE_DECELERATION		0x03
#define MOTOR_STATE_CONSTANT_VELOCITY	0x04
#define MOTOR_STATE_MOVE_TO_POSITION	0x05
#define MOTOR_STATE_SLOWDOWN_STOP		0x06
#define MOTOR_STATE_MOVE_TO_STOP_PIN	0x07

#define MOTOR_OUTPUT_LOW		1
#define MOTOR_OUTPUT_HIGH		0

#define MOTOR_CW				1	//for 宜笙
#define MOTOR_CCW				0

#define ROTATE_MODE_KEEP_SPEED				0
#define ROTATE_MODE_ACCELEARTION			1
#define ROTATE_MODE_DECELEARTION			2

/*
Digital pin 2 (PWM)	//TIMER3B 10
Digital pin 3 (PWM) //TIMER3C 11
Digital pin 4 (PWM) //TIMER0B 2
Digital pin 5 (PWM)	//TIMER3A 9
Digital pin 6 (PWM)	//TIMER4A 12
Digital pin 7 (PWM)	//TIMER4B 13
Digital pin 8 (PWM)	//TIMER4C 14
Digital pin 9 (PWM)	//TIMER2B 8
Digital pin 10 (PWM)	//TIMER2A 7
Digital pin 11 (PWM)	//TIMER1A 3
Digital pin 12 (PWM)	//TIMER1B 4
Digital pin 13 (PWM)	//TIMER0A 1
Digital pin 44 (PWM)	//TIMER5C 18
Digital pin 45 (PWM)	//TIMER5B 17
Digital pin 46 (PWM)	//TIMER5A 16

#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1C 5
#define TIMER2  6
#define TIMER2A 7
#define TIMER2B 8

#define TIMER3A 9
#define TIMER3B 10
#define TIMER3C 11
#define TIMER4A 12
#define TIMER4B 13
#define TIMER4C 14
#define TIMER4D 15
#define TIMER5A 16
#define TIMER5B 17
#define TIMER5C 18
*/

#define SENSOR_PIN_MODE_STOP		1
#define SENSOR_PIN_MODE_ONCE		2
#define SENSOR_PIN_MODE_SLOWDOWN	4
#define SENSOR_PIN_MODE_RANGE		8
	
#define SENSOR_PIN_STOP				0
#define SENSOR_PIN_STOP_RANGE		0
#define SENSOR_PIN_TOGGLE			0
#define SENSOR_PIN_SLOWDOWN			0
#define SENSOR_PIN_SLOWDOWN_STOP	0
#define SENSOR_PIN_LIMIT			1
#define SENSOR_PIN_TOTAL			2


typedef struct
{
	uint16_t OCRxA;
	uint16_t Prescale;
	uint16_t freq;
}TimerReg;

typedef struct
{
	int stepscnt;
	long startms;
	long endms;
}RotateInfo;

typedef struct
{
	int Pin;
	int State;
	int ToggleTimes;
	int ToggleCnt;
	int Range;
	uint8_t Mode;

	int PreHL;
}SensorPin;

class StepperMotor	//only for PWM pin
{
private:
	int Timer = -1; //

	int	State = MOTOR_STATE_STOP;

	// motor hardware definition
	int PulsePin;
	int DirPin;
	SensorPin Sensor[SENSOR_PIN_TOTAL][2];

	unsigned long Resolution;

	// motor control setting
	int Direction = MOTOR_CW;						// Direction of rotation
	unsigned long Frequence = 8000;			// Frequence , pulse per second
	unsigned long FrequenceStartup = ACCELERATE_TABLE_FREQ_STARTUP_MIN;			// Frequence , pulse per second
	unsigned long FrequenceNow = 0;			// Frequence , pulse per second
	int RotateMode = ROTATE_MODE_KEEP_SPEED;			// Frequence , pulse per second
	unsigned long RPM = 0; 					// RPM

	long motorOrigin = 0;

	int Vect_Num = -1;

	long Position = 0;
	long TargetPosition = 0;
	long SlowdownPosition = -1;
	int AccelerateTime = 100;

	
	long FrequenceIncValue = 0;

	int PWMInitCnt = 0;
	
	public:
		RotateInfo AccelerateInfo;
		RotateInfo DecelerateInfo;
		int FrequenceIncIndex = 0;			// Frequence , pulse per second
		int FrequenceIncCnt = 0;			// Frequence , pulse per second

	StepperMotor(int pulsepin, int dirpin);
	StepperMotor(int pulsepin, int dirpin, unsigned long resolution);
	StepperMotor(int pulsepin, int dirpin, unsigned long resolution, unsigned long frequence);

	void setSensorPin(int index, int pin, int hl, int ccwpin, int ccwhl, uint8_t mode, uint8_t toggletimes = 0, int range = 0);
	void setStopPin(int pin, int hl, uint8_t toggletimes);
	void setStopPin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes);
	void setStopRangePin(int pin, int hl, uint8_t toggletimes, int range);
	void setStopRangePin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes, int range);
	void setSlowdownPin(int pin, int hl, uint8_t toggletimes);
	void setSlowdownPin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes);
	void setSlowdownStopPin(int pin, int hl, uint8_t toggletimes);
	void setSlowdownStopPin(int pin, int hl, int ccwpin, int ccwhl, uint8_t toggletimes);
	void setLimitPin(int pin, int hl);
	void setLimitPin(int pin, int hl, int ccwpin, int ccwhl);
	
	int getTimer();
	int getState();
	int getPulsePin();
	int getDirPin();
	unsigned long getResolution();
	int getDirection();
	
	long getPosition();
	void setPosition(long position);
	int getRotateMode(void);
	unsigned long getFrequenceNow(void);
	void setFrequenceNow(long freq);
	unsigned long getFrequence(void);
	void setFrequence(long freq);
	void setFrequenceStartup(long freq);
	unsigned long getFrequenceStartup(void);
	void setAccelerateTime(int accms);
	int getAccelerateTime(void);
	
	void setRotateMode(int mode);
	void setRPM(unsigned long rpm);
	unsigned long getRPM();
	int getVectNum();
	long getTargetPosition(void);
	
	void Slowdown();
	void SlowdownStop();
	void Accelerate();
	void Stop(void);
	
	unsigned int Steps(long numberofsteps);
	unsigned int Steps(long numberofsteps, long frequence);

	unsigned int MoveTo(long targetpositon);
	unsigned int MoveTo(long targetpositon, long frequence);
	
	unsigned int MoveToStopPin(long targetpositon, long frequence);
	unsigned int MoveToStopPin(long targetpositon);
		
	void setDirection(int direction);

	void TimerProcess(int vect_num);

	
	unsigned int Speed(long freq, long freqstartup=OCRAXA_TABLE_FREQ_BASE, int accmsec=0);
	
	TimerReg ComputeTimerCCR(int timer, long freq);
	TimerReg updateTimer(long freq);
	bool SetPrescale(int timer, int prescale) ;
	bool SetPWMCompareReg(int timer, unsigned short ocrxa);
	bool turnOnTimer(int timer, int duty);
	bool turnOffTimer(int timer);
	bool setPWMOnOff(int timer, int onoff);
	int getPWMOnOff(int timer);
	bool ClearTimerCnt(int timer);

	int getLimitPinState();
};


#endif	//_STEPPER_MOTOR_H_
