#ifndef _MAIN_PROCESS_H_
#define _MAIN_PROCESS_H_

#include "Arduino.h"
#include "SoftwareSerial.h"

//for LH Hardware debuger

#define	VERSTR	"2021081101"

#define INO_DEBUG 				1
#define HRM_DEBUG 				0


static const uint8_t OutputPin[] = {A0, A1, A2, A3, A4, A5, A6, A7,
                                    A8, A9, A10, A11, A12, A13, A14, A15
                                   };
static const uint8_t InputPin[] = {22, 23, 24, 25, 26, 27, 28, 29,
                                   30, 31, 32, 33, 34, 35, 36, 37
                                  };
static const uint8_t ADC_PWMPin[] = {0, 0, 0, A8, 2, A9, 3, A10, 6, A11, 7, A12,
									11, A13, 12, A14, 44, A15, 45, 0, 0};




#define VENDOR		"ESONHRM"
#define RST_PIN		42          // RFID_RST 腳位 
#define MISO		50          // MISO 腳位
#define M0SO		51          // MOSO 腳位
#define SCK		    52          // SCK  腳位
#define SS_PIN		53          // SS   腳位
#define OUT_BUZZER	48	        // BUZZ 
#define BT_PWRC		49          // BT4.2 


#define	CMD_PORT		    Serial
#define	CMD_PORT_BR		    115200

#define	HRM_CMD_PORT		Serial1
#define	HRM_CMD_PORT_BR		115200

#define	EXTIO_NUM		    0
#define	INPUT_8_NUMBER	    2
#define OUTPUT_8_NUMBER	    1


#define MOTOR_LEFT			0
#define MOTOR_TOTAL			1


#define RUN_MODE_STOP				        0
#define RUN_MODE_NORMAL				        1
#define RUN_MODE_FIRST_GO_HOME              2
#define RUN_MODE_PRE_POSITION      	        3
#define RUN_MODE_CMD_MOVE         	        4
#define RUN_MODE_BREAK_Cylinder_LIMIT       5
#define RUN_MODE_INIT                       6
#define RUN_MODE_SEARCH_HOME_MOVE_OFFSET    7
#define RUN_MODE_RESET                      8
#define RUN_MODE_Cylinder_INIT              9



#define MOTOR_LEFT_PULSE_PIN		2

#define MOTOR_DEGREE_MAX_NUM		40
#define MOTOR_DEGREE_MIN_NUM		0
#define MOTOR_ZeroOffset            200

#define CALIBRATION_MAX_NUM         100
typedef struct _DigitalIO_
{
	uint8_t	Input[(INPUT_8_NUMBER+EXTIO_NUM)*8];
	uint8_t	Output[(OUTPUT_8_NUMBER+EXTIO_NUM)*8];
}DigitalIO;


typedef struct _MainDataStruct_
{
	char        Vendor[10];
	uint8_t     HMI_ID;
	long	    MotorAcceleration;
	long		MotorEESpeed;           //工程技師人員速度
    long        MotorOPSpeed;           //操作人員速度
    long        MotorInitSpeed;
	long		MotorOffset;
	uint16_t	ValueReInitMagicCode;
	uint32_t	StepsForDepth = 32;    //0.1mm 需要多少pulse
    uint32_t	PressTimes;             //按壓多少次1次=0.1mm, 20次=2mm 
    uint32_t	MaxPressTimes;          //最大按壓次數
    float       CylinderLimitDistance;  //移動氣壓缸機構限制深度
    uint16_t    Std_Act_Distance[CALIBRATION_MAX_NUM][2]; //2bytes * 200 = 400bytes
    //Std_Act_Distance[][] = {Std distance(mm), Act distance(mm)}
}MainDataStruct;

typedef struct 
{
	uint8_t 	ZeroProcessWorkindex = 0xFF;
	uint8_t	    PositionWorkIndex = 0xFF;
    uint8_t     CylLimitWorkIndex = 0xFF;
    uint8_t     CmdWorkIndex = 0xFF;
	uint8_t	 	RunMode = RUN_MODE_STOP;
	uint8_t		ZeroBtn = 0;
	uint8_t		MotorAddBtn = 0;
	uint8_t		MotorSubBtn = 0;
	uint8_t		HomePin = 0;
	uint8_t		MaxPin = 0;
	uint16_t	ErrorCode = 0x0000;
	uint8_t		CylinderControlPin = 0;
	uint8_t		CylinderFrontBtn = 0;
	uint8_t		CylinderBackBtn = 0;
	uint8_t		CylinderLeftSensorPin = 0;
	uint8_t		CylinderRightSensorPin = 0;
}MotorState;



typedef struct _RuntimeStruct_
{
	MotorState motorstate;
	bool 	UpdateEEPROM;
    bool    Cylinder_LeftLow_RightHigh;
    float   CmdMoveDepth;
    bool    needResetHMI = false;
//    float   SlopeFormula[CALIBRATION_MAX_NUM][2] = {0}; //4byte * 200 = 800 bytes
    float SlopeFormula_M_b[2];
}RuntimeStatus;


void MainPorcess_Timer();
void reboot();
void MainProcess_Task();
void MainProcess_Init();


uint8_t getInput(uint8_t index);
void setOutput(uint8_t index, uint8_t hl);
void setStopPinIndex(int motorindex, int pinindex, int hl);
bool MotorMoveToZeroProcess();
bool MotorNormalProcess();
bool MotorMoveToPrePosition();
bool MotorBreakCylinderLimit();
bool MotorCommandMove();
void SlopeCalculate(int num);
long Calibration_PWM_Count(float dipth_mm);
void SlopeCalculate_M_b(int num);

#endif	//_MAIN_PROCESS_H_
