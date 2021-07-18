#ifndef _USER_COMMAND_H_
#define	_USER_COMMAND_H_

#include "Arduino.h"

typedef struct __CMD {
  const char* cmd;
  void (*func)(void);
} CMD, *PCMD;

void UserCommand_Task(void);
void UserCommand_Timer(void);

void resetArduino(void);
void getAdc(void);
void getGpio(void);
void setGpio(void);
void echoOn(void);
void echoOff(void);
void cmd_CodeVer(void);
void showHelp(void);
bool getNextArg(String &arg);

void cmd_WriteToSerial(void);
void cmd_SetDO(void);
void cmd_GetDO(void);

void cmdMotorStep(void);
void cmdMotorMoveTo(void);
void cmdMotorStop(void);
void cmdMotorAccelerate(void);
void cmdMotorSetRPM(void);
void cmdMotorSpeed();
void cmdMotorInfo();
void cmdRunMode();
void cmdgetsetPosition();
void cmdOffset();
void cmdStepsForDepth();

void cmdInput();
void cmdOutput(void);
void cmdSaveEEPROM();
void cmdReadEEPROM(void);
void cmdClearEEPROM(void);

void cmdDepthRequest(void);
void cmdSetADC_PWM();
void cmdGetADC_PWM();
void cmd_ScanLCD_Address(void);

void cmdMotorEESpeed();
void cmdMotorOPSpeed();
void cmdMotorInitSpeed();
void cmdPressTimes();
void cmdMaxPressTimes();
void cmdCylinderLimitDistance();



#endif //_USER_COMMAND_H_
