/*
2021/05/25 提出新增 
Uart command:
1. A to B 移動設定 / 移動執行命令
2. speed 0.1mm/s
3. 白卡左右移動
4. 讀取左右移動的sensor
5. 白卡位置值
6. 按鈕功能保留

2021/06/24 提出新增
白卡左右移動要在7mm以下才能移動，不然會卡到機構。 
0~7mm之間不能左右移動

2021/07/02
1. 通訊WRITE 對方送過來的Command都完整回復回去
2. 硬體按鈕取消模式切換，只留下0.1mm
3. 對方34Pin的I/O控制移除(I/O觸發動作轉換為通訊)
4. 有機會量產，程式擴充方面須完善
5. 針對氣壓缸左右移動:
    需要可以設定限制高度參數調整
    強制離開到限制的高度後 再回到原來的地方
    Ex 限制高度為8mm，若目前高度在1mm，按下移進or移出，
    須強制馬達到達8mm才能繼續移進or移出，
    完成移進or移出後，在回到目前高度1mm
2021/07/23
1. motor position calibration
2. 需增加指令填寫(white_card_cal 1 0.1 0.1)
2. calibration後的數值需寫入EEPROM
*/

#include "Arduino.h"
#include "Display.h"
#include "EEPROM_Function.h"
#include "MainProcess.h"
#include "SPI.h"
#include "StepperMotor.h"
#include "Timer.h"
#include "UserCommand.h"
#include "HrmCommand.h"

#define BUZZ         48
#define LAN_CFG		 41

HardwareSerial *cmd_port;
HardwareSerial *HRM_cmd_port;

extern StepperMotor *motor[];

extern MainDataStruct maindata;
extern RuntimeStatus runtimedata;
extern DigitalIO digitalio;

void setup() {
	cmd_port = &CMD_PORT;
	cmd_port->begin(CMD_PORT_BR);
	HRM_cmd_port = &HRM_CMD_PORT;
	HRM_cmd_port->begin(HRM_CMD_PORT_BR);	
	READ_EEPROM();
	MainProcess_Init();
	Display_Init();
	TimerInit(1, 10000);
    
#if INO_DEBUG	
	cmd_port->println("Start of setup().");
	cmd_port->print("Version: ");
	cmd_port->println(VERSTR);
	cmd_port->println("Mode: Motor Ctrl");
#endif
	buzzerPlay(1000);
}

uint8_t datacnt = 0;
void loop()
{
	MainProcess_Task();
	UserCommand_Task();
	HRM_UserCommand_Task();
	if(runtimedata.UpdateEEPROM)
	{
		runtimedata.UpdateEEPROM = false;
		WRITE_EEPROM();
	}
}

void buzzerPlay(int playMS)
{
    digitalWrite(BUZZ, HIGH);
    delay(playMS);
    digitalWrite(BUZZ, LOW);
}

ISR(TIMER1_COMPA_vect)
{
	MainPorcess_Timer();
}

//for PWM 2
ISR(TIMER3_COMPB_vect)          // timer compare interrupt service routine
{
	motor[MOTOR_LEFT]->TimerProcess(TIMER3_COMPB_vect_num);
}
