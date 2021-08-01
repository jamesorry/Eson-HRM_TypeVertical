#include "avr/wdt.h"
#include <Adafruit_MCP23017.h>
#include "Display.h"
#include "MainProcess.h"
#include "StepperMotor.h"
#include "Timer.h"

#if INO_DEBUG
#define IO_DEBUG	1
#endif

#define MAIN_PROCESS_DEBUG	1
#define FREE_MODE 0

DigitalIO digitalio;
Adafruit_MCP23017 extio[EXTIO_NUM];
MainDataStruct maindata;
RuntimeStatus runtimedata;
StepperMotor *motor[MOTOR_TOTAL];
extern HardwareSerial *cmd_port;

uint16_t	TouchKeyTimer = 0;
void MainPorcess_Timer()
{
	if(TouchKeyTimer < 0xFF00)
		TouchKeyTimer += TIMER_INTERVAL_MS;
}

void reboot() 
{
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void MainProcess_ReCheckEEPROMValue()
{
    
	if(maindata.ValueReInitMagicCode != 728)	//20210705
	{
		maindata.ValueReInitMagicCode = 728;
		runtimedata.UpdateEEPROM = true;
	}
    if(maindata.HMI_ID < 0 || maindata.HMI_ID > 255)
    {
        maindata.HMI_ID = 0;
        runtimedata.UpdateEEPROM = true;
    }
	if(maindata.MotorAcceleration > 3000)
	{
		maindata.MotorAcceleration = 100;
		runtimedata.UpdateEEPROM = true;
	}
	if((maindata.MotorEESpeed > MOTOR_VELOCITY_MAX) || (maindata.MotorEESpeed < MOTOR_VELOCITY_MIN))
	{
		maindata.MotorEESpeed = MOTOR_VELOCITY_NORMAL;
		runtimedata.UpdateEEPROM = true;
	}
	if((maindata.MotorOPSpeed > MOTOR_VELOCITY_MAX) || (maindata.MotorOPSpeed < MOTOR_VELOCITY_MIN))
	{
		maindata.MotorOPSpeed = MOTOR_VELOCITY_NORMAL;
		runtimedata.UpdateEEPROM = true;
	}
    
	if((maindata.MotorInitSpeed > MOTOR_VELOCITY_MAX) || (maindata.MotorInitSpeed < MOTOR_VELOCITY_MIN))
	{
		maindata.MotorInitSpeed = MOTOR_VELOCITY_NORMAL;
		runtimedata.UpdateEEPROM = true;
	}
    
	if((maindata.MotorOffset < 0))
	{
		maindata.MotorOffset = 0;
		runtimedata.UpdateEEPROM = true;
	}
    if(maindata.PressTimes > 10000 || maindata.PressTimes < 0)
    {
        maindata.PressTimes = 0;
        runtimedata.UpdateEEPROM = true;
    }
    
    if(maindata.StepsForDepth > 10000 || maindata.StepsForDepth < 0)
    {
        maindata.StepsForDepth = 32;
        runtimedata.UpdateEEPROM = true;
    }
    
    if(maindata.MaxPressTimes > 10000 || maindata.MaxPressTimes < 0)
    {
        maindata.MaxPressTimes = 10000;
        runtimedata.UpdateEEPROM = true;
    }
    if(maindata.CylinderLimitDistance > 10000 || maindata.CylinderLimitDistance < 0)
    {
        maindata.CylinderLimitDistance = 8.0;
        runtimedata.UpdateEEPROM = true;
    }
    for(int i=0; i<CALIBRATION_MAX_NUM; i++){
        cmd_port->print(i);
        cmd_port->print(": ");
        for(int j=0; j<2; j++){
            if(maindata.Std_Act_Distance[i][j]*0.001 > 100 || maindata.Std_Act_Distance[i][j]*0.001 < 0)
                maindata.Std_Act_Distance[i][j] = 0;
            cmd_port->print(maindata.Std_Act_Distance[i][j]*0.001, 2);
            cmd_port->print(", ");
        }
        cmd_port->println();
    }
    runtimedata.UpdateEEPROM = true;
}


void MainProcess_Init()
{

	int i,j;
	runtimedata.UpdateEEPROM = false;


	MainProcess_ReCheckEEPROMValue();

	for(i=0;i<(INPUT_8_NUMBER+EXTIO_NUM)*8;i++)
		digitalio.Input[i] = 0;
	
	
	for(i=0;i<(OUTPUT_8_NUMBER+EXTIO_NUM)*8;i++)
			digitalio.Output[i]	= 0;


		
	for(i=0; i<INPUT_8_NUMBER * 8; i++)
	{
		//pinMode(InputPin[i], INPUT);
		pinMode(InputPin[i], INPUT_PULLUP);
	}
	
	for(i=0; i<OUTPUT_8_NUMBER * 8; i++)
	{
		pinMode(OutputPin[i], OUTPUT);	
	}
	
	for(j=0; j<EXTIO_NUM; j++)
	{
		extio[j].begin(j);	  	// Default device address 0x20+j

		for(i=0; i<8; i++)
		{
			extio[j].pinMode(i, OUTPUT);  // Toggle LED 1
			extio[j].digitalWrite(i, LOW);
		}
	}
	for(i=0; i<OUTPUT_8_NUMBER * 8; i++)
		digitalWrite(OutputPin[i], LOW);

	for(j=0; j<EXTIO_NUM; j++)
		for(i=0; i<8; i++)
		{
			extio[j].pinMode(i+8,INPUT);	 // Button i/p to GND
			extio[j].pullUp(i+8,HIGH);	 // Puled high to ~100k
		}



	runtimedata.motorstate.ZeroProcessWorkindex = 0xFF;
	runtimedata.motorstate.PositionWorkIndex = 0xFF;
    runtimedata.motorstate.CylLimitWorkIndex = 0xFF;
    runtimedata.motorstate.CmdWorkIndex = 0xFF;
	runtimedata.motorstate.RunMode = RUN_MODE_RESET;
	runtimedata.motorstate.ZeroBtn = 10;			    //隱藏回原點按鍵
	runtimedata.motorstate.MotorAddBtn = 2;			    //向上移動按鍵
	runtimedata.motorstate.MotorSubBtn = 3;			    //向下移動按鍵
	runtimedata.motorstate.HomePin = 6;			        //原點sensor
	runtimedata.motorstate.MaxPin = 7;			        //向下極限sensor
	runtimedata.motorstate.ErrorCode = 0x00;
	runtimedata.motorstate.CylinderControlPin = 0;		    //電磁閥開關
	runtimedata.motorstate.CylinderFrontBtn = 4;			//氣壓缸前進按鈕
	runtimedata.motorstate.CylinderBackBtn = 5;			    //氣壓缸後退按鈕
	runtimedata.motorstate.CylinderLeftSensorPin = 8;		//氣壓缸左sensor
	runtimedata.motorstate.CylinderRightSensorPin = 9;	    //氣壓缸右sensor
	//maindata.PressTimes = 0;
    motor[MOTOR_LEFT] = new StepperMotor(MOTOR_LEFT_PULSE_PIN, A8, 1000, MOTOR_VELOCITY_NORMAL);
	motor[0]->setLimitPin(InputPin[runtimedata.motorstate.MaxPin], LOW, InputPin[runtimedata.motorstate.HomePin], LOW);

    
    for(int i=0; i<CALIBRATION_MAX_NUM; i++)
    {
        SlopeCalculate_M_b(i);
#if 1
        cmd_port->print("runtimedata.SlopeFormula_M_b[");
        cmd_port->print(i);
        cmd_port->print("][0]= ");
        cmd_port->println(runtimedata.SlopeFormula_M_b[0], 5);
        cmd_port->print("runtimedata.SlopeFormula_M_b[");
        cmd_port->print(i);
        cmd_port->print("][1]= ");
        cmd_port->println(runtimedata.SlopeFormula_M_b[1], 5);
#endif
    }
}



void setOutput(uint8_t index, uint8_t hl)
{
	if(index < (OUTPUT_8_NUMBER*8))
	{
		digitalWrite(OutputPin[index], hl);
	}
	else
	{
		uint8_t extindex = index-(OUTPUT_8_NUMBER*8);
		uint8_t exi = extindex >> 3;
		uint8_t bi = extindex & 0x07;
		extio[exi].digitalWrite(bi, hl);
	}
	digitalio.Output[index] = hl;
}

uint8_t getInput(uint8_t index)
{
	uint8_t hl;
	if(index < (INPUT_8_NUMBER*8))
	{
		hl = digitalRead(InputPin[index]);
	}
	else
	{
		uint8_t extindex = index-(INPUT_8_NUMBER*8);
		uint8_t exi = extindex >> 3;
		uint8_t bi = extindex & 0x07;
		hl = extio[exi].digitalRead(bi+8);
	}

	digitalio.Input[index] = hl;
	return hl;
}

uint8_t preRunMode = 0xFF;
void MainProcess_Task()
{
    if(preRunMode != runtimedata.motorstate.RunMode){
        preRunMode = runtimedata.motorstate.RunMode;
        cmd_port->println("RunMode: " + String(preRunMode));
    }
	switch(runtimedata.motorstate.RunMode)
	{
		case RUN_MODE_STOP:
			cmd_port->println("RUN_MODE_STOP Success.");
			if(motor[0]->getState() != MOTOR_STATE_STOP)
				motor[0]->Stop();
			runtimedata.motorstate.RunMode = RUN_MODE_NORMAL;
			break;
		case RUN_MODE_NORMAL:
			MotorNormalProcess();
			break;
		case RUN_MODE_FIRST_GO_HOME: 
		    if(runtimedata.motorstate.ZeroProcessWorkindex == 0xFF){
                runtimedata.motorstate.ZeroProcessWorkindex = 0;
            }
            if(MotorMoveToZeroProcess()){
    			if(motor[0]->getState() == MOTOR_STATE_STOP){
                    runtimedata.motorstate.RunMode = RUN_MODE_PRE_POSITION;
                }
            }
			break;
		case RUN_MODE_PRE_POSITION: //馬達移動到上次儲存的位置
            if(runtimedata.motorstate.PositionWorkIndex == 0xFF){
                runtimedata.motorstate.PositionWorkIndex = 0;
            }
            if(MotorMoveToPrePosition()){
    			if(motor[0]->getState() == MOTOR_STATE_STOP){
                    runtimedata.motorstate.RunMode = RUN_MODE_Cylinder_INIT;
                }
            }
			break;
        case RUN_MODE_CMD_MOVE: //馬達移動到Command位置
            if(runtimedata.motorstate.CmdWorkIndex == 0xFF){
                runtimedata.motorstate.CmdWorkIndex = 0;
            }
            if(MotorCommandMove()){
                if(motor[0]->getState() == MOTOR_STATE_STOP){
                    runtimedata.motorstate.RunMode = RUN_MODE_NORMAL;
                }
            }
            break;
        case RUN_MODE_BREAK_Cylinder_LIMIT: //馬達需要先移動到指定高度，才能動氣壓缸
            if(runtimedata.motorstate.CylLimitWorkIndex == 0xFF){
                runtimedata.motorstate.CylLimitWorkIndex = 0;
            }
            if(MotorBreakCylinderLimit()){
    			if(motor[0]->getState() == MOTOR_STATE_STOP){
                    runtimedata.motorstate.RunMode = RUN_MODE_NORMAL;
                }
            }
            break;
        case RUN_MODE_INIT:
            //初始化狀態，確保氣壓缸在某個位置上，再讓馬達開始動，避免撞機
            //尚未確定初始化狀態
//            getInput(runtimedata.motorstate.CylinderRightSensorPin)
//            getInput(runtimedata.motorstate.CylinderLeftSensorPin)
            if(motor[0]->getState() == MOTOR_STATE_STOP){
                if(getInput(runtimedata.motorstate.CylinderRightSensorPin)){
                    runtimedata.motorstate.RunMode = RUN_MODE_FIRST_GO_HOME;
                }
                else{
                    setOutput(runtimedata.motorstate.CylinderControlPin, 1);
                }
            }
            break;
        case RUN_MODE_SEARCH_HOME_MOVE_OFFSET:
		    if(runtimedata.motorstate.ZeroProcessWorkindex == 0xFF){
                runtimedata.motorstate.ZeroProcessWorkindex = 0;
            }
            if(MotorMoveToZeroProcess()){
                if(motor[0]->getState() == MOTOR_STATE_STOP){
                    runtimedata.motorstate.RunMode = RUN_MODE_NORMAL;
                }
            }
            break;
        case RUN_MODE_RESET:
            motor[0]->Speed(maindata.MotorInitSpeed);
            Display(0, 0, 0, "HRM             ");
            Display(0, 0, 1, "Initialize         ");
            runtimedata.motorstate.RunMode = RUN_MODE_INIT;
            break;
        case RUN_MODE_Cylinder_INIT: //氣壓缸初始化要回到左側外面
            Display(0, 0, 0, "HRM             ");
            Display(0, 0, 1, "Break Cylinder        ");
            runtimedata.Cylinder_LeftLow_RightHigh = 0;
            runtimedata.motorstate.CylLimitWorkIndex = 0;
            runtimedata.motorstate.RunMode = RUN_MODE_BREAK_Cylinder_LIMIT;
            break;
	}
}


uint8_t PreCmdWorkIndex = 0xFF;
bool MotorCommandMove()
{
	bool result = false;
    long targetposition;
	char str[20];
	char str_dis[6];
    if(PreCmdWorkIndex != runtimedata.motorstate.CmdWorkIndex){
        PreCmdWorkIndex = runtimedata.motorstate.CmdWorkIndex;
        cmd_port->println("CmdWorkIndex: " + String(PreCmdWorkIndex));
    }
	switch(runtimedata.motorstate.CmdWorkIndex)
    {
        case 0:
            if(motor[0]->getState() != MOTOR_STATE_STOP){
                motor[0]->Stop();
            }
            else{
                runtimedata.motorstate.CmdWorkIndex += 10;
            }
            break;
        case 10:
            if(motor[0]->getState() == MOTOR_STATE_STOP){
                targetposition =  maindata.MotorOffset + maindata.StepsForDepth*runtimedata.CmdMoveDepth*10;
                motor[0]->MoveTo(targetposition, maindata.MotorEESpeed);
                Display(0, 0, 0, "Speed: " + String(maindata.MotorEESpeed));
                dtostrf(runtimedata.CmdMoveDepth, 4, 1, str_dis);
                sprintf(str,"Go: %smm        ", str_dis);
                Display(0, 0, 1, str);
                runtimedata.motorstate.CmdWorkIndex += 10;
            }
            break;
        case 20:
            if(motor[0]->getState() == MOTOR_STATE_STOP && motor[0]->getPosition() == maindata.MotorOffset + maindata.StepsForDepth*runtimedata.CmdMoveDepth*10){
                runtimedata.motorstate.CmdWorkIndex += 10;
            }
            break;
        default:
            runtimedata.motorstate.CmdWorkIndex = 0xFF;
            result = true;
            break;
    }
    return result;
}

uint8_t PreCylLimitWorkIndex = 0xFF;
bool MotorBreakCylinderLimit()
{
	bool result = false;
    long targetposition;
	char str[20];
	char str_dis[6];
    float distance;
    if(PreCylLimitWorkIndex != runtimedata.motorstate.CylLimitWorkIndex){
        PreCylLimitWorkIndex = runtimedata.motorstate.CylLimitWorkIndex;
        cmd_port->println("CylLimitWorkIndex: " + String(PreCylLimitWorkIndex));
    }
	switch(runtimedata.motorstate.CylLimitWorkIndex)
    {
        case 0:
            if(motor[0]->getState() != MOTOR_STATE_STOP){
                motor[0]->Stop();
            }
            else{
                runtimedata.motorstate.CylLimitWorkIndex += 10;
            }
            break;
        case 10: //馬達需要避免撞到機構，需要移動到設定高度
            if(motor[0]->getState() == MOTOR_STATE_STOP){
                targetposition = maindata.MotorOffset + maindata.StepsForDepth*maindata.CylinderLimitDistance*10;
                motor[0]->MoveTo(targetposition, maindata.MotorEESpeed);
                dtostrf(maindata.CylinderLimitDistance, 4, 1, str_dis);
    			sprintf(str,"Go: %smm        ", str_dis);
    			Display(0, 0, 1, str);
                runtimedata.motorstate.CylLimitWorkIndex += 10;
            }
            break;
        case 20: //移動氣壓缸
            if(motor[0]->getState() == MOTOR_STATE_STOP){
                if(runtimedata.Cylinder_LeftLow_RightHigh){ //向右
                    Display(0, 0, 0, "HRM             ");
                    sprintf(str, "Cylinder RIGHT        ");
                    Display(0, 0, 1, str);
                    setOutput(runtimedata.motorstate.CylinderControlPin, runtimedata.Cylinder_LeftLow_RightHigh);
                    runtimedata.motorstate.CylLimitWorkIndex = 25;
                }
                else{ //向左
                    Display(0, 0, 0, "HRM             ");
                    sprintf(str, "Cylinder LEFT        ");
                    Display(0, 0, 1, str);
                    setOutput(runtimedata.motorstate.CylinderControlPin, runtimedata.Cylinder_LeftLow_RightHigh);
                    runtimedata.motorstate.CylLimitWorkIndex = 30;
                }
            }
            break;
        case 25: //移動氣壓缸在右邊，確認已經到設定高度，馬達回到原始位置
            if(getInput(runtimedata.motorstate.CylinderRightSensorPin)){
                if(motor[0]->getState() == MOTOR_STATE_STOP){
                    targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
                    motor[0]->MoveTo(targetposition, maindata.MotorEESpeed);
                    distance = maindata.PressTimes/10.0;
                    dtostrf(distance, 4, 1, str_dis);
        			sprintf(str,"Back: %smm        ", str_dis);
        			Display(0, 0, 1, str);
                    runtimedata.motorstate.CylLimitWorkIndex = 40;
                }
            }
            break;
        case 30: //移動氣壓缸在左邊
            if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
                Display(0, 0, 0, "Cylinder in LEFT        ");
                dtostrf(maindata.CylinderLimitDistance, 4, 1, str_dis);
                sprintf(str,"Depth: %smm        ", str_dis);
                Display(0, 0, 1, str);
                runtimedata.motorstate.CylLimitWorkIndex = 50;
            }
            break;
        case 40: //確認已經馬達回到原始位置
            if(motor[0]->getState() == MOTOR_STATE_STOP 
//                && motor[0]->getPosition() == maindata.MotorOffset + maindata.StepsForDepth*maindata.PressTimes
                    )
            {
                runtimedata.motorstate.CylLimitWorkIndex += 10;
                TouchKeyTimer = 0;
            }
            break;
        default:
            runtimedata.motorstate.CylLimitWorkIndex = 0xFF;
            result = true;
            break;
    }
    return result;
}

uint8_t PrePositionWorkindex = 0xFF;
bool MotorMoveToPrePosition()
{
	char str[21];
	char str_dis[6];
	bool result = false;
	float distance;
    long targetposition;
    targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
    if(PrePositionWorkindex != runtimedata.motorstate.PositionWorkIndex){
        PrePositionWorkindex = runtimedata.motorstate.PositionWorkIndex;
        cmd_port->println("PositionWorkIndex: " + String(PrePositionWorkindex));
    }
	switch(runtimedata.motorstate.PositionWorkIndex)
    {
        case 0:
            if(motor[0]->getState() != MOTOR_STATE_STOP){
                motor[0]->Stop();
            }
            else{
                Display(0, 0, 0, "HRM             ");
                Display(0, 0, 1, "Go preposition          ");
                runtimedata.motorstate.PositionWorkIndex += 10;
            }
            break;
        case 10:    //確認已經到MotorOffset的位置，移動到上次儲存位置
            if(motor[0]->getPosition() == maindata.MotorOffset && motor[0]->getState() == MOTOR_STATE_STOP){
                cmd_port->println("targetposition: " + String(targetposition));
                motor[0]->MoveTo(targetposition, maindata.MotorInitSpeed);
                runtimedata.motorstate.PositionWorkIndex += 10;
            }
            break;
        case 20:    //確認已經到上次儲存位置
            if(motor[0]->getPosition() == targetposition && motor[0]->getState() == MOTOR_STATE_STOP){
                Display(0, 0, 0, "HRM             ");
                Display(0, 0, 1, "In preposition          ");
                runtimedata.motorstate.PositionWorkIndex += 10;
            }
            break;
        default:
            runtimedata.motorstate.PositionWorkIndex = 0xFF;
            result = true;
            break;
    }   
	return result;
}
uint8_t PreZeroProcessWorkindex = 0xFF;
bool MotorMoveToZeroProcess()
{
	uint8_t i;
	char str[21];
	bool result = false;
    
    if(PreZeroProcessWorkindex != runtimedata.motorstate.ZeroProcessWorkindex){
        PreZeroProcessWorkindex = runtimedata.motorstate.ZeroProcessWorkindex;
        cmd_port->println("ZeroProcessWorkindex: " + String(PreZeroProcessWorkindex));
    }
	switch(runtimedata.motorstate.ZeroProcessWorkindex)
    {
        case 0:
            if(motor[0]->getState() != MOTOR_STATE_STOP){
                motor[0]->Stop();
            }
            else{
                Display(0, 0, 0, "HRM             ");
                Display(0, 0, 1, "Motor go home          ");
                cmd_port->println("go hmoe");
                runtimedata.motorstate.ZeroProcessWorkindex += 10;
            }
            break;
        case 10: //馬達往負方向尋找原點senser
            if(motor[0]->getState() == MOTOR_STATE_STOP){
                motor[0]->Speed((-1)*maindata.MotorInitSpeed);
                runtimedata.motorstate.ZeroProcessWorkindex += 10;
            }
            break;
        case 20:
            if(motor[0]->getState() == MOTOR_STATE_STOP){
                motor[0]->setPosition(0);
                runtimedata.motorstate.ZeroProcessWorkindex += 10;
            }
            break;
        case 30: //移動到MotorOffset的位置
            if(motor[0]->getPosition() == 0){
                motor[0]->MoveTo(maindata.MotorOffset, maindata.MotorInitSpeed);
                runtimedata.motorstate.ZeroProcessWorkindex += 10;
            }
            break;
        case 40: //確認已經到MotorOffset的位置
            if(motor[0]->getPosition() == maindata.MotorOffset && motor[0]->getState() == MOTOR_STATE_STOP){
                runtimedata.motorstate.ZeroProcessWorkindex += 10;
            }
            break;
        default:
            runtimedata.motorstate.ZeroProcessWorkindex = 0xFF;
            result = true;
            break;
    }   
	return result;
}

uint8_t preAddBtnState = 0;
uint8_t preSubBtnState = 0;
uint8_t preFrontBtnState = 0;
uint8_t preBackBtnState = 0;
bool MotorNormalProcess()
{
	uint8_t i;
	char str[20];
	char str_dis[6];
	bool result = false;
	long targetposition;
    float distance;
    targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
    distance = maindata.PressTimes * (0.1);
    uint8_t addbtn = getInput(runtimedata.motorstate.MotorAddBtn);
	uint8_t subbtn = getInput(runtimedata.motorstate.MotorSubBtn);
	uint8_t frontbtn = getInput(runtimedata.motorstate.CylinderFrontBtn);
	uint8_t backbtn = getInput(runtimedata.motorstate.CylinderBackBtn);
    
//馬達到最高點或是最低點，送出訊號告知PC
    if(targetposition == maindata.MotorOffset)
    {    
        setOutput(5, 1);
        setOutput(6, 0);
        setOutput(7, 0);
//        cmd_port->println("motor in 最高點");
    }
    else if(targetposition == maindata.MotorOffset + maindata.StepsForDepth * maindata.MaxPressTimes)
    {
        setOutput(7, 1);
        setOutput(5, 0);
        setOutput(6, 0);
//        cmd_port->println("motor in 最低點");
    }
    else
    {
        setOutput(6, 1);
        setOutput(5, 0);
        setOutput(7, 0);
//        cmd_port->println("motor in 中間值");
    }
    
	if(motor[0]->getState() == MOTOR_STATE_STOP 
//		&& digitalRead(InputPin[runtimedata.motorstate.HomePin]) 
//		&& digitalRead(InputPin[runtimedata.motorstate.MaxPin])
		)
    {
//判斷馬達向上移動按鈕是否被按下，向下為加
        if( (subbtn && !preSubBtnState) && getInput(runtimedata.motorstate.CylinderRightSensorPin))
        {
            motor[0]->setStopPin(InputPin[runtimedata.motorstate.MaxPin], LOW, 0);
#if MAIN_PROCESS_DEBUG
            cmd_port->println("subbtn: " + String(subbtn) + ", " + String(preSubBtnState));
#endif
            if(maindata.PressTimes < maindata.MaxPressTimes){
                maindata.PressTimes++;
                targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
                motor[0]->MoveTo(targetposition, maindata.MotorOPSpeed);
#if MAIN_PROCESS_DEBUG
                cmd_port->println("targetposition: " + String(targetposition));
#endif
				Display(0, 0, 0, "HRM             ");
				distance = maindata.PressTimes * 0.1;
				dtostrf(distance, 4, 1, str_dis);
				sprintf(str,"Depth: %smm        ", str_dis);
				Display(0, 0, 1, str);
				TouchKeyTimer = 0;
            }
            
        }
            
//判斷馬達向下移動按鈕是否被按下，向上為減
        else if( (addbtn && !preAddBtnState) && getInput(runtimedata.motorstate.CylinderRightSensorPin))
        {
            //motor[0]->setStopPin(InputPin[runtimedata.motorstate.HomePin], LOW, 0);
#if MAIN_PROCESS_DEBUG
            cmd_port->println("addbtn: " + String(addbtn) + ", " + String(preAddBtnState));
#endif
            if(maindata.PressTimes > 0){
                maindata.PressTimes--;
                targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
                motor[0]->MoveTo(targetposition, maindata.MotorOPSpeed);
#if MAIN_PROCESS_DEBUG
                cmd_port->println("targetposition: " + String(targetposition));
#endif
				Display(0, 0, 0, "HRM             ");
				distance = maindata.PressTimes * 0.1;
				dtostrf(distance, 4, 1, str_dis);
				sprintf(str,"Depth: %smm        ", str_dis);
				Display(0, 0, 1, str);
				TouchKeyTimer = 0;
            }
        }
    
        //=======================================================================================================================================================================================
//判斷LEFT氣壓缸按鈕是否被按下
		else if( (frontbtn && !preFrontBtnState) && getInput(runtimedata.motorstate.CylinderRightSensorPin))
		{
#if MAIN_PROCESS_DEBUG
			cmd_port->println("frontbtn: " + String(frontbtn) + ", " + String(preFrontBtnState));
#endif
            /*
            白卡左右移動要在8mm以下才能移動，不然會卡到機構。 
            0~8mm之間不能左右移動
            */
            if(distance > maindata.CylinderLimitDistance){
                setOutput(runtimedata.motorstate.CylinderControlPin, 0);           
                Display(0, 0, 0, "HRM             ");
                sprintf(str, "Cylinder LEFT        ");
                Display(0, 0, 1, str);
                TouchKeyTimer = 0;
            }
            else{
                runtimedata.Cylinder_LeftLow_RightHigh = 0;
                runtimedata.motorstate.RunMode = RUN_MODE_BREAK_Cylinder_LIMIT;
                runtimedata.motorstate.CylLimitWorkIndex = 0;
                Display(0, 0, 0, "HRM             ");
                sprintf(str, "Break Cylinder        ");
                Display(0, 0, 1, str);
            }
		}
//判斷後退氣壓缸按鈕是否被按下
		else if( (backbtn && !preBackBtnState) && getInput(runtimedata.motorstate.CylinderLeftSensorPin))
		{
#if MAIN_PROCESS_DEBUG
			cmd_port->println("backbtn: " + String(backbtn) + ", " + String(preBackBtnState));
#endif
            /*
            白卡左右移動要在8mm以下才能移動，不然會卡到機構。 
            0~8mm之間不能左右移動
            */
            if(distance > maindata.CylinderLimitDistance){
                setOutput(runtimedata.motorstate.CylinderControlPin, 1);
                Display(0, 0, 0, "HRM             ");
                sprintf(str, "Cylinder RIGHT        ");
                Display(0, 0, 1, str);
                TouchKeyTimer = 0;
            }
            else{
                runtimedata.Cylinder_LeftLow_RightHigh = 1;
                runtimedata.motorstate.RunMode = RUN_MODE_BREAK_Cylinder_LIMIT;
                runtimedata.motorstate.CylLimitWorkIndex = 0;
                Display(0, 0, 0, "HRM             ");
                sprintf(str, "Break Cylinder        ");
                Display(0, 0, 1, str);
            }
		}
//動作完成後，LCD切換回深度資訊畫面
		else if(TouchKeyTimer>1000 && (getInput(runtimedata.motorstate.CylinderRightSensorPin)) )
        {
            Display(0, 0, 0, "HRM             ");
			dtostrf(distance, 4, 1, str_dis);
			sprintf(str,"Depth: %smm        ", str_dis);
			Display(0, 0, 1, str);
			TouchKeyTimer = 0;
		}
        else if(TouchKeyTimer>1000 && (getInput(runtimedata.motorstate.CylinderLeftSensorPin)) )
        {
            if(distance > maindata.CylinderLimitDistance){
                Display(0, 0, 0, "Cylinder in LEFT        ");
                dtostrf(distance, 4, 1, str_dis);
                sprintf(str,"Depth: %smm        ", str_dis);
    			Display(0, 0, 1, str);
            }
            else{
                Display(0, 0, 0, "Cylinder in LEFT        ");
                dtostrf(maindata.CylinderLimitDistance, 4, 1, str_dis);
                sprintf(str,"Depth: %smm        ", str_dis);
    			Display(0, 0, 1, str);
            }
            TouchKeyTimer = 0;
        }
	}
	preAddBtnState = addbtn;
	preSubBtnState = subbtn;
	preFrontBtnState = frontbtn;
	preBackBtnState = backbtn;

	return result;
}


//此方法可執行，但記憶體不足
#if 0
void SlopeCalculate(int num)
{
    if(num == 0){
        runtimedata.SlopeFormula[num][0] = 0.0;
        runtimedata.SlopeFormula[num][1] = 0.0;
    }
    else if(num < CALIBRATION_MAX_NUM){
        float x0 = (float)maindata.Std_Act_Distance[num-1][1] * 0.01;
        float x1 = (float)maindata.Std_Act_Distance[num][1] * 0.01;
        float y0 = (float)maindata.Std_Act_Distance[num-1][0] * 0.01;
        float y1 = (float)maindata.Std_Act_Distance[num][0] * 0.01;
        //M = (y1-y0)/(x1-x0)
        float M = ((y1-y0)*(float)maindata.StepsForDepth*10.0)/(x1-x0);
        //b = y - Mx
        float b = (y1*(float)maindata.StepsForDepth*10.0) - (x1*M);
        runtimedata.SlopeFormula[num][0] = M;
        runtimedata.SlopeFormula[num][1] = b;
    }
    cmd_port->print("runtimedata.SlopeFormula[");
    cmd_port->print(num);
    cmd_port->print("][0]= ");
    cmd_port->println(runtimedata.SlopeFormula[num][0], 5);
    cmd_port->print("runtimedata.SlopeFormula[");
    cmd_port->print(num);
    cmd_port->print("][1]= ");
    cmd_port->println(runtimedata.SlopeFormula[num][1], 5);
    //y = Mx + b
    //return M*y1 + b;
}
#endif


long Calibration_PWM_Count(float dipth_mm)
{
    //y = Mx + b
    if(dipth_mm != 0)
    {
        int num = (dipth_mm*2.0) - 1.0;
        SlopeCalculate_M_b(num);
//        if(!isnan(runtimedata.SlopeFormula[num][0]) && !isnan(runtimedata.SlopeFormula[num][1]))
        if(!isnan(runtimedata.SlopeFormula_M_b[0]) && !isnan(runtimedata.SlopeFormula_M_b[1]))
        {
            if(num <= 1.0)
            {
                num = 1.0;
                SlopeCalculate_M_b(num);
//                cmd_port->println(runtimedata.SlopeFormula[num][0]*dipth_mm + runtimedata.SlopeFormula[num][1]);
#if 0
                cmd_port->println(runtimedata.SlopeFormula_M_b[0]*dipth_mm + runtimedata.SlopeFormula_M_b[1]);
#endif
                return runtimedata.SlopeFormula_M_b[0]*dipth_mm + runtimedata.SlopeFormula_M_b[1];
            }
            else if(num < CALIBRATION_MAX_NUM)
            {
                SlopeCalculate_M_b(num);
//                cmd_port->println(runtimedata.SlopeFormula[num][0]*dipth_mm + runtimedata.SlopeFormula[num][1]);
#if 0
                cmd_port->println(runtimedata.SlopeFormula_M_b[0]*dipth_mm + runtimedata.SlopeFormula_M_b[1]);
#endif
                return runtimedata.SlopeFormula_M_b[0]*dipth_mm + runtimedata.SlopeFormula_M_b[1];
            }
        }
        else
        {
            return dipth_mm * 10.0 * (float)maindata.StepsForDepth;
        }
    }
    else{
        return 0;
    }
}

void SlopeCalculate_M_b(int num)
{
    if(num == 0){
//        runtimedata.SlopeFormula[num][0] = 0.0;
//        runtimedata.SlopeFormula[num][1] = 0.0;
        runtimedata.SlopeFormula_M_b[0] = 0.0;
        runtimedata.SlopeFormula_M_b[1] = 0.0;
    }
    else if(num < CALIBRATION_MAX_NUM){
        float x0 = (float)maindata.Std_Act_Distance[num-1][1] * 0.001;
        float x1 = (float)maindata.Std_Act_Distance[num][1] * 0.001;
        float y0 = (float)maindata.Std_Act_Distance[num-1][0] * 0.001;
        float y1 = (float)maindata.Std_Act_Distance[num][0] * 0.001;
        //M = (y1-y0)/(x1-x0)
        float M = ((y1-y0)*(float)maindata.StepsForDepth*10.0)/(x1-x0);
        //b = y - Mx
        float b = (y1*(float)maindata.StepsForDepth*10.0) - (x1*M);
//        runtimedata.SlopeFormula[num][0] = M;
//        runtimedata.SlopeFormula[num][1] = b;
        runtimedata.SlopeFormula_M_b[0] = M;
        runtimedata.SlopeFormula_M_b[1] = b;
    }
//    cmd_port->print("runtimedata.SlopeFormula[");
//    cmd_port->print(num);
//    cmd_port->print("][0]= ");
//    cmd_port->println(runtimedata.SlopeFormula[num][0], 5);
//    cmd_port->print("runtimedata.SlopeFormula[");
//    cmd_port->print(num);
//    cmd_port->print("][1]= ");
//    cmd_port->println(runtimedata.SlopeFormula[num][1], 5);
    
#if 0
    cmd_port->print("runtimedata.SlopeFormula_M_b[");
    cmd_port->print(num);
    cmd_port->print("][0]= ");
    cmd_port->println(runtimedata.SlopeFormula_M_b[0], 5);
    cmd_port->print("runtimedata.SlopeFormula_M_b[");
    cmd_port->print(num);
    cmd_port->print("][1]= ");
    cmd_port->println(runtimedata.SlopeFormula_M_b[1], 5);
#endif
    //y = Mx + b
    //return M*y1 + b;
}

