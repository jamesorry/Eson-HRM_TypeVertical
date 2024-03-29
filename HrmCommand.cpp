#include <avr/wdt.h>
#include "Display.h"
#include "MainProcess.h"
#include "SoftwareSerial.h"
#include "StepperMotor.h"
#include "Timer.h"
#include "HrmCommand.h"
#include "EEPROM_Function.h"

extern HardwareSerial *cmd_port;
extern HardwareSerial *HRM_cmd_port;
extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
extern RuntimeStatus	runtimedata;
extern MainDataStruct maindata;
extern DigitalIO digitalio;
extern StepperMotor *motor[];
extern uint16_t TouchKeyTimer;
HRMCMD HRM_g_cmdFunc[] = {
    {"fw_version", fw_version}, //確認FW版本
	{"white_card_position_read", white_card_position_read}, //讀氣壓缸sensor
	{"white_card_height_read", white_card_height_read},//讀高度位置
    {"white_card_dynamic", white_card_dynamic}, //馬達從目前的位置移動
    {"white_card_speed", white_card_speed}, //馬達速度
    {"white_card_save", white_card_save}, //儲存資料
    {"white_card_move_in", white_card_move_in}, //汽缸in:將白卡移動到中間
    {"white_card_move_out", white_card_move_out}, //汽缸out:將白卡移動到外側
    {"white_card_pos_set_offset", white_card_pos_set_offset},
    {"white_card_zero_offset", white_card_zero_offset},
    {"white_card_limit_depth", white_card_limit_depth},
    {"white_card_max_depth", white_card_max_depth},
    {"white_card_cal_show", white_card_cal_show},
    {"white_card_cal_reset", white_card_cal_reset},
    {"white_card_cal", white_card_cal},
    {"?", HRMshowHelp}
};

void fw_version()//Read:確認FW版本
{
    HRM_cmd_port->println(VERSTR);
}
void white_card_position_read()//Read:讀氣壓缸sensor
{
    if(getInput(runtimedata.motorstate.CylinderRightSensorPin))
    {
        HRM_cmd_port->println("in");
    }
    else if(getInput(runtimedata.motorstate.CylinderLeftSensorPin))
    {
       HRM_cmd_port->println("out");
    }
}
void white_card_height_read()//Read:讀高度位置
{
    long targetposition;
    float distance;
    distance = maindata.PressTimes/10.0;
    targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
    if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
        HRM_cmd_port->println(maindata.CylinderLimitDistance, 1);
    }
    else{
        HRM_cmd_port->println(distance, 1);
    }
}
void white_card_dynamic()//Write:馬達從目前的位置移動
{
	String arg1;
    float value;
	if(!HRMgetNextArg(arg1))	
	{
		return;
	}
    value = arg1.toFloat();
    runtimedata.CmdMoveDepth = value;
    if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
        HRM_cmd_port->println("please set white_card_move_in");
        return;
    }
#if HRM_DEBUG
    HRM_cmd_port->println("value: " + String(value));
    HRM_cmd_port->println("runtimedata.CmdMoveDepth: " + String(runtimedata.CmdMoveDepth));
#endif
    if((runtimedata.CmdMoveDepth*10) > maindata.MaxPressTimes){
        runtimedata.CmdMoveDepth = maindata.MaxPressTimes/10.0;
        HRM_cmd_port->print("white_card_max_depth ");
        HRM_cmd_port->println(runtimedata.CmdMoveDepth, 1);
    }
    else{
        HRM_cmd_port->print("white_card_dynamic ");
        HRM_cmd_port->println(value, 1);
    }
    maindata.PressTimes = runtimedata.CmdMoveDepth*10;
    runtimedata.motorstate.CmdWorkIndex = 0;
    runtimedata.motorstate.RunMode = RUN_MODE_CMD_MOVE;
}
void white_card_speed()//Write:馬達速度
{
	String arg1;
    float value;
	if(!HRMgetNextArg(arg1))	
	{
		return;
	}
    value = arg1.toFloat();
    maindata.MotorEESpeed = value * maindata.StepsForDepth * 10;
//    mm/s => 一秒走多少mm，等於一秒走多少pulse
    cmd_port->println("white_card_speed: " + String(maindata.MotorEESpeed));
    HRM_cmd_port->print("white_card_speed ");
    HRM_cmd_port->println(value, 1);
}
void white_card_save()//Write:儲存資料
{
//    runtimedata.UpdateEEPROM = true;
    WRITE_EEPROM();
    HRM_cmd_port->println("white_card_save");
}
void white_card_move_in()//Write:汽缸in:將白卡移動到中間Right
{
    float distance;
//    targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
    distance = maindata.PressTimes * (0.1);
    if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
        if(distance > maindata.CylinderLimitDistance){
            setOutput(runtimedata.motorstate.CylinderControlPin, 1);
            Display(0, 0, 0, "HRM             ");
            Display(0, 0, 1, "Cylinder RIGHT        ");
            TouchKeyTimer = 0;
        }
        else{
            runtimedata.Cylinder_LeftLow_RightHigh = 1;
            runtimedata.motorstate.CylLimitWorkIndex = 0;
            runtimedata.motorstate.RunMode = RUN_MODE_BREAK_Cylinder_LIMIT;
            Display(0, 0, 0, "HRM             ");
            Display(0, 0, 1, "Break Cylinder        ");
        }
    }
    HRM_cmd_port->println("white_card_move_in");
}
void white_card_move_out()//Write:汽缸out:將白卡移動到外側left
{
    float distance;
//    targetposition = maindata.MotorOffset + maindata.StepsForDepth * maindata.PressTimes;
    distance = maindata.PressTimes * (0.1);
    if(getInput(runtimedata.motorstate.CylinderRightSensorPin)){
        if(distance > maindata.CylinderLimitDistance){
            setOutput(runtimedata.motorstate.CylinderControlPin, 0);
            Display(0, 0, 0, "HRM             ");
            Display(0, 0, 1, "Cylinder Left        ");
            TouchKeyTimer = 0;
        }
        else{
            runtimedata.Cylinder_LeftLow_RightHigh = 0;
            runtimedata.motorstate.CylLimitWorkIndex = 0;
            runtimedata.motorstate.RunMode = RUN_MODE_BREAK_Cylinder_LIMIT;
            Display(0, 0, 0, "HRM             ");
            Display(0, 0, 1, "Break Cylinder        ");
        }
    }
    HRM_cmd_port->println("white_card_move_out");
}

void white_card_pos_set_offset()
{
    //不用參數，把目前的設定距離，設定為offset
    if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
        HRM_cmd_port->println("please set white_card_move_in");
        return;
    }
    maindata.MotorOffset = maindata.StepsForDepth * maindata.PressTimes;
    HRM_cmd_port->print("white_card_set_offset ");
    HRM_cmd_port->println(maindata.PressTimes/10.0, 1);
    maindata.PressTimes = 0;
    runtimedata.motorstate.ZeroProcessWorkindex = 0;
    runtimedata.motorstate.RunMode = RUN_MODE_FIRST_GO_HOME;
}

void white_card_zero_offset()
{
    //不用參數，直接去尋找原點，然後停下，offset設定為0，按壓次數也為0
    if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
        HRM_cmd_port->println("please set white_card_move_in");
        return;
    }
    maindata.MotorOffset = 0;
    runtimedata.motorstate.ZeroProcessWorkindex = 0;
    runtimedata.motorstate.RunMode = RUN_MODE_SEARCH_HOME_MOVE_OFFSET;
    maindata.PressTimes = 0;
    HRM_cmd_port->println("white_card_zero_offset");
}
void white_card_limit_depth()
{
    //需參數mm，設定氣壓缸限制的高度
    if(getInput(runtimedata.motorstate.CylinderLeftSensorPin)){
        HRM_cmd_port->println("please set white_card_move_in");
        return;
    }
	String arg1;
    float depth;
	if(!HRMgetNextArg(arg1))
	{
        HRM_cmd_port->print("now white_card_limit_depth ");
        HRM_cmd_port->println(maindata.CylinderLimitDistance, 1);
		return;
	}
    depth = arg1.toFloat();
    maindata.CylinderLimitDistance = depth;
    HRM_cmd_port->print("white_card_limit_depth ");
    HRM_cmd_port->println(maindata.CylinderLimitDistance, 1);
    runtimedata.Cylinder_LeftLow_RightHigh = 0;
    runtimedata.motorstate.CylLimitWorkIndex = 0;
    runtimedata.motorstate.RunMode = RUN_MODE_BREAK_Cylinder_LIMIT;
}

void white_card_max_depth()
{
    //需參數mm，設定最深距離
	String arg1;
    float max_depth;
	if(!HRMgetNextArg(arg1))
	{
        HRM_cmd_port->print("now white_card_max_depth: ");
        HRM_cmd_port->println((float)maindata.MaxPressTimes/10.0, 1);
		return;
	}
    max_depth = arg1.toFloat();
    maindata.MaxPressTimes = max_depth*10;
    cmd_port->println("MaxPressTimes: " + String(maindata.MaxPressTimes));
    HRM_cmd_port->print("white_card_max_depth ");
    HRM_cmd_port->println(max_depth, 1);
}

void white_card_cal_show()
{
    HRM_cmd_port->println("white_card_cal_show");
    for(int i=0; i<CALIBRATION_MAX_NUM; i++){
        HRM_cmd_port->print(i);
        HRM_cmd_port->print(": ");
        for(int j=0; j<2; j++){
            HRM_cmd_port->print(maindata.Std_Act_Distance[i][j]*0.001, 2);
            HRM_cmd_port->print(", ");
        }
        HRM_cmd_port->println();
    }
}
void white_card_cal_reset()
{
    //初始化SlopeFormula內所有數值
    maindata.Std_Act_Distance[0][0] = 100;
    maindata.Std_Act_Distance[0][1] = 100;
    for(int num=1; num<100; num++){
        maindata.Std_Act_Distance[num][0] = (num)*500;
        maindata.Std_Act_Distance[num][1] = (num)*500;
    }
    HRM_cmd_port->println("white_card_cal_reset");
    //runtimedata.UpdateEEPROM = true;
}
void white_card_cal(void)
{
    //white_card_cal 1 0.1 0.1
    String arg1, arg2, arg3;
    int num;
    float Std_distance, Act_distance;
	if(!HRMgetNextArg(arg1))
	{
		return;
	}
    num = arg1.toInt();
    if(!HRMgetNextArg(arg2))
	{
        HRM_cmd_port->println("please input Std_distance");
		return;
	}
    if(!HRMgetNextArg(arg3))
	{
        HRM_cmd_port->println("please input Act_distance");
		return;
	}
    Std_distance = arg2.toFloat();
    Act_distance = arg3.toFloat(); 
#if 0
    cmd_port->print("Std_distance: ");
    cmd_port->println(Std_distance, 3);
    cmd_port->print("Act_distance: ");
    cmd_port->println(Act_distance, 3);
#endif
    if(num >= 0 && num < CALIBRATION_MAX_NUM){
        maindata.Std_Act_Distance[num][0] = Std_distance*1000.0;
        maindata.Std_Act_Distance[num][1] = Act_distance*1000.0;
        HRM_cmd_port->print("white_card_cal ");
        HRM_cmd_port->print(num);
        HRM_cmd_port->print(" ");
        HRM_cmd_port->print(maindata.Std_Act_Distance[num][0]*0.001, 2);
        HRM_cmd_port->print(" ");
        HRM_cmd_port->println(maindata.Std_Act_Distance[num][1]*0.001, 2);
    }
    else{
        HRM_cmd_port->println("white_card_cal num is over");
    }
}


String HRM_g_inputBuffer0 = "";
String* HRM_g_inputBuffer = NULL;
String HRM_g_cmd = "";
String HRM_g_arg = "";

bool HRM_g_echoOn = true;

bool HRMgetNextArg(String &arg)
{
  	if (HRM_g_arg.length() == 0)
    	return false;
  	if (HRM_g_arg.indexOf(" ") == -1)
  	{
    	arg = HRM_g_arg;
    	HRM_g_arg.remove(0);
  	}
  	else
  	{
    	arg = HRM_g_arg.substring(0, HRM_g_arg.indexOf(" "));
    	HRM_g_arg = HRM_g_arg.substring(HRM_g_arg.indexOf(" ") + 1);
  	}
  	return true;
}

void HRMshowHelp(void)
{
	int i;

	HRM_cmd_port->println("");
	for (i = 0; i < (sizeof(HRM_g_cmdFunc) / sizeof(HRMCMD)); i++)
	{
		HRM_cmd_port->println(HRM_g_cmdFunc[i].cmd);
	}
}


uint8_t HRM_UserCommWorkindex = 0;
uint32_t HRM_UserCommandTimeCnt = 0;

void HRM_UserCommand_Task(void)
{
  	int i, incomingBytes, ret, cmdPortIndex;
 	char data[2] = {0};

	switch(HRM_UserCommWorkindex)
	{
		case 0:
		{
			if(HRM_cmd_port->available())
			{
				HRM_g_inputBuffer = &HRM_g_inputBuffer0;
				HRM_UserCommWorkindex ++;
				HRM_UserCommandTimeCnt = millis();
			}
			break;
		}
		case 1:
		{
			if((millis() - HRM_UserCommandTimeCnt) > 50)
				HRM_UserCommWorkindex ++;
			break;
		}
		case 2:
		{
		  	if ( incomingBytes = HRM_cmd_port->available() )
		  	{
#if HRM_DEBUG
				HRM_cmd_port->println("HRM_cmd_port datalen: " + String(incomingBytes));
#endif
				for ( i = 0; i < incomingBytes; i++ )
				{
			  		ret = HRM_cmd_port->read();
			  		if ( (ret >= 0x20) && (ret <= 0x7E) || (ret == 0x0D) || (ret == 0x0A) )
			  		{
						data[0] = (char)ret;
						(*HRM_g_inputBuffer) += data;
						if (HRM_g_echoOn)
						{
				  			if ( (data[0] != 0x0D) && (data[0] != 0x0A) )
							{
#if HRM_DEBUG
							HRM_cmd_port->write(data);
#endif
				  			}
						}
			  		}
			  		else if (ret == 0x08)
			  		{
						if (HRM_g_inputBuffer->length())
						{
				  			HRM_g_inputBuffer->remove(HRM_g_inputBuffer->length() - 1);
				  			if (HRM_g_echoOn)
				  			{
								data[0] = 0x08;
								HRM_cmd_port->write(data);
				  			}
						}
			  		}
				}
				if (HRM_g_inputBuffer->indexOf('\r') == -1)
				{
			  		if (HRM_g_inputBuffer->indexOf('\n') == -1)
						return;
				}
				HRM_g_inputBuffer->trim();
				while (HRM_g_inputBuffer->indexOf('\r') != -1)
			  		HRM_g_inputBuffer->remove(HRM_g_inputBuffer->indexOf('\r'), 1);
				while (HRM_g_inputBuffer->indexOf('\n') != -1)
			  		HRM_g_inputBuffer->remove(HRM_g_inputBuffer->indexOf('\n'), 1);
				while (HRM_g_inputBuffer->indexOf("  ") != -1)
			  		HRM_g_inputBuffer->remove(HRM_g_inputBuffer->indexOf("  "), 1);
#if HRM_DEBUG
				HRM_cmd_port->println();
#endif
				if (HRM_g_inputBuffer->length())
				{
			  		HRM_g_arg.remove(0);
			  		if (HRM_g_inputBuffer->indexOf(" ") == -1)
						HRM_g_cmd = (*HRM_g_inputBuffer);
			  		else
			  		{
						HRM_g_cmd = HRM_g_inputBuffer->substring(0, HRM_g_inputBuffer->indexOf(" "));
						HRM_g_arg = HRM_g_inputBuffer->substring(HRM_g_inputBuffer->indexOf(" ") + 1);
			  		}
			  		for (i = 0; i < (sizeof(HRM_g_cmdFunc) / sizeof(HRMCMD)); i++)
			  		{
						if (HRM_g_cmd.equalsIgnoreCase(HRM_g_cmdFunc[i].cmd))
						{
				  			HRM_g_cmdFunc[i].func();
#if HRM_DEBUG
				  			HRM_cmd_port->print("HRM>");
#endif
				  			break;
						}
						else if (i == (sizeof(HRM_g_cmdFunc) / sizeof(HRMCMD) - 1))
						{
#if HRM_DEBUG
							HRM_cmd_port->println("bad command !!");
				  			HRM_cmd_port->print("HRM>");
#endif
						}
			  		}
			  		*HRM_g_inputBuffer = "";
				}
				else
				{
#if HRM_DEBUG
			  		HRM_cmd_port->print("HRM>");
#endif
				}
				HRM_UserCommWorkindex = 0;
				break;
			}
		}
  	}
}


