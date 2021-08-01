#include <avr/wdt.h>
#include "Display.h"
#include "MainProcess.h"
#include "SoftwareSerial.h"
#include "StepperMotor.h"
#include "Timer.h"
#include "UserCommand.h"
#include <Wire.h>
#include "EEPROM_Function.h"

#if INO_DEBUG
#define USER_COMMAND_DEBUG	1
#endif

extern HardwareSerial *cmd_port;
extern RuntimeStatus	runtimedata;
extern MainDataStruct maindata;
extern DigitalIO digitalio;
extern StepperMotor *motor[];

CMD g_cmdFunc[] = {
//	{"adc", getAdc},
//	{"getgpio", getGpio},
//	{"setgpio", setGpio},
	{"reset", resetArduino},
	{"ver", cmd_CodeVer},
//	{"echoon", echoOn},
//	{"echooff", echoOff},
    {"out", cmdOutput},
    {"in", cmdInput},
//  {"GetADC_PWM", cmdGetADC_PWM},
//  {"SetADC_PWM", cmdSetADC_PWM},
    
    {"SD", cmdSaveEEPROM},
    {"RD", cmdReadEEPROM},
    {"CD", cmdClearEEPROM},
    {"scanlcd", cmd_ScanLCD_Address},
    
	{"step", cmdMotorStep},
	{"moveto", cmdMotorMoveTo},
	{"stop", cmdMotorStop},
	{"acc", cmdMotorAccelerate},
	{"speed", cmdMotorSpeed},
	{"info", cmdMotorInfo},
	{"position", cmdgetsetPosition},
	{"runmode", cmdRunMode},
	
	{"offset", cmdOffset},
    {"EESpeed", cmdMotorEESpeed},
    {"OPSpeed", cmdMotorOPSpeed},
    {"InitSpeed", cmdMotorInitSpeed},
	{"StepsForDepth", cmdStepsForDepth},
    {"PressTimes", cmdPressTimes},
    {"MaxPressTimes", cmdMaxPressTimes},
    {"CylinderLimitDistance", cmdCylinderLimitDistance},
    {"SlopeFormula",cmdSlopeFormula},
    {"cal_mm_pulse", cmdCalibration_PWM_Count},
	{"?", showHelp}
};
    

String g_inputBuffer0 = "";
String* g_inputBuffer = NULL;
String g_cmd = "";
String g_arg = "";

bool g_echoOn = true;

uint32_t targetcnt = 0;

bool getNextArg(String &arg)
{
  	if (g_arg.length() == 0)
    	return false;
  	if (g_arg.indexOf(" ") == -1)
  	{
    	arg = g_arg;
    	g_arg.remove(0);
  	}
  	else
  	{
    	arg = g_arg.substring(0, g_arg.indexOf(" "));
    	g_arg = g_arg.substring(g_arg.indexOf(" ") + 1);
  	}
  	return true;
}

void resetArduino(void)
{
    wdt_enable(WDTO_500MS);
    while (1);
}

void cmd_WriteToSerial(void)
{
	String arg1, arg2;
	char atcmd[32], ch;
	uint8_t i;
	if(!getNextArg(arg1))	
	{
		cmd_port->println("No parameter1");
		return;
	}
	if(!getNextArg(arg2))
		
	{
		cmd_port->println("No parameter2");
		return;
	}
	if(arg1 == "0")
	{
		Serial.write(arg2.c_str());
	}
	else if(arg1 == "1")
	{
		Serial1.write(arg2.c_str());
		cmd_port->println("send to Serial1.");
	}
	else if(arg1 == "2")
	{
		Serial2.write(arg2.c_str());
		cmd_port->println("send to Serial2.");
	}
	else if(arg1 == "3")
	{
		Serial3.write(arg2.c_str());
		cmd_port->println("send to Serial3.");
	}
	
}

void cmd_SetDO(void)
{
	String arg1, arg2;
	int i, bytei;
	int value;
	
	if (!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	if (!getNextArg(arg2))
	{
	  cmd_port->println("No parameter 2");
	  return;
	}
	cmd_GetDO();
	bytei = arg1.toInt();
	value = arg2.toInt();
	for(i=0; i<8; i++)
		setOutput(bytei*8+i, bitRead(value, i));

	cmd_port->print("set DO" + String(bytei) + ": ");
	cmd_port->println(String( value, HEX));

}

void cmdOutput(void)
{
	String arg1, arg2;
	int digitalPin;
	int value;

	if (!getNextArg(arg1))
	{
		cmd_port->println("No parameter 1");
		return;
	}
	if (!getNextArg(arg2))
	{
		cmd_port->println("No parameter 2");
		return;
	}
	digitalPin = arg1.toInt();
	value = arg2.toInt();

	cmd_port->print("PIN index:");
	cmd_port->println(arg1);
	cmd_port->print("level:");
	cmd_port->println(arg2);

	setOutput((uint8_t)digitalPin, (uint8_t)(value ? HIGH : LOW));
	//cmd_port->println("");
}

void cmdInput(void)
{
	String arg1, arg2;
	unsigned long pinindex;

	getNextArg(arg1);
	if( (arg1.length()==0))
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	pinindex = arg1.toInt();
	cmd_port->println("Sensor: " + String(getInput(pinindex)));
}

void cmd_GetDO(void)
{
	int i, bytei;
	uint8_t outbuf;
	cmd_port->print("get DO: ");
	for(bytei=0; bytei<(OUTPUT_8_NUMBER + EXTIO_NUM); bytei++)
	{
		outbuf = 0;
		for(i=0; i<8; i++)
		{
			outbuf |= (digitalio.Output[bytei*8+i] & 0x01) << i;
			cmd_port->print(String(digitalio.Output[i], HEX) + " ");
		}
		cmd_port->println(": " + String(outbuf, HEX));
	}
}


void showHelp(void)
{
	int i;

	cmd_port->println("");
	for (i = 0; i < (sizeof(g_cmdFunc) / sizeof(CMD)); i++)
	{
		cmd_port->println(g_cmdFunc[i].cmd);
	}
}

void getAdc(void)
{
	String arg1;
	int analogPin;
	int value;

	if (!getNextArg(arg1))
	{
		cmd_port->println("No parameter");
		return;
	}
	analogPin = arg1.toInt();
	value = analogRead(analogPin);
	cmd_port->print("ADC_");
	cmd_port->print(analogPin);
	cmd_port->print(" : ");
	cmd_port->println(value);
}

void getGpio(void)
{
	String arg1, arg2;
	int digitalPin, pullUp;
	int value;

	if (!getNextArg(arg1))
	{
		cmd_port->println("No parameter");
		return;
	}
	if (!getNextArg(arg2))
	{
		pullUp = 0;
	}
	else
	{
		pullUp = arg2.toInt();
	}
	digitalPin = arg1.toInt();
	if (arg2.compareTo("na") == 0)
	{
		cmd_port->println("pin mode keep original");
	}
	else
	{
		if (pullUp)
		{
			cmd_port->println("pull-up");
			pinMode(digitalPin, INPUT_PULLUP);
		}
		else
		{
			cmd_port->println("no-pull");
			pinMode(digitalPin, INPUT);
		}
	}

	cmd_port->print("GPIO:");
	cmd_port->println(arg1);

	value = digitalRead(digitalPin);

	cmd_port->print("input value:");
	cmd_port->println(value);
}

void setGpio(void)
{
	String arg1, arg2;
	int digitalPin;
	int value;

	if (!getNextArg(arg1))
	{
		cmd_port->println("No parameter 1");
		return;
	}
	if (!getNextArg(arg2))
	{
		cmd_port->println("No parameter 2");
		return;
	}
	digitalPin = arg1.toInt();
	value = arg2.toInt();

	cmd_port->print("GPIO:");
	cmd_port->println(arg1);
	cmd_port->print("level:");
	cmd_port->println(arg2);

	digitalWrite(digitalPin, value ? HIGH : LOW);
	pinMode(digitalPin, OUTPUT);
	//cmd_port->println("");
}

void echoOn(void)
{
	g_echoOn = true;
}

void echoOff(void)
{
	g_echoOn = false;
}

void cmd_CodeVer(void)
{
	cmd_port->println(VERSTR);
}

#if 0
#define RUN_MODE_STOP				        0
#define RUN_MODE_NORMAL				        1
#define RUN_MODE_FIRST_GO_HOME              2
#define RUN_MODE_PRE_POSITION      	        3
#define RUN_MODE_CMD_MOVE         	        4
#define RUN_MODE_BREAK_Cylinder_LIMIT       5
#define RUN_MODE_INIT                       6
#define RUN_MODE_SEARCH_HOME_MOVE_OFFSET    7
#endif
char *runmodeStrings[] = 
{   "RUN_MODE_STOP : 0", 
    "RUN_MODE_NORMAL : 1",
    "RUN_MODE_FIRST_GO_HOME : 2",
    "RUN_MODE_PRE_POSITION : 3",
    "RUN_MODE_CMD_MOVE : 4",
    "RUN_MODE_BREAK_Cylinder_LIMIT : 5",
    "RUN_MODE_INIT : 6",
    "RUN_MODE_SEARCH_HOME_MOVE_OFFSET : 7",
};

void cmdRunMode(void)
{
	String arg1, arg2;
	int runmode = 0;
	int motorindex = 0;
	
	getNextArg(arg1);
	if( (arg1.length()==0))
	{
	    for(int i=0; i<8; i++){
            cmd_port->println(runmodeStrings[i]);
        }
        cmd_port->println("---------------------");
        cmd_port->println("Now run mode: " + String(runtimedata.motorstate.RunMode));
		return;
	}
	runmode = arg1.toInt();
	runtimedata.motorstate.RunMode = runmode;
	cmd_port->println("Run mode: " + String(runtimedata.motorstate.RunMode));
}

void cmdOffset(void)
{
	String arg1;
	int Offset = 0;
	
	getNextArg(arg1);
	if( (arg1.length()==0))
	{
        cmd_port->println("Motor Offset: " + String(maindata.MotorOffset));
		return;
	}
	Offset = arg1.toInt();
	maindata.MotorOffset = Offset;
	runtimedata.UpdateEEPROM = true;
	cmd_port->println("Motor Offset: " + String(maindata.MotorOffset));
}

void cmdStepsForDepth(void)
{
	String arg1, arg2;
	int runmode = 0;
	int motorindex = 0;
    getNextArg(arg1);
	if( (arg1.length()==0))
	{
        cmd_port->println("Please input enough parameters");
        cmd_port->println("Now Steps For Degree: " + String(maindata.StepsForDepth));
		return;
	}
	//if(getNextArg(arg1))
	{
		maindata.StepsForDepth = arg1.toInt();
		runtimedata.UpdateEEPROM = true;
	}
	cmd_port->println("Set Steps For Degree: " + String(maindata.StepsForDepth));
}

void cmdgetsetPosition(void)
{
	String arg1, arg2;
	unsigned long motorNumber, steps;
	
	getNextArg(arg1);
	if( (arg1.length()==0))
	{
	  cmd_port->println("Please input enough parameters");
	  return;
	}
	motorNumber = arg1.toInt();

	getNextArg(arg2);
	if(arg2.length()>0)
		motor[motorNumber]->setPosition(arg2.toInt());
	cmd_port->println("Motor" + String(motorNumber) + " Position: " + String(motor[motorNumber]->getPosition()));
		
}


void cmdMotorStep(void)
{

	String arg1, arg2, arg3;
	long motorNumber, stepToMove, frequece;

	getNextArg(arg1);
	getNextArg(arg2);
	getNextArg(arg3);
	if( (arg1.length()==0)||(arg2.length()==0) )
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	motorNumber = arg1.toInt();
	stepToMove = arg2.toInt();
	if(arg3.length()==0)
	{
		if(motorNumber < MOTOR_TOTAL)
		{
			//if(motor[motorNumber]->getAccelerateTime() == 0)
			//	motor[motorNumber]->setAccelerateTime(200);
			targetcnt = motor[motorNumber]->Steps(stepToMove) >> 1; 
		}
		
	}
	else
	{
		frequece = arg3.toInt();
		if(motorNumber < MOTOR_TOTAL)
		{
			if(motor[motorNumber]->getAccelerateTime() == 0)
				motor[motorNumber]->setAccelerateTime(200);
			targetcnt = motor[motorNumber]->Steps(stepToMove, frequece) >> 1; 
		}
	}

}

void cmdMotorMoveTo(void)
{
	String arg1, arg2, arg3;
	long motorNumber, targetposition, frequece;

	getNextArg(arg1);
	getNextArg(arg2);
	getNextArg(arg3);
	if( (arg1.length()==0)||(arg2.length()==0))
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	motorNumber = arg1.toInt();
	targetposition = arg2.toInt();
	if(arg3.length()==0)
	{
		if(motorNumber < MOTOR_TOTAL)
			targetcnt = motor[motorNumber]->MoveTo(targetposition) >> 1; 
	}
	else
	{
		frequece = arg3.toInt();
		if(motorNumber < MOTOR_TOTAL)
			targetcnt = motor[motorNumber]->MoveTo(targetposition, frequece) >> 1; 
	}
}


void cmdMotorStop(void)
{
	String arg1, arg2;
	unsigned long motorNumber, steps;

	getNextArg(arg1);
	if( (arg1.length()==0))
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	motorNumber = arg1.toInt();
	if(motorNumber < MOTOR_TOTAL)
	{
		runtimedata.motorstate.RunMode = RUN_MODE_STOP;
		motor[motorNumber]->Stop();
	}
	else
		cmd_port->println("unknown motor number"); 
}

void cmdMotorAccelerate(void)
{
	String arg1, arg2;
	unsigned long motorNumber, steps;

	getNextArg(arg1);
	if( (arg1.length()==0))
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	motorNumber = arg1.toInt();
	if(motorNumber < MOTOR_TOTAL) 
		motor[motorNumber]->Accelerate(); 
	else
		cmd_port->println("unknown motor number"); 
  
}

void cmdMotorSetRPM(void)
{
	String arg1, arg2;
	unsigned long motorNumber, rpm;

	getNextArg(arg1);
	getNextArg(arg2);
	if( (arg1.length()==0)||(arg2.length()==0) )
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	motorNumber = arg1.toInt();
	rpm = arg2.toInt();
	if(motorNumber < MOTOR_TOTAL) 
		motor[motorNumber]->setRPM((unsigned long)rpm); 
	else
		cmd_port->println("unknown motor number"); 
}

void cmdMotorSpeed()
{
	String arg1, arg2, arg3, arg4;
	int motorNumber;
	long freq, freqstartup;
	int acctime = 0;
	
	if(!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	if(!getNextArg(arg2))
	{
	  cmd_port->println("No parameter 2");
	  return;
	}
	motorNumber = arg1.toInt();
	freq = arg2.toInt();
	getNextArg(arg3);
	if(arg3.length()>0)
	{
		freqstartup = arg3.toInt();
		getNextArg(arg4);
		if(arg4.length()>0)
			acctime = arg4.toInt();
	}
	
	if(motorNumber < MOTOR_TOTAL)
		targetcnt = motor[motorNumber]->Speed(freq, freqstartup, acctime) >> 1; 
	
}

void cmdMotorInfo()
{
	String arg1, arg2, arg3, arg4;
	int motorNumber;
	RotateInfo decinfo;
	RotateInfo accinfo;
	
	
	if(!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	motorNumber = arg1.toInt();

	cmd_port->println("State: " + String(motor[motorNumber]->getState(), DEC));
	cmd_port->println("Pin: " + String(motor[motorNumber]->getPulsePin(), DEC) + ", " + String(motor[motorNumber]->getDirPin(), DEC));
	cmd_port->println("Direction: " + String(motor[motorNumber]->getDirection(), DEC));
	cmd_port->println("Resolution: " + String(motor[motorNumber]->getResolution(), DEC));
	cmd_port->println("RPM: " + String(motor[motorNumber]->getRPM()));
	cmd_port->println("Freq Target: " + String(motor[motorNumber]->getFrequence(), DEC) + ", Startup: " + String(motor[motorNumber]->getFrequenceStartup(), DEC) + ", Now: " + String(motor[motorNumber]->getFrequenceNow(), DEC) + ", Rotate Mode: " + String(motor[motorNumber]->getRotateMode(), DEC));
	cmd_port->println("Accelerate time: " + String(motor[motorNumber]->getAccelerateTime(), DEC));
	cmd_port->println("TargetPosition: " + String(motor[motorNumber]->getTargetPosition(), DEC));
	cmd_port->println("Position: " + String(motor[motorNumber]->getPosition(), DEC) + ", PWMOnOff: " + String(motor[motorNumber]->getPWMOnOff(motor[motorNumber]->getTimer())));	
}

void cmdGetADC_PWM()
{
	String arg1;
	int value;
	if(!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	value = arg1.toInt();
    if(value/2 == 0)
        cmd_port->println(digitalRead(ADC_PWMPin[value]));
    else
        cmd_port->println(analogRead(ADC_PWMPin[value]));
}

void cmdSetADC_PWM()
{
	String arg1, arg2;
	int Pin,value;
	if(!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	if(!getNextArg(arg2))
	{
	  cmd_port->println("No parameter 2");
	  return;
	}
	Pin = arg1.toInt();
	value = arg2.toInt();
    if(value/2 == 0)
        digitalWrite(ADC_PWMPin[Pin], value ? HIGH : LOW);
    else
        analogWrite(ADC_PWMPin[Pin], value ? HIGH : LOW);
}


void cmd_ScanLCD_Address(void)
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16)
            Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error==4)
        {
            Serial.print("Unknow error at address 0x");
            if (address<16)
            Serial.print("0");
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

}

void cmdSaveEEPROM(void)
{
    runtimedata.UpdateEEPROM = true;
}
void cmdClearEEPROM(void)
{
    Clear_EEPROM();
}
void cmdReadEEPROM(void)
{
#if 0
char        Vendor[10];
uint8_t     HMI_ID;
long        MotorAcceleration;
long        MotorEESpeed;           //工程技師人員速度
long        MotorOPSpeed;           //操作人員速度
long        MotorInitSpeed;
long        MotorOffset;
uint16_t    ValueReInitMagicCode;
uint32_t    StepsForDepth = 100;    //0.1mm 需要多少pulse
uint32_t    PressTimes;             //按壓多少次1次=0.1mm, 20次=2mm 
uint32_t    MaxPressTimes;          //最大按壓次數
float       CylinderLimitDistance;  //移動氣壓缸機構限制深度

#endif
#if 1
    READ_EEPROM();
    cmd_port->println("Vendor: " + String(maindata.Vendor));
    cmd_port->println("HMI_ID: " + String(maindata.HMI_ID));
    cmd_port->println("MotorAcceleration: " + String(maindata.MotorAcceleration));
    cmd_port->println("MotorEESpeed: " + String(maindata.MotorEESpeed));
    cmd_port->println("MotorOPSpeed: " + String(maindata.MotorOPSpeed));
    cmd_port->println("MotorInitSpeed: " + String(maindata.MotorInitSpeed));
    cmd_port->println("MotorOffset: " + String(maindata.MotorOffset));
    cmd_port->println("ValueReInitMagicCode: " + String(maindata.ValueReInitMagicCode));
    cmd_port->println("StepsForDepth: " + String(maindata.StepsForDepth));
    cmd_port->println("PressTimes: " + String(maindata.PressTimes));
    cmd_port->println("MaxPressTimes: " + String(maindata.MaxPressTimes));
    cmd_port->println("CylinderLimitDistance: " + String(maindata.CylinderLimitDistance));
    for(int i=0; i<CALIBRATION_MAX_NUM; i++){
        cmd_port->print(i);
        cmd_port->print(": ");
        for(int j=0; j<2; j++){
            cmd_port->print(maindata.Std_Act_Distance[i][j]*0.01, 2);
            cmd_port->print(", ");
        }
        cmd_port->println();
    }
#endif
}

void cmdMotorEESpeed()
{
    String arg1;
    long value;
    if(!getNextArg(arg1))
    {
        cmd_port->println("No parameter EESpeed");
        cmd_port->println("EESpeed: " + String(maindata.MotorEESpeed));
        return;
    }
    value = arg1.toInt();
    maindata.MotorEESpeed = value;
    cmd_port->println("New EESpeed: " + String(maindata.MotorEESpeed));
    runtimedata.UpdateEEPROM = true;
}
void cmdMotorOPSpeed()
{
    String arg1;
    long value;
    if(!getNextArg(arg1))
    {
        cmd_port->println("No parameter OPSpeed");
        cmd_port->println("OPSpeed: " + String(maindata.MotorOPSpeed));
        return;
    }
    value = arg1.toInt();
    maindata.MotorOPSpeed = value;
    cmd_port->println("New OPSpeed: " + String(maindata.MotorOPSpeed));
    runtimedata.UpdateEEPROM = true;
}

void cmdMotorInitSpeed()
{
    String arg1;
    long value;
    if(!getNextArg(arg1))
    {
        cmd_port->println("No parameter InitSpeed");
        cmd_port->println("InitSpeed: " + String(maindata.MotorInitSpeed));
        return;
    }
    value = arg1.toInt();
    maindata.MotorInitSpeed = value;
    cmd_port->println("New InitSpeed: " + String(maindata.MotorInitSpeed));
    runtimedata.UpdateEEPROM = true;
}
void cmdPressTimes()
{
    String arg1;
    long value;
    if(!getNextArg(arg1))
    {
        cmd_port->println("No parameter PressTimes");
        cmd_port->println("PressTimes: " + String(maindata.PressTimes));
        return;
    }
    value = arg1.toInt();
    maindata.PressTimes = value;
    cmd_port->println("New PressTimes: " + String(maindata.PressTimes));
    runtimedata.UpdateEEPROM = true;
}
void cmdMaxPressTimes()
{
    String arg1;
    long value;
    if(!getNextArg(arg1))
    {
        cmd_port->println("No parameter MaxPressTimes");
        cmd_port->println("MaxPressTimes: " + String(maindata.MaxPressTimes));
        return;
    }
    value = arg1.toInt();
    maindata.MaxPressTimes = value;
    cmd_port->println("New MaxPressTimes: " + String(maindata.MaxPressTimes));
    runtimedata.UpdateEEPROM = true;
}

void cmdCylinderLimitDistance()
{
    String arg1;
    float value;
    if(!getNextArg(arg1))
    {
        cmd_port->println("No parameter CylinderLimitDistance");
        //cmd_port->println("CylinderLimitDistance: " + String(maindata.CylinderLimitDistance));
        return;
    }
    value = arg1.toFloat();
    //maindata.CylinderLimitDistance = value;
    //cmd_port->println("New CylinderLimitDistance: " + String(maindata.CylinderLimitDistance));
    runtimedata.UpdateEEPROM = true;
}

void cmdSlopeFormula()
{
    String arg1, arg2, arg3;
    int num;
    float Std_distance, Act_distance;
	if(!getNextArg(arg1))
	{
		return;
	}
    num = arg1.toInt();
    if(num >= 0 && num < CALIBRATION_MAX_NUM)
    {
        if(!getNextArg(arg2))
    	{
    	    cmd_port->print("maindata cal ");
            cmd_port->print(num);
            cmd_port->print(": ");
            cmd_port->print(maindata.Std_Act_Distance[num][0]);
            cmd_port->print(", ");
            cmd_port->println(maindata.Std_Act_Distance[num][1]);

    	    cmd_port->print("cal ");
            cmd_port->print(num);
            cmd_port->print(": ");
            cmd_port->print((float)maindata.Std_Act_Distance[num][0]*0.001, 3);
            cmd_port->print(", ");
            cmd_port->println((float)maindata.Std_Act_Distance[num][1]*0.001, 3);
    		return;
    	}
        if(!getNextArg(arg3))
    	{
    		return;
    	}
        Std_distance = arg2.toFloat();
        Act_distance = arg3.toFloat();
        cmd_port->print("Std_distance: ");
        cmd_port->println(Std_distance, 3);
        cmd_port->print("Act_distance: ");
        cmd_port->println(Act_distance, 3);
        maindata.Std_Act_Distance[num][0] = Std_distance*1000.0;
        maindata.Std_Act_Distance[num][1] = Act_distance*1000.0;
        cmd_port->print("maindata.Std_distance: ");
        cmd_port->println(maindata.Std_Act_Distance[num][0]);
        cmd_port->print("maindata.Act_distance: ");
        cmd_port->println(maindata.Std_Act_Distance[num][1]);
        SlopeCalculate_M_b(num);
    }
    else{
        cmd_port->println("Calibration min num: " + String(0));
        cmd_port->println("Calibration max num: " + String(CALIBRATION_MAX_NUM));
        cmd_port->println(String(num) + " is error num.");
    }
}
void cmdCalibration_PWM_Count()
{
    
    String arg1;
    float dipth_mm;
    long pulse;
    if(!getNextArg(arg1))
    {
        return;
    }
    dipth_mm = arg1.toFloat();
    if(dipth_mm <= (float)maindata.MaxPressTimes*0.1 && dipth_mm >= 0)
    {
        pulse = Calibration_PWM_Count(dipth_mm);
        cmd_port->print("dipth_mm: ");
        cmd_port->print(dipth_mm, 1);
        cmd_port->print("-->");
        cmd_port->println("Cal pulse: " + String(pulse));
    }
    else
    {
        cmd_port->println("MaxPressTimes: " + String(maindata.MaxPressTimes));
        cmd_port->println("Min Depth: 0");
        cmd_port->print("Max Depth: ");
        cmd_port->println((float)maindata.MaxPressTimes*0.1, 1);
        cmd_port->print("Input value: ");
        cmd_port->print(dipth_mm, 1);
        cmd_port->println(" is error.");
        return;
    }
}

//-----------------------------------------------------------------------------------
uint8_t UserCommWorkindex = 0;
uint32_t UserCommandTimeCnt = 0;
void UserCommand_Task(void)
{
  int i, incomingBytes, ret, cmdPortIndex;
  char data[2] = {0};

	switch(UserCommWorkindex)
	{
		case 0:
		{
			
			if(cmd_port->available())
			{
				g_inputBuffer = &g_inputBuffer0;
				UserCommWorkindex ++;
				UserCommandTimeCnt = millis();
			}
			break;
		}
		case 1:
		{
			if((millis() - UserCommandTimeCnt) > 50)
				UserCommWorkindex ++;
			break;
		}
		case 2:
		{
		  if ( incomingBytes = cmd_port->available() )
		  {

			cmd_port->println("cmd_port datalen: " + String(incomingBytes));

			for ( i = 0; i < incomingBytes; i++ )
			{
			  ret = cmd_port->read();
			  if ( (ret >= 0x20) && (ret <= 0x7E) || (ret == 0x0D) || (ret == 0x0A) )
			  {
				data[0] = (char)ret;
				(*g_inputBuffer) += data;
				if (g_echoOn)
				{
				  if ( (data[0] != 0x0D) && (data[0] != 0x0A) )
					cmd_port->write(data);
				}
			  }
			  else if (ret == 0x08)
			  {
				if (g_inputBuffer->length())
				{
				  g_inputBuffer->remove(g_inputBuffer->length() - 1);
				  if (g_echoOn)
				  {
					data[0] = 0x08;
					cmd_port->write(data);
				  }
				}
			  }
			}
			if (g_inputBuffer->indexOf('\r') == -1)
			{
			  if (g_inputBuffer->indexOf('\n') == -1)
				return;
			}
			g_inputBuffer->trim();
			while (g_inputBuffer->indexOf('\r') != -1)
			  g_inputBuffer->remove(g_inputBuffer->indexOf('\r'), 1);
			while (g_inputBuffer->indexOf('\n') != -1)
			  g_inputBuffer->remove(g_inputBuffer->indexOf('\n'), 1);
			while (g_inputBuffer->indexOf("  ") != -1)
			  g_inputBuffer->remove(g_inputBuffer->indexOf("  "), 1);
		
			cmd_port->println();
		
			if (g_inputBuffer->length())
			{
			  g_arg.remove(0);
			  if (g_inputBuffer->indexOf(" ") == -1)
				g_cmd = (*g_inputBuffer);
			  else
			  {
				g_cmd = g_inputBuffer->substring(0, g_inputBuffer->indexOf(" "));
				g_arg = g_inputBuffer->substring(g_inputBuffer->indexOf(" ") + 1);
			  }
			  for (i = 0; i < (sizeof(g_cmdFunc) / sizeof(CMD)); i++)
			  {
				//if(g_cmd==g_cmdFunc[i].cmd)
				if (g_cmd.equalsIgnoreCase(g_cmdFunc[i].cmd))
				{
				  g_cmdFunc[i].func();
				  cmd_port->print("ARDUINO>");
				  break;
				}
				else if (i == (sizeof(g_cmdFunc) / sizeof(CMD) - 1))
				{
				  cmd_port->println("bad command !!");
				  cmd_port->print("ARDUINO>");
				}
			  }
			  *g_inputBuffer = "";
			}
			else
			{
			  cmd_port->print("ARDUINO>");
			}
			UserCommWorkindex = 0;
			break;
		}
	}

  	}
}


