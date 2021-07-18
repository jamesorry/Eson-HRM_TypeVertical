
#include "EEPROM_Function.h"
#include "MainProcess.h"

#define EEPROM_DEBUG	1

extern MainDataStruct maindata;
extern HardwareSerial *cmd_port;


void READ_EEPROM()
{ 
  // 讀取EEPROM，回傳值count代表總共寫入幾個byte 
 
  int count = EEPROM_readAnything(0, maindata);
  #if EEPROM_DEBUG
  	if(cmd_port != NULL)
	{
  		cmd_port->println("Read EEPROM.");
  		cmd_port->print("Vendor: ");
  		cmd_port->println(maindata.Vendor);
  	}
  #endif
  
  if (!memcmp(maindata.Vendor, VENDOR, 10))  //未定義 時寫入初值 NNNNNNNNNNNN  
  {
     //Clear_EEPROM();
  }
  
//  Serial.println(KEEP_DATA.Vendor);
//  Serial.println(KEEP_DATA.RS485_ID);

  Serial.print(count);
  Serial.println(" bytes read.");
  

}

void WRITE_EEPROM()
{  
#if EEPROM_DEBUG	
	if(cmd_port != NULL)cmd_port->println("WRITE_EEPROM Start.");
#endif
//  Serial.println(KEEP_DATA.Vendor);
//  Serial.println(KEEP_DATA.RS485_ID);

  
  // 寫入EEPROM，回傳值count代表總共寫入幾個byte 
  int count = EEPROM_writeAnything(0, maindata);
//  Serial.print(count);
//  Serial.println(" bytes written.");
//  Serial.println("---------------");
  
#if EEPROM_DEBUG	
	if(cmd_port != NULL)cmd_port->println("WRITE_EEPROM End.");
#endif
}


void   Clear_EEPROM()
{ 
	uint8_t i;
#if EEPROM_DEBUG	
  	if(cmd_port != NULL)
	{
  		cmd_port->println("Clear EEPROM.");
  	}
#endif

	memset(&maindata, sizeof(MainDataStruct), 0x00);
	memcpy(maindata.Vendor, VENDOR, 10); 
	
    // 寫入EEPROM，回傳值count代表總共寫入幾個byte 
   int   count = EEPROM_writeAnything(0, maindata);
     
}

template <class T> int EEPROM_writeAnything(int address, const T &data)
{
  const byte *p = (const byte *)(const void *)&data;
  int i, n;
  for(i = 0, n = sizeof(data); i < n; i++)
    EEPROM.write(address++, *p++);
  return i;
}
template <class T> int EEPROM_readAnything(int address, T &data)
{
  byte *p = (byte *)(void *)&data;
  int i, n;
  for(i = 0, n = sizeof(data); i < n; i++)
    *p++ = EEPROM.read(address++);
  return i;
}
