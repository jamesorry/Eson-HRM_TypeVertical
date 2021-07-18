#ifndef _HRM_COMMAND_H_
#define	_HRM_COMMAND_H_

#include "Arduino.h"

typedef struct __HRMCMD {
  const char* cmd;
  void (*func)(void);
} HRMCMD, *HRMPCMD;

void HRM_UserCommand_Task(void);
void HRMshowHelp(void);
bool HRMgetNextArg(String &arg);

void fw_version();
void white_card_position_read();
void white_card_height_read();
void white_card_dynamic();
void white_card_speed();
void white_card_save();
void white_card_move_in();
void white_card_move_out();
void white_card_pos_set_offset();
void white_card_zero_offset();
void white_card_limit_depth();
void white_card_max_depth();

#endif //_HRM_COMMAND_H_

