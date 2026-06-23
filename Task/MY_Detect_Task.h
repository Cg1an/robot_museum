#ifndef _MY_Detect_Task_H
#define _MY_Detect_Task_H
#include "main.h"

extern u8 firemotor;
extern float rc_X, rc_Y;
extern uint8_t spin,win,Shift;
void Keyboard_coltrol(void);
void mecanum_Resolving(void);
extern void Start_Detect_Task(void const * argument);

#endif
