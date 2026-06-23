#include "main.h"
u8 firemotor = 0;
float rc_X, rc_Y;
uint8_t spin = 4,win=0,Shift=0;

void Keyboard_coltrol(void)
{
	/*
		F /F+CTRL 开关弹舱盖
		R /R+CTRL 开关摩擦轮
		Q /Q+CTRL 开关小陀螺
		C 使用上限射频  默认射频为 
	  shift 使用超电
	  鼠标右键开启自瞄


	*/
//		if(rc.F == 1)//弹仓盖开合操作
//		{
//			if(rc.Ctrl == 1)//CTRL+F关闭弹仓盖 
//			{
//				magazine_pwm = 2370;
//				magazine_flag = 0;
//			}
//			else	//按F开启弹仓盖
//			{
//				magazine_pwm = 500;
//				magazine_flag = 1;
//			}
//			TIM1->CCR1 = magazine_pwm;
//		}
		
		if(rc.R == 1 || rc.sr == 2)//激活摩擦轮
		{
			if(rc.Ctrl == 1)
				firemotor = 0;
			else
				firemotor = 1;
		}

		if(rc.sr ==2)
			firemotor = 1;
		else
			firemotor = 0;

}


void mecanum_Resolving(void)
{//其中一位是所有控制的标志位集合
	
	if (rc.Q == 1)
	{
		spin = 2;//spin
	}
	if (rc.sl == 1)
	{
		spin = 2;//spin
	}
	
	if (rc.sl == 3)
	{
		spin = 4;//spin
	}
	
	if(rc.E ==1)
	{
		spin = 4;//gimbal
	}
	if (rc.sl == 2)
	{
		win = 1;
	}
	else 
	{
		win = 0;
	}
	if(rc.Shift ==1)
	{
	  Shift = 1;
	}
	else
	{
		Shift = 0;
	}
	float rc_x, rc_y, X, Y, aa;  //云台的坐标系
	int W = 0,A = 0,S = 0,D = 0;

//	remote_fifo();
//	W = keypad_K * deal_W;
//	A = keypad_K * deal_A;
//	S = keypad_K * deal_S;
//	D = keypad_K * deal_D;
	rc_x = rc.R_x * remote_K;
	rc_y = rc.R_y * remote_K;
	
	Y = W - S + rc_y;
	X = -A + D + rc_x;
	
	aa = sqrt(X*X+Y*Y);
	X = aa <limit ? X : X*(limit/aa);
	X = aa >-limit ? X : X*(limit/aa);
	Y = aa <limit ? Y : Y*(limit/aa);
	Y = aa >-limit ? Y : Y*(limit/aa);
  Send_to_chassic(-X,-Y,Shift,spin,win);
}

void Start_Detect_Task(void const * argument)
{
    for(;;)
  {
	Keyboard_coltrol();
	mecanum_Resolving();
	  osDelay(1);
  }
}

