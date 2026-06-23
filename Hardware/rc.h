#ifndef RC_H
#define RC_H
#define keypad_K 16.5
#define remote_K 12.5
#define limit 8192
#include "main.h"
typedef struct
{
	int16_t R_x;  //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //右手y    上1684 下364 中1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//拨动左  上1 中3 下2
	uint8_t sr;
	
	uint8_t sl_change_flag;			//拨动左  上1 中3 下2
	uint8_t sr_change_flag;
	uint8_t mr_change_flag;
	uint8_t ml_change_flag;
	
	int16_t mouse_x;	//鼠标
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//鼠标按键
	uint8_t mouse_r;
	uint16_t key;		//键盘
	u8 W;
	u8 S;
	u8 A;
	u8 D;
	u8 Shift;
	u8 Ctrl;
	u8 Q;
	u8 E;
	u8 R;
	u8 F;
	u8 G;
	u8 Z;
	u8 X;
	u8 C;
	u8 V;
	u8 B;
	
	uint8_t last_sr;
	uint8_t last_sl;
	uint8_t last_mouse_l;
	uint8_t last_mouse_r;
	uint8_t last_R;
	uint64_t cnts;
	uint16_t trigger;
	int16_t Pulley;
	uint8_t off_line_flag;
	
}RC;


void chassis_out_update(int16_t x,int16_t y,int8_t accelerate,int8_t spin_mode,int8_t stop,int8_t other); //发送
void mecanum_Resolving(void);    //麦轮结算


extern RC rc; 
void rc_init(void);
void deal_dbus(u8 *RXBuff);

#endif 
