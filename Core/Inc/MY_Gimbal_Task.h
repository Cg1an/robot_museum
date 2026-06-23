#ifndef _MY_Gimbal_Task_H_
#define _MY_Gimbal_Task_H_

#include "main.h"

#define remote_K 12.5
#define limit 8192

#define shoot_frequency 10.0f//拨弹盘转速 
#define single_shoot_speed 600.0f
#define super_shoot_speed 1600.0f
/*
	以下选项只针对当前这21年步兵
	15:
	18:
	30:
*/
#define shoot_15rev 5000.0f//摩擦轮转速  对应速度
#define shoot_18rev 3500.0f
#define shoot_30rev 2300.0f//28射速
typedef struct Feed_Set
{
	float feed_speed_set;
	float feed_speed_real;
	int16_t feed_speed_max;
}Feed_set;


extern Feed_set feed_set;

typedef struct Angle
{
	float yaw_target;
	float pitch_target;							//target 为中间值
	float yaw_set;
	float yaw_speed_set;
	float pitch_set;								//两个轴的设定值
	float pitch_speed_set;
	float pitch_low;
	float pitch_top;
	float yaw_right;
	float yaw_left;									//上下左右极限值
//	float yaw_real;
//	float pitch_real;								//两个轴的真实值
//	float yaw_bias;
//	float pitch_bias;								//两个轴的基础角度
	u8 yaw_back_flag;									//回中标志位
	u8 pitch_back_flag;									//回中标志位
	u8 AUTO_MDOE;
	float pitch_remote_last;
	float pitch_remote_now;
	float pitch_differ;
	float pitch_set_last;
//	void (*f_angle_init)(struct Angle *angle);
//	void (*f_angle_update)(struct Angle *angle);
	float pitch_first;
	float yaw_first;
	
	float yaw_aim_set;
	float pitch_aim_set;
	float shoot_delay_offset;
	float fly_delay_offset;
	float edge_offset;
	float auto_distance;
	float total_offset;
	float auto_angle;
	float pitch_delta;
}GIMBAL_control;

void angle_set_update(void);
void gimbal_start(void);
void mecanum_Resolving(void);
void fireMotorSingleLoopPid_out_update(u8 shoot);


#endif
