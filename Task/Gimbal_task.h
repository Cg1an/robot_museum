#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

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
//	u8 yaw_back_flag;									//回中标志位
//	u8 pitch_back_flag;									//回中标志位
//	u8 AUTO_MDOE;
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
	float pitch_zimiao;
	float yaw_zimiao;
}GIMBAL_control;

typedef struct{

	float chassis_motor_real[4];	
  float chassis_motor_set[4];
	
	float value_3508_ID[4];//PID计算后的值

	float vx;//前后
	float vy;//左右
	
	float vx_set;   //车体三轴速度的设定
	float vy_set; 

	int   dianrong_flag;
  int   spin_flag;
	int   back_flag;
	
}CHASSIS_Task;

typedef enum{
  CHASSIS_STOP=0,//底盘轮子全给零	
	GIMBAL_FOLLOW_CHASSIS,//底盘不动(闭yaw轴)
	CHASSIS_NO_FOLLOW_GIMBAL_YAW,//不跟随云台
	CHASSIS_SPIN_R,//陀螺模式
	CHASSIS_FOLLOW_GIMBAL_YAW,//跟随云台	
}chassis_mode_e;

typedef enum{
	HAND=0,
    AUTOM,
	BUFF
}gimbal_mode_e;


void Gimbal_Control(void const * argument);

#endif
