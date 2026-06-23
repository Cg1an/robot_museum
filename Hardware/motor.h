#ifndef MOTHR__H
#define MOTHR__H
#include "stm32f4xx.h"                  // Device header

typedef struct{
	int32_t raw_value;   					 //编码器不经处理的原始值
	int32_t last_raw_value;					 //上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;							 //两次编码器之间的差值
	int32_t temp_count;                      //计数用
	uint8_t buf_count;						 //滤波更新buf用
	int32_t ecd_bias;						 //初始编码器值	
	int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
	int32_t rate_buf[6];	     //buf，for filter
	int32_t round_cnt;						 //圈数
	int32_t filter_rate;					 //速度
	int32_t filter_rate_max;				 //最大速度
	float ecd_angle;						 //角度
	float init_angle; 						 //开始读一次数据
	int16_t current;
	int16_t velo;
	int16_t electricity;
	int16_t re_speed;
	int8_t tempature;	
//	float real_angle;
//	float zero_angle;
//	float relative_zero_angle;
//	float ALL_angle;
//	int tool_flag;
}dji_motor;


typedef struct
{
	double kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	double dout;
	
	float poutmax;
	float ioutmax;
	double doutmax;
	float outmax;
	
	float set;
	float real;
	float out;
	
	float err;							
	double err_last;					
	double err_llast;				
	float integral;					
	
}PID_T;

extern volatile dji_motor FEED,SHOOT_L,SHOOT_R;
void GMmotor_Stop (float Pit_Groy,float Yaw_Groy);
void GMmotor_run (float setPit,float setYaw,float Pit_Groy,float Yaw_Groy, float Pit_ang, float Yaw_ang);
void STmotor_run (float setPit,float setYaw,float Pit_Groy,float Yaw_Groy, float Pit_ang, float Yaw_ang);
void PID_Init_All(void);
void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe);
void GMPID_Init_Zimiao(PID_T*GMPitchPid_Ang_ZM,PID_T*GMYawPid_Ang_ZM,PID_T*GMPitchPid_Spe_ZM,PID_T*GMYawPid_Spe_ZM);
void FirePID_Init(PID_T *Feedpid_Spe,PID_T*STLpid_Spe,PID_T*STRpid_Spe);
void Firemotor_run (float L_setST,float R_setST,float L_realspe,float R_realspe,float Feedget_spe,float Feedset_spe) ;
void GMmotor_Stop (float Pit_Groy,float Yaw_Groy);

#endif
