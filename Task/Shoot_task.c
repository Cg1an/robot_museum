#include "main.h"
float L_setST, R_setST, Feed_set,Feed_ji=3500;
int16_t shoot;
//extern int zimiao_flag;
extern int16_t left_motor_rpm;
extern int16_t right_motor_rpm;

int16_t x=3;
float y=1;
float right_motor;
float left_motor;
void Shoot_Control(void const * argument)
{
	PID_Init_All();
	
	for(;;)
	{
	 if(rc.sl==1)
		{
			if(rc.L_y!=0)   //前后
			{
				shoot = rc.L_y*x;
				L_setST = -(shoot*1.27);
				R_setST = (rc.L_y*x);
			
			}
			
			
			else if(rc.R_x>0)//右转
			{
				shoot = rc.R_x*x;
				L_setST = shoot;
				R_setST = rc.R_x*x;
				
			}
			
			else if(rc.R_x<0)//左转 
			{
				shoot = rc.R_x*x;
				L_setST = shoot;
				R_setST = rc.R_x*x;
				
			}
			
			else
			{
				L_setST = 0;
				R_setST = 0;
			}
			
			 Firemotor_run (L_setST,R_setST,SHOOT_L.re_speed,SHOOT_R.re_speed,FEED.re_speed,Feed_set) ;
			
	 }
	 else if(rc.sl==2)
	 {
		  right_motor = right_motor_rpm * y * 1.27;
		  left_motor = left_motor_rpm * y ;
		 right_motor = right_motor > 2000 ? 2000 : right_motor;
		 right_motor = right_motor < -2000 ? -2000 : right_motor;
		 left_motor = left_motor > 2000 ? 2000 : left_motor;
		 left_motor = left_motor < -2000 ? -2000 : left_motor;
		  Firemotor_run (right_motor,left_motor,SHOOT_L.re_speed,SHOOT_R.re_speed,FEED.re_speed,Feed_set) ;

	 }else if(rc.sl==3){
		 Firemotor_run (0.0,0.0,SHOOT_L.re_speed,SHOOT_R.re_speed,FEED.re_speed,Feed_set) ;
	 }

	
	//Firemotor_run ( right_motor_rpm,R_setST,SHOOT_L.filter_rate,SHOOT_R.filter_rate,FEED.re_speed,Feed_set) ;
	 //Firemotor_run (L_setST,R_setST,SHOOT_L.re_speed,SHOOT_R.re_speed,FEED.re_speed,Feed_set) ;

	osDelay(1);
	}
}
