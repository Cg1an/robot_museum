#include "main.h"
GIMBAL_control gimbal_control={0};
CHASSIS_Task chassis_task;
chassis_mode_e chassis_mode;
gimbal_mode_e gimbal_mode;
GIMBAL_control gimbal_control;
//extern extKalman_t yaw_RV;
extern int ZM_inter;
float ZMMM;
float ZMMMm;

int zimiao_flag;
extern struct SolveTrajectoryParams st;//+++

void Gimbal_Control(void const * argument)
{
	//KalmanCreate(&yaw_RV,1,40);//0.1   1 Q(大，信测量) R定140
	PID_Init_All();
	for(;;)
	{
		
		if(rc.sr==1)
		{
//			chassis_mode = CHASSIS_SPIN_R;//3  陀螺
			gimbal_control.yaw_set = (rc.L_x / 1000.0f + rc.mouse_x / 400.0f) ;///500
		    gimbal_control.pitch_set = rc.L_y / 1000.0f + rc.mouse_y / 400.0f;
		}
		else if(rc.sr==3||rc.sr==2)
		{
			zimiao_flag=0;
			chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;//4 跟随,手动
			gimbal_control.yaw_set = (rc.L_x / 1000.0f + rc.mouse_x / 400.0f) ;///500
			if(gimbal_control.pitch_set>20)/////////////pitch限位
			{
				gimbal_control.pitch_set
				= 20;
			}
			else if(gimbal_control.pitch_set<-27)
			{
				gimbal_control.pitch_set = -27;
			}
			else
			{
		gimbal_control.pitch_set = rc.L_y / 1000.0f + rc.mouse_y / 500.0f;//800
			}
		}
//		else if(rc.sr==2&&ZM_inter==1)               //自瞄1
//		{		
//		gimbal_control.yaw_set=gimbal_control.yaw_zimiao + imu_angle.yaw_counts * 360.0f;//防开自瞄转头
//		gimbal_control.pitch_set = 57.3*gimbal_control.pitch_zimiao;//角度制
//		ZMMM = imu_angle.yaw_real-gimbal_control.yaw_zimiao;

//		//if(fabs(ZMMM)<0.11f)/////0.07pp////0.4转
//		if((0.0f<ZMMM<0.11f) || (0.11f<ZMMM<0.0f))
//		{
//			zimiao_flag=1;
//		}
//		else{
//			zimiao_flag=0;
//		}
//		}
		
	  //   mecanum_Resolving();//发送信息给底盘c板	
		//Send_Current_Contorl_Data_1(rc.L_x,rc.L_y,rc.Shift,rc.R,rc.sl,rc.sr);//发送信息给底盘c板		

		GMmotor_run(gimbal_control.pitch_set,gimbal_control.yaw_set,imu_angle.gyro_y,imu_angle.gyro_z,imu_angle.pitch,imu_angle.yaw_real);
	osDelay(1);
	
	
	}
}



