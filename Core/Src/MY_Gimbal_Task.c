#include "main.h"

#include "main.h"

extern IMU_angle imu_angle;
extern RC rc;

GIMBAL_control gimbal_control = {0};

Feed_set feed_set = {0};
Feed_set shootL = {0};
Feed_set shootR = {0};
u8 motor_is_spin = 0;
// 30  1050   15 595  18 667   23.5 850  25 900   26 950   27.5 1000  28.5 1050
float speed1,speed2,speed3=0;
float motttspeed = 600;

float BoDan,Shoot_L,Shoot_R;

///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\//////一念通天  神魔无惧\\\\\\


void fireMotorPidReal_update(void)
{
	feed_set.feed_speed_real = INFANTRY_CLIP.filter_rate;
	shootL.feed_speed_real = SHOOT_L.filter_rate;
	shootR.feed_speed_real = SHOOT_R.filter_rate;
//	switch (refreedata.gunlimit)
//	{
//		case 15:
//			 motttspeed = 575;
//		break;
//		case 18:
//			 motttspeed = 667;
//		break;
//		case 30:
//			 motttspeed = 1050;
//		break;
//		default:
//			 motttspeed = 1050;
//		break;
//	}
	if(shootL.feed_speed_real !=0 && shootR.feed_speed_real !=0)
	{
			motor_is_spin = 1;
	}
	else
	{
			motor_is_spin = 0;
	}

//	if((rc.mouse_l == 1) && (((can_data.can_hot > 25 && can_data.can_hot <= 400)) || rc.C) && can_data.shoote_init_flag == 1)
//	{
//		feed_set.feed_speed_set = FEED_SPEED;
//	}
//	else 
//	{
//		feed_set.feed_speed_set = 0;
//	}

//	if((rc.mouse_l == 1 || rc.Pulley !=0)&&motor_is_spin == 1)//鼠标左键以及拨杆侧面
	if((rc.trigger<-20)&&rc.sr==2)
	{
		
		
//		if(refreedata.current_cool + 20 < refreedata.gun_limit_max)
//		{
			feed_set.feed_speed_set = 0 - rc.trigger*3 + rc.mouse_l * single_shoot_speed;	
//		}
//		else
//		{
//			feed_set.feed_speed_set = 0;
//		}	
//		if(rc.mouse_r == 1)
//			feed_set.feed_speed_set = rc.mouse_l * super_shoot_speed ;
	}
	else
	{
		feed_set.feed_speed_set = 0;
	}
	
	if(rc.Z == 1)
	{
		feed_set.feed_speed_set = -100;
	}
}


int shoot_type = 3;
void fireMotorSingleLoopPid_out_update(u8 shoot)
{
	
	switch(shoot_type)//根据裁判系统读取切换当前枪口摩擦轮转速 设定值 
	{
		case 1:
			shootL.feed_speed_set = shoot_15rev;
			shootR.feed_speed_set = shootL.feed_speed_set;
		break;
		
		case 2:
			shootL.feed_speed_set = shoot_18rev;
			shootR.feed_speed_set = shootL.feed_speed_set;
		break;
		
		case 3:
			shootL.feed_speed_set = shoot_30rev;
			shootR.feed_speed_set = shootL.feed_speed_set;
		break;
		
		default:
		break;
	}
	
	fireMotorPidReal_update();//真实值更新
	BoDan = PID_Cal(&FEED_MOTOR_SINGLE, feed_set.feed_speed_real, feed_set.feed_speed_set);
//	out[FEED_MOTOR_SINGLE]  = PID_Cal(&pid[FEED_MOTOR_SINGLE], feed_set.feed_speed_set, feed_set.feed_speed_real);
//	out[FIRE_MOTOR_SPEED_L] = PID_Cal(&pid[FIRE_MOTOR_SPEED_L], shoot_15rev, shootL.feed_speed_real);
//	out[FIRE_MOTOR_SPEED_R] = PID_Cal(&pid[FIRE_MOTOR_SPEED_R], shoot_15rev, shootR.feed_speed_real);	
	if(rc.sl != 2)//计算结束后根据遥控器状态对摩擦轮和拨弹盘电流值更新
	{	
		Set_FeedMotor_Current((int16_t)BoDan);
		if(shoot == 1)
		{
			Shoot_L = PID_Cal(&FIRE_MOTOR_SPEED_L, shootL.feed_speed_real, motttspeed);
			Shoot_R = PID_Cal(&FIRE_MOTOR_SPEED_R, shootR.feed_speed_real, -motttspeed);	
		}
		else
		{
			Shoot_L = PID_Cal(&FIRE_MOTOR_SPEED_L, shootL.feed_speed_real, 0);
	        Shoot_R = PID_Cal(&FIRE_MOTOR_SPEED_R, shootR.feed_speed_real, 0);	
		}
	}
	else
	{
		Set_FeedMotor_Current(0);
		Shoot_L = PID_Cal(&FIRE_MOTOR_SPEED_L, shootL.feed_speed_real, 0);
	    Shoot_R = PID_Cal(&FIRE_MOTOR_SPEED_R, shootR.feed_speed_real, 0);
	}
	Set_Shoot_Motor_Current((int16_t)Shoot_R,(int16_t)Shoot_L);
	Set_FeedMotor_Current(0); ///////////////////////////////////////////////////////////////////
}


///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\//////一念通天  神魔无惧\\\\\\

void angle_set_update(void)
{
	//gimbal back to center
	if(gimbal_control.yaw_back_flag == 0 || gimbal_control.pitch_back_flag == 0)
	{
		
		gimbal_control.yaw_back_flag = 1;

		gimbal_control.pitch_back_flag = 1;
		if(fabs(gimbal_control.pitch_set - gimbal_control.pitch_target) <= 0.1f)
		{
			gimbal_control.pitch_back_flag = 1;
		}
		if(gimbal_control.pitch_back_flag == 0)
		{
			gimbal_control.pitch_set = gimbal_control.pitch_set < gimbal_control.pitch_target ? gimbal_control.pitch_set + 0.035f : gimbal_control.pitch_set - 0.035f;
		}		
	}
	else 
	{
			gimbal_control.yaw_set -= (rc.L_x / 1000.0f - rc.mouse_x / 500.0f) ;
			gimbal_control.pitch_delta = rc.L_y / 1000.0f + rc.mouse_y / 800.0f;
     
				gimbal_control.pitch_set -= gimbal_control.pitch_delta;

	}
}

///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\//////一念通天  神魔无惧\\\\\\

void gimbal_start(void)
{
	motor_set_gimbal(gimbal_control.pitch_set,gimbal_control.yaw_set, imu_angle.pitch, imu_angle.yaw_real);
}

///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\///////一念通天  神魔无惧\\\\\\//////一念通天  神魔无惧\\\\\\


void StartGimbal_Task(void const * argument)
{
  /* USER CODE BEGIN StartGimbal_Task */
  /* Infinite loop */
  for(;;)
  {
		  angle_set_update();
		  gimbal_start();
		  fireMotorSingleLoopPid_out_update(firemotor);
	  
    osDelay(1);
  }
  /* USER CODE END StartGimbal_Task */
}
