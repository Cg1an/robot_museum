#include "main.h"
uint8_t usedata[8];
#define RATE_BUF_SIZE 6
#define MYABS(a)    ((a >= 0) ? (a) : -(a)) 
#define VALUE_LIMIT(v, l, h)   ((v) < (l) ? (l) : ((v) > (h) ? (h) : (v)))
#define VALUE_ABS_LIMIT(v, a)  VALUE_LIMIT(v, -a, a)
volatile dji_motor Pitch={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor Yaw={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor FEED,SHOOT_L,SHOOT_R;
PID_T GMPitchPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMPitchPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMPitchPid_Ang_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMPitchPid_Spe_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Ang_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Spe_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T STLpid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T STRpid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T Feedpid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


void getEncoderData(volatile dji_motor *v)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (usedata[0]<<8)|usedata[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
	
	if(fabs((float)v->filter_rate_max) < fabs((float)v->filter_rate))
	{
		v->filter_rate_max = v->filter_rate;
	}
	v->current = (uint16_t)(usedata[4] << 8 | usedata[5]);
	v->tempature = usedata[6];
	v->re_speed= (uint16_t)(usedata[2] << 8 | usedata[3]);//速度用这个

}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{   
  CAN_RxHeaderTypeDef	RxHeader;
//  if(hcan->Instance==CAN1)
//	{
//		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&RxHeader,usedata);
//		switch(RxHeader.StdId)
//		{

//			case 0x205 : getEncoderData(&Pitch);   break;  //6020电机
//			case 0x206 : getEncoderData(&Yaw);   break;		
//	    }	
//	}
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&RxHeader,usedata);
        switch(RxHeader.StdId)                   
        {
			case 0x201 : getEncoderData(&SHOOT_R);  break;  //2006
			case 0x202 : getEncoderData(&SHOOT_L);  break;	  
			case 0x203 : getEncoderData(&FEED);  break;
		
			case 0x205 : getEncoderData(&Pitch);   break;  //6020电机
			case 0x206 : getEncoderData(&Yaw);   break;		
	    }
    }
}



float PID_Cal(PID_T *pid, float get, double set)  // get为实际值  set为目标值
{	
	pid->set = set ;
	pid->real = get;
	
	pid->err_last = pid->err;
	pid->err = pid->set - pid->real;
	pid->integral += pid->err;
	
	pid->pout = pid->kp * pid->err;
	pid->pout = pid->pout < pid->poutmax ? pid->pout : pid->poutmax;
	pid->pout = pid->pout > -pid->poutmax ? pid->pout : -pid->poutmax;
	
	pid->iout = pid->ki * pid->integral;
	pid->iout = pid->iout < pid->ioutmax  ? pid->iout : pid->ioutmax;
	pid->iout = pid->iout > -pid->ioutmax ? pid->iout : -pid->ioutmax;
	
	pid->dout = pid->kd * (pid->err - pid->err_last);
	pid->dout = pid->dout < pid->doutmax ? pid->dout : pid->doutmax;
	pid->dout = pid->dout > -pid->doutmax ? pid->dout : -pid->doutmax;
	
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = pid->out < pid->outmax ? pid->out : pid->outmax;
	pid->out = pid->out > -pid->outmax ? pid->out : -pid->outmax;
	
	return pid->out;
}

//void GMmotor_Stop (float Pit_Groy,float Yaw_Groy) 
//{
//	float M0,M1=0;
//	
////	m0 = PID_Cal(&GMPitchPid_Ang, Pit_ang, setPit);
//	M0 = PID_Cal(&GMPitchPid_Spe, Pit_Groy, 0);
//	
////	m1 = PID_Cal(&GMYawPid_Ang, Yaw_ang, setYaw);
//    M1 = PID_Cal(&GMYawPid_Spe, Yaw_Groy, 0);
//	
//	
//	PHC_GIMBAL(M0,M1);        
//	
//}


//云台
//void GMmotor_run (float setPit,float setYaw,float Pit_Groy,float Yaw_Groy, float Pit_ang, float Yaw_ang) 
//{
//	float m0,m1=0;
//	float M0,M1=0;	

// if(rc.sl !=2)
// {
//	if(rc.sr ==3||rc.sr ==1)//手操
//	{
//	m0 = PID_Cal(&GMPitchPid_Ang, Pit_ang, setPit);
//	M0 = PID_Cal(&GMPitchPid_Spe, Pit_Groy, m0);
//	m1 = PID_Cal(&GMYawPid_Ang, Yaw_ang, setYaw);
//    M1 = PID_Cal(&GMYawPid_Spe, Yaw_Groy, m1);
//	}
////	else if(rc.sr ==2)	//自瞄
////   {
////	m0 = PID_Cal(&GMPitchPid_Ang_ZM, Pit_ang, setPit);
////	M0 = PID_Cal(&GMPitchPid_Spe_ZM, Pit_Groy, m0);
////	m1 = PID_Cal(&GMYawPid_Ang_ZM, Yaw_ang, setYaw);
////    M1 = PID_Cal(&GMYawPid_Spe_ZM, Yaw_Groy, m1);
////   }	
//	//PHC_GIMBAL(M0,M1); 

// }
//	

// else {
//	 	M0 = 0;
//		M1 = 0;
// }


// PHC_GIMBAL(M0,M1); 
//}
//发射机构

float M1,M2,M3 =0;
void Firemotor_run (float L_setST,float R_setST,float L_realspe,float R_realspe,float Feedget_spe,float Feedset_spe) 
{
	//float M1,M2,M3 =0;
	
	M1 = PID_Cal(&STRpid_Spe, R_realspe ,R_setST);
    M2 = PID_Cal(&STLpid_Spe, L_realspe, L_setST);
	M3 = PID_Cal(&Feedpid_Spe, Feedget_spe ,Feedset_spe );

    Set_Shoot_Motor_Current(M1,M2,M3); 
}

/*云台pid*/
void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe)
{
	GMPitchPid_Ang->kp = 28.0f;//62
	GMPitchPid_Ang->ki = 0.01f;//0.1
	GMPitchPid_Ang->kd = 10.0f;//96
	GMPitchPid_Ang->poutmax=12000.0f;
	GMPitchPid_Ang->ioutmax=100.0f;
	GMPitchPid_Ang->doutmax=12000.0f;
	GMPitchPid_Ang->outmax=12000.0f;
	
	GMYawPid_Ang->kp = 20.0f;//20
	GMYawPid_Ang->ki = 0.02f;//0.02
	GMYawPid_Ang->kd = 110.0f;//110
	GMYawPid_Ang->poutmax=8000.0f;
	GMYawPid_Ang->ioutmax=5000.0f;
	GMYawPid_Ang->doutmax=8000.0f;
	GMYawPid_Ang->outmax=11000.0f;  

	GMPitchPid_Spe->kp = 88.0f;  //88
	GMPitchPid_Spe->ki = 0.0f;
	GMPitchPid_Spe->kd = 0.0f;
	GMPitchPid_Spe->poutmax=30000.0f;
	GMPitchPid_Spe->ioutmax=1000.0f;
	GMPitchPid_Spe->doutmax=10000.0f;
	GMPitchPid_Spe->outmax=30000.0f;

	GMYawPid_Spe->kp = 600.0f;   //600 
	GMYawPid_Spe->ki = 0.01f;//0.1
	GMYawPid_Spe->kd = 50.0f; //60
	GMYawPid_Spe->poutmax=30000.0f;
	GMYawPid_Spe->ioutmax=1000.0f;
	GMYawPid_Spe->doutmax=10000.0f;
	GMYawPid_Spe->outmax=30000.0f;
}

void GMPID_Init_Zimiao(PID_T*GMPitchPid_Ang_ZM,PID_T*GMYawPid_Ang_ZM,PID_T*GMPitchPid_Spe_ZM,PID_T*GMYawPid_Spe_ZM)
{
	GMYawPid_Ang_ZM->kp = 28.0f;   //30
	GMYawPid_Ang_ZM->ki = 0.0f;
	GMYawPid_Ang_ZM->kd = 8.0f;//8
	GMYawPid_Ang_ZM->poutmax=10000.0f;
	GMYawPid_Ang_ZM->ioutmax=500.0f;
	GMYawPid_Ang_ZM->doutmax=9000.0f;
	GMYawPid_Ang_ZM->outmax=10000.0f; 
	
    GMYawPid_Spe_ZM->kp = 380.0f;//388 
	GMYawPid_Spe_ZM->ki = 0.0f;
	GMYawPid_Spe_ZM->kd = 40.0; //45
	GMYawPid_Spe_ZM->poutmax=30000.0f;
	GMYawPid_Spe_ZM->ioutmax=1000.0f;
	GMYawPid_Spe_ZM->doutmax=10000.0f;
	GMYawPid_Spe_ZM->outmax=30000.0f;	

	GMPitchPid_Ang_ZM->kp = 57.0f;//60
	GMPitchPid_Ang_ZM->ki = 0.1f;
	GMPitchPid_Ang_ZM->kd = 3.0f;
	GMPitchPid_Ang_ZM->poutmax=10000.0f;
	GMPitchPid_Ang_ZM->ioutmax=100.0f;
	GMPitchPid_Ang_ZM->doutmax=5000.0f;
	GMPitchPid_Ang_ZM->outmax=10000.0f;
	
	GMPitchPid_Spe_ZM->kp = 48.0f; //48
	GMPitchPid_Spe_ZM->ki = 0.0f;
	GMPitchPid_Spe_ZM->kd = 10.0f;
	GMPitchPid_Spe_ZM->poutmax=10000.0f;
	GMPitchPid_Spe_ZM->ioutmax=1000.0f;
	GMPitchPid_Spe_ZM->doutmax=10000.0f;
	GMPitchPid_Spe_ZM->outmax=10000.0f;

	
}
/*发射机构pid*/
void FirePID_Init(PID_T *Feedpid_Spe,PID_T*STLpid_Spe,PID_T*STRpid_Spe)
{
//	Feedpid_Spe->kp = 5.0f ;
//	Feedpid_Spe->ki = 0.0f ;
//	Feedpid_Spe->kd = 0.0f ;
//	Feedpid_Spe->poutmax = 16000.0f ;
//	Feedpid_Spe->ioutmax = 2000.0f;
//	Feedpid_Spe->doutmax = 1000.0f ;
//	Feedpid_Spe->outmax =  16000.0f;
	
	STLpid_Spe->kp = 2.0f;
	STLpid_Spe->ki = 0.0f;
	STLpid_Spe->kd = 0.2f;
	STLpid_Spe->poutmax = 16384.0f ;
	STLpid_Spe->ioutmax = 9000.0f;
	STLpid_Spe->doutmax = 10000.0f ;
	STLpid_Spe->outmax =  16384.0f;
	
	STRpid_Spe->kp = 3.0f;
	STRpid_Spe->ki = 0.0f;//右边积分小点
	STRpid_Spe->kd = 0.0f;
	STRpid_Spe->poutmax = 16384.0f ;
	STRpid_Spe->ioutmax = 9000.0f ;
	STRpid_Spe->doutmax = 10000.0f ;
	STRpid_Spe->outmax =  16384.0f;
	
}

void PID_Init_All(void)
{
//	GMPID_Init_Hand(&GMPitchPid_Ang,&GMYawPid_Ang,&GMPitchPid_Spe,&GMYawPid_Spe);
//	GMPID_Init_Zimiao(&GMPitchPid_Ang_ZM,&GMYawPid_Ang_ZM,&GMPitchPid_Spe_ZM,&GMYawPid_Spe_ZM);
	FirePID_Init(&Feedpid_Spe,&STLpid_Spe,&STRpid_Spe);
}
