#include "main.h"
#include "INS_task.h"

u8 offset_OK_flag=1;//校准时0，main.c中把读取开了，校准完给1，注释了read
extern uint16_t Man,WCIS;
float OFFSET_Buffer[6];
bmi088_real_data_t bmi088_real_data;
IMU_angle imu_angle = {0};
 fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
 fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
 fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //陀螺仪零漂
 fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
 fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //加速度零漂
 const float TimingTime = INS_DELTA_TICK * 0.001f;   //任务运行的时间 单位 s
 fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数
 fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
 fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
 fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
//static fp32 INS_tempature = 0.0f;
fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad

static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * Gyro_Scale_Factor[i][0] + bmi088->gyro[1] * Gyro_Scale_Factor[i][1] + bmi088->gyro[2] * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
        accel[i] = bmi088->accel[0] * Accel_Scale_Factor[i][0] + bmi088->accel[1] * Accel_Scale_Factor[i][1] + bmi088->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
    }
}


void man(void)
{
		BMI088_read_(&bmi088_real_data);
  //减去零漂以及旋转坐标系
  IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data);
}

void INS_task()
{
	BMI088_read_(&bmi088_real_data);
  //减去零漂以及旋转坐标系
  IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data);
	
		//加速度计低通滤波
	static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
	static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
	static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
	static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
	
		//判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
	static uint8_t updata_count = 0;
	if (updata_count == 0)
	{
			//初始化四元数		
			AHRS_init(INS_quat, INS_accel, INS_mag);
			get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

			accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
			accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
			accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
			updata_count++;
	}
	else
	{
			//加速度计低通滤波
			accel_fliter_1[0] = accel_fliter_2[0];
			accel_fliter_2[0] = accel_fliter_3[0];

			accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

			accel_fliter_1[1] = accel_fliter_2[1];
			accel_fliter_2[1] = accel_fliter_3[1];

			accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

			accel_fliter_1[2] = accel_fliter_2[2];
			accel_fliter_2[2] = accel_fliter_3[2];

			accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

		
			//更新四元数
			AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
			get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
			
			imu_angle.last_yaw = imu_angle.yaw;
			imu_angle.yaw = INS_Angle[0] * 57.3f;//yaw
			imu_angle.pitch = INS_Angle[1] * 57.3f;//pitch
			imu_angle.roll = INS_Angle[2]* 57.3f;//roll
			imu_angle.yaw_diff = imu_angle.yaw - imu_angle.last_yaw;
			if(imu_angle.yaw_diff > 150.0f)
				imu_angle.yaw_counts --;
			else if(imu_angle.yaw_diff < -150.0f)
				imu_angle.yaw_counts ++;
			imu_angle.yaw_real = imu_angle.yaw + imu_angle.yaw_counts * 360.0f;
			imu_angle.gyro_z = INS_gyro[2] * 57.3f;//yaw
			imu_angle.gyro_y = INS_gyro[1] * 57.3f;//pitch
			imu_angle.gyro_x = INS_gyro[0] * 57.3f;//roll
			//陀螺仪开机校准
			if(offset_OK_flag == 1)
			{
					static uint16_t start_gyro_cali_time = 0;
					if(start_gyro_cali_time == 0)
					{		
							Gyro_Offset[0] = gyro_cali_offset[0];
							Gyro_Offset[1] = gyro_cali_offset[1];
							Gyro_Offset[2] = gyro_cali_offset[2];
							start_gyro_cali_time++;
							offset_OK_flag = 2;
					}

			}//陀螺仪开机校准   code end

	}          
}

uint16_t count_time = 0;
int8_t cali_gyro_hook(void)
{
	imu_cali_t local_cali_t;
	INS_cali_gyro(local_cali_t.scale, local_cali_t.offset, &count_time);
	if (count_time > GYRO_CALIBRATE_TIME)
	{
		count_time = GYRO_CALIBRATE_TIME+1;
		OFFSET_Buffer[0] = local_cali_t.offset[0];
		OFFSET_Buffer[1] = local_cali_t.offset[1];
		OFFSET_Buffer[2] = local_cali_t.offset[2];
		
		flash_write();
		if(WCIS==0)
		{
		Read_Offset();
		return 1;
		}
		else
		{
			WCIS=0;
			count_time=0;
		OFFSET_Buffer[0]=0.0f;
		OFFSET_Buffer[1]=0.0f;
		OFFSET_Buffer[2]=0.0f;
			return 0;
		}
	}
	else
	{
//		led_red_on();
		return 0;
	}
	
}
/**
  * @brief          校准陀螺仪
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[in]      陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         返回空
  */

void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
	if( *time_count == 0)
	{
			Gyro_Offset[0] = gyro_cali_offset[0];
			Gyro_Offset[1] = gyro_cali_offset[1];
			Gyro_Offset[2] = gyro_cali_offset[2];
	}
	gyro_offset(Gyro_Offset, INS_gyro, 0, time_count);

	cali_offset[0] = Gyro_Offset[0];
	cali_offset[1] = Gyro_Offset[1];
	cali_offset[2] = Gyro_Offset[2];
	

}
void gyro_offset(float gyro_offset[3], float gyro[3], uint8_t imu_status, uint16_t *offset_time_count)
{

		gyro_offset[0] = gyro_offset[0] - GYRO_OFFSET_KP * gyro[0];
		gyro_offset[1] = gyro_offset[1] - GYRO_OFFSET_KP * gyro[1];
		gyro_offset[2] = gyro_offset[2] - GYRO_OFFSET_KP * gyro[2];
//		gyro_offset[0] = gyro_offset[0] - gyro[0]/GYRO_CALIBRATE_TIME;
//		gyro_offset[1] = gyro_offset[1] - gyro[1]/GYRO_CALIBRATE_TIME;
//		gyro_offset[2] = gyro_offset[2] - gyro[2]/GYRO_CALIBRATE_TIME;
		(*offset_time_count)++;
}


/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         返回空
  */

int16_t ins_set_counts = 0;
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[6])
{
	  ins_set_counts++;
    gyro_cali_offset[0]  = cali_offset[0];
    gyro_cali_offset[1]  = cali_offset[1];
    gyro_cali_offset[2]  = cali_offset[2];
		for(int i = 0; i < 3; i++)
		{
			if(fabs(gyro_cali_offset[i]) > 999.0f)
				gyro_cali_offset[0] = gyro_cali_offset[1] = gyro_cali_offset[2] = 0.0f;
		}

}
