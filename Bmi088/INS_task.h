#ifndef INS_TASK_H
#define INS_TASK_H

#include "stm32f4xx.h"
#include "BMI088driver.h"
typedef float fp32;
#define INS_DELTA_TICK 1 //任务调用的间隔
#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \
//处理陀螺仪，加速度计，磁力计数据的线性度，零漂	
																				
#define GYRO_CALIBRATE_TIME 20000   //较准时间 20000 / 1000 = 20s
#define GYRO_OFFSET_KP 0.0003f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大3
																				
extern u8 offset_OK_flag;																				
typedef struct 
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;
typedef struct 
{
  fp32 yaw;
	fp32 pitch;
	fp32 roll;
	
	fp32 last_yaw;					//将yaw轴角度连续起来
	fp32 yaw_diff;
	fp32 yaw_counts;
	fp32 yaw_real;
	
	fp32 gyro_x;
	fp32 gyro_y;
	fp32 gyro_z;
} IMU_angle;		//真实角度和角速度 （角度制）
extern IMU_angle imu_angle;

int8_t cali_gyro_hook(void);
void man(void);
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
void gyro_offset(float gyro_offset[3], float gyro[3], uint8_t imu_status, uint16_t *offset_time_count);
void INS_task(void);
//static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088);
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[6]);
#endif
