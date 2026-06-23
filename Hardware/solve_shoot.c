/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


#include "main.h"
#include <math.h>
#include <stdio.h>

#include "solve_shoot.h"
struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间



/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = 0.01;
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
//        printf("[WRAN]: Exceeding the maximum range!\n");
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    printf("model %f %f\n", t, z);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle)
{

    return 0;

}
//////////////////////////pitch解算/////////////////////////

float Trajectory_BulletModel(float distance,float bullet_k,float bullet_speed,float bullet_Time,float pitchangle)
{
	distance = sqrt(st.xw*st.xw+st.yw*st.yw);
    float height = 0.f;
    bullet_Time = (float)((exp(bullet_k * distance)- 1.f) / (bullet_k * bullet_speed * cos(pitchangle)));
    height = (float)(bullet_speed * sinf(pitchangle) * bullet_Time - 9.794f * bullet_Time * bullet_Time / 2.f);
    return height;
}

float height_calc = 0.f,pitchangle = 0.f;

float Trajectory_Picth_Update(float distance,float height)
{
    float z_temp = 0.f,dz = 0.f;
    z_temp = height;
    for(uint8_t i=0; i<200; i++)
    {
        pitchangle = (float)atan2(z_temp,distance);
        height_calc = Trajectory_BulletModel(distance,0.038f,20.2,0.4,pitchangle);//24//21.5    0.03
        dz = 0.3f*(height - height_calc);
        z_temp += dz;
        if(fabs(dz) < 0.001f)
        {
            break;
        }
    }
    return pitchangle;
}
 
/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
extern uint8_t get_vision;
float real_yaw;
extern float my_timeDelay;
extern extKalman_t yaw_RV;
float fly_time=0.4;//0.16pppp 1.5zzz
float rui=0.09;
int idx = 0; // 选择的装甲板
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    // 线性预测
   float timeDelay =my_timeDelay;
	  // float timeDelay =0.0142586;
	st.tar_yaw=st.current_yaw;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	if (st.tar_yaw<0 || st.v_yaw<0)
	{
		st.tar_yaw =0;
		st.v_yaw =0;
	}
	int use_1 = 1;
	int i = 0;
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
//			if (abs((int)st.v_yaw)<0.05)
//			{
//				st.v_yaw=0.0;
//			}
            float tmp_yaw = st.tar_yaw + 2.0 * PI/3.0+st.v_yaw*(timeDelay +rui);  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0 +st.v_yaw*(timeDelay +rui);//0.4是发弹延迟改改改2.17
			//float tmp_yaw = st.tar_yaw + i * PI/2.0 ;//0.4是发弹延迟改改改2.17
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw ;
         // tar_position[i].yaw = tmp_yaw +st.v_yaw*(timeDelay +fly_time);
			
            use_1 = !use_1;
        }

            
            //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }

    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
	*aim_y = tar_position[idx].y + st.vyw * (timeDelay +fly_time);//飞行时间

    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}


// 从坐标轴正向看向原点，逆时针方向为正





