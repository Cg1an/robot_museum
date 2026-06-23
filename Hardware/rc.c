#include "rc.h"
#include "main.h"
#include "can.h"
#define RX_BUF_SIZE 18		//DMA接收缓冲区大小
u8 RXBuff[RX_BUF_SIZE];		//DMA接收缓冲区
u8 sendchassic[5];

RC rc; 
 
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern u8 dbus_recieve_1[36],dbus_recieve_2[36];

u8 if_change(u8 flag,u8 last_flag)
{
if(flag!=last_flag) {return 1;}
else {return 0;}
}

u8 if_jump_up(u8 flag,u8 last_flag)
{
if(flag-last_flag==-1) {return 1;}
else {return 0;}
}

uint16_t rrr;
float keyboard_jump_flag_shift,keyboard_jump_flag_e,key_shift_last,key_e_last;
void deal_dbus(u8 *RXBuff)
{

		if(((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff)){
		rc.R_x = ((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff)-1024; 
		sendchassic[0]=RXBuff[0];
		sendchassic[1]=RXBuff[1];}
		if((((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff)){
		rc.R_y = (((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff)-1024;
		sendchassic[2]=RXBuff[2];} //!< Channel 1
		if((((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff)){
		rc.L_x = (((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff)-1024;}//!< Channel 2
		if((((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff)){
		rc.L_y = (((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff)-1024;} //!< Channel 3
		rc.sl = (((RXBuff[5] >> 4)& 0x000C) >> 2); //!< Switch left
		rc.sr = (((RXBuff[5] >> 4)& 0x0003)); //!< Switch left
		int16_t temp;
		temp = 1024 - (RXBuff[16]|(RXBuff[17] << 8));//遥控器侧面的拨轮
		if(temp>3000) {rc.trigger=65535-temp;}
		else {rc.trigger=temp;}

		rc.mouse_x = RXBuff[6] | (RXBuff[7] << 8); //!< Mouse X axis
		rc.mouse_y = RXBuff[8] | (RXBuff[9] << 8); //!< Mouse Y axis
		rc.mouse_z = RXBuff[10] | (RXBuff[11] << 8); //!< Mouse Z axis
		rc.mouse_l = RXBuff[12]; //!< Mouse Left Is Press ?
		rc.mouse_r = RXBuff[13]; //!< Mouse Right Is Press ?
		rc.key = RXBuff[14] | (RXBuff[15] << 8); //!< KeyBoard value
		rc.W = (rc.key&0x01)>>0;
		rc.S = (rc.key&0x02)>>1;
		rc.A = (rc.key&0x04)>>2;
		rc.D = (rc.key&0x08)>>3;
		rc.Shift = (rc.key&0x10)>>4;
		rc.Ctrl = (rc.key&0x20)>>5;
		rc.Q = (rc.key&0x40)>>6;
		rc.E = (rc.key&0x80)>>7;
		rc.R = (rc.key&0x100)>>8;
		rc.F = (rc.key&0x200)>>9;
		rc.G = (rc.key&0x400)>>10;
		rc.Z = (rc.key&0x800)>>11;
		rc.X = (rc.key&0x1000)>>12;
		rc.C = (rc.key&0x2000)>>13;
		rc.V = (rc.key&0x4000)>>14;
		rc.B = (rc.key&0x8000)>>15;
		//遥控器值抛出异常
		rc.R_x = rc.R_x > 660 ? 0 : rc.R_x;
		rc.R_x = rc.R_x < -660 ? 0 : rc.R_x;
		
		rc.R_y = rc.R_y > 660 ? 0 : rc.R_y;
		rc.R_y = rc.R_y < -660 ? 0 : rc.R_y;
		
		rc.L_x = rc.L_x > 660 ? 0 : rc.L_x;
		rc.L_x = rc.L_x < -660 ? 0 : rc.L_x;
		
		rc.L_y = rc.L_y > 660 ? 0 : rc.L_y;
		rc.L_y = rc.L_y < -660 ? 0 : rc.L_y;
		
		rc.mouse_x = rc.mouse_x > 1024 ? 0 : rc.mouse_x;
		rc.mouse_x = rc.mouse_x < -1024 ? 0 : rc.mouse_x;
		rc.mouse_y = rc.mouse_y > 1024 ? 0 : rc.mouse_y;
		rc.mouse_y = rc.mouse_y < -1024 ? 0 : rc.mouse_y;
		
		rc.sl_change_flag=if_change(rc.sl,rc.last_sl);
		rc.sr_change_flag=if_change(rc.sr,rc.last_sr);	
    rc.mr_change_flag=if_change(rc.mouse_r,rc.last_mouse_r);		
    rc.ml_change_flag=if_change(rc.mouse_l,rc.last_mouse_l);		
		
		rc.last_sl=rc.sl;
		rc.last_sr=rc.sr;
		rc.last_mouse_r=rc.mouse_r;
		rc.last_mouse_l=rc.mouse_l;
		
}


//////////////////////////////////////////////////////////////////////////////////////

#define ACC_STEP_NUM 1000
unsigned short AccStep[ACC_STEP_NUM] = 
{
	0,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,6,6,7,7,7,7,7,8,8,8,8,9,9,9,9,10,10,10,
	11,11,11,12,12,12,13,13,13,14,14,15,15,16,16,16,17,17,18,18,19,20,20,21,21,22,23,23,
	24,25,25,26,27,28,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,49,51,
	52,54,55,57,58,60,61,63,64,66,68,70,72,73,75,77,79,81,83,85,88,90,92,94,97,99,101,104,
	106,109,111,114,117,119,122,125,128,131,133,136,139,142,146,149,152,155,158,162,165,168,
  171,175,178,182,185,189,192,196,199,203,207,210,214,218,221,225,229,233,236,240,244,248,
	251,255,259,262,266,270,274,277,281,285,288,292,296,299,303,307,310,314,317,321,324,327,
	331,334,337,341,344,347,350,353,357,360,363,366,368,371,374,377,380,382,385,388,390,393,
	395,398,400,403,405,407,410,412,414,416,418,420,422,424,426,428,430,431,433,435,437,438,
	440,441,443,444,446,447,449,450,451,453,454,455,456,458,459,460,461,462,463,464,465,466,
	467,468,469,470,471,471,472,473,474,474,475,476,477,477,478,478,479,480,480,481,481,482,
	482,483,483,484,484,485,485,486,486,486,487,487,488,488,488,489,489,489,490,490,490,490,
	491,491,491,492,492,492,492,492,493,493,493,493,494,494,494,494,494,494,495,495,495,495,
	495,495,495,496,496,496,496,496,496,496,496,497,497
};

float deal_W, deal_A, deal_S, deal_D ;
int t_W, t_A, t_S, t_D;
void remote_fifo()
{
	if(rc.W!=0)
		t_W++;
	else
		t_W--;
	
	if(rc.A!=0)
		t_A++;
	else
		t_A--;
	
	if(rc.S!=0)
		t_S++;
	else
		t_S--;
	
	if(rc.D!=0)
		t_D++;
	else
		t_D--;

	t_W = t_W > 334 ? 334 : t_W;  t_W = t_W < 0 ? 0 : t_W;	
	t_A = t_A > 334 ? 334 : t_A;  t_A = t_A < 0 ? 0 : t_A;	
	t_S = t_S > 334 ? 334 : t_S;  t_S = t_S < 0 ? 0 : t_S;
	t_D = t_D > 334 ? 334 : t_D;  t_D = t_D < 0 ? 0 : t_D;

	deal_W=AccStep[t_W];
	deal_A=AccStep[t_A];
	deal_S=AccStep[t_S];
	deal_D=AccStep[t_D];
}




float rc_X, rc_Y;
uint8_t spin = 3,stop=0,Shift=0,other=0;
void mecanum_Resolving(void)
{//其中一位是所有控制的标志位集合
	
	if (rc.sr==1)
	{
		spin = 1;//spin+
	}
	if(rc.sr==2||rc.sr==3)
	{
		spin = 3;//gimbal
	}
	
	if (rc.sl == 2)//下电
	{
		stop = 1;
	}
	else 
	{
		stop = 0;
	}
	//if(rc.Shift ==1||rc.mouse_r==1||rc.sr==2)//超电       //调试
	if(rc.mouse_r==1)//超电       //调试
	{
	  Shift = 1;
	}
	else
	{
		Shift = 0;
	}
	if(rc.R==1)
	{other = 1;
	}
	else
	{other = 0;
	}
	float rc_x, rc_y, X, Y, aa;  //云台的坐标系
	int W = 0,A = 0,S = 0,D = 0;

	remote_fifo();
	W = keypad_K * deal_W;
	A = keypad_K * deal_A;
	S = keypad_K * deal_S;
	D = keypad_K * deal_D;
	rc_x = -rc.R_x * remote_K;
	rc_y = -rc.R_y * remote_K;
//	
//	Y = W - S + rc_y;
//	X = -A + D + rc_x;
	
	Y = -W + S + rc_y;
	X = +A - D + rc_x;
	
	aa = sqrt(X*X+Y*Y);
	X = aa <limit ? X : X*(limit/aa);
	X = aa >-limit ? X : X*(limit/aa);
	Y = aa <limit ? Y : Y*(limit/aa);
	Y = aa >-limit ? Y : Y*(limit/aa);
	
  //chassis_out_update(X,Y,Shift,spin,stop,other);/////这里发给底盘
}




//串口中断

void USART3_IRQHandler(void)
{
	

	  if(USART3->SR & UART_FLAG_IDLE)//接收到数据
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = 36;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == 18)
            {
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_1, 18);                                               
		deal_dbus(dbus_recieve_1);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 36 - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = 36;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == 18)
            {
		         HAL_UART_Receive_DMA(&huart3, (uint8_t*)dbus_recieve_2, 18);                                               
		         deal_dbus(dbus_recieve_2);
            }
        }
    }

}


