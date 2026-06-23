/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "crc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern DMA_HandleTypeDef hdma_usart3_rx;
u8 dbus_recieve_1[36],dbus_recieve_2[36];
// 数据包结构定义
typedef struct {
    uint8_t header;      // 0xA4
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
    uint16_t checksum;
} __attribute__((packed)) ReceivePacketTwist;
// 串口接收缓冲区
#define RX_BUFFER_SIZE sizeof(ReceivePacketTwist)
uint8_t uart_rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

// 底盘参数(根据实际底盘调整)
#define WHEEL_BASE 0.50f      // 左右驱动轮间距(米)
#define WHEEL_RADIUS 0.05f    // 驱动轮半径(米)
#define LINEAR_THRESHOLD 7   // m/s  
#define ANGULAR_THRESHOLD 10   // rad/s  


int16_t left_motor_rpm;
int16_t right_motor_rpm;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
uint8_t YiNianTongTian;
uint16_t calculate_checksum(uint8_t* data, uint16_t len);
void process_velocity_command(ReceivePacketTwist* packet);
int16_t wheel_speed_to_rpm(float wheel_speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t rx_byte;
   HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_TIM_Base_Start_IT(&htim6);
	can_filter_init();
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);              //CAN�ж�ʹ��
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	//HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	//HAL_NVIC_EnableIRQ(USART1_IRQn);

	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//开启空闲中断
	__HAL_DMA_DISABLE(&hdma_usart3_rx);  //失能DMA,解除保护
	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
	}
	__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, DMA_LISR_TCIF1);
	hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
	hdma_usart3_rx.Instance->M0AR = (uint32_t)(dbus_recieve_1);
	hdma_usart3_rx.Instance->M1AR = (uint32_t)(dbus_recieve_2);
	__HAL_DMA_SET_COUNTER(&hdma_usart3_rx, 36u);
	SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
	__HAL_DMA_ENABLE(&hdma_usart3_rx);
	
	YiNianTongTian=BMI088_init();
  while(YiNianTongTian)
  {
	  YiNianTongTian=BMI088_init();
  }
		Read_Offset();
        MX_USB_DEVICE_Init();
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static uint8_t rx_byte;
        
        // 寻找帧头
        if (rx_index == 0 && rx_byte != 0xA4) {
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
            return;
        }
        
        uart_rx_buffer[rx_index++] = rx_byte;
        
        // 接收完整数据包
        if (rx_index >= RX_BUFFER_SIZE) {
            // CRC16校验
            if (Verify_CRC16_Check_Sum(uart_rx_buffer, RX_BUFFER_SIZE)) {
                ReceivePacketTwist* packet = (ReceivePacketTwist*)uart_rx_buffer;
                process_velocity_command(packet);

            }
            
            rx_index = 0;
        }
        
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}


// 两轮差速运动学逆解
void differential_inverse_kinematics(float vx, float omega, float* wheel_speeds) {
    // 左轮速度 = 前进速度 - (角速度 * 轮距/2)
    wheel_speeds[0] = vx - (omega * WHEEL_BASE / 2.0f);  // 左轮
    
    // 右轮速度 = 前进速度 + (角速度 * 轮距/2)
    wheel_speeds[1] = vx + (omega * WHEEL_BASE / 2.0f);  // 右轮
}

// 速度指令处理
void process_velocity_command(ReceivePacketTwist* packet) {
    float wheel_speeds[2];
	if(packet->angular_z >= 1){
		packet->angular_z = 0;
	}
    
	float vx = packet->linear_x / 10.0f;  // 前进速度(m/s)
    float omega = packet->angular_z * 40;        // 旋转角速度(rad/s)
    // 差速底盘只使用linear_x和angular_z
    // linear_y被忽略(差速底盘无法横向移动)
	
	 // 判断运动模式:优先级为转向>前进  
    if (fabs(omega) > ANGULAR_THRESHOLD) {  
        // 转向模式:忽略线速度,只执行旋转  
        vx = 0.0f;  
    } else if (fabs(vx) > LINEAR_THRESHOLD) {  
        // 前进模式:忽略角速度,只执行直线运动  
        omega = 0.0f;  
    } else {  
        // 停止模式  
        vx = 0.0f;  
        omega = 0.0f;  
    }  
	
    differential_inverse_kinematics(
        vx,
        omega,
        wheel_speeds
    );
    
    // 转换为电机转速并发送CAN指令
     left_motor_rpm = wheel_speed_to_rpm(wheel_speeds[0]);
     right_motor_rpm = wheel_speed_to_rpm(wheel_speeds[1]);
	 right_motor_rpm = -right_motor_rpm;
    
}

// 轮速转换为电机RPM
int16_t wheel_speed_to_rpm(float wheel_speed) {
    // 线速度(m/s) -> 转速(RPM)
    float rpm = (wheel_speed / (2.0f * 3.14159f * WHEEL_RADIUS)) * 60.0f;
    
    // 3508电机减速比(如果有)
    #define MOTOR_REDUCTION_RATIO 1.0f
    rpm *= MOTOR_REDUCTION_RATIO;
    
    return (int16_t)rpm;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
