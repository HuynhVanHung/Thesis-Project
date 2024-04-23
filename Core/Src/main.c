/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

  // Defining PID using typedef struct 

  typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float Setpoint;
  float Error;
  float Last_Error;
  float Integral;
  float Derivative;
  float CurrentRPM;
} PID_Controller;

  PID_Controller pid_1;
  PID_Controller pid_2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

  #define in_1_T HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
  #define in_2_T HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)
  #define in_3_T HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
  #define in_4_T HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

  #define in_1_N HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
  #define in_2_N HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
  #define in_3_N HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
  #define in_4_N HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
  
  #define RX_SIZE 12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

// Encoder Resolution (so xung trên 1 vong quay cua Encoder)
const float EncoderResolution = 1500.0;

// Thoi gian lay mau (0.01 giây) = 10ms
const float T = 0.01;

// R la ban kinh banh xe , B la chieu rong cua Robot (khoang cach giua 2 banh)
const float R = 0.0325, B = 0.1, pi = 3.14 ;

short i,a = 0;

// data nhan tu PC va PI 

char str[12];

//uint8_t rxBuffer[RX_SIZE] = {0};

uint8_t data_RX[RX_SIZE] = {0};
uint8_t size = 0;

// bien lua gia tri doc tu encoder 
volatile short EncoderCount_1 = 0;
volatile short EncoderCount_2 = 0;
// float CurrentRPM_1,CurrentRPM_2 = 0;

// Dau ra cua ham pid 
float ControlOutput_1,ControlOutput_2,ControlOutput_3 = 0;

// Van toc goc banh trai va banh phai 
float Velocity_1,Velocity_2 = 0;

// Bien van toc 2 banh 
float V_R, V_L;

// Van toc tiep tuyen va goc lech theta 
float V_Tangent = 0;
float theta = 0;
float omega = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

float Conv_Velocity_R(void);
float Conv_Velocity_L(void) ;

//void PWM_SetDutyCycle(float dutyCycle);
float PID_Update_1(PID_Controller *pid, float Setpoint, float Feedback);
float PID_Update_2(PID_Controller *pid, float Setpoint, float Feedback);
//void PWM_SetDutyCycle_2DC (float dutyCycle_1, float dutyCycle_2);

float Conv_Omega_R (float V_Tangent, float theta);
float Conv_Omega_L (float V_Tangent, float theta);

void PWM_SetDutyCycle_1(float dutyCycle);
void PWM_SetDutyCycle_2(float dutyCycle);

void Recevie_DaTa();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 //ham ngat timer 1 xu li pid voi chu ki lay mau la 10ms 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == htim1.Instance)
 {
     //Tinh toan dieu khien PID
    
    // Nhan du lieu V, theta bang Uart tu raspberry 
    Recevie_DaTa();
    
    // doi theta tu don vi do sang radian 
    //theta = theta * (pi/180);
  
    pid_1.Setpoint = Conv_Omega_R(V_Tangent, theta);
    pid_2.Setpoint = Conv_Omega_L(V_Tangent, theta);
    
    pid_1.CurrentRPM = Conv_Velocity_R();
    pid_2.CurrentRPM = Conv_Velocity_L();
    
    ControlOutput_1 = PID_Update_1(&pid_1, pid_1.Setpoint, pid_1.CurrentRPM);
    ControlOutput_2 = PID_Update_2(&pid_2, pid_2.Setpoint, pid_2.CurrentRPM);
    //ControlOutput_3 = PID_Update_3(&pid_3, pid_3.Setpoint, pid_3.CurrentRPM);
    
    PWM_SetDutyCycle_1(ControlOutput_1);
    PWM_SetDutyCycle_2(ControlOutput_2);
    
    //PWM_SetDutyCycle_2DC(ControlOutput_1, ControlOutput_2);
    // Xoa co ngat timer 1
    //TIM1->SR &= ~TIM_SR_UIF;
 }
}

// HamPID Update 1 va 2 la PID van toc goc cua 2 dong co (vong tren phut)
float PID_Update_1(PID_Controller *pid, float Setpoint, float Feedback) {
  
  pid->Error = Setpoint - Feedback;
  pid->Integral += pid->Error;
  pid->Derivative = pid->Error - pid->Last_Error;

  // Tinh dieu khien PID
  float Output = pid->Kp * pid->Error + pid->Ki * pid->Integral + pid->Kd * pid->Derivative;
  
   pid->Last_Error = pid->Error;

  return Output;
}

float PID_Update_2(PID_Controller *pid, float Setpoint, float Feedback) {
  
  pid->Error = Setpoint - Feedback;
  pid->Integral += pid->Error;
  pid->Derivative = pid->Error - pid->Last_Error;

  // Tinh dieu khien PID
  float Output = pid->Kp * pid->Error + pid->Ki * pid->Integral + pid->Kd * pid->Derivative;
  
   pid->Last_Error = pid->Error;

  return Output;
}


void Recevie_DaTa(){
   // Phân tích chuoi va gan gia tri bien 
    // Sao chép du lieu tu mang uint8_t sang mang char
    for (int i = 0; i < 11; i++) {

          str[i] = (char)data_RX[i];  
    }
    
    sscanf(str,"V%f,T%f", &V_Tangent, &theta);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data_RX, RX_SIZE);
     size = Size;
}


// Ham doc encoder và quy doi van toc
float Conv_Velocity_R(void) {
  //static int EncoderCount = 0;
  //static int PrevEncoderCount = 0;

  // Ðoc giá tri tu Encoder 
  EncoderCount_1 = __HAL_TIM_GET_COUNTER(&htim2);
  
  /* Reset the Counter Register value */
  TIM2->CNT = 0;
  // Tinh van toc tu su thay doi vi tri cua encoder
  Velocity_1 = EncoderCount_1  * 60 / EncoderResolution / T;
  
  //Velocity = (EncoderCount - PrevEncoderCount) * 60 / EncoderResolution / T;
  // Luu gia tri encoder cho lan lay mau ke tiep
  // PrevEncoderCount = EncoderCount;

  return Velocity_1;
}

float Conv_Velocity_L(void) {
  //static int EncoderCount = 0;
  //static int PrevEncoderCount = 0;

  // Ðoc giá tri tu Encoder 
  EncoderCount_2 = __HAL_TIM_GET_COUNTER(&htim3);
  
  /* Reset the Counter Register value */
  TIM3->CNT = 0;
  // Tinh van toc tu su thay doi vi tri cua encoder
  Velocity_2 = EncoderCount_2  * 60 / EncoderResolution / T;
   
  //Velocity = (EncoderCount - PrevEncoderCount) * 60 / EncoderResolution / T;
  // Luu gia tri encoder cho lan lay mau ke tiep
  // PrevEncoderCount = EncoderCount;

  return Velocity_2;
}

//  R = 0.0325, B = 0.2, pi = 3.14 ;
// Tinh toan Van Toc goc banh trai va banh phai theo V tiep tuyen va Theta
float Conv_Omega_R (float V_Tangent, float theta){
  
  //theta = (theta / pi) * 180;   // doi radian sang do 
  // voi R la ban kinh banh xe , B la khoang cach giua hai banh xe (chieu rong cua Robot)
  float Velocity_R = (2*V_Tangent + (B*theta)) / (2*R); // don vi rad/s
  //float Omega_R = ((Velocity_R / R) / 2*pi ) * 60;
  float Omega_R = (Velocity_R * 60) / (2*pi); // doi sang RPM
  
  return -Omega_R;
}

float Conv_Omega_L (float V_Tangent, float theta){
  
  //theta = (theta / pi) * 180;   // doi radian sang do 
  // voi R la ban kinh banh xe , B la khoang cach giua hai banh xe (chieu rong cua Robot)
  float Velocity_L = (2*V_Tangent - (B*theta)) / (2*R); // don vi rad/s
  //float Omega_L = ((Velocity_L / R) / 2*pi ) * 60; 
  float Omega_L = (Velocity_L * 60) / (2*pi);  // doi sang RPM
  
  return Omega_L;
}

// ham pwm tan so 1Khz
void PWM_SetDutyCycle_1(float dutyCycle) {
    //  dutyCycle [0, 100]
  uint16_t pulseWidth_1;
  if(dutyCycle > 0){
    dutyCycle = abs(dutyCycle);
    in_1_T;
    in_2_T;
    pulseWidth_1 = (uint16_t)((dutyCycle / 100.0) * 1000);
  }
  if(dutyCycle < 0) {
    dutyCycle = abs(dutyCycle);
    in_1_N;
    in_2_N;
    pulseWidth_1 = (uint16_t)((dutyCycle / 100.0) * 1000);
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulseWidth_1);
   
}

void PWM_SetDutyCycle_2(float dutyCycle) {
    //  dutyCycle [0, 100]
  uint16_t pulseWidth_2;
  if(dutyCycle > 0){
    dutyCycle = abs(dutyCycle);
    in_3_N;
    in_4_N;
    pulseWidth_2 = (uint16_t)((dutyCycle / 100.0) * 1000);
  }
  if(dutyCycle < 0) {
    dutyCycle = abs(dutyCycle);
    in_3_T;
    in_4_T;
    pulseWidth_2 = (uint16_t)((dutyCycle / 100.0) * 1000);
  }
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulseWidth_2);
   
}


/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  // Cau hinh PID
  pid_1.Kp = 1.4;
  pid_1.Ki = 0.18;
  pid_1.Kd = 0.16;
  pid_1.Setpoint = 0.0;
  pid_1.Error = 0.0;
  pid_1.Integral = 0.0;
  pid_1.Derivative = 0.0;
  
  pid_2.Kp = 1.4;
  pid_2.Ki = 0.18;      
  pid_2.Kd = 0.16;
  pid_2.Setpoint = 0.0; 
  pid_2.Error = 0.0;
  pid_2.Integral = 0.0;
  pid_2.Derivative = 0.0;
  


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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start_IT(&htim1);
  
  //HAL_UART_Receive_DMA(&huart3, data_RX, sizeof(data_RX));
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data_RX, RX_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2); 
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
 
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 359;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
