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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensirion_common.h"
#include "sgp30.h"
#include <stdio.h>
#include <math.h>

#define PI 3.1415926f

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t measurement_counter = 0;
uint16_t tvoc_ppb = 0, co2_eq_ppm = 0;
int16_t err;

uint16_t tvoc_buf_av = 0;
uint16_t co2_buf_av = 0;







#ifdef __GNUC__
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#else
int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void fan_control_front(uint16_t speed);
void fan_control_back(uint16_t speed);
void fan_control_right(uint16_t speed);
void fan_control_left(uint16_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void fan_control_front(uint16_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);



   printf("front motor speed : %u\n\r",speed);

}

void fan_control_back(uint16_t speed)
{
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);



   printf("back motor speed : %u\n\r",speed);
}

void fan_control_right(uint16_t speed)
{
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);



   printf("right motor speed : %u\n\r",speed);
}

void fan_control_left(uint16_t speed)
{
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);



   printf("left motor speed : %u\n\r",speed);
}



void read_N_printSGP30value()
{//delay 및 과도응답에 대한 로직 포함
   tvoc_buf_av = 0;
   co2_buf_av = 0;


   uint16_t tvoc_buf[8];
   uint16_t co2_buf[8];

    int count = 0;


              while(1)
              { // for (int i = 0; i < 8; i++)

                if(count >= 5) {break;}

                  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);


                  if (err == STATUS_OK)
                  {
/*
                      tvoc_buf[count] = tvoc_ppb;
                      co2_buf[count]  = co2_eq_ppm;



                    tvoc_buf_av += tvoc_buf[count];
                    co2_buf_av  += co2_buf[count];
*/

                   tvoc_buf_av += tvoc_ppb;
                   co2_buf_av  += co2_eq_ppm;
                    count++;

                  }

                  else
                  {
                     /*
                      tvoc_buf[count] = 0xFFFF;
                      co2_buf[count]  = 0xFFFF;
                      */
                  }



                printf("sampling....(%d) tvoc_ppb : %u co2_eq_ppm : %u \n\r",count,tvoc_ppb,co2_eq_ppm);


                  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


                  HAL_Delay(1000);
              }

           tvoc_buf_av = (uint16_t)(tvoc_buf_av/5);
           co2_buf_av  = (uint16_t)(co2_buf_av/5);

           printf("\n\n[MODE2] tVOC: ");
           printf("%u ", tvoc_buf_av);
           printf("     CO2eq: ");
           printf("%u ", co2_buf_av);
           printf("count : %u ", count);
           printf("\r\n\n");

}

void blow_away(){
   printf("\nblowing away\n\r");
   fan_control_left(128);
   fan_control_right(128);
   fan_control_front(128);
   fan_control_back(128);
   HAL_Delay(3000);

   co2_eq_ppm=400;
    tvoc_ppb = 0;
   printf("\nblowing away end\n\r");

}


uint16_t Comparing_FnB(){


   int16_t  pre,aft,tem1=0, tem2=0;


   fan_control_front(13);
   fan_control_back(0);
   fan_control_left(0);
   fan_control_right(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
   read_N_printSGP30value();

   tem1 = co2_buf_av;
   printf("front level : %d \n\r", tem1);

   //blow_away();




   fan_control_back(13);
   fan_control_front(0);
   fan_control_left(0);
   fan_control_right(0);
   HAL_Delay(1500);
   read_N_printSGP30value();

      read_N_printSGP30value();

      tem2 = co2_buf_av;

   printf("back level : %d \n\r", tem2);

   //blow_away();

   if(tem1 > tem2){

      return 1;

   }

   else if(tem1 < tem2){

      return 2;

   }
   else{

      return 3;

   }



}


uint16_t NarrowingFront(){


   int16_t  pre,aft,tem1=0, tem2=0;


   fan_control_front(0);
   fan_control_right(16);
   fan_control_back(0);
   fan_control_left(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem1 = aft-pre;
   printf("The1_quadrant level : %d \n\r", tem1);

   //blow_away();

   fan_control_front(0);
   fan_control_left(16);
   fan_control_right(0);
   fan_control_back(0);
   HAL_Delay(1500);

   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem2 = aft-pre;
   printf("The2_quadrant level: %d \n\r", tem2);

   //blow_away();
   if(tem1 > tem2){

      return 1;

   }

   else if(tem1 < tem2){

      return 2;

   }
   else{

      return 0;

   }



}



uint16_t NarrowingBack(){



   int16_t  pre,aft,tem1=0, tem2=0;


   fan_control_back(0);
   fan_control_left(16);
   fan_control_right(0);
   fan_control_front(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem1 = aft-pre;

   printf("The3_quadrant level : %d \n\r", tem1);

   //blow_away();

   fan_control_back(0);
   fan_control_right(16);
   fan_control_left(0);
   fan_control_front(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem2 = aft-pre;

   printf("The4_quadrant level: %d \n\r", tem2);
   //blow_away();

   if(tem1 > tem2){

      return 3;

   }

   else if(tem1 < tem2){

      return 4;

   }
   else{

      return 0;

   }






}






void Narrowing_The1_quadrant(){


   int16_t  aft,pre,tem1=0, tem2=0;


   fan_control_front(0);
   fan_control_right(16);
   fan_control_left(0);
   fan_control_back(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
   pre = co2_buf_av;
   read_N_printSGP30value();
   aft = co2_buf_av;

   tem1 = aft-pre;

   printf("The1_section level : %d \n\r", tem1);
   //blow_away();

   fan_control_front(16);
   fan_control_right(0);
   fan_control_left(0);
   fan_control_back(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem2 = aft-pre;

   printf("The2_section level: %d \n\r", tem2);
   //blow_away();

   if(tem1 > tem2){

      printf("\nThe source point is section1 \n\r");

   }

   else if(tem1 < tem2){

      printf("\nThe source point is section2 \n\r");


   }
   else{

      printf("\n Error \n\r");


   }





}


void Narrowing_The2_quadrant(){

   int16_t  aft,pre,tem1=0, tem2=0;


   fan_control_front(0);
   fan_control_left(16);
   fan_control_right(0);
   fan_control_back(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
   pre = co2_buf_av;
   read_N_printSGP30value();
   aft = co2_buf_av;
   tem1 = aft-pre;

   printf("The4_section level : %d \n\r", tem1);
   //blow_away();


   fan_control_front(16);
   fan_control_left(0);
   fan_control_right(0);
   fan_control_back(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
   pre = co2_buf_av;
   read_N_printSGP30value();
   aft = co2_buf_av;

   tem2 = aft-pre;

   printf("The3_section level: %d \n\r", tem2);
   //blow_away();

   if(tem1 > tem2){

      printf("\nThe source point is section4 \n\r");

   }

   else if(tem1 < tem2){

      printf("\nThe source point is section3 \n\r");


   }
   else{

      printf("\n Error \n\r");


   }





}

void Narrowing_The3_quadrant(){

   int16_t  pre,aft,tem1=0, tem2=0;


   fan_control_back(0);
   fan_control_left(16);
   fan_control_front(0);
   fan_control_right(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem1 = aft-pre;

   printf("The5_section level : %d \n\r", tem1);
   //blow_away();

   fan_control_back(16);
   fan_control_left(0);
   fan_control_front(0);
   fan_control_right(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem2 = aft-pre;

   printf("The6_section level: %d \n\r", tem2);
   //blow_away();

   if(tem1 > tem2){

      printf("\nThe source point is section5 \n\r");

   }

   else if(tem1 < tem2){

      printf("\nThe source point is section6 \n\r");


   }
   else{

      printf("\n Error \n\r");


   }








}


void Narrowing_The4_quadrant(){

   int16_t  pre,aft,tem1=0, tem2=0;


   fan_control_back(0);
   fan_control_right(16);
   fan_control_left(0);
   fan_control_front(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;

   tem1 = aft-pre;
   printf("The8_section level : %d \n\r", tem1);
   //blow_away();

   fan_control_back(16);
   fan_control_right(0);
   fan_control_left(0);
   fan_control_front(0);
   HAL_Delay(1500);
   read_N_printSGP30value();
      pre = co2_buf_av;
      read_N_printSGP30value();
      aft = co2_buf_av;
   tem2 = aft-pre;
   printf("The7_section level: %d \n\r", tem2);
   //blow_away();
   if(tem1 > tem2){

      printf("\nThe source point is section8 \n\r");

   }

   else if(tem1 < tem2){

      printf("\nThe source point is section7 \n\r");


   }
   else{

      printf("\n Error \n\r");


   }




}

void search_StinkSource() {


   uint16_t quadrant = 0;
   uint16_t WhereIsHigh = 0;

   WhereIsHigh = Comparing_FnB();



   if(WhereIsHigh == 1){ //앞쪽이 높음

      printf("Front level is higher. So, start comparing The1_quadrant and The2_quadrant\n\n\r");

      quadrant = NarrowingFront();

   }

   else if(WhereIsHigh == 2){//뒤쪽이 높음


      printf("Back level is higher. So, start comparing The3_quadrant and The4_quadrant\n\n\r");

      quadrant = NarrowingBack();

   }

   else{

      printf(" Error occured at comparing point \n\n\r");

   }



   switch(quadrant){


      case 1://1사분면이 높은 경우
      Narrowing_The1_quadrant();
      break;

      case 2://2사분면이 높은 경우
      Narrowing_The2_quadrant();
      break;

      case 3://3사분면이 높은 경우
      Narrowing_The3_quadrant();
      break;

      case 4://4사분면이 높은 경우
      Narrowing_The4_quadrant();
      break;

      default :

      printf(" Error occured at switch in comparing point \n\n\r");
      break;

   }

   fan_control_front(0);
   fan_control_right(0);
   fan_control_left(0);
   fan_control_back(0);
   while(1);

}


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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  printf("\r\n[SGP30 Demo] STM32F4 + I2C1 + UART2\r\n");

  while (sgp_probe() != STATUS_OK)
  {
     printf("SGP30 probe failed. Check wiring & power.\r\n");
     HAL_Delay(500);
   }
   printf("SGP30 found.\r\n");

   // IAQ(공기질) 알고리즘 초기화
   if (sgp_iaq_init() != STATUS_OK) {
     printf("sgp_iaq_init() failed\r\n");
   }

   // (선택) 첫 RAW 시그널 읽기: 부동소수 printf 옵션 없으면 정수로 안내
   {
     uint16_t s_eth = 0, s_h2 = 0;
     if (sgp_measure_signals_blocking_read(&s_eth, &s_h2) == STATUS_OK) {
       printf("RAW Ethanol=%u (scaled/512), H2=%u (scaled/512)\r\n", s_eth, s_h2);
     }
   }

   //fan_control_left(32);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




     read_N_printSGP30value();
     read_N_printSGP30value();
     read_N_printSGP30value();
     read_N_printSGP30value();
     read_N_printSGP30value();

     search_StinkSource();


     //read_N_printSGP30value();


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 128-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 128-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
