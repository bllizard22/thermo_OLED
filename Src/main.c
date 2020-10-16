/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//=== CRC ===//
//uint8_t crc_8(uint8_t inCrc, uint8_t inData)
//{
//  uint8_t i;
//  uint8_t data;
//  data = inCrc ^ inData;
//  for ( i = 0; i < 8; i++ )
//  {
//    if (( data & 0x80 ) != 0 )
//    {
//      data <<= 1;
//      data ^= 0x07;
//    }
//    else
//    {
//      data <<= 1;
//    }
//  }
//  return data;
//}
//
//uint8_t crc(uint8_t *Data)
//{
//  uint8_t data = crc_8(0, *Data++);
//  data = crc_8(data, *Data++);
//  data = crc_8(data, *Data);
//  return data;
//}

void set_new_addr(int8_t new_adr, int32_t mem_adr, uint8_t *in_buff) 
{
  uint8_t Err = 0;
  uint8_t buff[4] = {0x2E, 0x00, 0x00};
  buff[3] = crc(buff);
  HAL_I2C_Master_Transmit(&hi2c1, 0x00, buff, 4, 100);
  HAL_Delay(10);
  uint8_t buff2[4] = {0x2E, new_adr, 0x00};
  buff2[3] = crc(buff2);
  HAL_I2C_Master_Transmit(&hi2c1, 0x00, buff2, 4, 100);
  for (uint8_t i=new_adr-2; i<=new_adr+2; i++){
    HAL_Delay(50);
    Err = HAL_I2C_Mem_Read(&hi2c1, i<<1, mem_adr, 1, in_buff, 4, 100);
    if (Err == HAL_OK)
      printf("Done!\n");
    else
      printf("%i\n", i);
  }
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
  SSD1306_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  uint16_t data1 = 0;
  float data1_fl = 0.0;
  uint8_t Err = 0;
  
  int8_t new_adr = 0xB4;
  uint32_t mem_adr = 0x07;
  uint8_t in_buff[0x04];
  
  
  SSD1306_GotoXY(0, 44); //????????????? ?????? ? ??????? 0;44. ??????? ?? ???????????, ????? ?????????.
  SSD1306_Puts("Hello, habrahabr!!", &Font_7x10, SSD1306_COLOR_WHITE); //????? ??????? ? ???????????? ??????? ??????? "Font_7x10" ????? ??????. 
  SSD1306_DrawCircle(10, 33, 7, SSD1306_COLOR_WHITE);
  
  SSD1306_UpdateScreen();
  
  // Set new_adr as new I2C address
//  set_new_addr(new_adr, mem_adr, in_buff)
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Err = HAL_I2C_Mem_Read(&hi2c1, new_adr, mem_adr, 1, in_buff, 3, 100);
//    HAL_Delay(10);
    if (Err == HAL_OK){
      data1 = (in_buff[1] <<8 | in_buff[0]); 
      data1_fl = (float)(data1 - 13658) / 50;
      uint8_t buf[16];
//      sprintf(buf, "Temp %.2F      ", data1_fl);
//      HAL_UART_Transmit(&huart6, (uint8_t*)&buf, sizeof(buf), 0xFFFF);
//      HAL_UART_Transmit(&huart6, (uint8_t*)"              \r\n", sizeof(buf), 0xFFFF);
//      printf("data1 %4.2F \n", data1_fl);
      //printf("data %s \n", buf);
    }
    else{
      printf("Error!\n ");
      //HAL_UART_Transmit(&huart6, (uint8_t*)"Error!", 6, 0xFFFF);
      HAL_I2C_DeInit(&hi2c1);
      HAL_Delay(150);
      MX_I2C1_Init();
    }
    
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      printf("data1 %4.2F \n", data1_fl);
//      HAL_Delay(500);
    }
    else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
//    HAL_Delay(70);
    
    
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
