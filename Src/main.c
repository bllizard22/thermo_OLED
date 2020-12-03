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
//#include "ssd1306.h"
#include "oled.h"
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

void loading_animation() {
  
  uint16_t delay_value = 300;
  
  SSD1306_ON();  
  
  TM_SSD1306_Fill(0); //clear oled  
  TM_SSD1306_GotoXY(10,15);
  TM_SSD1306_DrawFilledCircle(44, 50, 5, 1);
  TM_SSD1306_DrawCircle(64, 50, 3, 1);
  TM_SSD1306_DrawCircle(84, 50, 3, 1);
  TM_SSD1306_UpdateScreen(); //display
  HAL_Delay(delay_value);
  
  TM_SSD1306_Fill(0); //clear oled  
  TM_SSD1306_GotoXY(10,15);
  TM_SSD1306_DrawCircle(44, 50, 3, 1);
  TM_SSD1306_DrawFilledCircle(64, 50, 5, 1);
  TM_SSD1306_DrawCircle(84, 50, 3, 1);
  TM_SSD1306_UpdateScreen(); //display
  HAL_Delay(delay_value);
  
  TM_SSD1306_Fill(0); //clear oled  
  TM_SSD1306_GotoXY(10,15);
  TM_SSD1306_DrawCircle(44, 50, 3, 1);
  TM_SSD1306_DrawCircle(64, 50, 3, 1);
  TM_SSD1306_DrawFilledCircle(84, 50, 5, 1);
  TM_SSD1306_UpdateScreen(); //display
  HAL_Delay(delay_value);
  
  
}

void measuring_animation(uint8_t pos) {
  
  uint16_t delay_value = 210;
  
  switch (pos){
    case 1: {
    TM_SSD1306_Fill(0); //clear oled
    TM_SSD1306_GotoXY(10,15);
    TM_SSD1306_Puts("Temp:", &TM_Font_11x18, 1);
    TM_SSD1306_DrawFilledCircle(44, 50, 5, 1);
    TM_SSD1306_DrawCircle(64, 50, 3, 1);
    TM_SSD1306_DrawCircle(84, 50, 3, 1);
    TM_SSD1306_UpdateScreen(); //display 
    SSD1306_ON();
    HAL_Delay(delay_value);
    break;
    }
    
    case 2: {
    TM_SSD1306_Fill(0); //clear oled
    TM_SSD1306_GotoXY(10,15);
    TM_SSD1306_Puts("Temp:", &TM_Font_11x18, 1);
    TM_SSD1306_DrawCircle(44, 50, 3, 1);
    TM_SSD1306_DrawFilledCircle(64, 50, 5, 1);
    TM_SSD1306_DrawCircle(84, 50, 3, 1);
    TM_SSD1306_UpdateScreen(); //display
    HAL_Delay(delay_value);
    break;
    }
    
    case 3: {
    TM_SSD1306_Fill(0); //clear oled 
    TM_SSD1306_GotoXY(10,15);
    TM_SSD1306_Puts("Temp:", &TM_Font_11x18, 1);
    TM_SSD1306_DrawCircle(44, 50, 3, 1);
    TM_SSD1306_DrawCircle(64, 50, 3, 1);
    TM_SSD1306_DrawFilledCircle(84, 50, 5, 1);
    TM_SSD1306_UpdateScreen(); //display
    HAL_Delay(delay_value);
    break;
    }
  }
  
}

void display_temp(float data_temp) {
  if (data_temp == 0.0) {
    TM_SSD1306_Fill(0); //clear oled
    TM_SSD1306_GotoXY(10,15);
    TM_SSD1306_Puts("Temp: ", &TM_Font_11x18, 1);
    TM_SSD1306_UpdateScreen(); //display
  }
  else {
    if (data_temp == -1.0) {
      TM_SSD1306_Fill(0); //clear oled
      TM_SSD1306_GotoXY(10,15);
      TM_SSD1306_Puts("Temp:  Err", &TM_Font_11x18, 1);
      TM_SSD1306_UpdateScreen(); //display
    }
    else {
      uint8_t buf[16];
      if (data_temp < 34.5) {
        sprintf(buf, "Temp: Low", data_temp);
        TM_SSD1306_Fill(0); //clear oled
        TM_SSD1306_GotoXY(10,15);
        TM_SSD1306_Puts(buf, &TM_Font_11x18, 1);
        TM_SSD1306_UpdateScreen(); //display
      }
      else
        if (data_temp > 43.0) {
          sprintf(buf, "Temp: High", data_temp);
          TM_SSD1306_Fill(0); //clear oled
          TM_SSD1306_GotoXY(10,15);
          TM_SSD1306_Puts(buf, &TM_Font_11x18, 1);
          TM_SSD1306_UpdateScreen(); //display
        }
        else  
          if (data_temp >= 37.5) {
              sprintf(buf, "Temp: %.1F", data_temp);
              TM_SSD1306_Fill(0); //clear oled
              TM_SSD1306_GotoXY(10,15);
              TM_SSD1306_Puts(buf, &TM_Font_11x18, 1);
              
              TM_SSD1306_GotoXY(35,40);
              TM_SSD1306_Puts("ALERT!", &TM_Font_11x18, 1);
              TM_SSD1306_UpdateScreen(); //display
              
//              printf("Measured temp: %.2F\n\n", data_temp);
//              printf(" *===   ALERT!   ===*\n\n");
            } else {
              sprintf(buf, "Temp: %.1F", data_temp);
              TM_SSD1306_Fill(0); //clear oled
              TM_SSD1306_GotoXY(10,15);
              TM_SSD1306_Puts(buf, &TM_Font_11x18, 1);
              TM_SSD1306_UpdateScreen(); //display
//              printf("Measured temp: %.2F\n\n", data_temp);
        }
    }
  }
}

float adjust_temp(uint8_t msb_temp, uint8_t lsb_temp) {
  float data_temp = (float)(msb_temp << 8 | lsb_temp);
  data_temp = (data_temp - 13658) / 50;
  printf("Max%.3F\n", data_temp);
  float adjusted_temp = 0.0;
  //=== Here is the separating point between two calculation 
  //=== NOT high temp alert-point!
  if (data_temp >= 35.05) {
    adjusted_temp = (1025 * data_temp) / (1000 - data_temp);
  } 
  else {
    adjusted_temp = (283 * 21 * data_temp) / (100 * (21 + data_temp));
  }
  return adjusted_temp;
}

void process_error() {
  
  //=== Print "Err" instead temp
//  printf("Error!\n ");
  display_temp(-1.0);
//  HAL_UART_Transmit(&huart6, (uint8_t*)"Error!", 6, 0xFFFF);
  
  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(150);
  MX_I2C1_Init();
}

void send_uart(float data_temp) {
  uint8_t buf[16];
  sprintf(buf, "Temp %.2F      ", data_temp);
//  HAL_UART_Transmit(&huart6, (uint8_t*)&buf, sizeof(buf), 0xFFFF);
//  HAL_UART_Transmit(&huart6, (uint8_t*)"              \r\n", sizeof(buf), 0xFFFF);
//  printf("data %4.2F \n", data_temp);
//  printf("data %s \n", buf);
}

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
//    if (Err == HAL_OK)
//      printf("Done!\n");
//    else
//      printf("%i\n", i);
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
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // --- Achtung! --- Reset the I2C before use! Bug in STM32F100 !!!
  __HAL_RCC_I2C1_CLK_ENABLE();
  HAL_Delay(100);
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(100);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(300);
  uint8_t res = TM_SSD1306_Init();
//  printf("OLED init: %d\n", res);
  
  
  //=== Loop loading animation several times
  int counter = 3;
  while (--counter > 0) {
    loading_animation();
  }
  
  SSD1306_OFF();
  
  uint16_t data = 0;
  float data_fl = 0.0;
  float max_data = 0.0;
  uint8_t Err = 0;
  uint8_t measure_counter = 0;
  
  int8_t i2c_adr = 0xB4;
  uint32_t mem_adr = 0x07;
  uint8_t in_buff[0x04];
  
  uint16_t activity_flag = 100;
  
  //=== Set new_adr as new I2C address
//  int8_t new_adr = 0xB4;
//  set_new_addr(new_adr, mem_adr, in_buff)
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //=== Read data from sensor via I2C
    Err = HAL_I2C_Mem_Read(&hi2c1, i2c_adr, mem_adr, 1, in_buff, 3, 100);
    HAL_Delay(100);
        
    //=== Check if there is no error in I2C read
    if (Err != HAL_OK) {
      process_error();
    }
    else {
//      display_temp(0.0);
      //=== Display the Temp while button is pressed
      if (GPIO_PIN_SET == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//        printf("data %4.2F \n", data_fl);
//        printf("=== %d ===\n", measure_counter++);
//        printf("Button pressed\n");
        
        int delay = 6;
        while ((--delay >= 0)) {
          //=== Read data from sensor via I2C
          Err = HAL_I2C_Mem_Read(&hi2c1, i2c_adr, mem_adr, 1, in_buff, 3, 10);
  //        HAL_Delay(50);
          
          //=== Check if there is no error in I2C read
          if (Err == HAL_OK) {
//            data = (in_buff[1] << 8 | in_buff[0]); 
//            data_fl = (float)(data - 13658) / 50;
            data_fl = adjust_temp(in_buff[1], in_buff[0]);
//            printf("%.2F\n", data_fl);
            if (max_data < data_fl) {
              max_data = data_fl;
            }
          }
          else {
            process_error();
          }
          uint8_t pos = (5 - delay)/2 + 1; 
          measuring_animation(pos); 
        }
        
        //=== Print temperature in terminal and oled-display
        HAL_I2C_Mem_Read(&hi2c1, i2c_adr, mem_adr - 1, 1, in_buff, 3, 100);
        data = (in_buff[1] << 8 | in_buff[0]); 
        data_fl = (float)(data - 13658) / 50;
        
//        printf("Ta: %.2F\n", data_fl);
        
        display_temp(max_data);
        printf("\n");
        max_data = 0.0;
        HAL_Delay(2000);
  //      display_temp(0.0);
        activity_flag = 100;
      }
      else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        if (activity_flag == 0) {
          SSD1306_OFF();
        }
        else {
          activity_flag--;
//          printf("Activity timer: %d\n", activity_flag);
        }
      }  
    }
  }
    
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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
