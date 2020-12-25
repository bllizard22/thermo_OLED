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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "dwt_delay.h"
#include "stm32f1xx_hal_can.h"

#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOA
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//typedef enum
//{
//  HAL_CAN_STATE_RESET             = 0x00U,  /*!< CAN not yet initialized or disabled */
//  HAL_CAN_STATE_READY             = 0x01U,  /*!< CAN initialized and ready for use   */
//  HAL_CAN_STATE_BUSY              = 0x02U,  /*!< CAN process is ongoing              */
//  HAL_CAN_STATE_BUSY_TX           = 0x12U,  /*!< CAN process is ongoing              */
//  HAL_CAN_STATE_BUSY_RX           = 0x22U,  /*!< CAN process is ongoing              */
//  HAL_CAN_STATE_BUSY_TX_RX        = 0x32U,  /*!< CAN process is ongoing              */
//  HAL_CAN_STATE_TIMEOUT           = 0x03U,  /*!< Timeout state                       */
//  HAL_CAN_STATE_ERROR             = 0x04U   /*!< CAN error state                     */
//
//}HAL_CAN_StateTypeDef;

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

CAN_TxHeaderTypeDef    TxHeader1;
//CAN_FilterTypeDef  sFilterConfig;
uint32_t canFifoFulFil;
//HAL_StatusTypeDef HAL_CAN_Transmit_IT(CAN_HandleTypeDef* hcan)

void can_transmit_to(uint8_t *data) {
      /*##-4- Start the Transmission process #####################################*/

  //  TxHeader.TransmitGlobalTime = DISABLE;
      //TxHeader.DLC = sizeof data;
      TxHeader1.DLC = 6;
      TxHeader1.StdId = 0x100;
      //TxHeader.ExtId = 0x1;
      TxHeader1.RTR = CAN_RTR_DATA;
      TxHeader1.IDE = CAN_ID_STD;
//      for (uint8_t i = 0; i < 8; ++i) {
////        TxHeader1.Data[i] = data[i];
//      }
      uint32_t TxMailbox = 0;
      HAL_CAN_AddTxMessage(&hcan, &TxHeader1, data, &TxMailbox);

//      if (hcan.State == HAL_CAN_STATE_LISTENING) {
//        while (hcan.State == HAL_CAN_STATE_LISTENING)
//          __NOP;
//      }
//      else {
//        for (int i = 0; i < 1; i++) {
//          for (int i = 0; i < 15000; i++) {
//            __NOP;
//          }
//        }
//      }
      //HAL_CAN_Transmit_IT(&hcan1);
      //}
//      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader1, data, &TxMailbox) != HAL_OK)
//      {   
//              //Error_Handler();
//              canFifoFulFil = 1;
//      }
//      HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
      
	// Start receiving
	//HAL_CAN_Receive_IT( &hcan, CAN_FIFO0 );
        TxMailbox = 0;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader1, data, &TxMailbox);
  }


//=== CRC ===//
uint8_t crc_8(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;
  data = inCrc ^ inData;
  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}

uint8_t crc(uint8_t *Data)
{
  uint8_t data = crc_8(0, *Data++);
  data = crc_8(data, *Data++);
  data = crc_8(data, *Data);
  return data;
}

void array_sort(uint16_t *array , uint16_t amount)
{
    // declare some local variables
    uint16_t i=0 , j=0 , temp=0;

    for(i=0 ; i<amount ; i++)
    {
        for(j=0 ; j<amount-1 ; j++)
        {
            if(array[j]>array[j+1])
            {
                temp        = array[j];
                array[j]    = array[j+1];
                array[j+1]  = temp;
            }
        }
    }
}

//=== Convert temp from Kelvin to Celsius
//=== and make value adjustment
float adjust_temp(uint8_t msb_temp, uint8_t lsb_temp) {
  float data_temp = (float)(msb_temp << 8 | lsb_temp);
//  printf("msb %d\n", msb_temp);
//  printf("msb %d\n", lsb_temp);
//  printf("ADJ before %.4F\n", data_temp);
  data_temp = (data_temp - 13658) / 50;
//  printf("ADJ %.4F\n", data_temp);
  float adjusted_temp = 0.0;
  adjusted_temp = data_temp;
  
//=== Here is the separating point between two calculation
//=== to make it close to mercury thermometer values
//=== NOT high temp alert-point!
  
//  if (data_temp >= 35.05) {
//    adjusted_temp = (1025 * data_temp) / (1000 - data_temp);
//  }
//  else {
//    adjusted_temp = (283 * 21 * data_temp) / (100 * (21 + data_temp));
//  }
  return adjusted_temp;
}

//=== Find max value form buff in (int)-ed temps
float obtain_max_temp(uint16_t *log_array, uint16_t log_array_size){
  uint16_t max_value = 0;
  for (int i=0; i < log_array_size; i++) {
    if (log_array[i] > max_value) {
      max_value = log_array[i];
    }
  }
  printf("Local Max;");
  return ((float)max_value)/100;
}

//=== Two orders of sensors
//  uint8_t bar_sensors[6] = {0xA2, 0xA4, 0xA6, 0xB2, 0xB4, 0xB6};
  uint8_t bar_sensors[6] = {0xA2, 0xB2, 0xA4, 0xB4, 0xA6, 0xB6};
//uint8_t i2c_adr = 0xB2; // A2, B2, A4, B4, A6, B6
uint32_t mem_adr = 0x07;
uint8_t in_buff[0x04];
uint8_t Err = 0;
float data_fl = 0.0;


//=== Calculate average temp of enviroment in sensors field of view
float calibrate_ambient(uint8_t i2c_adr) {
  uint16_t cycles_value = 1;
  uint16_t avr_array[1]; // = cycles_value
  uint16_t amount = 0;
  uint32_t sum = 0;
  uint16_t ambient_array[50];

  for (uint8_t cycle=0; cycle < cycles_value; cycle++){
    for (uint32_t i=0; i < 50; i++) {
      Err = HAL_I2C_Mem_Read(&hi2c1, i2c_adr, mem_adr, 1, in_buff, 3, 50);
      if (Err == HAL_OK) {
        data_fl = adjust_temp(in_buff[1], in_buff[0]);
//        printf("%.2F\n", data_fl);
        ambient_array[amount] = (uint32_t)(data_fl*100);
        amount++;
      }
      HAL_Delay(40);
    }

    array_sort(ambient_array, amount);
    sum = 0;
    for (uint8_t pos=0; pos<amount; pos++) {
      sum += ambient_array[pos];
    }
    uint16_t avr_ambient_temp = sum / amount;
//    printf("AVR = %d\n", avr_ambient_temp);
    avr_array[cycle] = avr_ambient_temp;
    amount = 0;
  }

  sum = 0;
  for (uint8_t pos=0; pos < cycles_value; pos++) {
      sum += avr_array[pos];
//      printf("Ambient AVR = %d\n", avr_array[pos]);
    }
  uint16_t avr_ambient_temp = sum / cycles_value;

//  printf("Final AVR = %d\n", avr_ambient_temp);
//  printf("Ambient %d, %d...\n", ambient_array[2], ambient_array[3]);

  return ((float)avr_ambient_temp / 100);
}

//=== Function to calculate average ambient temp
float get_avr_ambient() {
  HAL_Delay(300);
  float ambient_avr[6];
  for (int i=0; i < 6; i++) {
    ambient_avr[i] = calibrate_ambient(bar_sensors[i]);
    printf("At 0x%X ambient_avr = %.2F\n", bar_sensors[i], ambient_avr[i]);
  }

  float sum = 0;
  for (uint8_t pos=0; pos < 6; pos++) {
      sum += ambient_avr[pos];
//      printf("sum = ambient_avr = %.2F\n", ambient_avr[pos]);
  }

  float avr_ambient_temp = sum / 6;
  printf("\nGlobal Ambient AVR = %.2F\n", avr_ambient_temp);

  for (uint8_t pos=0; pos < 6; pos++) {
    printf("Disp: %.2F\n", ambient_avr[pos] - avr_ambient_temp);
  }
  return avr_ambient_temp;
}


//=== Set new address for sensor
//=== (All connected sensors will recieve this address)
void set_new_addr(uint8_t new_adr, int32_t mem_adr, uint8_t *in_buff)
{
//  uint8_t Err = 0;
  uint8_t start_adr = new_adr - 4;
  uint8_t stop_adr = new_adr + 4;
  uint8_t buff[4] = {0x2E, 0x00, 0x00};
  buff[3] = crc(buff);
  HAL_I2C_Master_Transmit(&hi2c1, 0x00, buff, 4, 100);
  HAL_Delay(10);
  uint8_t buff2[4] = {0x2E, new_adr, 0x00};
  printf("\n\n new_adr = %X\n\n", new_adr);
  printf("\n\n start_adr = %X\n\n", start_adr);
  printf("\n\n stop_adr = %X\n\n", stop_adr);
  printf("\n\n");
  buff2[3] = crc(buff2);
  HAL_I2C_Master_Transmit(&hi2c1, 0x00, buff2, 4, 100);
  for (uint16_t i=start_adr; i<=stop_adr; i++){
    HAL_Delay(50);
    Err = HAL_I2C_Mem_Read(&hi2c1, i<<1, mem_adr, 1, in_buff, 4, 100);
    if (Err == HAL_OK)
      printf("Done! %X\n\n", i);
    else
      printf("No\n\n");
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
  DWT_Init();
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
  MX_TIM1_Init();
  MX_CAN_Init();
  inbus_init();
  
  /* USER CODE BEGIN 2 */
  printf("======= 000 =======\n\n");

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_Delay(300);
  
//=== Number of sensors in use
  uint8_t sensor_amount = 6;
  
  float sum_data = 0;  
  uint16_t log_array_size = 300;
  uint16_t log_array[300];
  float log_buff[6];
  float max_temp_on_sensor = 0;
  uint16_t log_array_pos = 0;
  
  uint32_t measure_counter = 0;
  
//=== CAN test transmit
  uint8_t TxData[8];
  TxData[0] = 0x01;
  TxData[1] = 0xFE;
  TxData[2] = 1;
  TxData[3] = 0;
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;
  can_transmit_to(TxData);

  
  printf("Can");

//=== Set new_adr as new I2C address
//  uint8_t new_adr = i2c_adr;
//  set_new_addr(new_adr>>1, mem_adr, in_buff);

//=== Temp ambient calibration
//  HAL_Delay(300);
//  float ambient_avr[6];
//  for (int i=0; i < 6; i++) {
//    ambient_avr[i] = calibrate_ambient(bar_sensors[i]);
//    printf("At 0x%X ambient_avr = %.2F\n", bar_sensors[i], ambient_avr[i]);
//  }
//
//  float sum = 0;
//  for (uint8_t pos=0; pos < 6; pos++) {
//      sum += ambient_avr[pos];
////      printf("sum = ambient_avr = %.2F\n", ambient_avr[pos]);
//  }
//
//  float avr_ambient_temp = sum / 6;
//  printf("\nGlobal Ambient AVR = %.2F\n", avr_ambient_temp);
//
//  for (uint8_t pos=0; pos < 6; pos++) {
//    printf("Disp: %.2F\n", ambient_avr[pos] - avr_ambient_temp);
//  }
//  printf("\n");
  float avr_ambient_temp = get_avr_ambient();
  printf("\n");
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("%d;", measure_counter++);
    
    for (uint8_t i=0; i<sensor_amount; i++) {
      
    //=== Read data from sensor via I2C
        Err = HAL_I2C_Mem_Read(&hi2c1, bar_sensors[i], mem_adr, 1, in_buff, 3, 0x100);
        HAL_Delay(5); // Delay for I2C responce

    //=== Check if there is no error in I2C read
        if (Err == HAL_OK) {

          //=== Read 2 bytes and convert to Celsius
              data_fl = adjust_temp(in_buff[1], in_buff[0]);
              
          //=== Check if temp close to ambient
//              if ((data_fl <= (avr_ambient_temp * 1.005)) & (data_fl >= (avr_ambient_temp * 0.995))) {
//                data_fl = avr_ambient_temp;
//              }
              
          //=== Set the value from sensor with max temp
//              if (max_data < data_fl) {
//                  max_data = data_fl;
//              }
//              float_max_data = max_data;

          //=== Print temperature in terminal
              //		  printf("    Adj %.2F\n", data_fl);
//              printf("%X;%.2F\n", bar_sensors[i], data_fl);
                printf("%.2F;", data_fl);
              sum_data += data_fl;
              log_buff[i] = data_fl;

//              new_max_data = (int)(data_fl*100);
//              printf("    NewAdj ==%d==\n\n", new_max_data);


          //=== Read ambient Temp - Ta
//              HAL_I2C_Mem_Read(&hi2c1, i2c_adr, mem_adr - 1, 1, in_buff, 3, 100);
//              data_fl = adjust_temp(in_buff[1], in_buff[0]);
//              ambient_data = (int)(data_fl*100);
//              diff_data = float_max_data - ambient_data;
//              float_amb_data = data_fl;
//              printf("Ta: %.2F\n", data_fl);

//              float_max_data = max_data;
//              diff_float_data = float_max_data - float_amb_data;

        }
        else {
              printf("\nError on I2C. Address %X\n", bar_sensors[i]);
        }
//        HAL_Delay(20); 
    }
    
//=== Check for threshold above avr_ambient_temp after 
//=== every full sensors cycleto catch pass
    
    if ((sum_data / sensor_amount >= (avr_ambient_temp * 1.01)) 
        && (log_array_pos +sensor_amount < log_array_size)) {
          
    //=== Fill buff of previous temp with float to int
      for (int i=0; i < sensor_amount; i++) {
        log_array[log_array_pos+i] = (int)(log_buff[i]*100);
//        printf("log %d   at pos %d\n", log_array[log_array_pos+i], log_array_pos+i);
      }
      log_array_pos += sensor_amount;
    } 
    else 
    { 
    //=== Check if high temp not a noise pump
      if (log_array_pos >= 40) {
      
      //=== Find max temp in last measures
        max_temp_on_sensor = obtain_max_temp(log_array, log_array_pos);
//        printf("\n===== %0.3F = at %d===\n\n", max_temp_on_sensor, measure_counter);
        printf("%0.3F;", max_temp_on_sensor);
        
      //=== Send alarm msg to CPU board via CAN
        if (max_temp_on_sensor >= avr_ambient_temp) {
          uint8_t TxData[8];
          TxData[0] = 0x01;
          TxData[1] = 0xFE;
          TxData[2] = 1;
          TxData[3] = 0;
          TxData[4] = 0;
          TxData[5] = 0;
          TxData[6] = 0;
          TxData[7] = 0;
          can_transmit_to(TxData);
        }
      }
      log_array_pos = 0;
    }
    
//=== Print average temp
    printf("%.3F\n", sum_data / sensor_amount);
//    printf("%.3F;%d\n", sum_data / sensor_amount, measure_counter);
    sum_data = 0;
    
    
//    printf("\n");
//    float_max_data = max_data;
//    diff_float_data = float_max_data - float_amb_data;
//    new_max_data = (int)(max_data*100);
//    printf("\n");
//    printf("-%d- MaxAdj ==%.2F==\n", measure_counter++, max_data);
//    printf("%.2F\n", max_data);
//    max_data = 0.0;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//HAL_CAN_TxCpltCallback() {
////  HAL_CAN_Transmit_IT(); // Rearm transmit
//  HAL_CAN_Transmit(); 
//}

//HAL_StatusTypeDef HAL_CAN_Transmit_IT(CAN_HandleTypeDef* hcan)
//{
//  uint32_t  transmitmailbox = CAN_TXSTATUS_NOMAILBOX;
//  
//  /* Check the parameters */
//  assert_param(IS_CAN_IDTYPE(hcan->pTxMsg->IDE));
//  assert_param(IS_CAN_RTR(hcan->pTxMsg->RTR));
//  assert_param(IS_CAN_DLC(hcan->pTxMsg->DLC));
// 
//  if(((hcan->Instance->TSR&CAN_TSR_TME0) == CAN_TSR_TME0) || \
//     ((hcan->Instance->TSR&CAN_TSR_TME1) == CAN_TSR_TME1) || \
//     ((hcan->Instance->TSR&CAN_TSR_TME2) == CAN_TSR_TME2))
//  {
//    /* Process Locked */
//    __HAL_LOCK(hcan);
//    
//    /* Select one empty transmit mailbox */
//    if((hcan->Instance->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
//    {
//      transmitmailbox = CAN_TXMAILBOX_0;
//    }
//    else if((hcan->Instance->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
//    {
//      transmitmailbox = CAN_TXMAILBOX_1;
//    }
//    else
//    {
//      transmitmailbox = CAN_TXMAILBOX_2;
//    }
//	
//    /* Set up the Id */
//    hcan->Instance->sTxMailBox[transmitmailbox].TIR &= CAN_TI0R_TXRQ;
//    if(hcan->pTxMsg->IDE == CAN_ID_STD)
//    {
//      assert_param(IS_CAN_STDID(hcan->pTxMsg->StdId));  
//      hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->StdId << 21) | \
//                                                hcan->pTxMsg->RTR);
//    }
//    else
//    {
//      assert_param(IS_CAN_EXTID(hcan->pTxMsg->ExtId));
//      hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->ExtId << 3) | \
//                                                hcan->pTxMsg->IDE | \
//                                                hcan->pTxMsg->RTR);
//    }
//    
//    /* Set up the DLC */
//    hcan->pTxMsg->DLC &= (uint8_t)0x0000000FU;
//    hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t)0xFFFFFFF0U;
//    hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= hcan->pTxMsg->DLC;
//
//    /* Set up the data field */
//    hcan->Instance->sTxMailBox[transmitmailbox].TDLR = (((uint32_t)hcan->pTxMsg->Data[3] << 24) | 
//                                           ((uint32_t)hcan->pTxMsg->Data[2] << 16) |
//                                           ((uint32_t)hcan->pTxMsg->Data[1] << 8) | 
//                                           ((uint32_t)hcan->pTxMsg->Data[0]));
//    hcan->Instance->sTxMailBox[transmitmailbox].TDHR = (((uint32_t)hcan->pTxMsg->Data[7] << 24) | 
//                                           ((uint32_t)hcan->pTxMsg->Data[6] << 16) |
//                                           ((uint32_t)hcan->pTxMsg->Data[5] << 8) |
//                                           ((uint32_t)hcan->pTxMsg->Data[4]));
//
//
//    
//    if(hcan->State == HAL_CAN_STATE_BUSY_RX) 
//    {
//      /* Change CAN state */
//      hcan->State = HAL_CAN_STATE_BUSY_TX_RX;
//    }
//    else
//    {
//      /* Change CAN state */
//      hcan->State = HAL_CAN_STATE_BUSY_TX;
//    }
//      
//    /* Set CAN error code to none */
//    hcan->ErrorCode = HAL_CAN_ERROR_NONE;
//      
//    /* Process Unlocked */
//    __HAL_UNLOCK(hcan);
//	
//    /* Enable Error warning, Error passive, Bus-off,
//       Last error and Error Interrupts */	
//    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG |
//                              CAN_IT_EPV |
//                              CAN_IT_BOF |
//                              CAN_IT_LEC |
//                              CAN_IT_ERR |
//							  CAN_IT_TME);
//      
//    /* Request transmission */
//    hcan->Instance->sTxMailBox[transmitmailbox].TIR |= CAN_TI0R_TXRQ;
//  }
//  else
//  {
//    /* Change CAN state */
//    hcan->State = HAL_CAN_STATE_ERROR; 
//
//    /* Return function status */
//    return HAL_ERROR;
//  }
//  
//  return HAL_OK;
//}
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
