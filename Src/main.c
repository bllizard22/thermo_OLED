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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "dwt_delay.h"

#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOA
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
uint8_t len = 0;
uint8_t value_read[6][2];
uint8_t value_read1[2];
uint16_t regist = 0;
uint8_t uart_buff[32];
uint8_t uart_buff1[32];
float fl_temperature[6];
int i = 0;
uint8_t transmitData[15];

uint8_t transmitData1[364];
float flTemperatureSum = 0;
float flTemperatureAverage = 0;
float flTemperatureAverage1 = 0;
float lenf = 0.0;

uint32_t timer;
uint16_t timeToSend = 0;
uint16_t j = 0;

uint16_t value_readHigh = 0;
uint16_t value_readLow = 0;
uint8_t value_readHigh_aver = 0;
uint8_t value_readLow_aver = 0;
float fl_mlx_temp_aver = 0;
uint8_t stateAver = 0;
uint16_t k = 0;
uint8_t kk = 0;
float tt[190];
float fl_temperatureA = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_USART6_UART_Init(void);
//static void MX_USART1_UART_Init(void);
///* USER CODE BEGIN PFP */
//float moving_average(float a, float *buff1, float *buff2, int N) {
//  float summ_buff_obj_temp = 0;
//
//  //buff_obj_temp1[0] = a;
//  buff1[0] = a;
//
//  for (uint8_t u = 0; u < N; u++) {
//    summ_buff_obj_temp = summ_buff_obj_temp + buff1[u];
//  }
//  float f_t_obj = summ_buff_obj_temp / N;
//  float t_obj = f_t_obj;
//  for (uint8_t u = 0; u < N - 1; u++) {
//    buff2[u + 1] = buff1[u];
//
//  }
//  for (uint8_t u = 0; u < N; u++) {
//    buff1[u] = buff2[u];
//
//  }
//  return t_obj;
//}
////================================
//void temp1(){
//   //???? ?????? ??-???????? ? ????????
//     for(int i = 0; i<6; i++){
//
//
//              fl_temperature[i] = i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i], 0x100);
//              HAL_I2C_Mem_Read(&hi2c1, i2c_adr, mem_adr, 1, value_read[i], 3, 50);
//
//              HAL_Delay(10);
//            flTemperatureSum = flTemperatureSum + fl_temperature[i];
//
//
//      }
//
//      flTemperatureAverage = flTemperatureSum/6;
//      flTemperatureSum = 0;
//
////      if(flTemperatureAverage>22){
////        timer = 0;
////        while(flTemperatureAverage>22){
////          for(int i = 0; i<6; i++){
////
////
////                    fl_temperature[i] = i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i], 0x100);
////
////                    HAL_Delay(10);
////                  flTemperatureSum = flTemperatureSum + fl_temperature[i];
////
////  k++;
////            }
////
////
////            flTemperatureAverage = flTemperatureSum/6;
////           tt[k] = flTemperatureAverage;
////
////            flTemperatureSum = 0;
////        }
////      k = 0;
////        timeToSend = timer;
////      }
//      fl_temperatureA = i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(1), 0x06, value_read[i], 0x100);
//}
//
////=====================================
//void dataCollectByte(){
//   //???? ?????? ??-???????? ?? ??????
//     for(int i = 0; i<6; i++){
//
//
//
//            i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i]);
//            HAL_Delay(10);
//
//                transmitData[2*i+1] = value_read[i][0];
//                transmitData[2*i+2] = value_read[i][1];
//
//          }
//
//             transmitData[0] = 's';
//
//             transmitData[14] = 'e';
//             fl_temperatureA = i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(1), 0x06, value_read1, 0x100);
//
//                HAL_Delay(10);
//
//                fl_temperatureA = fl_temperatureA*10;
//                transmitData[13] = ((int)fl_temperatureA);
//
//             HAL_UART_Transmit_DMA(&huart1, (uint8_t*) transmitData, 15);
//                            HAL_Delay(20);
//}
////==========================================
//void lvProxsonar(){
//   //========================================
//      //????????? ??????? ??????????
//      //======================================
//               uint16_t temp_len = 0;
//                uint8_t n_len = 0;
//                 for(int j = 0; j<26; j++){
//
//                    if(uart_buff[j]=='R'&&uart_buff[j+5]=='P'){
//
//
//                    temp_len = temp_len + ((uart_buff[j+1]-48)*100 + (uart_buff[j+2]-48)*10 + (uart_buff[j+3]-48));
//
//                      n_len++;
//                  }
//
//                  }
//
//                 if(n_len) {
//                   len = temp_len/n_len;
//                   lenf = len*2.54;
//                 }
//                 if(len<255) transmitData[13] = len;
//}
//
////threshold
//void tempThreshold(uint8_t tempThreshold){
//  for(int i = 0; i<6; i++){
//
//
//                          //fl_temperature[i] = i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i], 0x100);
//                      i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i]);
//                      HAL_Delay(10);
//
//                            value_readHigh = value_readHigh + value_read[i][1];
//                                    value_readLow = value_readLow + value_read[i][0];
//
//                }
//
//              value_readHigh_aver = value_readHigh/6;
//              value_readLow_aver = value_readLow/6;
//              fl_mlx_temp_aver = (value_readHigh_aver*256 + value_readLow_aver)*0.02 - 273.15;
//              value_readHigh = 0;
//              value_readLow = 0;
//      if(fl_mlx_temp_aver>tempThreshold){
//        timer = 0;
//           stateAver = 1;
//
//       for(j = 0; j<30; j++){
//
//         //???? ?????? ??-???????? ?? ??????
//          for(int i = 0; i<6; i++){
//
//
//                      //fl_temperature[i] = i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i], 0x100);
//                  i2cMLX.mlxRead(hi2c1, i2cMLX.getMlxSlaveAddr(i+1), i2cMLX.getRegAddr("TOBJ1"), value_read[i]);
//                  HAL_Delay(10);
//
//                      transmitData1[2*i +1+ 12*j] = value_read[i][0];
//                      transmitData1[2*i+2 + 12*j] = value_read[i][1];
//                                value_readHigh = value_readHigh + value_read[i][1];
//                                value_readLow = value_readLow + value_read[i][0];
//
//            }
//           k++;
//          value_readHigh_aver = value_readHigh/6;
//          value_readLow_aver = value_readLow/6;
//          fl_mlx_temp_aver = (value_readHigh_aver * 256 + value_readLow_aver)*0.02 - 273.15;
//          value_readHigh = 0;
//          value_readLow = 0;
//          tt[k] = fl_mlx_temp_aver;
//          if(fl_mlx_temp_aver<22&&stateAver==1){
//            stateAver = 0;
//            timeToSend = timer;
//            kk = k;
//          }
//
//        }
//
//           transmitData1[0] = 's';
//           transmitData1[363] = 'e';
//           transmitData1[361] = timer;
//           transmitData1[362] = timer>>8;
//
//      }
//}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=== HC-SR04 ===//
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint16_t Distance  = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
  {
    if (Is_First_Captured==0) // if the first value is not captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
      Is_First_Captured = 1;  // set the first captured as true
      // Now change the polarity to falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    else if (Is_First_Captured==1)   // if the first is already captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
      __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

      if (IC_Val2 > IC_Val1)
      {
        Difference = IC_Val2-IC_Val1;
      }

      else if (IC_Val1 > IC_Val2)
      {
        Difference = (0xFFFF - IC_Val1) + IC_Val2;
      }

      //Distance = Difference * .034/2;
//      Distance = Difference / 58.8 * 2;
      if ((Difference / 58.8 * 2) >= 390) {
    	  Distance = 0;
      } else {
    	  Distance = Difference / 58.8 * 2;
      }
      Is_First_Captured = 0; // set it back to false

      // set polarity to rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
//      printf("Val1 %d\n", IC_Val1);
//      printf("Val2 %d\n", IC_Val2);
    }
  }
}

void HCSR04_Read(void)
{
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin HIGH
  DWT_Delay(2);  // wait for 2 us

  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  DWT_Delay(10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
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
  //=== NOT high temp alert-point!
//  if (data_temp >= 35.05) {
//    adjusted_temp = (1025 * data_temp) / (1000 - data_temp);
//  }
//  else {
//    adjusted_temp = (283 * 21 * data_temp) / (100 * (21 + data_temp));
//  }
  return adjusted_temp;
}

float obtain_max_temp(uint16_t *log_array, uint16_t log_array_size){
  uint16_t max_value = 0;
  for (int i=0; i < log_array_size; i++) {
    if (log_array[i] > max_value) {
      max_value = log_array[i];
    }
  }
  printf("%d;Local Max;", max_value);
  return ((float)max_value)/100;
}

void Array_sort(uint16_t *array , uint16_t amount)
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

/*
// function to calculate the median of the array
float Find_median(uint32_t array[] , uint16_t amount)
{
    float median=0;

    // if number of elements are even
    if(amount%2 == 0)
        median = (array[(amount-1)/2] + array[amount/2])/2.0;
    // if number of elements are odd
    else
        median = array[amount/2];

    return median;
}
*/

uint8_t bar_sensors[6] = {0xA2, 0xA4, 0xA6, 0xB2, 0xB4, 0xB6};
uint8_t i2c_adr = 0xB2; // A2, B2, A4, B4, A6, B6
uint32_t mem_adr = 0x07;
uint8_t in_buff[0x04];
uint8_t Err = 0;
float data_fl = 0.0;

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

    Array_sort(ambient_array, amount);
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

//void send_uart(float data_temp) {
//  uint8_t buf[16];
//  sprintf(buf, "Temp %.2F      ", data_temp);
//  HAL_UART_Transmit(&huart6, (uint8_t*)&buf, sizeof(buf), 0xFFFF);
//  HAL_UART_Transmit(&huart6, (uint8_t*)"              \r\n", sizeof(buf), 0xFFFF);
//  printf("data %4.2F \n", data_temp);
//  printf("data %s \n", buf);
//}

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
  /* USER CODE BEGIN 2 */
  printf("======= 000 =======\n\n");

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_Delay(300);
  
  uint8_t sensor_amount = 4;

//  float ambient_avr[6];
//  float max_data = 0.0;
//  float float_max_data = 30.0;
//  float float_amb_data = 30.0;
//  float diff_float_data = 3.0;
  float sum_data = 0;
  
  const uint16_t log_array_size = 300;
  uint16_t log_array[300];
  float log_buff[6];
  float max_temp_on_sensor = 0;
  uint16_t log_array_pos = 0;
  
  uint32_t measure_counter = 0;

//  int new_max_data = 3000;
//  int ambient_data = 3000;
//  int diff_data = 3000;
//  int data_val = 0;
//  int data_dec = 0;
//  int8_t Err = 0;

//  uint8_t bar_sensors[6] = {0xA2, 0xA4, 0xA6, 0xB2, 0xB4, 0xB6};
  uint8_t bar_sensors[6] = {0xA2, 0xB2, 0xA4, 0xB4, 0xA6, 0xB6};
//  uint8_t i2c_adr = 0xB2; // A2, B2, A4, B4, A6, B6
  uint32_t mem_adr = 0x07;
  uint8_t in_buff[0x04];

//=== Set new_adr as new I2C address
//  uint8_t new_adr = i2c_adr;
//  set_new_addr(new_adr>>1, mem_adr, in_buff);

//=== Temp ambient calibration
//  HAL_Delay(500);
//  for (int i=0; i < 6; i++) {
//    ambient_avr[i] = calibrate_ambient(bar_sensors[i]);
//    printf("At 0x%X ambient_avr = %.2F\n", bar_sensors[i], ambient_avr[i]);
//  }
//
//  float sum = 0;
//  for (uint8_t pos=0; pos < 6; pos++) {
//      sum += ambient_avr[pos];
////      printf("sum = ambient_avr = %.2F\n", ambient_avr[pos]);
//    }
//
//  float avr_ambient_temp = sum / 6;
//  printf("\nGlobal Ambient AVR = %.2F\n", avr_ambient_temp);
//
//  for (uint8_t pos=0; pos < 6; pos++) {
//    printf("Disp: %.2F\n", ambient_avr[pos] - avr_ambient_temp);
//  }
//  printf("\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      //=== Read data from HC-SR04
      /*
          HCSR04_Read();
          printf("Distance = %d\n", Distance);
      */

//    printf("=== %d ===\n", measure_counter++);
    measure_counter++;
    
    for (uint8_t i=1; i<sensor_amount+1; i++) {
      
    //=== Read data from sensor via I2C
        Err = HAL_I2C_Mem_Read(&hi2c1, bar_sensors[i], mem_adr, 1, in_buff, 3, 0x100);
        HAL_Delay(3); // Delay for I2C responce

    //=== Check if there is no error in I2C read
        if (Err == HAL_OK) {

          //=== Check if there is no error in I2C read
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
//                printf("%.4F;", data_fl);
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
              //        printf("Ta: %.2F\n", data_fl);

//			  float_max_data = max_data;
//			  diff_float_data = float_max_data - float_amb_data;

        }
        else {
              printf("\nError on I2C. Address %X\n", bar_sensors[i]);
        }
//        HAL_Delay(20); 
    }
    
//=== Check for threshold after every sensor cycle
    if ((sum_data / sensor_amount >= 25.6) 
        && (log_array_pos +sensor_amount < log_array_size)) {
      for (int i=0; i < sensor_amount; i++) {
        log_array[log_array_pos+i] = (int)(log_buff[i]*100);
//        printf("log %d   at pos %d\n", log_array[log_array_pos+i], log_array_pos+i);
      }
      log_array_pos += sensor_amount;
    } 
    else 
    { 
      if (log_array_pos >= 40) {
        max_temp_on_sensor = obtain_max_temp(log_array, log_array_pos);
//        printf("\n===== %0.3F = at %d===\n\n", max_temp_on_sensor, measure_counter);
        printf("%0.3F;", max_temp_on_sensor);
      }
      log_array_pos = 0;
    }
    
//    printf("%.4F\n", sum_data);
    printf("%d;%.3F\n",measure_counter, sum_data / sensor_amount);
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
