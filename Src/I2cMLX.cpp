/*
 * I2cMLX.cpp
 *
 *  Created on: 21 окт. 2020 г.
 *      Author: hasanshin
 */

#include "I2cMLX.h"

    uint8_t MLX11 = 0x11;
    uint8_t MLX12 = 0x12;
    uint8_t MLX13 = 0x13;
    uint8_t MLX14 = 0x14;
    uint8_t MLX15 = 0x15;
    uint8_t MLX16 = 0x16;

    uint8_t MLX90614_RAWIR1 = 0x04;
    uint8_t MLX90614_RAWIR2 = 0x05;
	uint8_t MLX90614_TA = 0x06;
	uint8_t MLX90614_TOBJ1 =  0x07;
	uint8_t MLX90614_TOBJ2 = 0x08;
	// EEPROM
	uint8_t MLX90614_TOMAX =  0x20;
	uint8_t MLX90614_TOMIN = 0x21;
	uint8_t MLX90614_PWMCTRL = 0x22;
	uint8_t MLX90614_TARANGE = 0x23;
	uint8_t MLX90614_EMISS = 0x24;
	uint8_t MLX90614_CONFIG = 0x25;
	uint8_t MLX90614_ADDR = 0x2E;
	uint8_t MLX90614_ID1  = 0x3C;
	uint8_t MLX90614_ID2 = 0x3D;
	uint8_t MLX90614_ID3 = 0x3E;
	uint8_t MLX90614_ID4 = 0x3F;

I2cMLX::I2cMLX() {
	// TODO Auto-generated constructor stub

}

I2cMLX::~I2cMLX() {
	// TODO Auto-generated destructor stub
}

//==========================================
//read from sensor
//===========================================
void I2cMLX::mlxRead(I2C_HandleTypeDef i2c, uint8_t Addr, uint8_t Reg, uint8_t *data){

	HAL_StatusTypeDef status = HAL_OK;
  //while(status!= HAL_OK){
  status = HAL_I2C_Mem_Read(&i2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, sizeof(data), 0x100);


  //}
  if (status!= HAL_OK)  {

  }
}

//==========================================
//read from sensor
//===========================================
float I2cMLX::mlxRead(I2C_HandleTypeDef i2c, uint8_t Addr, uint8_t Reg, uint8_t *data, int time){

	HAL_StatusTypeDef status = HAL_OK;

  //while(status!= HAL_OK){
  status = HAL_I2C_Mem_Read(&i2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, sizeof(data), time);

  float fl_mlx_temp_array_amb1 = (data[1] * 256 + data[0])*0.02 - 273.15;
  //}
  if (status!= HAL_OK)  {

  }

    return fl_mlx_temp_array_amb1;
}

//==========================
// crc calculation
//================================
uint8_t I2cMLX::crc8ccitt(uint8_t *data, uint8_t size) {
	uint8_t val = 0;

	uint8_t * pos = (uint8_t *) data;
	uint8_t * end = pos + size;

	while (pos < end) {
		val = CRC_TABLE[val ^ *pos];
		pos++;
	}

	return val;
}
//=================================
// get slava address
//=================================
uint8_t I2cMLX::getMlxSlaveAddr(int id){

	int idMlx = 0x5A;
	switch(id){
	case 1:
		idMlx = MLX11;
	break;
	case 2:
			idMlx = MLX12;
		break;
	case 3:
			idMlx = MLX13;
		break;
	case 4:
			idMlx = MLX14;
		break;
	case 5:
			idMlx = MLX15;
		break;
	case 6:
				idMlx = MLX16;
			break;

	}

	return (idMlx<<1)|0x01;
}
//===========================
// get settings addr
//======================
uint8_t I2cMLX::getRegAddr(std::string settings){

	int SettingsAddr;
	if(settings=="TOBJ1"){
		SettingsAddr = MLX90614_TOBJ1;
	}
	if(settings=="TOBJ2"){
			SettingsAddr = MLX90614_TOBJ2;
		}
	if(settings=="CONFIG"){
			SettingsAddr = MLX90614_CONFIG;
		}
	return SettingsAddr;
}
