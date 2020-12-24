/*
 * Sonar.cpp
 *
 *  Created on: Oct 22, 2020
 *      Author: hasanshin
 */

#include "Sonar.h"


Sonar::Sonar() {
	// TODO Auto-generated constructor stub

}

Sonar::~Sonar() {
	// TODO Auto-generated destructor stub
}

void Sonar::receiveData(UART_HandleTypeDef *huart, uint8_t *data){

	HAL_UART_Receive_DMA(huart, (uint8_t*)data, sizeof data);


}
