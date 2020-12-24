/*
 * Sonar.h
 *
 *  Created on: Oct 22, 2020
 *      Author: hasanshin
 */

#ifndef SRC_SONAR_H_
#define SRC_SONAR_H_
#include "main.h"
#include "stm32f4xx_hal_uart.h"
class Sonar {
public:
	Sonar();
	virtual ~Sonar();
	void receiveData(UART_HandleTypeDef *huart, uint8_t *data);

};


#endif /* SRC_SONAR_H_ */
