/*
 * sensor.c
 *
 *  Created on: 9 Nov 2017
 *      Author: tinova
 */

#include "sensor.h"

extern uint8_t tO1, tO2;
extern uint8_t teken1, teken2;

uint8_t Advance(void) {
	uint8_t FlagDetect = 0;
	if (HAL_GPIO_ReadPin(ADVANCE_GPIO_Port, ADVANCE_Pin) == GPIO_PIN_RESET) {
		teken1 = 1;
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(ADVANCE_GPIO_Port, ADVANCE_Pin)
				== GPIO_PIN_RESET) {
			while (HAL_GPIO_ReadPin(ADVANCE_GPIO_Port, ADVANCE_Pin)
					== GPIO_PIN_RESET && tO1 <= 100)
				;
			if (tO1 > 100) {
				FlagDetect = 2;
			} else {
				FlagDetect = 1;
				tO1 = 0;
			}
		}
	} else {
		tO1 = 0;
	}
	teken1 = 0;
	return FlagDetect;
}

uint8_t Mode(void) {
	tO2 = 0;
	uint8_t FlagDetect1 = 0;
	if (HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == GPIO_PIN_RESET) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == GPIO_PIN_RESET) {
			while (HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == GPIO_PIN_RESET)
				;
			FlagDetect1 = 1;
		}
	}
	teken2 = 0;
	return FlagDetect1;
}
