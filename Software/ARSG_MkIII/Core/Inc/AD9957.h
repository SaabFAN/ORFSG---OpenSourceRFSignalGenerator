/*
 * AD9957.h
 *
 *  Created on: Aug 23, 2020
 *      Author: voyag
 */

#ifndef INC_AD9957_H_
#define INC_AD9957_H_

#include "main.h"

#define AD9957_IOUP_Pin GPIO_PIN_8
#define AD9957_IOUP_GPIO_Port GPIOE
#define AD9957_CS_Pin GPIO_PIN_9
#define AD9957_CS_GPIO_Port GPIOE
#define AD9957_RESET_Pin GPIO_PIN_10
#define AD9957_RESET_GPIO_Port GPIOE
#define AD9957_IORESET_Pin GPIO_PIN_11
#define AD9957_IORESET_GPIO_Port GPIOE
#define AD9957_PLL_LOCK_Pin GPIO_PIN_12
#define AD9957_PLL_LOCK_GPIO_Port GPIOE

void AD9957_Setup();	// Setup of the AD9957-Chip
void AD9957_PLLCheck();	// Checks the Lock-Signal of the AD9957 PLL
void AD9957_SetMode(uint8_t mode);	// Set the mode of the AD9957
void AD9957_CWFreq(int frequency);
void AD9957_CWAmplitude(int amplitude);
void AD9957_CWPhase(int phase);
void AD9957_FillIntRAM(int *databuffer);


#endif /* INC_AD9957_H_ */
