/*
 * ADF4355.h
 *
 *  Created on: Aug 23, 2020
 *      Author: voyag
 */

#ifndef INC_ADF4355_H_
#define INC_ADF4355_H_

#include "main.h"
#include "math.h"

typedef struct ADF4355_Configuration{	// Configuration-Data and Mirror of ADF4355-Registers in CPU-RAM. Last 4 bit of each 32bit value indicate the Register-Address and are constant
	long REFCLK;
	uint8_t ERROR;
	uint8_t MODE;
	uint8_t FPGA_MODE;
	int AMPLITUDE;
	float FREQUENCY;
	float PHASE;
	int REG0;
	int REG1;
	int REG2;
	int REG3;
	int REG4;
	int REG5;
	int REG6;
	int REG7;
	int REG8;
	int REG9;
	int REG10;
	int REG11;
	int REG12;
}ADF_CONF;

#define DIRECT_CTRL 0xDC	// Sets the DIRECT-Control: This will send data through the FPGA directly to the ADF4355
#define FPGA_CTRL 0xFC		// Sets the FPGA-Control: This will send data to a Buffer inside the FPGA and set up the FPGA

#define FREQ_UPDATE 0xF0
#define PHASE_UPDATE 0xE0
#define AMPLITUDE_UPDATE 0xA0

#define ADF4355_CS_Pin GPIO_PIN_13
#define ADF4355_CS_GPIO_Port GPIOE
#define ADF4355_LE_Pin GPIO_PIN_14
#define ADF4355_LE_GPIO_Port GPIOE
#define ADF4355_MUX_Pin GPIO_PIN_15
#define ADF4355_MUX_GPIO_Port GPIOE

uint8_t ADF4355_Setup(uint8_t startup, long refCLK, uint8_t mode);
uint8_t ADF4355_FPGA_Setup(unsigned char directMode, long updateFreq, int last_address, uint8_t stream_mode);	// Sets up the FPGA. directMode decides if the FPGA is just working as a Multiplexer or is acting as a buffer for the data. updateFreq decides the frequency with which the Samples are being sent to the ADF4355. last_address specifies the last Memory-Address (Up to 1 Mega-Samples if all 13 Registers are being saved for each Sample). streamMode decides if the FPGA is streaming the data just once or continuously.
uint8_t ADF4355_FPGA_START(unsigned char start);	// Signals the FPGA to START or STOP sending the data-buffer to the ADF4355
void ADF4355_FPGA_WriteData(float frequency, float phase, int amplitude); // Calculate all register-values for the ADF4355 and store one Sample inside the FPGA-Memory.
void ADF4355_FPGA_DUMP(int address);	// FPGA-Dump of one Sample plus the FPGA-Status Registers into an internal data-buffer of the function for debugging with Breakpoints.
void ADF4355_SetFrequency(float frequency);	// Sets the output-frequency.
void ADF4355_SetPhase(float phase);
void ADF4355_GetConfig(int *responseBuffer);




#endif /* INC_ADF4355_H_ */
