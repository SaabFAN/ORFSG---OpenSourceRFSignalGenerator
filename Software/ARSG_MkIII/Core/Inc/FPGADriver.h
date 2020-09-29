/*
 * FPGADriver.h
 *
 *  Created on: Sep 27, 2020
 *      Author: voyag
 */

#ifndef INC_FPGADRIVER_H_
#define INC_FPGADRIVER_H_

#include "Main.h"

extern SPI_HandleTypeDef hspi2;

/* Note on how to address the FPGA and its subsystems:
 * - MAX10_CS selects the FPGA. The FPGA ignores inputs or is transparent if MAX10_CS is HIGH
 * - FPGA_IRQ-PIN is active LOW and used in two ways determined by the status of MAX10_CS:
 * 		- MAX10_CS = HIGH: Interrupt-Request by the FPGA signaled by a LOW
 * 		- MAX10_CS = LOW: BUSY-Flag - LOW = Ready, HIGH = Busy (e.g. Waiting for SDRAM to be ready) - An Interrupt is automatically Cleared by asserting MAX10_CS LOW. To check the cause for the Interrupt, perform a 1 Byte READ in FPGA-STATUS Mode.
 * - AD9957_RESET Is multiplexed as the FPGA-Reset Line if MAX10_CS is LOW. Signal-Polarity is Active HIGH.
 * - AD9957_IOUP, AD9957_IORESET and ADF4355_LE are used to address the Registers inside the FPGA:
 *
 * 						Register-Table:
 * 		AD9957_IOUP | AD9957_IORESET | ADF4355_LE | Function
 * 			LOW				LOW				LOW			FPGA-STATUS <- READ ONLY
 * 			LOW				LOW				HIGH		SDRAM-Access to CPU-Memory
 * 			LOW				HIGH			LOW			TBD
 * 			LOW				HIGH			HIGH		TBD
 * 			HIGH			LOW				LOW			TBD
 * 			HIGH			LOW				HIGH		TBD
 * 			HIGH			HIGH			LOW			FPGA-SETUP
 * 			HIGH			HIGH			HIGH		SubsystemAddress (2 Bytes: Subsystem, Register-Address of Subsystem) <- This mode allows access to subsystems that are not directly addressable by the limited amount of Pins.
 *
 * - A combination of MAX10_CS, AD9957_CS and ADF4355_CS select the Subsystems responsible for the AD9957 and ADF4355. AD9957_IOUP, AD9957_IORESET and ADF4355_LE are used to address the registers of the Subsystems.
 * 						Register-Table (AD9957):
 * 		AD9957_IOUP | AD9957_IORESET | ADF4355_LE | Function
 * 			LOW				LOW				LOW			STATUS <- READ ONLY
 * 			LOW				LOW				HIGH		TBD
 * 			LOW				HIGH			LOW			TBD
 * 			LOW				HIGH			HIGH		TBD
 * 			HIGH			LOW				LOW			TBD
 * 			HIGH			LOW				HIGH		TBD
 * 			HIGH			HIGH			LOW			TBD
 * 			HIGH			HIGH			HIGH		SubsystemAddress (2 Bytes: Subsystem, Register-Address of Subsystem) <- This mode allows access to subsystems that are not directly addressable by the limited amount of Pins.
 *
 * 						Register-Table (ADF4355):
 * 		AD9957_IOUP | AD9957_IORESET | ADF4355_LE | Function
 * 			LOW				LOW				LOW			STATUS <- READ ONLY
 * 			LOW				LOW				HIGH		TBD
 * 			LOW				HIGH			LOW			TBD
 * 			LOW				HIGH			HIGH		TBD
 * 			HIGH			LOW				LOW			TBD
 * 			HIGH			LOW				HIGH		TBD
 * 			HIGH			HIGH			LOW			TBD
 * 			HIGH			HIGH			HIGH		SubsystemAddress (2 Bytes: Subsystem, Register-Address of Subsystem) <- This mode allows access to subsystems that are not directly addressable by the limited amount of Pins.
 */
#define MAX10_CS_Pin GPIO_PIN_12
#define MAX10_CS_GPIO_Port GPIOB
#define FPGA_IRQ_Pin GPIO_PIN_7
#define FPGA_IRQ_GPIO_Port GPIOE
#define AD9957_RESET_Pin GPIO_PIN_10
#define AD9957_RESET_GPIO_Port GPIOE
#define AD9957_IOUP_Pin GPIO_PIN_8
#define AD9957_IOUP_GPIO_Port GPIOE
#define AD9957_CS_Pin GPIO_PIN_9
#define AD9957_CS_GPIO_Port GPIOE
#define AD9957_RESET_Pin GPIO_PIN_10
#define AD9957_RESET_GPIO_Port GPIOE
#define AD9957_IORESET_Pin GPIO_PIN_11
#define AD9957_IORESET_GPIO_Port GPIOE

class FPGA_Driver {
public:
	FPGA_Driver();
	virtual ~FPGA_Driver();
	// All Functions return at least 0x00 (success) or 0xEE (error encountered) upon completion.
	uint8_t Setup(uint8_t mode, uint8_t reset);
	// Functions to access the SDRAM-Area reserved for the CPU. ATTENTION: These functions are unavailable if the FPGA is streaming data to the AD9957 or the ADF4356 at full speed.
	uint8_t SDRAM_WriteWord(uint16_t *dataword, uint16_t *address);	// Write one 16bit Word to the SDRAM inside the Memory-Area
	uint8_t SDRAM_Write(uint16_t *datawords, uint16_t *startAddress, uint16_t *dataLength);	// Write an arbitrary number (defined by dataLength) of 16bit Words to the SDRAM.
	uint8_t SDRAM_ReadWord(uint16_t *dataBuffer, uint16_t *address);	// Read one 16bit Word from the SDRAM and store the data in the location pointed to by dataBuffer
	uint8_t SDRAM_Read(uint16_t *dataBuffer, uint16_t *startAddress, uint16_t *dataLength);	// Read an arbitrary amount of data from the SDRAM. MAKE SURE DATA BUFFER IS LARGE ENOUGH!
	// Functions to Access the AD9957 Subsystem of the FPGA Directly
	uint8_t AD9957_Setup(uint32_t *configData); // Setup for the AD9957-Subsystem. It uses the Configuration-Data of the AD9957-Functions.
	uint8_t AD9957_Status(uint32_t *dataBuffer); // Dumps the registers of the AD9957 Subsystem to the memory-Location specified by dataBuffer. Make sure the dataBuffer is large enough!

	uint8_t AD9957_FillBuffer(uint32_t *dataBuffer);	// Fills the Buffer of the
};

#endif /* INC_FPGADRIVER_H_ */
