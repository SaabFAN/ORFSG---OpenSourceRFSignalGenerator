/*
 * AD9957Driver.cpp
 *
 *  Created on: Sep 27, 2020
 *      Author: voyag
 */

#include <AD9957_Driver.h>

// Variables
AD9957_CONF ad9957_config;
AD9957_DATA data;
AD9957_REG reg;

AD9957_Driver::AD9957_Driver() {
	// TODO Auto-generated constructor stub
	error = 0x00;
}

AD9957_Driver::~AD9957_Driver() {
	// TODO Auto-generated destructor stub
}

uint8_t AD9957_Driver::Setup(uint8_t startup, long refCLK, uint8_t ctrl_mode,
		uint8_t chip_mode, uint8_t int_RAM_mode) {
	error = 0x00;
	if (startup > 0) {
		ad9957_config.REFCLK = refCLK;
	}
	// Setting up the Chip-Configuration
	ad9957_config.CFR1_OP_MODE = chip_mode;
	ad9957_config.CFR1_SDIO_IN_ONLY = 1;
	switch (chip_mode) { // Update the config-variable according to selected Chip Mode
	case AD_QDUC:
		ad9957_config.CFR1_DIGI_PWRDN = 0;
		ad9957_config.CFR1_DAC_PWRDN = 0;
		ad9957_config.CFR1_REFCLK_IN_PWRDN = 0;
		ad9957_config.CFR1_AUXDAC_PWRDN = 0;
		ad9957_config.CFR1_EXT_PWRDN_MODE = 0;
		ad9957_config.CFR2_PDCLK_ENABLE = 1;
		ad9957_config.CFR3_DRV0 = 0x01;
		ad9957_config.CFR3_VCO_SELECT = 0x05;
		ad9957_config.CFR3_ICP = 0x03;
		ad9957_config.CFR3_REFCLK_DIV_BYPASS = 0;
		ad9957_config.CFR3_REFCLK_DIV_RESET = 1;
		ad9957_config.CFR3_PLL_ENABLE = 1;
		ad9957_config.CFR3_N = 50;
		ad9957_config.FSC = 0xFF;
		ad9957_config.AMP_RAMP_RATE = 128;
		ad9957_config.AMP_ASF = (int) (1.0 * (pow(2, 14) - 1));
		ad9957_config.AMP_STEP_SIZE = 0;
		break;
	case AD_SINGLE_TONE:
		ad9957_config.CFR1_DIGI_PWRDN = 0;
		ad9957_config.CFR1_DAC_PWRDN = 0;
		ad9957_config.CFR1_REFCLK_IN_PWRDN = 0;
		ad9957_config.CFR1_AUXDAC_PWRDN = 0;
		ad9957_config.CFR1_EXT_PWRDN_MODE = 0;
		ad9957_config.CFR2_EN_PROFILE_REG_ASF_SRC = 1;
		ad9957_config.CFR2_PDCLK_ENABLE = 1;
		ad9957_config.CFR3_DRV0 = 0x02;
		ad9957_config.CFR3_VCO_SELECT = 0x05;
		ad9957_config.CFR3_ICP = 0x03;
		ad9957_config.CFR3_REFCLK_DIV_BYPASS = 0;
		ad9957_config.CFR3_REFCLK_DIV_RESET = 1;
		ad9957_config.CFR3_PLL_ENABLE = 1;
		ad9957_config.CFR3_N = 50;
		ad9957_config.FSC = 0xFF;
		ad9957_config.AMP_RAMP_RATE = 128;
		ad9957_config.AMP_ASF = (int) (1.0 * (pow(2, 14) - 1));
		ad9957_config.AMP_STEP_SIZE = 0;
		break;
	case AD_IDAC:
		ad9957_config.CFR1_DIGI_PWRDN = 0;
		ad9957_config.CFR1_DAC_PWRDN = 0;
		ad9957_config.CFR1_REFCLK_IN_PWRDN = 0;
		ad9957_config.CFR1_AUXDAC_PWRDN = 0;
		ad9957_config.CFR1_EXT_PWRDN_MODE = 0;
		ad9957_config.CFR2_PDCLK_ENABLE = 1;
		ad9957_config.CFR3_DRV0 = 0x03;
		ad9957_config.CFR3_VCO_SELECT = 0x05;
		ad9957_config.CFR3_ICP = 0x03;
		ad9957_config.CFR3_REFCLK_DIV_BYPASS = 0;
		ad9957_config.CFR3_REFCLK_DIV_RESET = 1;
		ad9957_config.CFR3_PLL_ENABLE = 1;
		ad9957_config.CFR3_N = 50;
		ad9957_config.FSC = 0xFF;
		ad9957_config.AMP_RAMP_RATE = 128;
		ad9957_config.AMP_ASF = (int) (1.0 * (pow(2, 14) - 1));
		ad9957_config.AMP_STEP_SIZE = 0;
		break;
	case AD_OFF:
		ad9957_config.CFR1_DIGI_PWRDN = 1;
		ad9957_config.CFR1_DAC_PWRDN = 1;
		ad9957_config.CFR1_REFCLK_IN_PWRDN = 1;
		ad9957_config.CFR1_AUXDAC_PWRDN = 1;
		ad9957_config.CFR1_EXT_PWRDN_MODE = 1;
		break;
	}
	switch (int_RAM_mode) {
	// TODO: Configure Chip to use internal RAM depending on the settings
	case RAM_OFF:
		ad9957_config.CFR1_RAM_EN = 0;
		break;
	default:
		ad9957_config.CFR1_RAM_EN = 0;
		break;
	}
	ad9957_config.SYSCLK = ad9957_config.REFCLK * ad9957_config.CFR3_N;
	ST_SetFrequency(5000000.0);
	ST_SetAmplitude(1.0);
	ST_SetPhase(0.0);
	Generate_Registers(0xFF, PROFILE_0);
	return error;
}

uint8_t AD9957_Driver::ReportError() {
	return error; // Returns the last Error-Code that occurred.
}

uint8_t AD9957_Driver::IOReset() {
	// Performs an IO-Reset of the AD9957 (Toggling the IOReset-Line)
	return error;
}

uint8_t AD9957_Driver::SetMode(uint8_t mode) {
	return error;
}

uint8_t AD9957_Driver::SetupFPGA(uint8_t directMode, uint8_t FPGAMode) {
	// Setup the FPGA. Must be called after Setup has been called and all the Driver-Variables are properly initialized.
	return error;
}

uint8_t AD9957_Driver::WriteFPGA(uint16_t *dataByte, uint32_t datalength,
		uint32_t startAddress) {
	//Function to Write 16bit long Data-Words to the SDRAM Connected to the FPGA. The Start-Address will be incremented in the FPGA for each word transferred
	return error;
}

uint16_t AD9957_Driver::ReadFPGA(uint32_t startAddress) {
	// Read ONE DataWord from the SDRAM-Area reserved for the AD9957-Subsystem
}

uint8_t AD9957_Driver::SetupRamp(double startFreq, double stopFreq,
		int stepSize, double sampleRate, uint8_t mode) {
	// Setup a Frequency-Ramp that starts at a specified frequency and moves to the specified stop-frequency in the specified steps and at the specified Samplerate (Up to 125 MS/s)
	return error;
}

// Single Tone Mode-Functions
uint8_t AD9957_Driver::ST_SetFrequency(double frequency) {
	double ftw = pow(2,32) - 1;
	ftw = ftw * (frequency / ad9957_config.SYSCLK);
	ad9957_config.ST_FTW = (int) ftw;
	ad9957_config.QDUC_FTW = ad9957_config.ST_FTW;
	return error;
}

uint8_t AD9957_Driver::ST_SetPhase(double phase) {
	// Calculate the Phase Offset Word: 360°/14bit breites Register = 0.022°/LSB
	ad9957_config.ST_POW = (int) (phase / 0.022);
	return error;
}

uint8_t AD9957_Driver::ST_SetAmplitude(double amplitude) {
	ad9957_config.AMP_ASF = (int) (pow(2, 14) * amplitude);
	return error;
}

uint8_t AD9957_Driver::ST_FillRAM_Internal(double frequency, double phase,
		double amplitude, uint16_t address) {
	// Fill the RAM internal to the AD9957 with data that can be played back in Single Tone-Mode
	return error;
}

uint8_t AD9957_Driver::ST_FillRAM_FPGA(double frequency, double phase,
		double amplitude, uint32_t address) {
	// Fill the RAM of the FPGA with data that can be played back in Single Tone-Mode
	// TODO: Implement FPGA AD9957 Statemachine
	return error;
}
uint8_t AD9957_Driver::ST_StartRAMPB() {
	return error;
}

uint8_t AD9957_Driver::ST_StopRAMPB() {
	return error;
}

// Interpolating DAC-Functions TODO: Implement FPGA-Statemachine for the AD9957
uint8_t AD9957_Driver::IDAC_WriteBuffer(uint16_t address, uint16_t DACword) {
	// Write the DAC-Buffer one Data-Word at a time. The selected Mode determines where the data goes (AD9957 Internal RAM, or FPGA SDRAM)
	return error;
}

uint8_t AD9957_Driver::IDAC_Start() {
	return error;
}

uint8_t AD9957_Driver::IDAC_Stop() {
	return error;
}

// Quadrature Modulation Mode
uint8_t AD9957_Driver::QDUC_Setup(double carrier) {
	// Setup for the QDUC-Mode. This will configure the AD9957 for QDUC-Mode
	return error;
}

uint8_t AD9957_Driver::QDUC_WriteBuffer(uint16_t address, uint16_t IWord,
		uint16_t QWord) {
	// Write the IQ-DataBuffer two Words at a Time.
	return error;
}

uint8_t AD9957_Driver::QDUC_Start() {
	// Signal the FPGA to start sending the IQ-Datastream to the AD9957.
	return error;
}

uint8_t AD9957_Driver::QDUC_Stop() {
	// Signal the FPGA to stop sending the IQ-Datastream.
	return error;
}

// RAM Control-Functions
uint8_t AD9957_Driver::RAM_Read(uint16_t address, uint32_t *pointer) {
	return error;
}

uint8_t AD9957_Driver::RAM_Write(uint16_t address, uint32_t word) {
	return error;
}

uint8_t AD9957_Driver::Generate_Registers(uint8_t UpdateMode, uint8_t profile) {
	switch (UpdateMode) {
	case 0xFF:
		reg.REG_CFR1 = (int) ((ad9957_config.CFR1_LSB_FIRST * pow(2, 0))
				+ (ad9957_config.CFR1_SDIO_IN_ONLY * pow(2, 1))
				+ (ad9957_config.CFR1_AUTO_PWRDN * pow(2, 2))
				+ (ad9957_config.CFR1_EXT_PWRDN_MODE * pow(2, 3))
				+ (ad9957_config.CFR1_AUXDAC_PWRDN * pow(2, 4))
				+ (ad9957_config.CFR1_REFCLK_IN_PWRDN * pow(2, 5))
				+ (ad9957_config.CFR1_DAC_PWRDN * pow(2, 6))
				+ (ad9957_config.CFR1_DIGI_PWRDN * pow(2, 7))
				+ (ad9957_config.CFR1_OSK_AUTO * pow(2, 8))
				+ (ad9957_config.CFR1_OSK_ENABLE * pow(2, 9))
				+ (ad9957_config.CFR1_LOAD_ARR_IOUP * pow(2, 10))
				+ (ad9957_config.CFR1_CLR_PHASE_ACCU * pow(2, 11))
				+ (ad9957_config.CFR1_AUTOCLR_PHASE_ACCU * pow(2, 13))
				+ (ad9957_config.CFR1_SEL_DDS_OUTPUT * pow(2, 16))
				+ (ad9957_config.CFR1_CLEAR_CCI * pow(2, 21))
				+ (ad9957_config.CFR1_INV_SINC_FILT * pow(2, 22))
				+ (ad9957_config.CFR1_MAN_OSK_CTRL * pow(2, 23))
				+ (ad9957_config.CFR1_OP_MODE * pow(2, 25))
				+ (ad9957_config.CFR1_RAM_PB_DEST * pow(2, 28))
				+ (ad9957_config.CFR1_RAM_EN * pow(2, 31)));// Calculate CFR1
		reg.REG_CFR2 = (int) ((ad9957_config.CFR2_SYNC_TIMING_VAL_DISABLE
				* pow(2, 5))
				+ (ad9957_config.CFR2_DATA_ASSEMB_HOLD_LAST * pow(2, 6))
				+ (ad9957_config.CFR2_Q_FIRST_DATA_PROC * pow(2, 8))
				+ (ad9957_config.CFR2_TXENABLE_INVERT * pow(2, 9))
				+ (ad9957_config.CFR2_PDCLK_INVERT * pow(2, 10))
				+ (ad9957_config.CFR2_PDCLK_ENABLE * pow(2, 11))
				+ (ad9957_config.CFR2_DATA_FORMAT * pow(2, 12))
				+ (ad9957_config.CFR2_PDCLK_RATE * pow(2, 13))
				+ (ad9957_config.CFR2_IOUP_RATE_DIV * pow(2, 14))
				+ (ad9957_config.CFR2_READ_EFF_FTW * pow(2, 16))
				+ (ad9957_config.CFR2_SYNC_CLK_EN * pow(2, 22))
				+ (ad9957_config.CFR2_INT_IOUP_ACTIVE * pow(2, 23))
				+ (ad9957_config.CFR2_EN_PROFILE_REG_ASF_SRC * pow(2, 24))
				+ (ad9957_config.CFR2_BLACKFIN_EARLY_FRAME_SYNC * pow(2, 29))
				+ (ad9957_config.CFR2_BLACKFIN_BIT_ORDER * pow(2, 30))
				+ (ad9957_config.CFR2_BLACKFIN_ACTIVE * pow(2, 31)));
		reg.REG_CFR3 = (int) ((ad9957_config.CFR3_N * pow(2, 1))
				+ (ad9957_config.CFR3_PLL_ENABLE * pow(2, 8))
				+ (ad9957_config.CFR3_REFCLK_DIV_RESET * pow(2, 14))
				+ (ad9957_config.CFR3_REFCLK_DIV_BYPASS * pow(2, 15))
				+ (ad9957_config.CFR3_ICP * pow(2, 19))
				+ (ad9957_config.CFR3_VCO_SELECT * pow(2, 24))
				+ (ad9957_config.CFR3_DRV0 * pow(2, 28)));
		reg.REG_AUX_DAC = (int) ad9957_config.FSC;
		reg.REG_IOUP_RATE = (int) ad9957_config.IOUPDATE_RATE;
		reg.REG_RAMSEG0_1 = (int) ad9957_config.RAM_ADDRESS_STEP_RATE_0;
		reg.REG_RAMSEG0_0 = (int) ((ad9957_config.RAM_PB_MODE_0 * pow(2, 0))
				+ (ad9957_config.RAM_START_ADDR_0 * pow(2, 6))
				+ (ad9957_config.RAM_END_ADDR_0 * pow(2, 22)));
		reg.REG_RAMSEG1_1 = (int) ad9957_config.RAM_ADDRESS_STEP_RATE_1;
		reg.REG_RAMSEG1_0 = (int) ((ad9957_config.RAM_PB_MODE_1 * pow(2, 0))
				+ (ad9957_config.RAM_START_ADDR_1 * pow(2, 6))
				+ (ad9957_config.RAM_END_ADDR_1 * pow(2, 22)));
		reg.REG_AMP = (int) ((ad9957_config.AMP_STEP_SIZE * pow(2, 0))
				+ (ad9957_config.AMP_ASF * pow(2, 2))
				+ (ad9957_config.AMP_RAMP_RATE * pow(2, 16)));
		// reg.REG_MULTICHIP_SYNC = Assemble Multichip-Sync Register - Nothign to do here, since only one chip used.
		reg.REG_ST1 = (int) ((ad9957_config.ST_POW * pow(2, 0))
				+ (ad9957_config.ST_ASF * pow(2, 16)));
		reg.REG_ST0 = (int) ad9957_config.ST_FTW;
		reg.REG_QDUC1 = (int) ((ad9957_config.QDUC_POW * pow(2, 0))
				+ (ad9957_config.QDUC_OSK * pow(2, 16))
				+ (ad9957_config.QDUC_CCI_INVERSE_BYPASS * pow(2, 24))
				+ (ad9957_config.QDUC_SPECTRAL_INVERT * pow(2, 25))
				+ (ad9957_config.QDUC_CCI_INTERPOLATION_RATE * 2, 26));
		reg.REG_QDUC0 = (int) ad9957_config.QDUC_FTW;
		reg.REG_RAM = (int) 0;// TODO: Generate the content of the RAM-Register
		// TODO: Update All Registers
		WriteRegister(CFR1);
		WriteRegister(CFR2);
		WriteRegister(CFR3);
		WriteRegister(AUXDAC);
		WriteRegister(AMP);
		WriteRegister(PROFILE_0);
		break;
	case 0x01:
		// TODO: Update Only Registers for Single Tone-Mode
		break;
	case 0x00:
		// TODO: Update the Registers necessary for QDUC-Mode
		break;
	case 0x10:
		// TODO: Update the Registers necessary for Interpolating DAC-Mode
		break;
		// TODO: Add more Cases to speed up the Register-Calculation Process by leaving out even more registers (e.g. Only Update Frequency Tuning Word)
	}
	return error;
}
uint8_t AD9957_Driver::WriteRegister(uint8_t regAddr) { // Subroutine that Writes a Register-Set
	// TODO: Assemble the Register-Values into 32bit Datawords
	uint8_t result;	// Success? Yes / NO
	uint8_t reglength;
	uint32_t *datawordH;
	uint32_t *datawordL;
	switch (regAddr) {
	case PROFILE_7:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_6:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_5:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_4:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_3:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_2:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_1:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case PROFILE_0:
		reglength = 9; // Profile-Register is 8 Bytes long, plus  Instruction-Byte
		switch (ad9957_config.CFR1_OP_MODE) {
		case AD_QDUC:
			datawordH = &reg.REG_QDUC1;
			datawordL = &reg.REG_QDUC0;
			break;
		case AD_SINGLE_TONE:
			datawordH = &reg.REG_ST1;
			datawordL = &reg.REG_ST0;
			break;
		}
		break;
	case RAMSEG0:
		reglength = 7; // Ramsegment-Register is 6 Bytes (48 Bits) long, plus Instruction-Byte
		break;
	case RAMSEG1:
		reglength = 7; // Ramsegment-Register is 6 Bytes (48 Bits) long, plus Instruction-Byte
		break;
	case CFR1:
		reglength = 5;
		datawordL = &reg.REG_CFR1;
		break;
	case CFR2:
		reglength = 5;
		datawordL = &reg.REG_CFR2;
		break;
	case CFR3:
		reglength = 5;
		datawordL = &reg.REG_CFR3;
		break;
	case AUXDAC:
		reglength = 5;
		datawordL = &reg.REG_AUX_DAC;
	case AMP:
		reglength = 5;
		datawordL = &reg.REG_AMP;
		break;
	}
	uint8_t buf[reglength];
	uint8_t transmitSize = reglength;
	buf[0] = regAddr;
	reglength--;

	// TODO: REWORK THIS SECTION TO MAKE IT COMPATIBLE WITH 6 BYTE LONG REGISTERS!
	if (reglength > 6) {
		for (int i = 0; i < 4; i++) {
			buf[reglength - i] = (uint8_t) (*datawordH >> (i * 8)); // Split the value into 4 Bytes
		}
		reglength = reglength - 4;
	}
	if (reglength > 4) {
		for (int i = 0; i < 2; i++) {
			buf[reglength - i] = (uint8_t) (*datawordH >> (i * 8)); // Split the value into 4 Bytes
		}
		reglength = reglength - 2;
	}
	for (int i = 0; i < 4; i++) {
		buf[reglength - i] = (uint8_t) (*datawordL >> (i * 8)); // Split the value into 4 Bytes
	}
	HAL_GPIO_WritePin(AD9957_CS_GPIO_Port, AD9957_CS_Pin, GPIO_PIN_RESET); // Select the Chip
	HAL_GPIO_WritePin(AD9957_IOUP_GPIO_Port, AD9957_IOUP_Pin, GPIO_PIN_RESET); // Set the IOUpdate-Pin to LOW
	HAL_SPI_Transmit(&hspi2, buf, transmitSize, 800); // Send the Data to the Chip via SPI2
	HAL_GPIO_WritePin(AD9957_IOUP_GPIO_Port, AD9957_IOUP_Pin, GPIO_PIN_SET); // Set the IOUpdate-Pin HIGH to load the data from the shift register into the register set by the instruction-byte
	HAL_GPIO_WritePin(AD9957_CS_GPIO_Port, AD9957_CS_Pin, GPIO_PIN_SET); // Transfer over, Deselect Chip
	return result;
}
uint32_t AD9957_Driver::ReadRegister(uint8_t regAddr) {
	// Read one 32bit Register
}
