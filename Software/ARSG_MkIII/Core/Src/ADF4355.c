/*
 * ADF4355.c
 *
 *  Created on: Aug 23, 2020
 *      Author: voyag
 */

#include "ADF4355.h"

ADF_CONF adf_config;

extern SPI_HandleTypeDef hspi2;

unsigned char setup_complete = 0x00;

void ADF4355_ConvertFreq(double frequency);
void ADF4355_WriteData();
void ADF4355_WriteRegister(int adfreg);
void ADF4355_WriteDataLite(uint8_t mode);
#define ADF_DEBUG

uint8_t ADF4355_Setup(uint8_t startup, long refCLK, uint8_t mode, int MOD2) { // Setup the ADF4355 with standard values (Output: ON, Freqency: 100 MHz (for debug-purposes)). Requires Reference-Frequency, Mode, MOD2-Value
	switch (setup_complete) {
	case 0x01:
		break;
	default:
		startup = 0x01;	// Setup has not been completed, set Startup to true to make sure the chip is being set up correctly.
		break;
	}
	switch (startup) {
	case 0x01: // This is the first time we're running the Setup - ADF4355-Configuration needs to be set up.
		if (refCLK < 10000000) {
// RefCLK Lower than 10 MHz and out of Range!
			adf_config.ERROR = 0xF0; // Save the REFERENCE-ERROR and
		}
		adf_config.REFCLK = refCLK;
		adf_config.AMPLITUDE = 0;	// Set Amplitude to lowest possible value
		adf_config.MODE = mode;	// Store the supplied Mode in the Mode-Variable
		adf_config.MOD1 = 16777216;	// MOD1 (Constant)

		adf_config.R0_PRESCAL = 0;	// Set Prescaler to 4/5 Mode
		adf_config.R0_AUTOCAL = 1;	// Enable the Autocalibration
		adf_config.R0_Nmin = 23;
		// TODO: Split the MOD2-Value to have the MSB and LSB-Value
		if (MOD2 <= 2) {
			adf_config.R2_MOD2_LSB = 2;	// Default-Value for MOD2
		} else {
			adf_config.R2_MOD2_LSB = MOD2;
		}
		adf_config.MOD2 = MOD2;	// MOD2 Value (Defines the Channel-Spacing

		adf_config.R4_MUXOUT = 4;	// Set the MUXOUT to Digital Lock-Detect
		adf_config.R4_REFDOUBLE = 1;
		adf_config.R4_REFDIV2 = 1;
		adf_config.R4_RCOUNTER = 1;
		adf_config.R4_DOUBLEBUF = 1;
		adf_config.R4_CHGPUMP = 0x03;	// Set Charge-Pump to 3.3mA
		adf_config.R4_REFMODE = 1;	// Differential Reference Select
		adf_config.R4_MUXLOG = 1;
		adf_config.R4_PDPOLA = 1;
		adf_config.R4_PWRDN = 0;
		adf_config.R4_CP3STATE = 0;
		adf_config.R4_COUNTRES = 0;
		// Calculate the PFD-Frequency
		adf_config.PFD = ((adf_config.REFCLK * (adf_config.R4_REFDOUBLE + 1))
				/ (adf_config.R4_REFDIV2 + 1)) / adf_config.R4_RCOUNTER;

		adf_config.REG5 = 0x00800025; // // REGISTER 5 IS RESERVED AND NEEDS TO BE SET TO 0x00800025

		adf_config.R6_BLEED_POLARITY = 1;
		adf_config.R6_GATED_BLEED = 1;
		adf_config.R6_NEGATIVE_BLEED = 1;
		adf_config.R6_RFOUTB_SELECT = 0;
		adf_config.R6_FEEDBACK_SELECT = 1;
		adf_config.R6_RF_DIV_SEL = 6;
		adf_config.R6_BLEED_CURRENT = 8;	// Default-Value for N = 23
		adf_config.R6_MUTE_TILL_LOCK = 0;
		adf_config.R6_AUXRF_ENABLE = 1;
		adf_config.R6_AUXRF_PWR = 3;
		adf_config.R6_RFOUT_ENABLE = 0;	// TODO: Set this to ON when RF-Path is populated
		adf_config.R6_RFOUT_PWR = 0;

		adf_config.R7_LE_SYNC_EDGE = 0;
		adf_config.R7_LE_SYNC = 1;
		adf_config.R7_LDC = 2; 	// 2048 Cycles until LOCK-DETECT = TRUE
		adf_config.R7_LOL_MODE = 0;	// Disable LOL-Mode since Differential Reference is used
		adf_config.R7_LDP = 3;
		adf_config.R7_LDM = 0;	// Set to 1 if using a Integer-N

		adf_config.REG8 = 0x15596568; //REGISTER 8 IS RESERVED AND NEEDS TO BE SET TO 0x102D0428 (ADF4355) or 0x15596568 (ADF4356)

		adf_config.R9_VCO_BAND_DIV = (int)((adf_config.PFD / 1600000) + 1);	//Calculate the divider for the VCO Band and add 1 to round up the result.
		adf_config.R9_TIMEOUT = 34;
		adf_config.R9_AUTOLVL_CAL_TIMEOUT = (int) (((0.00005 * adf_config.PFD)
				/ adf_config.R9_TIMEOUT) + 1);	// Calculate the Autolevel Calibration Timeout. The result makes sure that it will take at least 50us (0.00005 seconds) before Timeout occurs
		adf_config.R9_SYNTH_LOCK_TIMEOUT = (int) (((0.00002 * adf_config.PFD)
				/ adf_config.R9_TIMEOUT) + 1);	// Calculate the Synthesizer Lock Timeout. The result makes sure that it will take at least 20us (0.00002 seconds) before Timeout occurs.

		adf_config.R10_ADC_CLK_DIV =
				(int) ((((adf_config.PFD / 100000) - 2) / 4) + 1);	// Calculate the divider for the internal ADC-Clock in a way that the ADC is clocked at approximately 100kHz. +1 ensures the value is rounded UP when converted to INTEGER.
		adf_config.R10_ADC_CONV = 1;
		adf_config.R10_ADC_ENABLE = 1;

		adf_config.REG11 = 0x0061200B; // REGISTER 11 IS RESERVED AND NEEDS TO BE SET TO 0x0061300B in case of a ADF4355 and 0x61200B in case of a ADF4356

		adf_config.R12_PHASE_RESYNC_CLK_DIV = (int) (0.01 * adf_config.PFD);

		adf_config.R13_FRAC2_MSB = 0;
		adf_config.R13_MOD2_MSB = 0;

		ADF4355_ConvertFreq(150000000);
		ADF4355_WriteData();
		ADF4355_ConvertFreq(120000000);
		ADF4355_WriteData(FREQ_UPDATE);
		break;
	case 0x00: // Chip has already been set up. No need for all the config-data to be set up again.
		// TODO: Configure the operating-Mode
		break;
	default:

		break;
	}
	setup_complete = 0x01;
	return adf_config.ERROR;
}

uint8_t ADF4355_FPGA_Setup(unsigned char directMode, long updateFreq,
		int last_address, uint8_t stream_mode) {
	uint8_t adf4355_fpga = 0x00;// FPGA State-Value that is read fro the FPGA after performing the setup
	// Setup the FPGA to for either Direct or FPGA Control
	// TODO: DEVELOP A STATE-MACHINE INSIDE THE FPGA
	// TODO: SETUP THE STATE-MACHINE
	// TODO: Check if FPGA was configured correctly and report back
	return adf4355_fpga;
}

uint8_t ADF4355_FPGA_START(unsigned char start) {// Signals the FPGA to START or STOP sending the data-buffer to the ADF4355
	uint8_t fpga_state = 0x00;// FPGA-State Value - To be read from the FPGA at the end of the START / STOP-Command
	// TODO: Write the Start-Command to the FPGA
	// TODO: Report Back the FPGA-State
	return fpga_state;
}

void ADF4355_FPGA_WriteData(float frequency, float phase, int amplitude) {
	// TODO: Implement-Functions that copy an entire sample (up to all 13 Registers, depending on the "stream_mode"-value) to the FPGA, where it'll be stored in SDRAM to be sent to the ADF4355 upon issuing the START-Command.
}

void ADF4355_FPGA_DUMP(int address) {//Dumps the FPGA-Registers and Memory at the specified FPGA Memory-Address

}

void ADF4355_SetFrequency(double frequency) {
	ADF4355_ConvertFreq(frequency);
	ADF4355_WriteData();
	// ADF4355_WriteDataLite(FREQ_UPDATE);
}

void ADF4355_SetPhase(float phase) {

}

void ADF4355_GetConfig(int *responseBuffer) {

}

void ADF4355_ConvertFreq(double frequency) {
	// int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;
	adf_config.FREQUENCY = (double) frequency; // Desired Output-Frequency

	// Calculate the RF-Divider
	if (adf_config.FREQUENCY >= 6800000000) { // Driver only considers the RFOUTA of ADF4355 / ADF5355. Frequencies higher than 6.8 GHz are not routed through the RF-Deck of the ARSG but can be taken from "J_rfb". Frequency at J_rfb = RFOUT * 2.
		adf_config.FREQUENCY = adf_config.FREQUENCY / 2;
		adf_config.RFDIV = 0;
		adf_config.R6_RF_DIV_SEL = 0;
		adf_config.R6_AUXRF_ENABLE = 1;	// Frequency above 6.8 GHz Selected - Assume that a ADF5355 is in the system and make sure the Doubled Output is ON
		adf_config.R6_AUXRF_PWR = 3;
	}
	if (adf_config.FREQUENCY < 6800000000) {
		adf_config.RFDIV = 0;
		adf_config.R6_RF_DIV_SEL = 0;
	}
	if (adf_config.FREQUENCY < 3400000000) {
		adf_config.RFDIV = 2;
		adf_config.R6_RF_DIV_SEL = 1;
	}
	if (adf_config.FREQUENCY < 1700000000) {
		adf_config.RFDIV = 4;
		adf_config.R6_RF_DIV_SEL = 2;
	}
	if (adf_config.FREQUENCY < 850000000) {
		adf_config.RFDIV = 8;
		adf_config.R6_RF_DIV_SEL = 3;
	}
	if (adf_config.FREQUENCY < 425000000) {
		adf_config.RFDIV = 16;
		adf_config.R6_RF_DIV_SEL = 4;
	}
	if (adf_config.FREQUENCY < 212500000) {
		adf_config.RFDIV = 32;
		adf_config.R6_RF_DIV_SEL = 5;
	}
	if (adf_config.FREQUENCY < 106250000) {
		adf_config.RFDIV = 64;
		adf_config.R6_RF_DIV_SEL = 6;
	}

	/////////////////////////////////////////////////////////////////////////////
	//////////////////////// N and Frac1 and Frac2 calculations /////////////////
	//////////////////////// Done using double precision 64 bit /////////////////
	//////////////////////// Results agree exactly with AD demo /////////////////
	/////////////////////////////////////////////////////////////////////////////

//	double PFDFreq = adf_config.REFCLK
//			* ((1.0 + adf_config.R4_REFDOUBLE) / (adf_config.R4_RCOUNTER * (1.0 + adf_config.R4_REFDIV2))); //Phase detector frequency

	adf_config.N = ((adf_config.FREQUENCY) * adf_config.RFDIV) / adf_config.PFD; // Calculate N

	adf_config.R0_N = (int)adf_config.N;  // Turn N into integer
	adf_config.R0_N = adf_config.R0_N & 0x0000FFFF;	// Clear the first 16 bit of the N-Value to make sure

	adf_config.FRAC1 = adf_config.N - adf_config.R0_N;

	adf_config.FRAC1 = adf_config.FRAC1 * adf_config.MOD1; // Calculate Frac1: Multiply the remainder of N - Integer-Part of N with MOD1

	adf_config.R1_FRAC1 = (int)adf_config.FRAC1;  // turn Frac1 into an integer
	adf_config.R1_FRAC1 = adf_config.R1_FRAC1 & 0x00FFFFFF;	// Clear the first 8 bit of FRAC1

	adf_config.FRAC2 = adf_config.FRAC1 - adf_config.R1_FRAC1;	// Subtract the Integer-Part of FRAC1 from FRAC1 to get FRAC2.

	adf_config.FRAC2 = adf_config.FRAC2 * adf_config.MOD2;	// Multiply the remainder with MOD2 to get the Integer-Part of FRAC2.

	adf_config.R2_FRAC2_LSB = (int)adf_config.FRAC2;	// Turn FRAC2 into an Integer and store the value in the Mirrored register.
	adf_config.R13_FRAC2_MSB = (int)adf_config.FRAC2;	// Turn FRAC2 into an Integer and store the value in the Mirrored register.

	adf_config.R2_FRAC2_LSB = adf_config.R2_FRAC2_LSB & 0x00003FFF; // Clear the first 18 bits of the LSB of FRAC2.
	adf_config.R13_FRAC2_MSB = adf_config.R13_FRAC2_MSB >> 18; // Clear the lowest 18 bits of the MSB of FRAC2 and shift the value RIGHT by 18 bits to align it with the register-value.

	if (adf_config.R1_FRAC1 == 0) {	// Check if the Fractional Values are 0 and if so, configure Negative Bleed and Lock Detect Mode to Integer N-Mode.
		if (adf_config.R2_FRAC2_LSB == 0) {
			adf_config.R6_NEGATIVE_BLEED = 0;
			adf_config.R7_LDM = 1;
		}
	} else {
		adf_config.R6_NEGATIVE_BLEED = 1;
		adf_config.R7_LDM = 0;
	}

	////////////////// Set 32 bit register values R0 to R12 ///////////////////////////

	adf_config.REG0 = (int) (0 + adf_config.R0_N * pow(2, 4)
			+ adf_config.R0_PRESCAL * pow(2, 20)
			+ adf_config.R0_AUTOCAL * pow(2, 21));	// Calculate R0

	adf_config.REG1 = (int) (1 + adf_config.R1_FRAC1 * pow(2, 4));// Calculate R1

	adf_config.REG2 = (int) (2 + adf_config.R2_MOD2_LSB * pow(2, 4)
			+ adf_config.R2_FRAC2_LSB * pow(2, 18)); //

	adf_config.REG3 = (int) (3 + adf_config.R3_PHASE_VALUE * pow(2, 4)
			+ adf_config.R3_PHASE_ADJUST * pow(2, 28)
			+ adf_config.R3_PHASE_RESYNC * pow(2, 29)
			+ adf_config.R3_SD_LOAD_RESET * pow(2, 30)); // TODO: CHECK Phase-Register Calculation

	adf_config.REG4 = (int) (4 + adf_config.R4_COUNTRES * pow(2, 4)
			+ adf_config.R4_CP3STATE * pow(2, 5)
			+ adf_config.R4_PWRDN * pow(2, 6) + adf_config.R4_PDPOLA * pow(2, 7)
			+ adf_config.R4_MUXLOG * pow(2, 8)
			+ adf_config.R4_REFMODE * pow(2, 9)
			+ adf_config.R4_CHGPUMP * pow(2, 10)
			+ adf_config.R4_DOUBLEBUF * pow(2, 14)
			+ adf_config.R4_RCOUNTER * pow(2, 15)
			+ adf_config.R4_REFDIV2 * pow(2, 25)
			+ adf_config.R4_REFDOUBLE * pow(2, 26)
			+ adf_config.R4_MUXOUT * pow(2, 27));

	adf_config.REG6 = (int) (6 + adf_config.R6_RFOUT_PWR * pow(2, 4)
			+ adf_config.R6_RFOUT_ENABLE * pow(2, 6)
			+ adf_config.R6_AUXRF_PWR * pow(2, 7)
			+ adf_config.R6_AUXRF_ENABLE * pow(2, 9) + 0 * pow(2, 10)
			+ adf_config.R6_MUTE_TILL_LOCK * pow(2, 11) + 0 * pow(2, 12)
			+ adf_config.R6_BLEED_CURRENT * pow(2, 13)
			+ adf_config.R6_RF_DIV_SEL * pow(2, 21)
			+ adf_config.R6_FEEDBACK_SELECT * pow(2, 24) + adf_config.R6_RFOUTB_SELECT * pow(2,25) + 1 * pow(2, 26) +  1 * pow(2, 28)
			+ adf_config.R6_NEGATIVE_BLEED * pow(2, 29)
			+ adf_config.R6_GATED_BLEED * pow(2, 30) + adf_config.R6_BLEED_POLARITY * pow(2,31));

	adf_config.REG7 = (int) (7 + adf_config.R7_LDM * pow(2, 4)
			+ adf_config.R7_LDP * pow(2, 5) + adf_config.R7_LOL_MODE * pow(2, 7)
			+ adf_config.R7_LDC * pow(2, 8) + adf_config.R7_LE_SYNC * pow(2, 25) + 1 * pow(2,26)
			+ adf_config.R7_LE_SYNC_EDGE * pow(2, 27));

	//REGISTER 8 IS RESERVED AND NEEDS TO BE SET TO 0x102D0428 (ADF4355) or 0x15596568 (ADF4356)

	adf_config.REG9 = (int) (9 + adf_config.R9_SYNTH_LOCK_TIMEOUT * pow(2, 4)
			+ adf_config.R9_AUTOLVL_CAL_TIMEOUT * pow(2, 9)
			+ adf_config.R9_TIMEOUT * pow(2, 14)
			+ adf_config.R9_VCO_BAND_DIV * pow(2, 24));

	adf_config.REG10 = (int) (10 + adf_config.R10_ADC_ENABLE * pow(2, 4)
			+ adf_config.R10_ADC_CONV * pow(2, 5)
			+ adf_config.R10_ADC_CLK_DIV * pow(2, 6) + 1 * pow(2, 22)
			+ 1 * pow(2, 23));

	// REGISTER 11 IS RESERVED AND NEEDS TO BE SET TO 0x0061300B in case of a ADF4355 and 0x61200B in case of a ADF4356

	adf_config.REG12 = (int) (12 + 0x5F * pow(2, 4)
			+ adf_config.R12_PHASE_RESYNC_CLK_DIV * pow(2, 12));

	adf_config.REG13 = (int) (13 + adf_config.R13_MOD2_MSB * pow(2,4) + adf_config.R13_FRAC2_MSB * pow(2,18));

#ifdef ADF_DEBUG
	// Send Precalculated Register-Values from the Evaluation Board-Software to the Chip
	adf_config.REG0 = 0x200A50;
	adf_config.REG1 = 0x1;
	adf_config.REG2 = 0x1902;
	adf_config.REG3 = 0x3;
	adf_config.REG4 = 0x3400C984;
	adf_config.REG5 = 0x800025;
	adf_config.REG6 = 0x75C20386;
	adf_config.REG7 = 0x60000E7;
	adf_config.REG8 = 0x15596568;
	adf_config.REG9 = 0x1910FCC9;
	adf_config.REG10 = 0xC0193A;
	adf_config.REG11 = 0x61200B;
	adf_config.REG12 = 0x7FFFF5FC;
	adf_config.REG13 = 0xD;
#endif
}

void ADF4355_WriteData() {
	HAL_GPIO_WritePin(ADF4355_CS_GPIO_Port, ADF4355_CS_Pin, GPIO_PIN_RESET); // SELECT THE CHIP / PRIME THE INPUT-REGISTER
	ADF4355_WriteRegister(adf_config.REG13);
	ADF4355_WriteRegister(adf_config.REG12);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG11);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG10);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG9);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG8);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG7);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG6);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG5);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG4);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG3);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG2);
	//HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG1);
	// TODO: Calculate the Wait-Time: 16/ADF4355_ADC-CLK
	HAL_Delay(4);
	ADF4355_WriteRegister(adf_config.REG0);
	HAL_GPIO_WritePin(ADF4355_CS_GPIO_Port, ADF4355_CS_Pin, GPIO_PIN_SET); // SELECT THE CHIP / PRIME THE INPUT-REGISTER
}

void ADF4355_WriteDataLite(uint8_t mode) {	// Updates only the Registers that
	switch (mode) {
	case FREQ_UPDATE:// Update the Frequency - See Datasheet for the Register-Sequence
		ADF4355_WriteRegister(adf_config.REG13);
		ADF4355_WriteRegister(adf_config.REG10);
		//HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG6);
		//HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG2);
		//HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG1);
		//HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG0);
		break;
	case PHASE_UPDATE:// Update the Phase-Angle - Only Register 3 needs to be sent
		ADF4355_WriteRegister(adf_config.REG3);
		break;
	case AMPLITUDE_UPDATE:// Update the Amplitude - Only Register 6 Needs to be sent
		ADF4355_WriteRegister(adf_config.REG6);
		break;
	}
}

void ADF4355_WriteRegister(int adfreg) {
	uint8_t buf[4];
	for (int i = 0; i < 4; i++) {
		buf[3 - i] = (uint8_t) (adfreg >> (i * 8));	// Split the value into 4 Bytes
	}
	HAL_GPIO_WritePin(ADF4355_LE_GPIO_Port, ADF4355_LE_Pin, GPIO_PIN_RESET);// SELECT THE CHIP / PRIME THE INPUT-REGISTER
	HAL_SPI_Transmit(&hspi2, buf, 4, 800);// Send the Data to the Chip via SPI2
	HAL_GPIO_WritePin(ADF4355_LE_GPIO_Port, ADF4355_LE_Pin, GPIO_PIN_SET);// DESELECT THE CHIP AND LOAD THE INPUT-REGISTER INTO THE INTERNAL REGISTER SPECIFIED BY THE 4 LSB
}
