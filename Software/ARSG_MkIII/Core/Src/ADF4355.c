/*
 * ADF4355.c
 *
 *  Created on: Aug 23, 2020
 *      Author: voyag
 */

#include "ADF4355.h"

ADF_CONF adf_config;

extern SPI_HandleTypeDef hspi2;

void ADF4355_ConvertFreq();
void ADF4355_WriteData();
void ADF4355_WriteRegister(int adfreg);
void ADF4355_WriteDataLite(uint8_t mode);

uint8_t ADF4355_Setup(uint8_t startup, long refCLK, uint8_t mode) {	// Setup the ADF4355 with standard values (Output: ON, Freqency: 100 MHz (for debug-purposes)
	switch (startup) {
	case 0x01: // This is the first time we're running the Setup - ADF4355-Configuration needs to be set up.
		if (refCLK < 10000000) {
// RefCLK Lower than 10 MHz and out of Range!
			adf_config.ERROR = 0xF0; // Save the REFERENCE-ERROR and
		}
		break;
	case 0x00: // Chip has already been set up. No need for all the config-data to be set up again.
		// TODO: Configure the operating-Mode
		break;
	}
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

void ADF4355_FPGA_DUMP(int address) {

}

void ADF4355_SetFrequency(float frequency) {
	ADF4355_ConvertFreq();
	ADF4355_WriteData();
}

void ADF4355_SetPhase(float phase) {

}

void ADF4355_GetConfig(int *responseBuffer) {

}

void ADF4355_ConvertFreq() {
	// PLL-Reg-R0         =  32bit
	//   Registerselect        4bit
	//  int N_Int = 92;       // 16bit
	int Prescal = 0;         // 1bit geht nicht ??? it does not work
	int Autocal = 1;          //1 bit
	//  reserved           // 10bit

	// PLL-Reg-R1         =  32bit
	//   Registerselect        4bit
	//   int FRAC1 = 10;       // 24 bit
	//   reserved              // 4bit

	// PLL-Reg-R2         =  32bit
	//    Registerselect        4bit
	int M_Mod2 = 16383;            // 14 bit
	//    int Frac2 = 0;            // 14 bit

	// PLL-Reg-R3         =  32bit - FIXED !
	// Registerselect        4bit
	//Fixed value to be written = 0x3 =3

	// PLL-Reg-R4         =  32bit
	// Registerselect        4bit
	int U1_CountRes = 0;     // 1bit
	int U2_Cp3state = 0;     // 1bit
	int U3_PwrDown = 0;      // 1bit
	int U4_PDpola = 1;       // 1bit
	int U5_MuxLog = 1;          // 1bit
	int U6_RefMode = 1;          // 1bit
	//  int U5_LPD = 0;          // 1bit
	//  int U6_LPF = 1;          // 1bit 1=Integer, 0=Frac not spported yet
	int CP_ChgPump = 9;      // 4bit
	int D1_DoublBuf = 0;     // 1bit
	int R_Counter = 1;       // 10bit
	int RD1_Rdiv2 = 0;       // 1bit
	int RD2refdoubl = 0;     // 1bit
	int M_Muxout = 6;        // 3bit
	// reserved              // 2bit

	// PLL-Reg-R5         =  32bit
	// Registerselect        // 4bit
	// Phase Select: Not of partcular interst in Amatuer radio applications. Leave at a string of zeros.

	// PLL-Reg-R6         =  32bit
	// Registerselect        // 4bit
	//Variable value to be written!!!
	int D_out_PWR = adf_config.AMPLITUDE; // 2bit  OutPwr 0-3 3= +5dBm   Power out 1
	int D_RF_ena = 1; // 1bit  OutPwr 1=on           0 = off  Outport Null freischalten
	int Reserved = 0;                 // 3bit
	int D_RFoutB = 1;         // 1bit  aux OutSel
	int D_MTLD = 0;              // 1bit
	int CPBleed = 126;   // 8bit
	int D_RfDivSel = 3;      // 3bit 3=70cm 4=2m    lokale Variable
	int D_FeedBack = 1;       // 1bit
	// reserved              // 7bit

	// PLL-Reg-R7         =  32bit
	// Registerselect        // 4bit
	//Fixed value to be written = 0x120000E7 = 301990119 (dec)

	// PLL-Reg-R8         =  32bit
	// Registerselect        // 4bit
	//Fixed value to be written = 0x102D0428 = 271385640 (dec)

	// PLL-Reg-R9         =  32bit
	// Registerselect        // 4bit
	//Fixed value to be written = 0x5047CC9 = 84180169 (dec)

	// PLL-Reg-R10         =  32bit
	// Registerselect        // 4bit
	//Fixed value to be written = 0xC0067A = 12584570 9dec)

	// PLL-Reg-R11         =  32bit
	// Registerselect        // 4bit
	//Fixed value to be written = 0x61300B = 6369291 (dec)

	// PLL-Reg-R12         =  32bit
	// Registerselect        // 4bit
	//Fixed value to be written = 0x1041C = 66588 (dec)

	// Referenz Freg Calc

	// int F4_BandSel = 10.0 * B_BandSelClk / PFDFreq;
	double RFout = adf_config.FREQUENCY; // VCO-Frequenz  144200000  Freq ist global, RFout ist lokal

	// calc bandselect und RF-div
	float outdiv = 1;
	if (RFout >= 680000000) {
		outdiv = 0.5;
		D_RfDivSel = 0;
		D_RFoutB = 0;
		D_RF_ena = 0;
	}
	if (RFout < 680000000) {
		outdiv = 1;
		D_RfDivSel = 0;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}
	if (RFout < 340000000) {
		outdiv = 2;
		D_RfDivSel = 1;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}
	if (RFout < 170000000) {
		outdiv = 4;
		D_RfDivSel = 2;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}
	if (RFout < 85000000) {
		outdiv = 8;
		D_RfDivSel = 3;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}
	if (RFout < 42500000) {
		outdiv = 16;
		D_RfDivSel = 4;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}
	if (RFout < 21250000) {
		outdiv = 32;
		D_RfDivSel = 5;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}
	if (RFout < 10625000) {
		outdiv = 64;
		D_RfDivSel = 6;
		D_RFoutB = 1;
		D_RF_ena = 1;
	}

	/////////////////////////////////////////////////////////////////////////////
	//////////////////////// N and Frac1 and Frac2 calculations /////////////////
	//////////////////////// Done using double precision 64 bit /////////////////
	//////////////////////// Results agree exactly with AD demo /////////////////
	/////////////////////////////////////////////////////////////////////////////

	double PFDFreq = adf_config.REFCLK
			* ((1.0 + RD2refdoubl) / (R_Counter * (1.0 + RD1_Rdiv2))); //Phase detector frequency

	double N = ((RFout) * outdiv) / PFDFreq;   // Calculate N

	int N_Int = N;   // N= 50 for 5 GHz   // Turn N into integer

	double F_Frac1x = (N - N_Int) * pow(2, 24); // Calculate Frac1 (N remainder * 2^24)

	int F_FracN = F_Frac1x;  // turn Frac1 into an integer

	double F_Frac2x = ((F_Frac1x - F_FracN)) * pow(2, 14); // Claculate Frac2 (F_FracN remainder * 2^14)

	int F_Frac1 = F_Frac1x;  // turn Frac1 into integer
	int F_Frac2 = F_Frac2x;  // turn Frac2 into integer

	////////////////// Set 32 bit register values R0 to R12 ///////////////////////////

	adf_config.REG0 = (int) (0 + N_Int * pow(2, 4) + Prescal * pow(2, 20)
			+ Autocal * pow(2, 21)); // R0 für Startfrequenz ok

	adf_config.REG1 = (int) (1 + F_Frac1 * pow(2, 4));

	adf_config.REG2 = (int) (2 + M_Mod2 * pow(2, 4) + F_Frac2 * pow(2, 18)); //

	adf_config.REG3 = (int) (0x3); //Fixed value (Phase control not needed)

	adf_config.REG4 = (int) (4 + U1_CountRes * pow(2, 4)
			+ U2_Cp3state * pow(2, 5) + U3_PwrDown * pow(2, 6)
			+ U4_PDpola * pow(2, 7) + U5_MuxLog * pow(2, 8)
			+ U6_RefMode * pow(2, 9) + CP_ChgPump * pow(2, 10)
			+ D1_DoublBuf * pow(2, 14) + R_Counter * pow(2, 15)
			+ RD1_Rdiv2 * pow(2, 25) + RD2refdoubl * pow(2, 26)
			+ M_Muxout * pow(2, 27));

	adf_config.REG5 = 0x800025; // Fixed (Reserved)

	adf_config.REG6 = (int) (6 + D_out_PWR * pow(2, 4) + D_RF_ena * pow(2, 6)
			+ Reserved * pow(2, 7) + D_RFoutB * pow(2, 10) + D_MTLD * pow(2, 11)
			+ Reserved * pow(2, 12) + CPBleed * pow(2, 13)
			+ D_RfDivSel * pow(2, 21) + D_FeedBack * pow(2, 24)
			+ 10 * pow(2, 25));

	adf_config.REG7 = 0x120000E7;
	adf_config.REG8 = 0x102D0428;
	adf_config.REG9 = 0x2A29FCC9;
	adf_config.REG10 = 0xC0043A;
	adf_config.REG11 = 0x61300B;
	adf_config.REG12 = 0x1041C;
}

void ADF4355_WriteData() {
	ADF4355_WriteRegister(adf_config.REG12);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG11);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG10);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG9);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG8);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG7);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG6);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG5);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG4);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG3);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG2);
	HAL_Delay(1);
	ADF4355_WriteRegister(adf_config.REG1);
	// TODO: Calculate the Wait-Time: 16/ADF4355_ADC-CLK
	HAL_Delay(50);
	ADF4355_WriteRegister(adf_config.REG0);
	HAL_Delay(1);
}

void ADF4355_WriteDataLite(uint8_t mode) {	// Updates only the Registers that
	switch (mode) {
	case FREQ_UPDATE:// Update the Frequency - See Datasheet for the Register-Sequence
		ADF4355_WriteRegister(adf_config.REG10);
		HAL_Delay(1);
		// TODO: set BIT4 of REGISTER 4 TRUE
		ADF4355_WriteRegister(adf_config.REG4);
		HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG2);
		HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG1);
		HAL_Delay(1);
		ADF4355_WriteRegister(adf_config.REG0);
		HAL_Delay(1);
		// TODO: Set BIT4 of REGISTER 4 to FALSE
		ADF4355_WriteRegister(adf_config.REG0);
		HAL_Delay(50);	// TODO: Calculate the Wait-Time: 16/ADF4355_ADC-CLK
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
	HAL_GPIO_WritePin(ADF4355_CS_GPIO_Port, ADF4355_LE_Pin, GPIO_PIN_RESET);// SELECT THE CHIP / PRIME THE INPUT-REGISTER
	uint8_t buf[4];
	for (int i = 0; i < 4; i++) {
		buf[i] = (uint8_t) (adfreg >> (i * 8));	// Split the value into 4 Bytes
	}
	HAL_SPI_Transmit(&hspi2, buf, 4, 800);// Send the Data to the Chip via SPI2
	HAL_GPIO_WritePin(ADF4355_CS_GPIO_Port, ADF4355_LE_Pin, GPIO_PIN_SET);// DESELECT THE CHIP AND LOAD THE INPUT-REGISTER INTO THE INTERNAL REGISTER SPECIFIED BY THE 4 LSB
}
