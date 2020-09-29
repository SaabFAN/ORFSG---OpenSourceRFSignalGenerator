/*
 * ADF4356Driver.h
 *
 *  Created on: Sep 24, 2020
 *      Author: voyag
 */

#include "Main.h"
#include "Math.h"

#ifndef SRC_ADF4356DRIVER_H_
#define SRC_ADF4356DRIVER_H_

#define ADF_DIRECT_CTRL 0xDC	// Sets the DIRECT-Control: This will send data through the FPGA directly to the ADF4355
#define ADF_FPGA_CTRL 0xFC		// Sets the FPGA-Control: This will send data to a Buffer inside the FPGA and set up the FPGA

#define FREQ_UPDATE 0xF0
#define PHASE_UPDATE 0xE0
#define AMPLITUDE_UPDATE 0xA0

#define ADF4355_CS_Pin GPIO_PIN_13
#define ADF4355_CS_GPIO_Port GPIOE
#define ADF4355_LE_Pin GPIO_PIN_14
#define ADF4355_LE_GPIO_Port GPIOE
#define ADF4355_MUX_Pin GPIO_PIN_15
#define ADF4355_MUX_GPIO_Port GPIOE

extern SPI_HandleTypeDef hspi2;

typedef struct ADF4355_Configuration{	// Configuration-Data and Mirror of ADF4355-Registers in CPU-RAM. Last 4 bit of each 32bit value indicate the Register-Address and are constant
	double REFCLK;
	double PFD;
	unsigned char RFDIV;
	double N;
	double FRAC1;
	double FRAC2;
	unsigned int MOD1;
	unsigned int MOD2;
	uint8_t ERROR;
	uint8_t MODE;
	uint8_t FPGA_MODE;
	int AMPLITUDE;
	double FREQUENCY;
	double PHASE;
	// REGISTER 0
	int R0_PRESCAL;			// Dual Modulus Prescaler
	int R0_AUTOCAL;			// Autocal-Select (Should only be off for fixed frequency applications)
	int R0_N;				// INTEGER-Value of the Fractional N-PLL
	int R0_Nmin; 			// Minimal Value of N. Determined by Prescaler-Value. Nmin = 23 if Prescaler = 4/5-Mode

	// REGISTER 1
	int R1_FRAC1;			// 24bit Main Fractional Value

	// REGISTER 2
	int R2_FRAC2_LSB;			// 14bit Auxilliary Fractional Value
	int R2_MOD2_LSB;			// Auxiliary Modulus Value

	// REGISTER 3
	int R3_SD_LOAD_RESET;	// Sigma Delta Modulator-RESET ENABLE <- Disable for Operating Modes that involve Phase Modulation
	int R3_PHASE_RESYNC;	// Phase Resync ENABLE - ONLY WORKS WHEN FRAC2 = 0 and needs PLL-Feedback to come from Divider if Frequency is less than 3.4 GHz
	int R3_PHASE_ADJUST;	// Phase Adjust ENABLE - A 0 here turns off Phase Adjust Features
	int R3_PHASE_VALUE;		// Phase Value in Steps of 0.000021457672119140625 degrees

	// REGISTER 4
	int R4_MUXOUT;			// Muxout-Selector
	int R4_REFDOUBLE;		// Reference-Doubler
	int R4_REFDIV2;			// Reference-Prescaler (Divide by 2)
	int R4_RCOUNTER;		// 10bit Reference-Counter
	int R4_DOUBLEBUF;		// Enable or Disable Double Buffering
	int R4_CHGPUMP;			// Chargepump-Current
	int R4_REFMODE;			// Differential or Single Ended Reference
	int R4_MUXLOG;
	int R4_PDPOLA;			// Phase Detector Polarity
	int R4_PWRDN;
	int R4_CP3STATE;		// Chargepump Threestate
	int R4_COUNTRES;		// Counter Reset

	// REGISTER 5
	// REGISTER 5 IS RESERVED AND NEEDS TO BE SET TO 0x00800025

	// REGISTER 6
	int R6_BLEED_POLARITY;	// Set the Bleed Polarity
	int R6_GATED_BLEED;		// Gated Bleed: Bleed Feature only enabled AFTER Lock
	int R6_NEGATIVE_BLEED;	// Negative Bleed Enable
	int R6_RFOUTB_SELECT;	// What will be sent out at RFOUTB
	int R6_FEEDBACK_SELECT;	// Feedback-Selector: 1 = VCO, 0 = Divided
	int R6_RF_DIV_SEL;		// RF-Divider: 0 - 64
	int R6_BLEED_CURRENT;	// Bleed-Current Select: 3.75uA Steps from 3.75uA up to 956.25uA. Best Value = 4 / N < Ibleed / IchargePump < 10 / N with N being the value of the Feedback-Counter of the PFD
	int R6_MUTE_TILL_LOCK;	// Disable Output until LOCK is Achieved
	int R6_AUXRF_ENABLE;	// Enable/Disable RFOUT_B
	int R6_AUXRF_PWR;		// Select the Output-Power of RFOUT_B: -4dBm (0), -1dBm (1), +1dBm (2), +5dBm (3)
	int R6_RFOUT_ENABLE;	// Enable/Disable RFOUT_A
	int R6_RFOUT_PWR;		// Select the Output-Power of RFOUT_B: -4dBm, -1dBm, +1dBm, +5dBm

	// REGISTER 7
	int R7_LE_SYNC_EDGE;
	int R7_LE_SYNC;			// Load Enable Sync: If Enabled, the Latch of the shift register is Synchronized with the Reference-Clock
	int R7_LDC;				// Lock Detect-Count: How many Cycles until Lock-Detect Circuit reports Lock
	int R7_LOL_MODE;		// Loss of Lock-Mode: SET TO 0! Datasheet states LOL-Mode does not work with differential RefCLK
	int R7_LDP;				// Lock Detect Precision: Should be set to 12ns = 11
	int R7_LDM;				// Lock Detect-Mode: Set to 0 if Fractional N-Mode is used

	// REGISTER 8:
	// REGISTER 8 IS RESERVED AND NEEDS TO BE SET TO 0x102D0428

	// REGISTER 9:
	unsigned char R9_VCO_BAND_DIV;	// 8bit Divider-Value for VCO Band Division-Clock. Determine by: (PFD / (Band-Division * 16)) < 150kHz
	int R9_TIMEOUT;			// Timeout for VCO Band-Division Select - Used as variable in the next Timeout-Settings
	int R9_AUTOLVL_CAL_TIMEOUT;	// Timeout for the VCO Automatic Level Calibration. Configure thus: ((Timeout * ALC WAIT) / PFD Frequency) > 50us
	int R9_SYNTH_LOCK_TIMEOUT;	// Timeout for the Synthersizer Lock Timeout: Configure thus: ((Timeout * Synth Lock Timeout) / PFD Frequency) > 20us

	// REGISTER 10
	int R10_ADC_CLK_DIV;	// Divider for the internal ADC. ADC-CLK = R counter / ADC_CLK_DIV. Configure thus: ADC_CLK_DIV = (((PFD Frequency / 100000) - 2) / 4) and Round up to nearest Integer
	int R10_ADC_CONV;		// ADC Conversion Enable: A 1 here starts an ADC-Conversion. Always write a 1 here
	int R10_ADC_ENABLE;		// Enable or Disable the ADC. Always Enable the ADC (Write 1)!

	// REGISTER 11
	// REGISTER 11 IS RESERVED AND NEEDS TO BE SET TO 0x0061300B

	// REGISTER 12
	int R12_PHASE_RESYNC_CLK_DIV;	// Timeout-Counter for Phase Resync Activation. Set to: Phase Resync Cycles / PFD Frequency > PLL Lock-Time (See ADISIMPll for Lock-Times + Experimentation)

	// REGISTER 13
	uint16_t R13_FRAC2_MSB;
	uint16_t R13_MOD2_MSB;

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
	int REG13;
}ADF_CONF;

class ADF4356_Driver {
public:
	ADF4356_Driver();
	virtual ~ADF4356_Driver();
	uint8_t Setup(uint8_t startup, long refCLK, uint8_t mode, int MOD2);
	uint8_t FPGA_Setup(unsigned char directMode, long updateFreq, int last_address, uint8_t stream_mode);	// Sets up the FPGA. directMode decides if the FPGA is just working as a Multiplexer or is acting as a buffer for the data. updateFreq decides the frequency with which the Samples are being sent to the ADF4355. last_address specifies the last Memory-Address (Up to 1 Mega-Samples if all 13 Registers are being saved for each Sample). streamMode decides if the FPGA is streaming the data just once or continuously.
	uint8_t FPGA_START(unsigned char start);	// Signals the FPGA to START or STOP sending the data-buffer to the ADF4355
	void FPGA_WriteData(float frequency, float phase, int amplitude); // Calculate all register-values for the ADF4355 and store one Sample inside the FPGA-Memory.
	void FPGA_DUMP(int address);	// FPGA-Dump of one Sample plus the FPGA-Status Registers into an internal data-buffer of the function for debugging with Breakpoints.
	void SetFrequency(double frequency);	// Sets the output-frequency.
	void SetPhase(float phase);
	void GetConfig(int *responseBuffer);
private:
	void ConvertFreq(double frequency);
	void WriteData();
	void WriteRegister(int adfreg);
	void WriteDataLite(uint8_t mode);
	uint32_t ReadRegister(uint8_t reg);	// Read a Register and Return the 32 Bit long Value
	unsigned char setup_complete;
	ADF_CONF adf_config;
};

#endif /* SRC_ADF4356DRIVER_H_ */
