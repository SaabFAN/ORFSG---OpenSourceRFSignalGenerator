# ORFSG---OpenSourceRFSignalGenerator
RF Signal Generator with filtered outputs, modulation features and adjustable Amplitude with at least 80dB of dynamic range. 

This git-repo contains the basic information about the design for users to develop their own application, as well as some simple firmware to get the mainboard up and running for testing. 

The FPGA-Directory contains information specific to the FPGA on the mainboard, as well as the Source-Code (Quartus Lite-Project) for the FPGA. You need Quartus Lite and an Altera USB Blaster to program the FPGA!

The "Software"-Directory contains the Source-Code for the Firmware. 
The Firmware-Source is a STM32CUBEIDE-Project that can be imported into the STM32CUBEIDE to be compiled and programmed into the main Processor. 