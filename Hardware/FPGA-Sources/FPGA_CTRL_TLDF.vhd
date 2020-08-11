-- Copyright (C) 2020  Intel Corporation. All rights reserved.
-- Your use of Intel Corporation's design tools, logic functions 
-- and other software and tools, and any partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Intel Program License 
-- Subscription Agreement, the Intel Quartus Prime License Agreement,
-- the Intel FPGA IP License Agreement, or other applicable license
-- agreement, including, without limitation, that your use is for
-- the sole purpose of programming logic devices manufactured by
-- Intel and sold by Intel or its authorized distributors.  Please
-- refer to the applicable agreement for further details, at
-- https://fpgasoftware.intel.com/eula.

library ieee;
use ieee.std_logic_1164.all;
library altera;
use altera.altera_syn_attributes.all;

entity FPGA_CTRL_TLDF is
	port
	(
-- {ALTERA_IO_BEGIN} DO NOT REMOVE THIS LINE!

		inclk0n : in std_logic;
		inclk0 : in std_logic;
		AD9957_RESET : in std_logic;
		sdram_cas_n : in std_logic;
		sdram_cke : in std_logic;
		SDRAM_CLK : in std_logic;
		sdram_cs_n : in std_logic;
		sdram_dqmh : in std_logic;
		sdram_dqml : in std_logic;
		sdram_ras_n : in std_logic;
		sdram_we_n : in std_logic;
		MAX_CS : in std_logic;
		AD5355_CS : in std_logic;
		AD5355_LE : in std_logic;
		AD5355_MOSI : in std_logic;
		AD5355_SCK : in std_logic;
		AD5355_MUX_CPU : in std_logic;
		MAX_MISO : in std_logic;
		MAX_MOSI : in std_logic;
		MAX_SCK : in std_logic;
		ADF5355_CS_CPU : in std_logic;
		ADF5355_LE_CPU : in std_logic;
		ADF5355_MUX : in std_logic;
		AD9957_RESET_CPU : in std_logic;
		AD9957_SCK : in std_logic;
		AD9957_MISO : in std_logic;
		AD9957_MOSI : in std_logic;
		AD9957_P0 : in std_logic;
		AD9957_P1 : in std_logic;
		AD9957_P2 : in std_logic;
		AD9957_CS : in std_logic;
		AD9957_CS_CPU : in std_logic;
		AD9957_IOUP : in std_logic;
		AD9957_IOUP_CPU : in std_logic;
		AD9957_CCI_OSK : in std_logic;
		AD9957_CCI_RT : in std_logic;
		AD9957_CCI_OVFL : in std_logic;
		AD9957_PDCLK : in std_logic;
		AD9957_SYNC_CLK : in std_logic;
		FPGA_IRQ : in std_logic;
		FPGA_LED1 : in std_logic;
		AD9957_IO_RST : in std_logic;
		AD9957_IO_RST_CPU : in std_logic;
		AD9957_TXENABLE : in std_logic;
		sdram_a : in std_logic_vector(0 to 12);
		sdram_ba : in std_logic_vector(0 to 1);
		sdram_dq : in std_logic_vector(0 to 15);
		AD9957_DATA : in std_logic_vector(0 to 17)
-- {ALTERA_IO_END} DO NOT REMOVE THIS LINE!

	);

-- {ALTERA_ATTRIBUTE_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_ATTRIBUTE_END} DO NOT REMOVE THIS LINE!
end FPGA_CTRL_TLDF;

architecture ppl_type of FPGA_CTRL_TLDF is

-- {ALTERA_COMPONENTS_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_COMPONENTS_END} DO NOT REMOVE THIS LINE!
begin
-- {ALTERA_INSTANTIATION_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_INSTANTIATION_END} DO NOT REMOVE THIS LINE!

end;


