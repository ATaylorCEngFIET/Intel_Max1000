## Generated SDC file "io_expansion.sdc"

## Copyright (C) 2018  Intel Corporation. All rights reserved.
## Your use of Intel Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Intel Program License 
## Subscription Agreement, the Intel Quartus Prime License Agreement,
## the Intel FPGA IP License Agreement, or other applicable license
## agreement, including, without limitation, that your use is for
## the sole purpose of programming logic devices manufactured by
## Intel and sold by Intel or its authorized distributors.  Please
## refer to the applicable agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 18.1.0 Build 625 09/12/2018 SJ Lite Edition"

## DATE    "Tue Jun 22 18:46:49 2021"

##
## DEVICE  "10M08SAU169C8G"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name clk [get_ports {clk}] -period 10.000 -waveform { 0.000 5.000 } 
create_clock -name sclk [get_ports {sclk}] -period 10.000 -waveform { 0.000 5.000 } 

#**************************************************************
# Create Generated Clock
#**************************************************************



#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************



#**************************************************************
# Set False Path
#**************************************************************

set_false_path -from [get_pins -nocase -compatibility_mode {*SPIPhy_altera_avalon_st_idle_inserter|received_esc*|*}] -to [get_pins -nocase -compatibility_mode {*|rdshiftreg*|*}]
set_false_path -from [get_pins -nocase -compatibility_mode {*|stsourcedata*|*}] -to [get_registers *]
set_false_path -to [get_keepers {*altera_std_synchronizer:*|din_s1}]
set_false_path -to [get_pins -nocase -compatibility_mode {*|alt_rst_sync_uq1|altera_reset_synchronizer_int_chain*|clrn}]
set_false_path -from [get_pins -nocase -compatibility_mode {*SPIPhy_MOSIctl|stsourcedata*|*}] -to [get_registers *]
set_false_path -from [get_pins -nocase -compatibility_mode {*SPIPhy_altera_avalon_st_idle_inserter|received_esc*|*}] -to [get_pins -nocase -compatibility_mode {*SPIPhy_MISOctl|rdshiftreg*|*}]


#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

