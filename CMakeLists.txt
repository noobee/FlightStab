#====================================================================#
#  FlightStab cmake                                                  #
#====================================================================#

# basic build instructions for windows
# cd ./Builds
# optional: rd /s .  (to wipe out all intermediate files)
# cmake [-Dtoday=yyyymmdd] -G"MSYS Makefiles" ..  (cmake will use the 
#   system date if -Dtoday is not specified)
# make

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)
cmake_minimum_required(VERSION 2.8)
project(FlightStab C CXX)

#====================================================================#
#  macros                                                            #
#====================================================================#

# get current date
macro(get_today today)
	if (MSYS)
		EXECUTE_PROCESS(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE ${today})
		STRING(REGEX REPLACE ".*(..)/(..)/(....).*" "\\3\\1\\2" ${today} ${${today}})
	else (MSYS)
		set(today "NODATE")
	endif (MSYS)
endmacro(get_today)

# build specific target
macro(add_arduino_firmware target sketch board defines)
	set(${target}_SKETCH ${sketch})
	set(${target}_BOARD ${board})
	generate_arduino_firmware(${target})
	set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${defines}")
endmacro(add_arduino_firmware)

#====================================================================#
#  build all targets                                                 #
#====================================================================#

print_board_list()
print_programmer_list()

# get system date if <today> is not specified in cmake command line (cmake -Dtoday=yyyymmdd .....)
if (NOT TODAY)
  get_today(TODAY)
endif (NOT TODAY)

# defines = semicolon-separated list of key-value pairs "RX3S_V1;DEF=VAL;DEF2=VAL2"

add_arduino_firmware(${TODAY}_RX3S_V1 FlightStab pro5v "RX3S_V1")
add_arduino_firmware(${TODAY}_RX3S_V1_8MHZ FlightStab pro "RX3S_V1")
add_arduino_firmware(${TODAY}_RX3S_V2 FlightStab pro5v "RX3S_V2")
add_arduino_firmware(${TODAY}_RX3S_V2_8MHZ FlightStab pro "RX3S_V2")






