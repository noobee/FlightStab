#====================================================================#
#  FlightStab cmake                                                  #
#====================================================================#

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
		STRING(REGEX REPLACE ".*(..)/(..)/(....).*" "\\3\\2\\1" ${today} ${${today}})
	else (MSYS)
		set(today "NODATE")
	endif (MSYS)
endmacro(get_today)

# build specific target
macro(build_target target sketch board definitions)
	set(${target}_SKETCH ${sketch})
	set(${target}_BOARD ${board})
	generate_arduino_firmware(${target})
	set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${definitions}")
endmacro(build_target)

#====================================================================#
#  build all targets                                                 #
#====================================================================#

print_board_list()
print_programmer_list()

get_today(today)

build_target(${today}_RX3S_V1 FlightStab pro5v "RX3S_V1")
build_target(${today}_RX3S_V1_8MHZ FlightStab pro "RX3S_V1")
build_target(${today}_RX3S_V2 FlightStab pro5v "RX3S_V2")
build_target(${today}_RX3S_V2_8MHZ FlightStab pro "RX3S_V2")






