#====================================================================#
#  FlightStab cmake                                                  #
#====================================================================#

# build instructions for windows
# ==============================
# cd ./Builds
# optional: rd /s .  (to wipe out all intermediate files)
# cmake [-Dtoday=yyyymmdd] -G"MSYS Makefiles" ..
#   (cmake will use the system date if -Dtoday is not specified)
# make [VERBOSE=1]

# install instructions for windows
# ================================
# install C:\Program Files (x86)\CMake 2.8
# ensure PATH includes C:\Program Files (x86)\CMake 2.8\bin
# install C:\Program Files\arduino-1.0.3 (this exact path is important!)
# ensure PATH includes C:\Program Files\arduino-1.0.3\hardware\tools\avr\utils\bin

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
  include_directories("C:\\Users\\z570\\Documents\\Arduino\\Libraries\\I2Cdev"
                      "C:\\Users\\z570\\Documents\\Arduino\\Libraries\\ITG3200")
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

# standard (16MHz) build
add_arduino_firmware(${TODAY}_RX3S_V1 FlightStab pro5v "RX3S_V1")
add_arduino_firmware(${TODAY}_RX3S_V2 FlightStab pro5v "RX3S_V2")

# 8MHz
add_arduino_firmware(${TODAY}_RX3S_V1_8MHZ FlightStab pro "RX3S_V1")
add_arduino_firmware(${TODAY}_RX3S_V2_8MHZ FlightStab pro "RX3S_V2")

# CPPM enabled
add_arduino_firmware(${TODAY}_RX3S_V1_CPPM FlightStab pro5v "RX3S_V1;USE_CPPM")
add_arduino_firmware(${TODAY}_RX3S_V2_CPPM FlightStab pro5v "RX3S_V2;USE_CPPM")
         
# test permutations (TODO: build only on "make test_targets")          
add_arduino_firmware(TEST_RX3S_V2_MOD_PCINT0 FlightStab pro5v "RX3S_V2;MOD_PCINT0")
add_arduino_firmware(TEST_RX3S_V2_SERIAL FlightStab pro5v "RX3S_V2;USE_SERIAL")
#add_arduino_firmware(TEST_RX3S_V2_USE_I2CDEVLIB FlightStab pro5v "RX3S_V2;USE_I2CDEVLIB")
         
         