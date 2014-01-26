#====================================================================#
#  FlightStab cmake                                                  #
#====================================================================#

# build instructions for windows
# ==============================
# cd ./Builds
# optional: rd /s .  (to wipe out all intermediate files)
# cmake [-DTODAY=yyyymmdd] -G"MSYS Makefiles" ..
#   (cmake will use the system date if -DTODAY is not specified)
# make [VERBOSE=1]

# install instructions for windows
# ================================
# install C:\Program Files (x86)\CMake 2.8
# ensure PATH includes C:\Program Files (x86)\CMake 2.8\bin
# install C:\Program Files\arduino-1.0.5 (this exact path is important!)
# ensure PATH includes C:\Program Files\arduino-1.0.5\hardware\tools\avr\utils\bin

# other useful commands
# =====================
# avr-readelf -a <filename>.elf
# avr-objdump -dS <filename>.elf
# avr-nm -Cn <filename>.elf
# avr-size [-C] <filename>.elf
# avr-strings <filename>.elf
# avrdude -C "C:\Program Files\arduino-1.0.5\hardware\tools\avr\etc\avrdude.conf" -c usbasp -p m168p -U flash:r:<filename>.hex

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)
cmake_minimum_required(VERSION 2.8)
project(FlightStab C CXX)

if (MSYS)
  link_directories("$ENV{USERPROFILE}\\Documents\\Arduino\\libraries")
else (MSYS)
  link_directories("~/Arduino/libraries")
endif (MSYS)

#====================================================================#
#  macros                                                            #
#====================================================================#

# get current date
macro(get_today today)
	if (MSYS)
    EXECUTE_PROCESS(COMMAND wmic os get localdatetime /format:list OUTPUT_VARIABLE ${today})
    STRING(REGEX REPLACE ".*LocalDateTime=(........).*" "\\1" ${today} ${${today}})
    #EXECUTE_PROCESS(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE ${today})
    #STRING(REGEX REPLACE ".*(..)/(..)/(....).*" "\\3\\1\\2" ${today} ${${today}})
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

# AQUASTAR/DLUX Programming Box
add_arduino_firmware(${TODAY}_AQUASTAR Aquastar atmega8 "AQUASTAR")
add_arduino_firmware(${TODAY}_DLUX Aquastar atmega8 "DLUX")

# NANOWII
add_arduino_firmware(${TODAY}_NANOWII FlightStab leonardo "NANOWII")
add_arduino_firmware(${TODAY}_NANOWII_CPPM FlightStab leonardo "NANOWII;SERIALRX_CPPM")
add_arduino_firmware(${TODAY}_NANOWII_SPEKTRUM FlightStab leonardo "NANOWII;SERIALRX_SPEKTRUM")
add_arduino_firmware(${TODAY}_NANOWII_SBUS FlightStab leonardo "NANOWII;SERIALRX_SBUS")

# MINI MWC
add_arduino_firmware(${TODAY}_MINI_MWC_CPPM FlightStab pro5v "MINI_MWC;SERIALRX_CPPM")
add_arduino_firmware(${TODAY}_MINI_MWC_CPPM_EXTERNAL_RX FlightStab pro5v "MINI_MWC;SERIALRX_CPPM;MINI_MWC_EXTERNAL_RX")
add_arduino_firmware(${TODAY}_MINI_MWC_SPEKTRUM FlightStab pro5v "MINI_MWC;SERIALRX_SPEKTRUM")
add_arduino_firmware(${TODAY}_MINI_MWC_SBUS FlightStab pro5v "MINI_MWC;SERIALRX_SBUS")

# RX3S V1/V2V3/RX3SM 16MHz
add_arduino_firmware(${TODAY}_RX3S_V1 FlightStab pro5v "RX3S_V1")
add_arduino_firmware(${TODAY}_RX3S_V1_CPPM FlightStab pro5v "RX3S_V1;SERIALRX_CPPM")
add_arduino_firmware(${TODAY}_RX3S_V1_SPEKTRUM FlightStab pro5v "RX3S_V1;SERIALRX_SPEKTRUM")
add_arduino_firmware(${TODAY}_RX3S_V1_SBUS FlightStab pro5v "RX3S_V1;SERIALRX_SBUS")
add_arduino_firmware(${TODAY}_RX3S_V2 FlightStab pro5v "RX3S_V2")
add_arduino_firmware(${TODAY}_RX3S_V2_CPPM FlightStab pro5v "RX3S_V2;SERIALRX_CPPM")
add_arduino_firmware(${TODAY}_RX3S_V2_SPEKTRUM FlightStab pro5v "RX3S_V2;SERIALRX_SPEKTRUM")
add_arduino_firmware(${TODAY}_RX3S_V2_SBUS FlightStab pro5v "RX3S_V2;SERIALRX_SBUS")
add_arduino_firmware(${TODAY}_RX3SM FlightStab pro5v "RX3SM")
         
# EagleA3Pro
add_arduino_firmware(${TODAY}_EAGLE_A3PRO FlightStab pro5v "EAGLE_A3PRO")
add_arduino_firmware(${TODAY}_EAGLE_A3PRO_NO_STICKCONFIG FlightStab pro5v "EAGLE_A3PRO;NO_STICKCONFIG")

# test permutations          
add_arduino_firmware(TEST_RX3S_V1_8MHZ FlightStab pro "RX3S_V1")
add_arduino_firmware(TEST_RX3S_V2_8MHZ FlightStab pro "RX3S_V2")
add_arduino_firmware(TEST_RX3S_V2_SERIAL_DEBUG FlightStab pro5v "RX3S_V2;SERIAL_DEBUG")
add_arduino_firmware(TEST_RX3S_V2_DUMP_SENSORS FlightStab pro5v "RX3S_V2;DUMP_SENSORS")
add_arduino_firmware(TEST_RX3S_V2_LED_TIMING FlightStab pro5v "RX3S_V2;LED_TIMING")
add_arduino_firmware(TEST_RX3S_V2_I2CDEVLIB FlightStab pro5v "RX3S_V2;USE_I2CDEVLIB")
add_arduino_firmware(TEST_RX3S_V2_NO_ONEWIRE FlightStab pro5v "RX3S_V2;NO_ONEWIRE")
add_arduino_firmware(TEST_RX3S_V2_NO_STICKCONFIG FlightStab pro5v "RX3S_V2;NO_STICKCONFIG")

        