@ECHO OFF
set AVRASM2PATH=c:\Program Files (x86)\Atmel\Atmel Toolchain\AVR Assembler\Native\2.1.39.1005\avrassembler

call :buildhex AQUASTAR
call :buildhex RX3S_V1V2V3_AILR_OUT
call :buildhex MINI_MWC_AILR_OUT
goto :eof

:buildhex
set OUT=default\AVRootloader_%~1
echo %OUT%
"%AVRASM2PATH%\avrasm2.exe" -fI -w -W+ie -D %~1 -I "%AVRASM2PATH%\include" -o "%OUT%.hex" -S "%OUT%.info" -d "%OUT%.obj" -e "%OUT%.eep" -m "%OUT%.map" -l "%OUT%.lst" "AVRootloader.asm"
goto :eof