#FlightStab
Flight Stabilizer Firmware for RX3S (V1, V2 later) and Arduino-based microcontrollers.

###*USE AT YOUR OWN RISK  USE AT YOUR OWN RISK  USE AT YOUR OWN RISK  USE AT YOUR OWN RISK  USE AT YOUR OWN RISK  USE AT YOUR OWN RISK  USE AT YOUR OWN RISK*

##DEVICE SUPPORT
* NANO_MPU6050 (my development platform: Arduino Nano 328p 5V 16Mhz and MPU6050 sensor)
* RX3S V1 (http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=25448)
* RX3S V2 (not yet supported) (http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=28456)
 
##SETUP TO BUILD RX3S FIRMWARE (Optional, can use precompiled hex file)
1. install arduino v1.0x (http://arduino.cc/en/Main/Software)
2. download i2cdevlib (http://www.i2cdevlib.com/usage)
3. place i2cdev directories in `C:\users\<user>\My Documents\Arduino\Libraries\`

```
for example:
C:\users\<user>\My Documents\Arduino\Libraries\_Stub
C:\users\<user>\My Documents\Arduino\Libraries\I2Cdev
C:\users\<user>\My Documents\Arduino\Libraries\ITG3200
...
```
4. run Arduino.exe and select Board as `Arduino Pro or Pro Mini (5V, 16MHz) w/ ATmega 168`
5. load `FlightStab.ino`
6. uncomment `#define RX3S_V1` for RX3S V1 and check all other devices are commented out.
7. verify/compile the program.
8. locate the generated hex file at `C:\users\<user>\AppData\Temp\build<numbers>.tmp\FlightStab.cpp.hex`. sort the directory by modified-date to help find the most recent `build<numbers>.tmp` entry.
9. verify the timestamp of `FlightStab.cpp.hex` matches current time.
 
##SETUP TO FLASH FIRMWARE (using eXtreme Burner - AVR as the USBASP programmer)
1. install eXtreme Burner - AVR (http://extremeelectronics.co.in/category/software/)
2. edit the properties of `C:\Program Files\eXtreme Burner - AVR\Data` to allow the user full control over the directory. otherwise, the contents are read-only.
3. edit the file `C:\Program Files\eXtreme Burner - AVR\Data\chips.xml` to add support for the new **ATmega168PA** chip type.
4. add the following XML section just after the ATmega168 section

```xml
	<CHIP>
		<NAME>ATmega168PA</NAME>
		<FLASH>16384</FLASH>
		<EEPROM>512</EEPROM>
		<SIG>0x000B941E</SIG>
		<PAGE>128</PAGE>
		<LFUSE layout="2">YES</LFUSE>
		<HFUSE layout="3">YES</HFUSE>
		<EFUSE layout="2">YES</EFUSE>
		<LOCK>YES</LOCK>
		<CALIB>YES</CALIB>
		<PLACEMENT>.\Images\Placements\ZIF_DIP_40.bmp</PLACEMENT>
	</CHIP>
```
5. save the file. if there is an error saving, then check the security properties in step 2 again.
		
##FLASHING THE FIRMWARE (using eXtreme Burner - AVR)
1. use a USBSAP programmer capable of operating at 3.3v 
2. i think it is possible to use a 5V programmer, but you have to disconnect the ISP Vcc line and also self power the RX3S. i have not verified this.
3. run eXtreme Burner and select Chip type as ATmega168PA
4. connect the ISP connector to the RX3S
5. select `"Read All"`. the program should recognize the chip and proceed to read the flash, eeprom and fuse/lock bits
6. the flash and eeprom will be read back as 0xff (since the lock bits LB1/2 are set). that is correct.
7. the fuse/lock bits should read:

```
    lo=0xf7
    hi=0xdf
    ext=0xf9
    lock=0xfc
    cal=0xffffff95
```

###__DO NOT PROCEED IF THE VALUES DO NOT MATCH OR IF THE PROGRAM CANNOT RECOGNIZE THE CHIP (AFTER 2-3 TRIES).__

FROM THIS POINT ONWARDS, YOU WILL ERASE THE CHIP AND REFLASH WITH NEW FIRMWARE. THERE IS NO WAY TO RESTORE THE ORIGINAL FIRMWARE.

8. select `Chip Erase`. the program will erase the chip and reset the lock bits.
9. select `Read All` again, this time the lock fuse should lock=0xff.
10. open the generated hex file `FlightStab.cpp.hex`
11. select `Write` from the menu bar then `Flash`. the program will write the updated firmware to the flash. sometimes it will fail to recognize the chip ID. try again 2-3 times and it would usually succeed.
12. disconnect the ISP connector from the RX3S.


##USING THE RX3S V1
1. connect RX AIL/ELE/RUD to AIL/ELE/RUD (standard). connect RX AUX to AIL-R (used to be output, is now input).
2. set RUD switch to "rev" (to the left) to enable DELTA wing mode. In DELTA wing mode, the AIL and ELE output connect to the wing servos, the RUD output is unchanged from NORMAL mode.
3. set ELE switch to "rev" (to the left) to enable VTAIL mode. In VTAIL mode, the ELE and RUD output connect to the vtail servos, the AIL output is unchanged from NORMAL mode.
4. the VRs control the individual AIL/ELE/RUD gain (linearly) AND direction of correction.
   * 12 o'clock = zero gain
   * 7 o'clock = max gain in one direction
   * 5 o'clock = max gain in opposite direction
5. the RX AUX control the master gain (linearly). you can set it with 2P switch (MIN/MAX), 3P switch (MIN/MID/MAX) or KNOB on the TX. RX pulse width 1100us == MIN GAIN and 1900us == MAX GAIN (linearly)
6. the stick position also controls the gain to reduce the stabilizer from over correcting your controls during manoeuvres.
   * center position = max gain
   * extreme position = zero gain
   * in between = linear between max and zero.

##OTHER NOTES
* the FlightStabGUI is still incomplete and is likely to be supported by ATmega328-based devices (instead of the ATmega168-based ones) in the future.
* RX3S V2 support NOT DONE YET, but will be next.

##FIGHTSTAB FEATURES
* AUX master gain. RX aux channel controls the master gain linearly (1100us-1900us PWM)
* VRs controls both servo gain and direction. pot position at 12 oclock = zero gain, 7 oclock / 5 oclock = max gain in opposite directions.
* stick proportional master gain. channel gain reduces as stick position goes away from center position, which prevents the stabilizer from overcorrecting or "fighting" the control stick during manoeuvres.
* support DELTA wing and VTAIL modes.
* oscillation detection (NOT DONE YET). detects oscillation with no stick input and reduces channel gain if needed.
* stick controlled rotation "fly by wire" (NOT DONE YET). stick controls the stabilizer PID input so it will try to rotate at the commanded rate (instead of simple stabilization, which tries to correct any detected rate of rotation to zero). 
* looking for other suggestions, please send message.


