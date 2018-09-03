# HEAVILY MODIFIED

This firmware is a heavily modified version of https://github.com/NiklasFauth/hoverboard-firmware-hack which allows you to use the hoverboard AS A HOVERBOARD, as well as external serial control.

My thoughts were:

a/ to make it work as a hoverboard again. Done.

b/ to provide serial control, but not via USART2/3. Done.  Machine protocol needs further work (like and example javascript driver).

c/ to include a RPi ZeroW inside the hoverboard. Done.


# Added:

*Software serial:* This allows you to use ANY GPIO pins (with modififcation) as serial.  Probably best to stay at 9600 baud, as the receive interrupt is serviced at 8x the bitrate.

*Sensor reading:* It reads the original serial data (9 bit) from the original sensor boards from USART2&3.

*Sensor control:* Sensor data can control the PWM demands (power to the wheels).  Double tap on pads to enable.

*Serial diagnostic control:* Protocol.c implements a simple ASCII serial protocol which allows for manual control of the board.

*Serial Machine control:* Protocol.c implements the bones of an acked/checksummed serial protocol.  Embryonic as yet, but intended to be a generic control protocol for the hoverboard.

*PID Control:* PID control loops for control of Speed (in mm/sec) and Position (in mm).  Currently separate control modes, and parameter need better tuning.

*Hall Interupts:* used to read Position and Speed data.

*Flash settings:* implements a flash page available for efficitent storage of parameters (currently unused, but tested).

Should work with original control settings (in config.h), but not tested...

# RPiZeroW
 
I installed an RPiZeroW inside the hoverboard, powered from a DC-DC convertor off 12v, adn connect to the hoverboard via an STLINK and a USB serial.

To program the firmware from the RPiZeroW:

Install openocd (sudo apt-get install openocd).

```
openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c "program firmware.bin 0x08000000" -c "halt 100" -c "reset"
```

(I made a script flash.sh, and copy the firmware.bin over with winscp over ssh).

To connect to the serial, I use a USB serial dongle, and use 

```
screen /dev/ttyUSB0
```

again, another script 't.sh'.

For control, I will be running Node-Red.

# Simple ASCII interface:

*?* - show options

*A n m l* -set buzzer (freq, patt, len_ms)

*B* -toggle sensor Board control

*E* - dEbug 'E'-disable all, EC-enable consoleLog, ES enable Scope\r\n"\

*P* - power control - P -disablepoweroff, PE enable poweroff, Pn power off in n seconds

*I* -enable Immediate commands:
  
  *W/S/A/D/X* -Faster/Slower/Lefter/Righter/DisableDrive
  
  *H/C/G/Q* -read Hall posn,speed/read Currents/read GPIOs/Quit immediate mode
  
  *N/O/R* - read seNsor data/toggle cOntrol/dangeR
  
*T* -send a machine test message A-ack N-nack T-test

*?* -show this



# Original readme from here on:

This firmware is much better than the old one. tested up to 40A / 60V, no dead board so far :)

# hoverboard-firmware-hack

This repo contains open source firmware for generic Hoverboard Mainboards.
The firmware you can find here allows you to use your Hoverboard Hardware (like the Mainboard, Motors and Battery) for cool projects like driving armchairs, person-tracking transportation robots and every other application you can imagine that requires controlling the Motors.

If you want an overview of what you can do with this firmware, here is a ~40min video of a talk about this project:
https://media.ccc.de/v/gpn18-95-howto-moving-objects

---

## Hardware
![otter](https://raw.githubusercontent.com/NiklasFauth/hoverboard-firmware-hack/master/pinout.png)

The original Hardware supports two 4-pin cables that originally were connected to the two sensor boards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard.
Both USART2 & 3 can be used for UART and I2C, PA2&3 can be used as 12bit ADCs.

The reverse-engineered schematics of the mainboard can be found here:
http://vocke.tv/lib/exe/fetch.php?media=20150722_hoverboard_sch.pdf

---

## Building
To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary (version 7 works, there is a version that does not!) location in the Makefile ("PREFIX = ..."). 

The firmware will also build (and flash) very easily from platform.io, plaformio.ini file included.  Simply open the folder in the IDE of choice (vscode or Atom), and press the 'PlatformIO:Build' or the 'PlatformIO:Upload' button (bottom left in vscode). 

(Note: if you have no buttons, use Debug/Add Configuration, and select 'PlatformIO Debugger'; seems to kick it into life).


## Flashing
Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing. Battery > 36V have to be connected while flashing.

To flash the STM32, use the ST-Flash utility (https://github.com/texane/stlink).

If you never flashed your mainboard before, the STM is probably locked. To unlock the flash, use the following OpenOCD command:
```
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0"
```

If that does not work:
```
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" -c "mww 0x40022010 0x2220" -c "sleep 100" -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" -c targets -c "halt" -c "stm32f1x unlock 0"
```
```
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c targets -c "halt" -c "stm32f1x unlock 0"
```
Or use the Windows ST-Link utility.

Then you can simply flash the firmware:
```
st-flash --reset write build/hover.bin 0x8000000
```

---
## Troubleshooting
First, check that power is connected and voltage is >36V while flashing.
If the board draws more than 100mA in idle, it's probably broken.

If the motors do something, but don't rotate smooth and quietly, try to use an alternative phase mapping. Usually, color-correct mapping (blue to blue, green to green, yellow to yellow) works fine. However, some hoverboards have a different layout then others, and this might be the reason your motor isn't spinning.

Nunchuck not working: Use the right one of the 2 types of nunchucks. Use i2c pullups.

Nunchuck or PPM working bad: The i2c bus and PPM signal are very sensitive to emv distortions of the motor controller. They get stronger the faster you are. Keep cables short, use shielded cable, use ferrits, stabalize voltage in nunchuck or reviever, add i2c pullups. To many errors leads to very high accelerations which triggers the protection board within the battery to shut everything down.

Most robust way for input is to use the ADC and potis. It works well even on 1m unshielded cable. Solder ~100k Ohm resistors between ADC-inputs and gnd directly on the mainboard. Use potis as pullups to 3.3V.

---


## Examples

Have a look at the config.h in the Inc directory. That's where you configure to firmware to match your project.
Currently supported: Wii Nunchuck, analog potentiometer and PPM-Sum signal from a RC remote.
If you need additional features like a boost button, have a look at the while(1) loop in the main.c

### Projects based on it
* [bobbycar-optimized firmware](https://github.com/larsmm/hoverboard-firmware-hack-bbcar)  based on this one with driving modes, acceleration ramps and some other features
* [wheel chair](https://github.com/Lahorde/steer_speed_ctrl) controlled with a joystick or using a CC2650 sensortag to control it over  bluetooth with pitch/roll.
