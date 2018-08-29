I used a 'blue pill' STLink-v2 from 

Install Openocd

In windows from here http://gnutoolchains.com/arm-eabi/openocd/

run go to the bin folder, and run 

openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg

telnet to 127.0.0.1:4444

type:

reset halt

stm32f1x unlock 0

if you have a .hex file (like the repo contains):

flash write_image erase C:\\DataNoBackup\\hoverboard\\src\\build\\hover.hex 0 ihex

if you have a .bin file (like made by platformio):

flash write_image firmware.bin 0 

programming the hover.hex from the original repo certainly operates the wheels, and generates debug output!

for platformio generated bin:

flash write_image erase C:\\DataNoBackup\\hoverboard\\.pioenvs\\genericSTM32F103RC\\firmware.bin 0x08000000 bin

both should work.......

(check diagnostic output if configured)


On linux (raspberian/debian), installing openocd from apt, I use:

openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c "program firmware.bin 0x08000000" -c "halt 100" -c "reset"

to flash.  My Rpi is powered from the hoverboard, so I have to hold the power button until the board is restarted, else it powers off mid flash!.


