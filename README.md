# INFO
STM32F103C8T6
ARM cortex-m3
Flash: 128kB
RAM: 20kB

# DEBUG
Enable in the .ioc file in SYS (Serial Wire Debug swd)

Debugger configuration:
	- "connect under reset" seems to work
	- have to hold the reset button while transfering
	- release it before debugging

# Flash release firmware
Project properties -> C/C++ Build -> MCU Post build outputs -> convert to binary file

st-flash write foobar.bin 0x8000000

# Memory
stlink-gui
st-flash erase

+---------------+ 0x1FFF F800
|               |
| System memory |
| (bootloader)  |
+---------------+ 0x1FFF F000
|               |
|               |
|   reserved    |
|               |
|               |
+---------------+ 0x0801 FFFF
|               |
| Flash memory  |
|               |
+---------------+ 0x0800 0000
| aliased to    |
| flash or      |
| system memory |
| depending on  |
| BOOT pins     |
+---------------+ 0x0000 0000

# Boot
At startup, boot pins are used to select one of three boot options:
- Boot from User Flash (BOOT1 == X && BOOT0 == 0)
- Boot from System Memory (BOOT1 == 0 && BOOT0 == 1)
- Boot from embedded SRAM (BOOT1 == 1 && BOOT0 == 1)

The boot loader is located in System Memory. It is used to reprogram the Flash memory by using USART1 (i think this is what is used by arduino IDE, when we're not using the programmer).
For further details please refer to AN2606.

# Clock
- Increase HCLK to 76MHz (max).
- Use the HSE (High Speed External) crystal oscillator.

# UART
## Test (only need the computer)
Shortcut usb to ttl serial cable with a jumper cable.

Configure tty
stty raw -F /dev/ttyUSB0 -echo -echonl -echoe -echok -echoprt -echoctl -parenb

Write to serial
sudo cat hi > /dev/ttyUSB0

Listen to serial
sudo cat /dev/ttyUSB0
sudo screen /dev/ttyUSBO 115200

## COM (with board)
Use cutecom (easier to set up than with tty)
Baud rate		4800
Word length		8
Parity			None
Stop Bits		1
HW Flow Ctrl 	Disable

## Pinout
USB to TTL serial cable

red     5V      ---------------
black   GND     --------------- GND                 G
white   RX      --------------- TX  USART1  PA_9    A9
green   TX      --------------- RX  USART1  PA_10   A10

# Tools
## Software
- stm32cubeIDE
- stlink-gui
- st-flash
- cutecom

## Hardware
- stm32f103c8t6 board
- st-link v2
