# MCU information
STM32F103C8T6 ARM cortex-m3

## General
```
Flash	:	128kB
RAM		:	20kB
HCLK	:	76MHz
```

## Memory
### Mapping

```
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
```

### Useful commands
```
stlink-gui
st-flash erase
```

## Boot
At startup, boot pins are used to select one of three boot options:
- Boot from User Flash (BOOT1 == X && BOOT0 == 0)
- Boot from System Memory (BOOT1 == 0 && BOOT0 == 1)
- Boot from embedded SRAM (BOOT1 == 1 && BOOT0 == 1)

The boot loader is located in System Memory. It is used to reprogram the Flash memory by using USART1 (this is what is used by arduino IDE, when we're not using the programmer).
For further details please refer to AN2606.

# Tools
## Software
- stm32cubeIDE
- stlink-gui
- st-flash
- cutecom

## Hardware
- stm32f103c8t6 board
- st-link v2
- USB to TTL serial cable

# Clock
- HCLCK increased to maximum of 76MHz.
- Use the HSE (High Speed External) crystal oscillator.

# UART
The board simply echo a 16 byte message received from its UART RX to its UART TX.

## Test (only need a computer)
Shortcut usb to ttl serial cable with a jumper cable.

Configure tty
`stty raw -F /dev/ttyUSB0 -echo -echonl -echoe -echok -echoprt -echoctl -parenb`

Write to serial
`sudo cat hi > /dev/ttyUSB0`

Listen to serial
```
sudo cat /dev/ttyUSB0
sudo screen /dev/ttyUSBO 115200
```

## COM (with board)
Use cutecom (easier to set up than with tty)
```
Baud rate		115200
Word length		8
Parity			None
Stop Bits		1
HW Flow Ctrl 	Disable
```

## Pinout
USB to TTL serial cable

```
RED     5V      ---------------
BLACK   GND     --------------- GND                 G
WHITE   RX      --------------- TX  USART1  PA_9    A9
GREEN   TX      --------------- RX  USART1  PA_10   A10
```

# LED
LED is blinking for a short period after receiving data from UART.

```
LED		------------	GPIO 	PC_13
```

# Usage
## Debug
Enable in the .ioc file in SYS (Serial Wire Debug swd).

## Flash release firmware
Project properties -> C/C++ Build -> MCU Post build outputs -> convert to binary file

`st-flash write foobar.bin 0x8000000`
