# Information
STM32 project for learning purposes.
- UART driver
- Bootloader

## Hardware
- STM32F103C8T6 board
- st-link v2
- USB to TTL serial cable

## Software
- stm32cubeIDE
- stlink-gui
- st-flash
- cutecom

# MCU information
STM32F103C8T6 ARM cortex-m3
```
Flash: 128kB
RAM  : 20kB
HCLK : 76MHz High Speed External crystal oscillator
```

## Memory
### Mapping

```
+----------------+ 0x1FFF F800
|                 |
|  System memory  |
| (ST bootloader) |
|                 |
+-----------------+ 0x1FFF F000
|                 |
|    Reserved     |
|                 |
+-----------------+ 0x0801 FFFF -+
|                 |              |
|   Application   |              |
|                 |              | Flash
+-----------------+ 0x0800 4000  | memory
|                 |              |
|     Custom      |              | 
|   bootloader    |              |
|                 |              |
+-----------------+ 0x0800 0000 -+
|                 |
|  Aliased to     |
|  flash or       |
|  system memory  |
|  depending on   |
|  BOOT pins      |
|                 |
+-----------------+ 0x0000 0000
```

### Useful commands
```
stlink-gui
st-flash erase
```

# Bootloader
At startup, boot pins are used to select one of three boot options:
- Boot from User Flash (BOOT1 == X && BOOT0 == 0)
- Boot from System Memory (BOOT1 == 0 && BOOT0 == 1)
- Boot from embedded SRAM (BOOT1 == 1 && BOOT0 == 1)

The ST bootloader is located in System Memory. It is used to reprogram the Flash memory by using USART1.

Custom bootloader is located at the start of the flash memory.

# UART
The board simply echoes a 16 byte message received from its UART RX to its UART TX.

## Test TTL serial cable (if using a Linux computer)
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
Use cutecom (easier to set up than with a tty)
```
Baud rate       115200
Word length     8
Parity          None
Stop Bits       1
HW Flow Ctrl    Disable
```

## Pinout
USB to TTL serial cable

```
RED     5V    ---------------
BLACK   GND   ---------------  GND                 G
WHITE   RX    ---------------  TX  USART1  PA_9    A9
GREEN   TX    ---------------  RX  USART1  PA_10   A10
```

## Driver
Use a dummy custom driver instead of the `stm32f1xx_hal_uart.c` (cubeMX generated code not used).

- Uses interrupt but not DMA. 
- Hardcoded configuration.
- No timeout management.
- No error checks.
- Still use the HAL for the GPIO initialization (TODO).

# LED
LED is blinking for a short period after receiving data from UART.

```
LED  ------------  GPIO  PC_13
```

# Usage
## Debug
Enable in the .ioc file in SYS (Serial Wire Debug swd).

## Debug bootloader and application
Run -> Debug Configurations -> Startup -> Add 

Download both binaries and load both their symbols for the debugger to be aware of application symbols after the jump.

## Flash release firmware (without custom bootloader)
Project properties -> C/C++ Build -> MCU Post build outputs -> convert to binary file

`st-flash write stm32f103c8t6.bin 0x8000000`
