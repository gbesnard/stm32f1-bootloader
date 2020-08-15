# Information
STM32 project for learning purposes.
- Bootloader
- UART driver

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

Custom bootloader is located at the start of the flash memory, hence we set BOOT0 pin to 0.

```
BOOTLOADER
+-----------------------------------------------------+
|State 1: LED green ON.                               |
|Wait for one and only one dummy char on UART         |-->+
|for 30 seconds maximum                               |   |
+-----------------------------------------------------+   |
                          |                               |
                          v                               |
+-----------------------------------------------------+   |
|State 2: LED Green OFF.                              |   |
|Erase flash applicative area.                        |   |
+-----------------------------------------------------+   |
                          |                               |
                          v                               |
+-----------------------------------------------------+   |
|State 3: LED green ON.                               |   |
|Wait for the application binary on UART              |-->+ 30 secs delay
|for 30 secs maximum.                                 |   | elapsed without
+-----------------------------------------------------+   | data received on 
                          |                               | UART.
                          v                               |
+-----------------------------------------------------+   |
|State 4: LED green OFF.                              |   |
|Receiving and flashing application binary.           |   |
+-----------------------------------------------------+   |
                          |                               |
                          v                               |
+-----------------------------------------------------+   |
|State 5: LED green ON.                               |<--+
|Branch to main application.                          |   
+-----------------------------------------------------+   
                          |                               
APPLICATION               v                               
+-----------------------------------------------------+   
|State 6: LED green blinking.                         |
|Main application execution.                          |
+-----------------------------------------------------+
```


# UART

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

The application binary file can be sent to the bootloader using UART to update it.

No check is done, what is received from UART is directly copied into FLASH from 0x08004000 upwards.

## Notes
The application sent must be a .bin file and not an .elf file.
```
Project properties -> C/C++ Build -> MCU Post build outputs -> convert to binary file
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
Use a dummy driver instead of the `stm32f1xx_hal_uart.c`.

- Circular buffer for rx. 
- No use of interrupt nor DMA.
- No timeout management.

# LED
LED main program make the LED blink to show the branching from bootloader worked.

```
LED  ------------  GPIO  PC_13
```

# Usage
## Debug
Enable in the .ioc file in SYS (Serial Wire Debug swd).

## Debug bootloader and application
Run -> Debug Configurations -> Startup -> Add 

Download both binaries and load both their symbols for the debugger to be aware of application symbols after the jump.

## Flash release bootloader
`st-flash write stm32f103c8t6_bootloader.bin 0x8000000`
