# stm32-frequencies2canbus
Software for an STM32F103-board that accurately measures 6 frequencies and reports them on the CAN bus

## Introduction
This project was made to solve an automotive prototyping use-case of measuring several frequencies 
and reporting them to an ECU on a CAN-Bus
(while being a learning exercise on how to work with STM32 microcontrollers).
The hardware this is designed for is the
[OLIMEXINO-STM32](https://www.olimex.com/Products/Duino/STM32/OLIMEXINO-STM32/open-source-hardware) board

The primary requirements were:
* Accurately read 6 frequencies in parallel (in the range of 0.1Hz - 600Hz)
  and report to them on a CAN-bus (250kBaud) in fixed intervals  
  (although frequencies of 20kHz or more are still working fine)
  * Frequencies are read using InputCapture, with relatively fast running timers
  * Timer-overflows are tracked so that low frequencies can be measured
  * OverCapture errors are detected and handled (although they shouldn't really happen)
  * Measurements get invalidated after a certain duration without capture-events
  * Measurements get lowered gradually when there is no capture-event within the valid period

Apart from this, the current version has the additional features/properties. It:
* can produce 2 configurable PWM-frequencies to develop/test without a vehicle.
* can output measurement-statistics for debugging on the serial port (without impacting with the measurement)
* can read debug commands from the serial-port, to tweak certain parameters
* can can output time spent in ISRs on GPIOs (to be profiled with a Logic-Analyzer)

## Setup 

### Flashing the bootloader
I'm using a slightly modified version of "STM32duino-bootloader" which provides a DFU interface for programming (which the STM32F103 does not have natively).
(My fork adds the Olimexino-board definition and disarms the bootloader for POR and WDT resets, so we need to actually press RESET if we wanna boot the bootloader)

* Install a flash-tool for the stm32 serial bootloader. I used "stm32flash" which I took from an STM32 Arduino Board-Package
* Connect the board via to a 3,3V USB-Serial adapter, the pinning should be:
  * GND  <-->  STM32-GND  (OR UExt-02)
  * RX   <-->  STM32-D7   (OR UExt-03)
  * TD   <-->  STM32-D8   (OR UExt-04)
* Get the bootloader binary from [my fork](https://github.com/teschi/STM32duino-bootloader/blob/master/bootloader_only_binaries/olimexino_stm32_boot20.bin)
* Enter native STM32 bootloader by holding button "BUT" and then pressing "RST"
* Run: stm32flash -w olimexino_stm32_boot20.bin -v -g 0x0 /dev/ttyUSB0
  * you might have to replace /dev/ttyUSB0 with your serial device
* A blinking green LED indicates the bootloader is running and awaiting the firmware to be flashed

### Flashing the firmware
* Install dfu-util
* Connect to the Olimexino board via it's USB connector 
* get the [firmware binary](https://github.com/teschi/stm32-frequencies2canbus/tree/master/binary)
* Run: dfu-util -d 1EAF:0003 -a 1 -D stm32-frequencies2canbus.bin -R
* Green LED should now blink fast (indicating CAN errors, if CAN bus is not connected)


## Usage

### Normal usage:

* Connect the CAN-bus (250kBaud)
  * if everything is OK with the bus, the green flicker should stop
* Connect the input pins for the frequency measurement to digital 3,3V signals.
  If there are frequencies measured, the yellow LED should be blinking/flickering
  The pin-mapping is defined as follows:
  * Frequency 1-3 (reported on CAN-ID 302): D2, D1, D0
  * Frequency 4-6 (reported on CAN-ID 303): EXT14, EXT15, EXT16 

### Test/Debug usage:
To enable "Debug mode" boot the Olimexino with UEXT-7 and UEXT-8 being connected (e.g. by jumper) 

* This will enable debug output to be printed to a serial terminal (115200, 8N1, using newlines to break).
  * Note: unfortunately we cannot use USB for this debugging, because CAN and USB cannot be used at the same time on the STM32F103
* Pins D6 & D5 will be configured as PWM to output frequencies. Those can be measured again using the input pins (e.g. if you connect D5 with D0)
* There is a serial debug terminal that can be used to tweak several settings without having to rebuild.
  send "h" followed by a newline to get a list of commands, which are: 
  * p<uint>  - change prescaler for InputCapture (default: 100)
  * F<float> - change frequency on test-signal from D5
  * f<float> - change frequency on test-signal from D6
  * i<uint>  - change input-filter value [IC1F] (-1) for input-captures (allowed: 1-16, default: 6) 
  * c<uint>  - change input-filter Clock-division [CKD] (-1) for input-captures (allowed: 1-3, default: 1) 
  * d        - disable debug GPIO-PINS (D5, D6 and A0-4)
  * e        - enable debug GPIO-PINS (D5, D6 and A0-4) [A0-4 are used to visualize time spent in interrupts on an oscilluscope]
  * u        - toggle USART debugging on/off
  * w        - trigger a watchdog reset
  * s        - execute a software reset
  * r<uint>  - CAN report rate (default: 20 (ms))
  * n        - Print information (uptime, reset-reason, error-counts)
  * l        - Print license info

## Development / Building

The project was implemented using:
* STM32CubeMX v4.24
* and SW4STM32 v2.4
(not going to go into detail on the setup, but you only need CubeMX for if you want to use it for changing stuff in the generated parts.)
Also you should be able to re-generate the project for different development-environments, e.g. plain makefile.

For debugging (breakpoints, etc) I used a Raspberry-Pi running OpenOCD, connected to the SDW port of the Olimexino
I roughly followed [this guide](https://github.com/TurtleRover/firmware-shield/wiki/Wireless-Programming-and-Debugging-with-STM32-and-RPi) on how to set this up.
