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

## Setup & Usage

TODO


## Building

TODO
