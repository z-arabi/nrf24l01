<h1> NRF24L01+ </h1>

<h2> Introduction </h2>
In this project, we use SPI protocol to connect the module to the STM32F103 ARM microcontroller. SPI (Serial Peripheral Interface) is a synchronous serial communication interface specification used for short-distance communication. SPI communication is in full duplex mode using a master-slave architecture.

NRF24L01 is a single chip radio transceiver for the world wide 2.4 - 2.5 GHz ISM band. The transceiver consists of a fully integrated frequency synthesizer, a power amplifier, a crystal oscillator, a demodulator, modulator and Enhanced ShockBurst™ protocol engine. Output power, frequency channels, and protocol setup are easily programmable through a SPI interface. Current consumption is very low, only 9.0mA at an output power of -6dBm and 12.3mA in RX mode. Built-in Power Down and Standby modes makes power saving easily realizable.

The targer of this project is to write a library for for SPI protocol for connecting the module to the STM32F103 ARM microcontroller which has the ability to adjust the frequency channel, output power, telecommunication link speed. Unlike UART protocol, NRF modules are either transmitter or receiver at the moment; therefore, we are not allowed to have one module on each side, so we managed this issue  by checking the send and receive buffer size, and in the case we face a problem an Error message will come up. 

<h2>
  Implementation
</h2>
The library is based on ARM Programming Language and written in Keil uVision, a window-based software development platform for ARM-based microcontrollers. 
The microcontroller used is STM32F103 - Arm Cortex-M3 Microcontrollers.
There are multiple libraries for writing the program to implement on ARM microcontrollers, and generally, I would prefer to use the HAL library for STM32 ARM Cortex.

