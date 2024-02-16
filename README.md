# STM32-inclinometer
## STM32 ARM Cortex-M3 based inclinometer

### 1. Introduction
> An **inclinometer** or **clinometer** is an instrument used for measuring angles of slope, elevation, or depression of an object with respect to gravity's direction. [...]
> 
> Clinometers measure both inclines and declines using three different units of measure: degrees, percentage points, and topos.

Quote : [Wikipedia](https://en.wikipedia.org/wiki/Inclinometer)

This project is a crude implementation of an inclinometer, using :
- A Bluepill (STM32f103 ARM microcontroller development board)
- An ADXL345 accelerometer
- An SSD1306 128x64 OLED screen