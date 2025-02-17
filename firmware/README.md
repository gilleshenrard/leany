# Leany firmware

### 5. How to build
1. Open the *.ioc file with STM32CubeMX and click "Generate Code" to add the STM32CubeMX-generated files to the directory.
2. Hit CTRL + SHIFT + P, then launch "CMake: Configure", then select "Debug" (or simply hit F7)
3. Once done, hit CTRL + SHIFT + P, then launch "CMake: Build" (or simply hit F5)

### 6. Operation principles
This devices functions in 4 steps :
1. Wait for the LSM6DSO to gather measurements
    - accelerometer : linear acceleration with a digital low-pass filter on the X, Y and Z axis
    - gyroscope : angle rate with digital low-pass and high-pass filters on the x, y and z axis
2. Apply a complementary filter (with Euler angles transformation) on the measurements
3. Format the angles with their sign and print them on the screen (if the angle changed)
4. Rinse and repeat

### 7. Wiring

STLink V2 pinout :

![](img/STLinkV2_pinout.jpg)

Photo : [PlayEmbedded](https://www.playembedded.org/blog/mikroe-clicker-2-for-stm32-and-stlink-v2/)

STLink to Bluepill wiring :
| STLink V2 pin | STLink V2 pin number | Bluepill pin | Circuit  |
|:-------------:|:--------------------:|:------------:|:--------:|
| MCU VDD       | 1                    | 3V3          |          |
| SWDIO         | 7                    | SWDIO        |          |
| SWCLK         | 9                    | SWCLK        |          |
| VDD           | 19                   |              | VDD rail |
| GND           | 20                   |              | GND rail |

Bluepill to peripherals wiring :
| STM32/Bluepill pin | Alternate use | LSM6DSO pin | SSD1306 pin | Zero button      | Hold button      | Power latch      |
|:------------------:|:-------------:|:-----------:|:-----------:|:----------------:|:----------------:|:----------------:|
| PA4                | SPI1 NSS      | CS          |             |                  |                  |                  |
| PA5                | SPI1 SCK      | SCL         |             |                  |                  |                  |
| PA6                | SPI1 MISO     | SDO         |             |                  |                  |                  |
| PA7                | SPI1 MOSI     | SDA         |             |                  |                  |                  |
| PB0                | GPIO input PU*| INT1        |             |                  |                  |                  |
| PB12               | SPI2 NSS      |             | CS          |                  |                  |                  |
| PB13               | SPI2 SCK      |             | D0          |                  |                  |                  |
| PB15               | SPI2 MOSI     |             | D1          |                  |                  |                  |
| PA9                | GPIO output   |             | D/C         |                  |                  |                  |
| PA10               | GPIO output   |             | RES         |                  |                  |                  |
| PB1                | GPIO input PU*|             |             |                  |                  | Button           |
| PB10               | GPIO input PU*|             |             | X (other to GND) |                  |                  |
| PB11               | GPIO input PU*|             |             |                  | X (other to GND) |                  |
| PB14               | GPIO out. PU* |             |             |                  |                  | Power ON output  |

*PU : Pull-up

Note : Two different SPI are used because, while the SSD1306 can go at full speed, the ADXL345 can go at max. 5MHz.

In addition, SPI2 is a transmit-only master because the SSD1306 does not allow any read operation in serial mode. 
