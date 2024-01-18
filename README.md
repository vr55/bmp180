[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) ![Static Badge](https://img.shields.io/badge/1.0.0-brightgreen?label=version)

# Bosch BMP180 Sensor Library for STM32F10x/CH32F10x with StdPeriph
This library provides functions to interface with the BMP180 temperature and pressure sensor on STM32F10x/CH32F10x microcontrollers using STM32 Standard Peripheral Libraries.

## BMP180 Sensor Technical Specifications

- **Interface**: I2C (TWI) or SPI
- **Supply Voltage**: 1.8V to 3.6V
- **Operating Voltage**: 1.8V to 3.6V
- **Operating Temperature**: -40°C to 85°C
- **Pressure Range**: 300hPa to 1100hPa (9000m to -500m above sea level)
- **Temperature Measurement Range**: 0°C to 65°C
- **Temperature Accuracy**: ±2°C
- **Pressure Accuracy**: ±1hPa

Datasheet: [https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)

### Usage
#### example
```C
#include "bmp180.h"

int main(void)
{
  bmp180_t  bmp180Structure;
  int32_t   temperature, pressure ;
  float     altitude;
  
  char result = BMP180_Setup( I2C1, &bmp180Structure );
  
  if( result == 1 )
  {
    temperature = BMP180_GetTemperature();
  
    //Pressure Pa
    pressure = BMP180_GetPressure( BMP180_ACC_ULTRA_HIGH_RES );
  
    altitude = BMP180_GetAltitude( pressure );
  
    //Pressure mmHg
    pressure = pressure * 0.00750062;
  }
  
  while (1);
}
```
