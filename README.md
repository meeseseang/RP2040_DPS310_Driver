# DPS310 Driver for RP2040/RP2350

# Table of Contents
- [Functions](#functions)
- [Example Code](#example-code)

# Functions
- [Constructor](#dps310constructor)
- [Reset](#reset)
- [Config](#config)
- [whoAmI](#whoami)
- [writeRegister](#writeregister)
- [readRegister](#readregister)
- [readCoefficients](#readcoefficients)
- [checkMeasureStatus](#checkmeasurestatus)
- [rawData](#rawdata)
- [scaledData](#scaleddata)

## DPS310(Constructor)
### Description
Initializes sensor communication with SPI and retrieves the coefficient values from their registers.
### Parameters
- CS (int): Chip select pin number
- MOSI(int): Master Out Slave In pin number
- MISO(int): Master In Slave Out pin number
- SCLK(int): Serial Clock pin number
- SPI(spi_inst_t*): Pointer to the SPI instance (refer to Raspberry Pi documentation for more details)

## reset
### Description
Sends a soft reset command to the RST register of the sensor.

## config
### Description
### Parameters
- CFG_DAT(uint8_t): Configuration byte for the CFG_REG
- MEAS_DAT(uint8_t): Configuration byte for measurement mode (Standby, Continuous, Command)
- PRS_DAT(uint8_t): Configuration byte for the pressure sample rate (including oversampling) and precision
- TMP_DAT(uint8_t): Configuration byte for the temperature sample rate (including oversampling) and precision

## whoAmI
### Description
Returns the Product ID and Revision ID of the chip.

## writeRegister
### Description
Writes a byte to the specified register.
### Parameters
- reg(uint8_t): Hex address of register to write to
- data(uint8_t): Hex data to write to the selected register

## readRegister
### Description
Reads data from a specific register.
### Parameters
- reg(uint8_t): Hex address of register to read

## readCoefficients
### Description
Read the calibration coefficients from their registers.

## checkMeasureStatus
### Description
Return true if there is a new pressure or temperature measurement, return false if there is no new measurement ready.

## rawData
### Description
Get raw temperature and pressure data not scaled or temperature compensated for the pressure readings.

## scaledData
### Description
Pressure and temperature data scaled for non-linearity and temperature compensation for pressure readings.

# Example Code
```cpp
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <iostream>
#include "dps_310.hpp"

// Pin definitions for SPI communication
#define CS_PIN   5    // Chip Select pin
#define MOSI_PIN 3    // Master Out Slave In pin
#define MISO_PIN 4    // Master In Slave Out pin
#define SCLK_PIN 2    // Serial Clock pin

// SPI port
#define SPI_PORT spi0

int main() {
    // Initialize stdio for debugging output
    stdio_init_all();

    // Create a DPS310 object
    DPS310 sensor(CS_PIN, SPI_PORT);

    // Wait for the sensor to initialize (minimum 40ms as per documentation)
    sleep_ms(40);

    // Main loop
    while (true) {
        // Read raw barometric pressure and temperature
        DPS310::PressureData rawData = sensor.rawData();
        float raw_pressure = rawData.pressure;
        float raw_temperature = rawData.temperature;

        // Read scaled pressure and temperature
        DPS310::PressureData scaledData = sensor.scaledData(true, true);
        float scaled_pressure = scaledData.pressure;
        float scaled_temperature = scaledData.temperature;

        std::cout << "Raw Pressure: " << raw_pressure << " Pa"<< std::endl;
        std::cout << "Raw Temperature: " << raw_temperature << " C" << std::endl;
        std::cout << "Scaled Pressure" << scaled_pressure << " PSI" << std::endl;
        std::cout << "Scaled Temperature" << scaled_temperature << " Fahrenheit" << std::endl;

        // Delay for 1 second
        sleep_ms(1000);
    }

    return 0;
}
```