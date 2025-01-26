#ifndef DPS_310_HPP
#define DPS_310_HPP

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <iostream>

// ***NOTE*** wait at least 40ms before trying to access registers

class DPS310{
    private:
        // Pinout config
        int CS_PIN;
        spi_inst_t* SPI_PORT;

        // Coefficient scaling factors
        int16_t c0;
        int16_t c1;
        int32_t c00;
        int32_t c10;
        int16_t c01;
        int16_t c11;
        int16_t c20;
        int16_t c21;
        int16_t c30;

    public:

        // Constructor
        DPS310(int CS, spi_inst_t* SPI_PORT);

        // Register definitions
        static constexpr uint8_t PSR_B2     = 0x00;
        static constexpr uint8_t PSR_B1     = 0x01;
        static constexpr uint8_t PSR_B0     = 0x02;
        // Pressure Data PRS_Bn, contains 24 bit pressure measurement value, if FIFO enabled it will contain the FIFO results
        // B2 is highest byte and B0 is least byte  

        static constexpr uint8_t TMP_B2     = 0x03;
        static constexpr uint8_t TMP_B1     = 0x04;
        static constexpr uint8_t TMP_B0     = 0x05;
        // Same as pressure just with temp data

        static constexpr uint8_t PRS_CFG    = 0x06;
        // Pressure measurement rate and resolution
        // PRS_CFG has a rate and precision 
        // MSB is blank, then the first octet is the rate and the last byte is precision
        // OCTET rate settings:
        // 000 - 1   Hz
        // 001 - 2   Hz
        // 010 - 4   Hz
        // 011 - 8   Hz
        // 100 - 16  Hz
        // 101 - 32  Hz
        // 110 - 64  Hz
        // 111 - 128 Hz
        //
        // Precision oversampling rate
        // 0000 - 1   sample
        // 0001 - 2   samples
        // 0010 - 4   samples
        // 0011 - 8   samples
        // 0100 - 16  samples (standard)*
        // 0101 - 32  samples*
        // 0110 - 64  samples (high precision)*
        // 0111 - 128 samples*
        // 
        // * - Use in combination with a bit shift in INT and FIFO configuration
        // 7 - reserved
        // 6:4 - rate octect
        // 3:0 - oversampling rate (resolution) byte
        // Measurement times for oversampling (ms) and associated precision:
        // 1   - 3.6   - 2.5
        // 2   - 5.2   - 1
        // 4   - 8.4   - 0.5
        // 8   - 14.8  - 0.4
        // 16  - 27.6  - 0.35
        // 32  - 53.2  - 0.3
        // 64  - 104.4 - 0.2
        // 128 - 206.8

        static constexpr uint8_t TMP_CFG    = 0x07;
        // Same as pressure config with slight difference
        // 7 - TMP_EXT - Temperature measurement sensor selection: 0-internal sensor (ASIC), 1-external sensor (MEMS) use the same as what is the COEF_SRC register
        // Rest of bits are same as PRS_CFG

        static constexpr uint8_t MEAS_CFG   = 0x08;
        // Setup measurement mode, reset value C0h
        // 7 - COEF_RDY - are coefficients available: 0-no, 1-yes
        // 6 - SENSOR_RDY - pressure sensor is running through initialization: 0-initialization not complete, 1-initialization complete
        // 5 - TMP_RDY - temp measurement ready: 1-new pressure measurement ready, cleared when temp measurement is read
        // 4 - PRS_RDY - pressure measurement ready: 1-new pressure measurement ready, cleared when temp measurement is read
        // 3 - reserved
        // 2:0 - MEAS_CTRL - set measurement mode and type
        // Standby mode:
        // 000 - Idle/Stop background measurement
        // Command Mode:
        // 001 - Pressure measurement
        // 010 - Temperature measurement
        // 011 - N/A
        // 100 - N/A
        // Background mode:
        // 101 - Continuous pressure measurement
        // 110 - Continuous temperature measurement
        // 111 - Continuous pressure/temperature measurement

        static constexpr uint8_t CFG_REG    = 0x09;
        // INT & FIFO CFG_REG configuration
        // Bit - associated field - function:options
        // 7 - INT_HL - Interrupt (on SDO pin) active level: 0-active low, 1-active high
        // 6 - INT_FIFO - Generate interrupt when FIFO is full: 0-disable, 1-enable
        // 5 - INT_TMP - Generate interrupt when a temperature measurement is ready: 0-disable, 1-enable
        // 4 - INT_PRS - Generate interrupt when pressure measurement is available: 0-disable, 1-enable
        // 3 - T_SHIFT - Temperature result bit-shift: 0-no shift, 1-shift result right in data register
        // 2 - P_SHIFT - Pressure result bit-shift: 0-no shift, 1-shift result right in data register
        // 1 - FIFO_EN - Enable the FIFO: 0-disable, 1-enable
        // 0 - SPI_MODE - Set SPI mode: 0-4 wire interface, 1-3-wire interface

        static constexpr uint8_t INT_STS    = 0x0A;
        // Interrupt status register, cleared on read, 1 octet of data
        // 7:3 - reserved
        // 2 - INT_FIFO_FULL - Status of FIFO interrupt:0-interrupt not active, 1-interrupt active
        // 1 - INT_TMP - Status of temp interrupt: 0-not active, 1-active
        // 0 - INT_PRS - Status of pressure interrupt: 0-not active, 1-active
        
        static constexpr uint8_t FIFO_STS   = 0x0B;
        // FIFO Status register
        // 7:2 - reserved
        // 1 - FIFO_FULL: 0-FIFO is not full, 1-FIFO is full
        // 0 - FIFO_EMPTY: 0-FIFO is not empty, 1-FIFO is empty

        static constexpr uint8_t RESET      = 0x0C;
        // FIFO flush - soft reset
        // 7 - FIFO_FLUSH: write 1 to flush FIFO
        // 6:4 - reserved
        // 3:0 - SOFT_RST: write 1001 (09h) to generate a soft reset

        static constexpr uint8_t PRODUCT_ID = 0x0D;
        // Product and Revision ID, reset value: 0x10
        // 7:4 - Revision ID
        // 3:0 - Product ID

        static constexpr uint8_t COEF_SRCE  = 0x28;
        // States which internal temperature sensor the calibration coefficients are based on - the ASIC sensor or MEMS sensor
        // 7 - TMP_COEF_SRCE: 0-Internal temperature sensor (ASIC, 1-External temperature sensor (MEMS)
        // 6:0 - reserved

        struct PressureData {
            int32_t pressure;
            int32_t temperature;
        };

        void reset();
        bool config(uint8_t CFG_DAT, uint8_t MEAS_DAT, uint8_t PRS_DAT, uint8_t TMP_DAT);
        uint8_t whoAmI();
        void writeRegister(uint8_t reg, uint8_t data);
        uint8_t readRegister(uint8_t reg);
        void readCoefficients();
        bool checkMeasureStatus();
        PressureData rawData();
        PressureData scaledData(bool isPSI, bool isFahrenheit);
};
#endif