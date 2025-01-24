#include "dps_310.hpp"
#include <iostream>

DPS310::DPS310(int CS, int MOSI, int MISO, int SCLK, spi_inst_t* SPI)
{
    CS_PIN = CS;
    SPI_PORT = SPI;
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1); // Set CS high initially

    // SPI0 interface initialization
    spi_init(SPI_PORT, 1000000); // Initialize spi0 at 1MHz
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    spi_set_format( SPI_PORT,       // SPI instance
                    8,              //Number of bits per transfer
                    SPI_CPOL_1,     // Polarity (CPOL)
                    SPI_CPHA_1,     // Phase (CPHA)
                    SPI_MSB_FIRST);
}

uint8_t DPS310::whoAmI()
{
    uint8_t data;
    gpio_put(CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &PRODUCT_ID, 1);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(CS_PIN, 1);
    return data;
}

void DPS310::writeRegister(uint8_t reg, uint8_t data)
{
    gpio_put(CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_write_blocking(SPI_PORT, &data, 1);
    gpio_put(CS_PIN, 1);
}

uint8_t DPS310::readRegister(uint8_t reg)
{
    uint8_t data;
    gpio_put(CS_PIN,0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(CS_PIN, 1);
    return data;
}

// Perform soft reset of sensor
void DPS310::reset()
{
    writeRegister(RESET, 0x09);
}

// NOTE** Include verification of registers after base function
void DPS310::config(uint8_t CFG_DAT, uint8_t MEAS_DAT, uint8_t PRS_DAT, uint8_t TMP_DAT)
{
    // read PRODUCT_ID and verify product works
    if (readRegister(PRODUCT_ID)!=0x10)
    {}
    // set CFG_REG
    writeRegister(CFG_REG, CFG_DAT);
    if (DPS310::readRegister(CFG_REG) != CFG_DAT)
    {}
    // set MEAS_CFG
    writeRegister(MEAS_CFG, MEAS_DAT);
    if (readRegister(MEAS_CFG) != MEAS_DAT)
    {}
    // set PRS_CFG
    DPS310::writeRegister(PRS_CFG, PRS_DAT);
    if (readRegister(PRS_CFG) != PRS_DAT)
    {}
    // read COEF_SRCE and set MEMS if MSB=1 or ASIC if MSB=0
    if ((readRegister(COEF_SRCE)|0x7f) == 0xff)
    {
        uint8_t tmpData = (0x80 | TMP_DAT);
        writeRegister(TMP_CFG, tmpData);
        if(readRegister(TMP_CFG) != tmpData)
        {}
    }
    else {
        // Set ASIC temp
        writeRegister(TMP_CFG, TMP_DAT);
        if (readRegister(TMP_CFG) != TMP_DAT)
        {}
    }
}

DPS310::PressureData DPS310::readData()
{}