#include "dps_310.hpp"

DPS310::DPS310(int CS, spi_inst_t* SPI)
{
    CS_PIN = CS;
    SPI_PORT = SPI;
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1); // Set CS high initially
    sleep_ms(100);
    readCoefficients();
}

uint8_t DPS310::whoAmI()
{
    uint8_t data;
    gpio_put(CS_PIN, 0);
    spi_write_blocking(SPI_PORT, &PRODUCT_ID, 1);
    spi_read_blocking(SPI_PORT, 0x00, &data, 1);
    gpio_put(CS_PIN, 1);
    return data;
}

void DPS310::writeRegister(uint8_t reg, uint8_t data)
{
    uint8_t data_to_send[] = {reg | 0x00, data};
    gpio_put(CS_PIN, 0);
    spi_write_blocking(SPI_PORT, data_to_send, sizeof(data_to_send));
    gpio_put(CS_PIN, 1);
}

uint8_t DPS310::readRegister(uint8_t reg)
{
    uint8_t buffer[] = {reg | 0x80};
    uint8_t data;
    gpio_put(CS_PIN,0);
    spi_write_blocking(SPI_PORT, buffer, sizeof(buffer));
    spi_read_blocking(SPI_PORT, 0x00, &data, 1);
    gpio_put(CS_PIN, 1);
    return data;
}

// Perform soft reset of sensor
void DPS310::reset()
{
    writeRegister(RESET, 0x09);
}

bool DPS310::config(uint8_t CFG_DAT, uint8_t MEAS_DAT, uint8_t PRS_DAT, uint8_t TMP_DAT)
{
    // Check for bitshifting and set scaling factor based on oversample rate
    uint8_t NEW_CFG;
    switch (PRS_DAT | 0xF0){
        case 0xF0:
            NEW_CFG = CFG_DAT;
            kP = 524288.0f;
            break;
        case 0xF1:
            NEW_CFG = CFG_DAT;
            kP = 1572864.0f;
            break;
        case 0xF2:
            NEW_CFG = CFG_DAT;
            kP = 3670016.0f;
            break;
        case 0xF3:
            NEW_CFG = CFG_DAT;
            kP = 7864320.0f;
            break;
        case 0xF4:
            bitShiftPress = true;
            NEW_CFG = CFG_DAT | 0x04;
            kP = 253952.0f;
            break;
        case 0xF5:
            bitShiftPress = true;
            NEW_CFG = CFG_DAT | 0x04;
            kP = 516096.0f;
            break;
        case 0xF6:
            bitShiftPress = true;
            NEW_CFG = CFG_DAT | 0x04;
            kP = 1040384.0f;
            break;
        case 0xF7:
            bitShiftPress = true;
            NEW_CFG = CFG_DAT | 0x04;
            kP = 2088960.0f;
            break;
    }

    switch (TMP_DAT | 0xF0){
        case 0xF0:
            kT = 524288.0f;
            break;
        case 0xF1:
            kT = 1572864.0f;
            break;
        case 0xF2:
            kT = 3670016.0f;
            break;
        case 0xF3:
            kT = 7864320.0f;
            break;
        case 0xF4:
            bitShiftTemp = true;
            NEW_CFG = NEW_CFG | 0x08;
            kT = 253952.0f;
            break;
        case 0xF5:
            bitShiftTemp = true;
            NEW_CFG = NEW_CFG | 0x08;
            kT = 516096.0f;
            break;
        case 0xF6:
            bitShiftTemp = true;
            NEW_CFG = NEW_CFG | 0x08;
            kT = 1040384.0f;
            break;
        case 0xF7:
            bitShiftTemp = true;
            NEW_CFG = NEW_CFG | 0x08;
            kT = 2088960.0f;
            break;
    }

    // Soft reset
    reset();

    // read PRODUCT_ID and verify product works
    uint8_t PID = whoAmI();
    if (PID!=0x10)
    {
        std::cerr << "Reset value in Product ID not found, register reads: " << std::hex << PID << std::endl;
        return false;
    }

    // Set to standby to verify writing of other config variables
    writeRegister(MEAS_CFG, 0x00);
    sleep_ms(10);

    // set CFG_REG
    writeRegister(CFG_REG, NEW_CFG);
    sleep_ms(10);
    uint8_t cfgVal = readRegister(CFG_REG);
    if (cfgVal != CFG_DAT)
    {
        std::cerr << "Set CFG_REG value not found, register reads: " << cfgVal << std::endl;
        return false;
    }

    // set PRS_CFG
    DPS310::writeRegister(PRS_CFG, PRS_DAT);
    sleep_ms(10);
    uint8_t prsVal = readRegister(PRS_CFG);
    if (prsVal != PRS_DAT)
    {
        std::cerr << "Set PRS_CFG value not found, register reads: " << prsVal << std::endl;
        return false;
    }

    // read COEF_SRCE and set MEMS if MSB=1 or ASIC if MSB=0
    if ((readRegister(COEF_SRCE)|0x7f) == 0xff)
    {
        uint8_t tmpData = (0x80 | TMP_DAT);
        writeRegister(TMP_CFG, tmpData);
        sleep_ms(10);
        uint8_t tmpVal = readRegister(TMP_CFG);
        if(tmpVal != tmpData)
        {
            std::cerr << "Set TMP CFG value not found, register reads: " << tmpVal << std::endl;
            return false;
        }
    }
    else {
        // Set ASIC temp
        writeRegister(TMP_CFG, TMP_DAT);
        sleep_ms(2);
        uint8_t tmpVal2 = readRegister(TMP_CFG);
        if (tmpVal2 != TMP_DAT)
        {
            std::cerr << "Set TMP_CFG value not found, register reads: " << tmpVal2 << std::endl;
            return false;
        }
    }

    // set MEAS_CFG
    writeRegister(MEAS_CFG, MEAS_DAT);
    sleep_ms(10);
    uint8_t measVal = readRegister(MEAS_CFG);
    if (measVal != MEAS_DAT)
    {
        std::cerr << "Set MEAS_CFG value not found, register reads: " << measVal << std::endl;
        return false;
    }
    return true;
}

bool DPS310::checkMeasureStatus()
{
    // Check if data is ready
    uint8_t status = readRegister(MEAS_CFG);
    if (!(status & 0x10)){
        // Check PRS_RDY (bit 4)
        std::cerr << "Pressure data not ready!" << std::endl;
        return false;
    }
    if (!(status & 0x20)){   
        // Check TMP_RDY (bit 5)
        std::cerr << "Temperature data not ready!" << std::endl;
        return false;
    }
    return true;
}


DPS310::PressureData DPS310::rawData()
{
    PressureData data; // Initialize pressure and temperature to 0

    if (!checkMeasureStatus())
    {
        sleep_ms(70);
    }

    // Read pressure data (24-bit two's complement)
    uint32_t rawPressure = (readRegister(PSR_B2) << 16) |
                           (readRegister(PSR_B1) << 8) |
                           (readRegister(PSR_B0));
    if (rawPressure & 0x800000) { // Sign extension for 24-bit
        rawPressure |= 0xFF000000;
    }

    // Check if bit-shifting is needed
    if (bitShiftPress) {
        rawPressure >>= 2; // Shift raw pressure right by 2 bits
    }

    data.pressure = static_cast<int32_t>(rawPressure); // Convert to signed 32-bit integer

    // Read temperature data (24-bit two's complement)
    uint32_t rawTemperature = (readRegister(TMP_B2) << 16) |
                              (readRegister(TMP_B1) << 8)  |
                              (readRegister(TMP_B0));
    if (rawTemperature & 0x800000) { // Sign extension for 24-bit
        rawTemperature |= 0xFF000000;
    }

    // Check for bit shifting
    if (bitShiftTemp) {
        rawTemperature >>= 2; // Shift temp data by 2
    }
    data.temperature = static_cast<int32_t>(rawTemperature); // Convert to signed 32-bit integer

    return data;
}

DPS310::PressureData DPS310::scaledData(bool isPSI, bool isFahrenheit) {
    // Read raw pressure and temperature data
    PressureData rawDat = rawData();

    float Traw_sc = rawDat.temperature / kT;
    float Tcomp = c0*0.5f+c1*Traw_sc;

    // Convert raw pressure to scaled pressure
    float Praw_sc = rawDat.pressure / kP;
    float Pcomp = c00 +
                  Praw_sc*(c10+Praw_sc*(c20+Praw_sc*c30))+ Traw_sc*c01+Traw_sc*Praw_sc*(c11+Praw_sc*c21);

    // Convert temperature to Fahrenheit if requested
    if (isFahrenheit) {
        Tcomp = Tcomp*(9.0f/5.0f)+32.0f;
    }

    // Convert pressure to PSI if requested
    if (isPSI) {
        Pcomp /= 6894.757f; // Convert Pascal to PSI (1 PSI = 6894.757 Pa)
    }

    // Return the compensated and scaled data
    PressureData scaledData;
    scaledData.pressure = static_cast<int32_t>(Pcomp); // Pressure in Pa or PSI
    scaledData.temperature = static_cast<int32_t>(Tcomp); // Temperature in °C or °F
    return scaledData;
}

void DPS310::readCoefficients() {
    // Read raw coefficient data from registers
    sleep_ms(50);
    uint8_t coefData[18];
    for (uint8_t i = 0; i < 18; ++i) {
        coefData[i] = readRegister(0x10 + i);
    }

    // Parse and combine coefficients based on Table 18
    c0 = static_cast<int16_t>((coefData[0] << 4) | (coefData[1] >> 4));
    if (c0 & 0x0800) c0 |= 0xF000; // Sign extension for 12-bit

    c1 = static_cast<int16_t>(((coefData[1] & 0x0F) << 8) | coefData[2]);
    if (c1 & 0x0800) c1 |= 0xF000; // Sign extension for 12-bit

    c00 = static_cast<int32_t>((coefData[3] << 12) | (coefData[4] << 4) | (coefData[5] >> 4));
    if (c00 & 0x80000) c00 |= 0xFFF00000; // Sign extension for 20-bit

    c10 = static_cast<int32_t>(((coefData[5] & 0x0F) << 16) | (coefData[6] << 8) | coefData[7]);
    if (c10 & 0x80000) c10 |= 0xFFF00000; // Sign extension for 20-bit

    c01 = static_cast<int16_t>((coefData[8] << 8) | coefData[9]);
    c11 = static_cast<int16_t>((coefData[10] << 8) | coefData[11]);
    c20 = static_cast<int16_t>((coefData[12] << 8) | coefData[13]);
    c21 = static_cast<int16_t>((coefData[14] << 8) | coefData[15]);
    c30 = static_cast<int16_t>((coefData[16] << 8) | coefData[17]);

    std::cout << "Coefficients:\n" << std::hex << c0 << std::endl
                                  << std::hex << c1 << std::endl
                                  << std::hex << c00 << std::endl
                                  << std::hex << c10 << std::endl
                                  << std::hex << c01 << std::endl
                                  << std::hex << c11 << std::endl
                                  << std::hex << c20 << std::endl
                                  << std::hex << c21 << std::endl
                                  << std::hex << c30 << std::endl;
    sleep_ms(50);
}
