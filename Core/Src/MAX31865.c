/*
 * MAX31865.C
 *
 *  Created on: Sep 12, 2024
 *      Author: DuttEngi
 */



#include "MAX31865.h"

extern SPI_HandleTypeDef hspi1;
bool initialized = false;


/**
 * @brief Write an 8-bit value to the specified register.
 * @param addr Register address.
 * @param data Data to write.
 */
void writeRegister8(uint8_t addr, uint8_t data) {
    addr |= 0x80; // MSB=1 for write, make sure top bit is set

    uint8_t buffer[2] = {addr, data};

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin, GPIO_PIN_SET);
}
/**
 * @brief Read multiple bytes from the specified register.
 * @param addr Register address.
 * @param buffer Buffer to store the read data.
 * @param n Number of bytes to read.
 */
void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {
    addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, n, SPI_DELAY);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Read an 8-bit value from the specified register.
 * @param addr Register address.
 * @return 8-bit value read from the register.
 */
uint8_t readRegister8(uint8_t addr) {
    uint8_t ret = 0;
    readRegisterN(addr, &ret, 1);
    return ret;
}

/**
 * @brief Read a 16-bit value from the specified register.
 * @param addr Register address.
 * @return 16-bit value read from the register.
 */
uint16_t readRegister16(uint8_t addr) {
    uint8_t buffer[2] = {0, 0};
    readRegisterN(addr, buffer, 2);

    uint16_t ret = (uint16_t)buffer[0]; // Cast to uint16_t before left shift
    ret <<= 8;
    ret |= buffer[1];

    return ret;
}

/**
 * @brief Read a 24-bit value from the specified register.
 * @param addr Register address.
 * @return 24-bit value read from the register.
 */
uint32_t readRegister24(uint8_t addr) {
    uint8_t buffer[3] = {0, 0, 0};
    readRegisterN(addr, buffer, 3);

    uint32_t ret = (uint32_t)buffer[0];
    ret <<= 8;
    ret |= buffer[1];
    ret <<= 8;
    ret |= buffer[2];

    return ret;
}
/**
 * @brief Set the number of wires for the RTD configuration.
 * @param wires Number of wires configuration (2, 3, or 4-wire).
 */
void setWires(max31865_numwires_t wires) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (wires == MAX31865_3WIRE) {
        t |= MAX31865_CONFIG_3WIRE;
    } else {
        t &= ~MAX31865_CONFIG_3WIRE;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}
/**
 * @brief Enable or disable the bias voltage on the RTD sensor.
 * @param b True to enable, false to disable.
 */
void enableBias(bool b) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
        t |= MAX31865_CONFIG_BIAS; // Enable bias
    } else {
        t &= ~MAX31865_CONFIG_BIAS; // Disable bias
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

/**
 * @brief Enable or disable automatic conversion mode.
 * @param b True to enable, false to disable.
 */
void autoConvert(bool b) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
        t |= MAX31865_CONFIG_MODEAUTO; // Enable auto convert
    } else {
        t &= ~MAX31865_CONFIG_MODEAUTO; // Disable auto convert
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

/**
 * @brief Enable or disable 50Hz filter.
 * @param b True to enable, false to disable.
 */
void enable50Hz(bool b) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
        t |= MAX31865_CONFIG_FILT50HZ;
    } else {
        t &= ~MAX31865_CONFIG_FILT50HZ;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

/**
 * @brief Set the high and low fault thresholds.
 * @param lower Lower threshold.
 * @param upper Upper threshold.
 */
void setThresholds(uint16_t lower, uint16_t upper) {
    writeRegister8(MAX31865_LFAULTLSB_REG, lower & 0xFF);
    writeRegister8(MAX31865_LFAULTMSB_REG, lower >> 8);
    writeRegister8(MAX31865_HFAULTLSB_REG, upper & 0xFF);
    writeRegister8(MAX31865_HFAULTMSB_REG, upper >> 8);
}

void setTemperatureThresholds() {
    // Use the predefined constants for -50°C and 400°C
    setThresholds(LOWER_THRESHOLD_REG_VALUE, UPPER_THRESHOLD_REG_VALUE);
}
/**
 * @brief Get the lower fault threshold.
 * @return Lower fault threshold.
 */
//uint16_t getLowerThreshold(void) {
//    return readRegister16(MAX31865_LFAULTMSB_REG);
//}

/**
 * @brief Get the upper fault threshold.
 * @return Upper fault threshold.
 */
//uint16_t getUpperThreshold(void) {
//    return readRegister16(MAX31865_HFAULTMSB_REG);
//}
/**
 * @brief Initialize the MAX31865 device.
 * @param wires Number of wires configuration (2, 3, or 4-wire).
 * @return True if initialization is successful, otherwise false.
 */
bool begin(max31865_numwires_t wires) {
    if (HAL_SPI_Init(&hspi1) == HAL_OK) {
        initialized = true;
    } else {
        initialized = false;
    }

    setWires(wires);
    enableBias(false);
    autoConvert(false);
    setThresholds(0, 0xFFFF);
    clearFault();

    return initialized;
}

/**
 * @brief Read the RTD resistance value.
 * @return Raw RTD resistance value.
 */
uint16_t readRTD(void) {
    clearFault();
    enableBias(true);
    HAL_Delay(10);

    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, t);
    HAL_Delay(65);

    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);
    enableBias(false); // Disable bias current to reduce self-heating.

    rtd >>= 1; // Remove fault bit

    return rtd;
}

/**
 * @brief Calculate the temperature from the RTD resistance.
 * @param RTDnominal Nominal resistance of the RTD at 0°C.
 * @param refResistor Reference resistance value.
 * @return Calculated temperature.
 */
float temperature(float RTDnominal, float refResistor) {
    return calculateTemperature(readRTD(), RTDnominal, refResistor);
}

/**
 * @brief Calculate the temperature from the raw RTD resistance value.
 * @param RTDraw Raw RTD resistance value.
 * @param RTDnominal Nominal resistance of the RTD at 0°C.
 * @param refResistor Reference resistance value.
 * @return Calculated temperature.
 */
float calculateTemperature(uint16_t RTDraw, float RTDnominal, float refResistor) {
    float Z1, Z2, Z3, Z4, Rt, temp;

    Rt = RTDraw;
    Rt /= 32768;
    Rt *= refResistor;

    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / RTDnominal;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;

    if (temp >= 0) return temp;

    Rt /= RTDnominal;
    Rt *= 100; // Normalize to 100 ohm

    float rpoly = Rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt; // Square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt; // ^5
    temp += 1.5243e-10 * rpoly;

    return temp;
}

/**
 * @brief Read fault status from the MAX31865.
 * @param fault_cycle Fault detection cycle mode.
 * @return Fault status register value.
 */
uint8_t readFault(max31865_fault_cycle_t fault_cycle) {
    if (fault_cycle) {
        uint8_t cfg_reg = readRegister8(MAX31865_CONFIG_REG);
        cfg_reg &= 0x11; // Mask out wire and filter bits
        switch (fault_cycle) {
        case MAX31865_FAULT_AUTO:
            writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10000100));
            HAL_Delay(1);
            break;
        case MAX31865_FAULT_MANUAL_RUN:
            writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001000));
            return 0;
        case MAX31865_FAULT_MANUAL_FINISH:
            writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001100));
            return 0;
        case MAX31865_FAULT_NONE:
        default:
            break;
        }
    }
    return readRegister8(MAX31865_FAULTSTAT_REG);
}

/**
 * @brief Clear fault status on the MAX31865.
 */
void clearFault(void) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t &= ~0x2C;
    t |= MAX31865_CONFIG_FAULTSTAT;
    writeRegister8(MAX31865_CONFIG_REG, t);
}
