/*
 * MAX31865.h
 *
 *  Created on: Sep 12, 2024
 *      Author: DuttEngi
 */

#ifndef INC_MAX31865_H_
#define INC_MAX31865_H_

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// MAX31865 register addresses and configuration bit masks
#define MAX31865_CONFIG_REG       0x00
#define MAX31865_CONFIG_BIAS      0x80
#define MAX31865_CONFIG_MODEAUTO  0x40
#define MAX31865_CONFIG_MODEOFF   0x00
#define MAX31865_CONFIG_1SHOT     0x20
#define MAX31865_CONFIG_3WIRE     0x10
#define MAX31865_CONFIG_24WIRE    0x00
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ  0x01
#define MAX31865_CONFIG_FILT60HZ  0x00

#define MAX31865_RTDMSB_REG       0x01
#define MAX31865_RTDLSB_REG       0x02
#define MAX31865_HFAULTMSB_REG    0x03
#define MAX31865_HFAULTLSB_REG    0x04
#define MAX31865_LFAULTMSB_REG    0x05
#define MAX31865_LFAULTLSB_REG    0x06
#define MAX31865_FAULTSTAT_REG    0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH  0x40
#define MAX31865_FAULT_REFINLOW   0x20
#define MAX31865_FAULT_REFINHIGH  0x10
#define MAX31865_FAULT_RTDINLOW   0x08
#define MAX31865_FAULT_OVUV       0x04

// Constants for the RTD Data Register values corresponding to the thresholds
#define LOWER_THRESHOLD_REG_VALUE 0x3366  // -50°C corresponds to 13158 in decimal (3366h)
#define UPPER_THRESHOLD_REG_VALUE 0x9E24  // 400°C corresponds to 40484 in decimal (9E24h)

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#define SPI_DELAY 0xFF
extern bool initialized;

#define RREF      400.0
#define RNOMINAL  100.0

// Number of wires configuration for the RTD sensor
typedef enum max31865_numwires {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE = 1,
    MAX31865_4WIRE = 0
} max31865_numwires_t;


// Fault detection cycle mode
typedef enum max31865_fault_cycle{
    MAX31865_FAULT_NONE = 0,
    MAX31865_FAULT_AUTO,
    MAX31865_FAULT_MANUAL_RUN,
    MAX31865_FAULT_MANUAL_FINISH
} max31865_fault_cycle_t;



// Function prototypes
bool begin(max31865_numwires_t wires);
uint8_t  readFault(max31865_fault_cycle_t fault_cycle);
uint8_t  readRegister8(uint8_t addr);
uint16_t readRegister16(uint8_t addr);
uint16_t readRTD(void);
//uint16_t getLowerThreshold(void);
//uint16_t getUpperThreshold(void);
void setTemperatureThresholds();

void clearFault(void);
void setThresholds(uint16_t lower, uint16_t upper);
void setWires(max31865_numwires_t wires);
void autoConvert(bool b);
void enable50Hz(bool b);
void enableBias(bool b);
void writeRegister8(uint8_t addr, uint8_t reg);
void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);

float temperature(float RTDnominal, float refResistor);
float calculateTemperature(uint16_t RTDraw, float RTDnominal, float refResistor);



#endif /* INC_MAX31865_H_ */
