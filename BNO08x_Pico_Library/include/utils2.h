#ifndef UTILS_H_2
#define UTILS_H_2

#include <cstdio>
#include <math.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

namespace CONFIG_2 {

    inline uint I2C_TIMEOUT_US = 5000;

    constexpr uint I2C_BAUD_RATE = 350 * 1000; // 350kHz
    constexpr uint I2C_SDA_PIN = 2; // GP4 pin 6;
    constexpr uint I2C_SCL_PIN = 3; // GP5 pin 7;

    constexpr uint8_t BNO08x2_ADDR = 0x4A;

}

void initI2C2(i2c_inst_t* &i2c_port, bool force_recovery);

void handleI2CError2(i2c_inst_t* &i2c_port);

void i2cBusRecovery2(uint sda_pin, uint scl_pin);

bool reserved_addr2(uint8_t addr);

void scan_i2c_bus2();

bool non_blocking_delay_us2(uint32_t delay_us);

#endif // UTILS_H
