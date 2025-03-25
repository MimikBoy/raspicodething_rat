#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

//Readout sensor data (IMUs and Honeycomb, >250Hz)
//Check that the pins are the correct ones we want to use
#define BNO085_ADDR 0x4A  // Default I2C address for BNO085
#define BNO085_REG_OPR_MODE 0x3D
#define BNO085_REG_PWR_MODE 0x3E
#define BNO085_REG_EULER_H 0x1A

// Initialize I2C
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define I2C_BAUD 400000  // 400 kHz I2C speed

// Function to initialize I2C
void init_i2c() {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Function to write data to a register
void write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, BNO085_ADDR, data, sizeof(data), false);
}

// Function to read a byte from a register
uint8_t read_register(uint8_t reg) {
    uint8_t value;
    i2c_write_blocking(I2C_PORT, BNO085_ADDR, &reg, 1, true);  // Write register address
    i2c_read_blocking(I2C_PORT, BNO085_ADDR, &value, 1, false); // Read data
    return value;
}

// Function to read Euler angles (Yaw, Pitch, Roll)
void read_euler_angles(float &yaw, float &pitch, float &roll) {
    yaw = read_register(BNO085_REG_EULER_H) / 16.0;
    pitch = read_register(BNO085_REG_EULER_H + 1) / 16.0;
    roll = read_register(BNO085_REG_EULER_H + 2) / 16.0;
}

// Function to initialize the BNO085 sensor
void init_bno085() {
    // Set operation mode to NDOF (9-DOF)
    write_register(BNO085_REG_OPR_MODE, 0x0C);  // NDOF mode
    // Set power mode to NORMAL
    write_register(BNO085_REG_PWR_MODE, 0x00);  // Normal power mode
}

int main() {
    // Initialize stdio for USB serial output
    stdio_init_all();
    
    // Initialize I2C
    init_i2c();
    
    // Initialize BNO085 sensor
    init_bno085();

    // Read Euler angles and print them
    float yaw, pitch, roll;

    while (true) {
        // Read Euler angles (Yaw, Pitch, Roll)
        read_euler_angles(yaw, pitch, roll);
        
        // Print Euler angles (in degrees)
        printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n", yaw, pitch, roll);
        
        // Delay before the next reading
        sleep_ms(100);  // 10 Hz update rate
    }
}

//Input of runners mass, shank length, thigh length
typedef struct {
    double mass;
    double shank_length;
    double thigh_length;
} Runner


//Transforming sensor data (accelerometer xyz and gyrometer xyz) to good inputs for the math

//Math 
int N = w_shank.size();
double integrate_theeta(w_shank,theeta_initial, deltatime)
    for (int i= 0; i<N-1; i++)
        0.5*(w_shank(i)+w_shank(i+1))*deltatime
        
    end    

