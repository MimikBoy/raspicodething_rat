#include <stdio.h>
#include <math.h>
#include <iostream>
//#include <vector>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
using namespace std;

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cmath>
#include <iostream>

// I2C Settings
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define BNO085_ADDR 0x4C

// Constants
const float COM_shank = 0.5726;  // Values taken from paper in Zotero
const float COM_thigh = 0.4095; 

struct SensorData {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float angle_x;
    float angle_y;
    float angle_z;
};

// Initialize I2C
void init_i2c() {
    i2c_init(I2C_PORT, 400000);  // 400kHz clock
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);  // Enable pull-up on SDA
    gpio_pull_up(I2C_SCL_PIN);  // Enable pull-up on SCL
}

// Function to read 16-bit signed data from IMU 
int16_t read_sensor_data(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, BNO085_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BNO085_ADDR, buf, 2, false);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

// Read accelerometer, gyroscope, and angle data (make sure we can read the angle from IMU)
SensorData read_sensor() {
    SensorData data;
    
    // Read acceleration data
    data.accel_x = read_sensor_data(0x08) * 0.001f;
    data.accel_y = read_sensor_data(0x0A) * 0.001f;
    data.accel_z = read_sensor_data(0x0C) * 0.001f;

    // Read gyroscope data
    data.gyro_x = read_sensor_data(0x14) * 0.001f;
    data.gyro_y = read_sensor_data(0x16) * 0.001f;
    data.gyro_z = read_sensor_data(0x18) * 0.001f;

    // Read angle
    data.angle_x = read_sensor_data(0x1A) * 0.1f;  // Assuming 0.1 degree per unit Not sure how much we need
    data.angle_y = read_sensor_data(0x1C) * 0.1f;
    data.angle_z = read_sensor_data(0x1E) * 0.1f;

    return data;
}

// Function to compute the length vectors for shank and thigh
void calculate_length_vectors(float theta_shank, float L_shank_y, float L_thigh_y, float theta_thigh, float &L_shank_x, float &L_shank_z, float &L_thigh_x, float &L_thigh_z) {
    // Shank Length Vector
    L_shank_x = L_shank * sin(theta_shank);
    L_shank_y=0;
    L_shank_z = L_shank * cos(theta_shank);

    // Thigh Length Vector
    L_thigh_x = L_thigh * sin(theta_thigh);
    L_thigh_y=0;
    L_thigh_z = L_thigh * cos(theta_thigh);
}

// Function to compute the velocities of shank, knee, thigh, hip
void calculate_kinematics(SensorData &data, float &velocity_shank_x, float &velocity_shank_y, float &velocity_shank_z,
                        float &velocity_thigh_x, float &velocity_thigh_y, float &velocity_thigh_z, float velocity_knee_x, 
                        float velocity_knee_y, float velocity_knee_z, float velocity_hip_x, float velocity_hip_y, 
                        float velocity_hip_z, float velocity_IMU_x, float velocity_IMU_y, float velocity_IMU_z,  float dt) {

    // Use direct IMU angle for shank and thigh
    float theta_shank = data.angle_x //same as theta_IMU by property of z-corner
    float theta_thigh = data.angle_y * M_PI / 180.0f;  // CHANGE THIS

    // Calculate length vectors for shank and thigh
    float L_shank_x, L_shank_z, L_thigh_x, L_thigh_z;
    calculate_length_vectors(theta_shank, theta_thigh, L_shank_x, L_shank_z, L_thigh_x, L_thigh_z);

    // Integrate acceleration to velocity for shank and thigh
    integrate_acceleration(velocity_shank_x, velocity_shank_y, velocity_shank_z, data, dt);

    // Calculate the velocity of the shank, knee, thigh with cross product approximation
    velocity_shank_x = velocity_IMU_x + data.gyro_z * L_shank_x * COM_shank;
    velocity_shank_y = velocity_IMU_y + data.gyro_z * L_shank_x * COM_shank;
    velocity_shank_z = velocity_IMU_z + data.gyro_z * L_shank_z * COM_shank; 

    velocity_knee_x = velocity_shank_x + data.gyro_z * L_shank_x;
    velocity_knee_y = velocity_shank_y + data.gyro_z * L_shank_x;
    velocity_knee_z = velocity_shank_z + data.gyro_z * L_shank_z;

    velocity_thigh_x = velocity_knee_x + w_shank_z * (1-COM_thigh) * L_thigh_x;
    velocity_thigh_y = velocity_knee_y + w_shank_z * (1-COM_thigh) * L_thigh_x;
    velocity_thigh_z = velocity_knee_z + w_shank_z * (1-COM_thigh) * L_thigh_z; 
    
    velocity_hip_x = velocity_knee_x + w_shank_z * L_thigh_x;
    velocity_hip_y = velocity_knee_y + w_shank_z * L_thigh_x;
    velocity_hip_z = velocity_knee_z + w_shank_z * L_thigh_z; 
}

// Function to integrate acceleration to get velocity
void integrate_acceleration(float &velocity_x, float &velocity_y, float &velocity_z, const SensorData &accel_data, float dt) {
    velocity_x += accel_data.accel_x * dt;
    velocity_y += accel_data.accel_y * dt;
    velocity_z += accel_data.accel_z * dt;
}

// Function to compute Ground Reaction Force
void calculate_grf(float m_shank, float m_thigh, float a_shank, float a_thigh, float g) {
    float F_shank = m_shank * (a_shank - g);
    float F_thigh = m_thigh * (a_thigh - g);
    float F_ground = F_shank + F_thigh;
    
    std::cout << "GRF: " << F_ground << " N" << std::endl;
}

int main() {
    init_i2c();  // Initialize I2C communication

    // User input
    float L_shank, L_thigh, m;
    std::cin >> L_shank;
    std::cout << "Enter the Length of Thigh (L_thigh) in meters: ";
    std::cin >> L_thigh;
    std::cout << "Enter the Total Mass of the Body (m) in kg: ";
    std::cin >> m;

    // Initialize velocity variables
    float velocity_shank_x = 0.0f, velocity_shank_y = 0.0f, velocity_shank_z = 0.0f;
    float velocity_thigh_x = 0.0f, velocity_thigh_y = 0.0f, velocity_thigh_z = 0.0f;
    float velocity_IMU_x = 0.0f, velocity_IMU_y = 0.0f, velocity_IMU_z = 0.0f;
    float velocity_knee_x = 0.0f, velocity_knee_y = 0.0f, velocity_knee_z = 0.0f;

    float dt = 0.000004f;  // Time step for 250Hz (4 microseconds between calculations)

    while (true) {
        // Read sensor data
        SensorData data = read_sensor();

        // Calculate velocities for shank and thigh
        calculate_kinematics(data, velocity_shank_x, velocity_shank_y, velocity_shank_z, 
                           velocity_thigh_x, velocity_thigh_y, velocity_thigh_z, velocity_IMU_x, 
                           velocity_IMU_y, velocity_IMU_z, dt);

        // Calculate GRF
        float a_shank = sqrt(velocity_shank_x * velocity_shank_x + velocity_shank_y * velocity_shank_y);
        float a_thigh = sqrt(velocity_thigh_x * velocity_thigh_x + velocity_thigh_y * velocity_thigh_y);
        calculate_grf(m * 0.057f, m * 0.1416f, a_shank, a_thigh, 9.81f);  // Using 9.81m/s^2 for gravity

        sleep_ms(4);  // Sleep 4 mili sec until next sample to be taken
    }

    return 0;
}
// Function to calculate velocity of the shank
void(float v_IMU, float w_IMU, const COM_shank, float L_shank){
    v_shank
}

//a_shank, a_thigh, v_hip, a_hip, v_shank, v_knee, w_thigh, v_thigh 
//Make sure pins match datasheet
//how much sleep time between samples? 
//not sure how to write the communication between raspberry pies here to do the sum
//how to match time instants between raspberry pies
//COM creo que no es un input sino que lo definimos nosotros? 
//Assuming 0.1 degree per unit Not sure how much we need