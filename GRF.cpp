#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
using namespace std;

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cmath>
#include <iostream>
#include <bno08x.h>
#include "utils.h"

// I2C Settings
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define BNO085_ADDR 0x4C

// Constants
const float COM_shank = 0.5726;  // Values taken from paper in Zotero
const float COM_thigh = 0.4095; 
float L_shank, L_thigh, m; // Defined by user (lower in the code)

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
}data;


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
    float theta_shank_x = data.angle_x //same as theta_IMU by property of z-corner
    float theta_shank_y = data.angle_y 
    float theta_shank_z = data.angle_z
    float theta_thigh_x = data.angle_x * M_PI / 180.0f;  // CHANGE THIS
    float theta_thigh_y = data.angle_y * M_PI / 180.0f;  // CHANGE THIS
    float theta_thigh_z = data.angle_z * M_PI / 180.0f;  // CHANGE THIS

    // Calculate length vectors for shank and thigh
    float L_shank_x, L_shank_z, L_thigh_x, L_thigh_z;
    calculate_length_vectors(theta_shank_x, theta_thigh_x, L_shank_x, L_shank_z, L_thigh_x, L_thigh_z);

    // Integrate acceleration to get velocity of the IMU
    integrate(velocity_IMU_x, velocity_IMU_y, velocity_IMU_z, data, dt);

    // Function to calculate cross product of two 3D vectores
    void crossProduct(float vect_A[3], float vect_B[3], float cross_P[3]) {
        cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
        cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
        cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    }


    // Calculate the velocity of the shank, knee, thigh and hip with cross product approximation
    velocity_shank_x = velocity_IMU_x + crossProduct(data.gyro_y, L_shank_z * COM_shank, cross_P); //check axis for cross prod
    velocity_shank_y = velocity_IMU_y + crossProduct(data.gyro_x, L_shank_z * COM_shank, cross_P); //check axis for cross prod
    velocity_shank_z = velocity_IMU_z + crossProduct(data.gyro_y, L_shank_x * COM_shank, cross_P); //check axis for cross prod

    velocity_knee_x = velocity_IMU_x + crossProduct(data.gyro_z, L_shank_x, cross_P); //check axis for cross prod
    velocity_knee_y = velocity_IMU_y + crossProduct(data.gyro_z, L_shank_x, cross_P); //check axis for cross prod
    velocity_knee_z = velocity_IMU_z + crossProduct(data.gyro_z, L_shank_z, cross_P); //check axis for cross prod

    velocity_thigh_x = velocity_knee_x + crossProduct(w_shank_z, (1-COM_thigh) * L_thigh_x, cross_P); //check axis for cross prod
    velocity_thigh_y = velocity_knee_y + crossProduct(w_shank_z, (1-COM_thigh) * L_thigh_x, cross_P); //check axis for cross prod
    velocity_thigh_z = velocity_knee_z + crossProduct(w_shank_z, (1-COM_thigh) * L_thigh_z, cross_P); //check axis for cross prod
    
    velocity_hip_x = velocity_knee_x + crossProduct(w_thigh_z, L_thigh_x, cross_P); //check axis for cross prod
    velocity_hip_y = velocity_knee_y + crossProduct(w_thigh_z, L_thigh_x, cross_P); //check axis for cross prod
    velocity_hip_z = velocity_knee_z + crossProduct(w_thigh_z, L_thigh_z, cross_P); //check axis for cross prod
}

// Function to integrate acceleration to get velocity
void integrate(float &velocity_x, float &velocity_y, float &velocity_z, const SensorData &accel_data, float dt) {
    velocity_x += accel_data.accel_x * dt;
    velocity_y += accel_data.accel_y * dt;
    velocity_z += accel_data.accel_z * dt;
}

// Function to differentiate 
void differentiate(float x, float y, float z, float pre_x, float pre_y, float pre_z,
    float dt, vector<float> &acc) {
    
    acc[0] += (pre_x - x) / dt;
    acc[1] += (pre_y - y) / dt;
    acc[2] += (pre_z - z) / dt;
}

int main() {
    i2c_inst_t* i2c_port0;
    initI2C(i2c_port0, false);

    BNO08x IMU;

    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c_port0)==false) {
        printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
        scan_i2c_bus();
        sleep_ms(1000);
    }

    IMU.enableAccelerometer(2500);
    IMU.enableGameRotationVector(2500);
    IMU.enableGyro(2500);

    // User input
    std::cin >> L_shank;
    std::cout << "Enter the Length of Thigh (L_thigh) in meters: ";
    std::cin >> L_thigh;
    std::cout << "Enter the Total Mass of the Body (m) in kg: ";
    std::cin >> m;

    float pitch = 0.0f, yaw = 0.0f;// Quaternion values for IMU
    uint8_t accAccuracy = 0; // Quaternion accuracy
    float accX = 0.0f, accY = 0.0f, accZ = 0.0f; // Accelerometer values
    float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f; // Gyroscope values
    uint8_t gyroAccuracy = 0.0f; // Gyroscope accuracy

    // Initialize velocity variables
    float velocity_shank_x = 0.0f, velocity_shank_y = 0.0f, velocity_shank_z = 0.0f;
    float velocity_thigh_x = 0.0f, velocity_thigh_y = 0.0f, velocity_thigh_z = 0.0f;
    float velocity_IMU_x = 0.0f, velocity_IMU_y = 0.0f, velocity_IMU_z = 0.0f;
    float velocity_knee_x = 0.0f, velocity_knee_y = 0.0f, velocity_knee_z = 0.0f;
    float velocity_hip_x = 0.0f, velocity_hip_y = 0.0f, velocity_hip_z = 0.0f;
    vector<float> w_thigh = {0,0,0};

    // Initialize acceleration variables
    vector<float> a_shank = {0,0,0};
    vector<float> a_hip = {0,0,0};
    vector<float> a_thigh = {0,0,0};
    vector <float> acc = {0,0,0};
    
    // Initialize pre variables (not the most efficient way but... :)
    float dt = 0.000004f;  // Time step for 250Hz (4 microseconds between calculations)
    float pre_velocity_shank_x = 0.00f;
    float pre_velocity_shank_y = 0.00f;
    float pre_velocity_shank_z = 0.00f;
    float pre_theta_thigh_x = 0.00f;
    float pre_theta_thigh_y = 0.00f;
    float pre_theta_thigh_z = 0.00f;
    float pre_velocity_thigh_x = 0.00f;
    float pre_velocity_thigh_y = 0.00f;
    float pre_velocity_thigh_z = 0.00f;
    float pre_velocity_hip_x = 0.00f;
    float pre_velocity_hip_y = 0.00f;
    float pre_velocity_hip_z = 0.00f;
    
    while (true) {
        // Read sensor data
        if (IMU.getSensorEvent() == true){
            if (IMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                pitch = IMU.getGamePitch();
                yaw = IMU.getGameYaw();
            }
    
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
                IMU.getAccel(accX, accY, accZ, accAccuracy);
            }

            if (IMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
                IMU.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
            }
            // Differentiate shank velocity to obtain a_shank 
            differentiate(velocity_shank_x, velocity_shank_y, velocity_shank_z, pre_velocity_shank_x,
                pre_velocity_shank_y, pre_velocity_shank_z, dt, a_shank);

            // Differentiate theta thigh to obtain w_thigh
            differentiate(theta_thigh_x, theta_thigh_y, theta_thigh_z, pre_theta_thigh_x,
                pre_theta_thigh_y, pre_theta_thigh_z, dt, w_thigh);
                
            // Differentiate velocity thigh to obtain a_thigh
            differentiate(velocity_thigh_x, velocity_thigh_y, velocity_thigh_z, pre_velocity_thigh_x,
                pre_velocity_thigh_y, pre_velocity_thigh_z, dt, a_thigh);

            // Differentiate velocity hip to obtain a_hip
            differentiate(velocity_hip_x, velocity_hip_y, velocity_hip_z, pre_velocity_hip_x,
                pre_velocity_hip_y, pre_velocity_hip_z, dt, a_hip);    

            pre_velocity_shank_x= velocity_shank_x;
            pre_velocity_shank_y= velocity_shank_y;
            pre_velocity_shank_z= velocity_shank_z;

            pre_theta_thigh_x= theta_thigh_x;
            pre_theta_thigh_y= theta_thigh_y;
            pre_theta_thigh_z= theta_thigh_z;

            pre_velocity_thigh_x= velocity_thigh_x;
            pre_velocity_thigh_y= velocity_thigh_y;
            pre_velocity_thigh_z= velocity_thigh_z;

            pre_velocity_hip_x= velocity_hip_x;
            pre_velocity_hip_y= velocity_hip_y;
            pre_velocity_hip_z= velocity_hip_z;    
        }

        // Calculate velocities for shank and thigh
        calculate_kinematics(data, velocity_shank_x, velocity_shank_y, velocity_shank_z, 
                           velocity_thigh_x, velocity_thigh_y, velocity_thigh_z, velocity_IMU_x, 
                           velocity_IMU_y, velocity_IMU_z, dt);
        // Check GRF function
        // Function to calculate GRF
        void calculate_grf(float m_shank, float m_thigh, float a_shank, float a_thigh, 
            float a_hip, float g) {
        float F_shank = m_shank * (a_shank - g);
        float F_thigh = m_thigh * (a_thigh - g);
        float F_hip = m_hip * (a_hip - g)        
        float F_ground = F_shank + F_thigh + F_hip;
    
        std::cout << "GRF: " << F_ground << " N" << std::endl;
        }         
        // Calculate GRF
        calculate_grf(m * COM_shank, m * COM_thigh, a_shank, a_thigh, a_hip, 9.81f);  // Using 9.81m/s^2 for gravity

        sleep_ms(4);  // Sleep 4 mili sec until next sample to be taken
    }

    return 0;
}

//Make sure pins match datasheet
//Calibration?
//Axis in cross product and variables in general