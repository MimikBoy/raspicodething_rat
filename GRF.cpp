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
void calculate_kinematics(SensorData &data, float &v_shank_x, float &v_shank_y, float &v_shank_z,
                        float &v_thigh_x, float &v_thigh_y, float &v_thigh_z, float v_knee_x, 
                        float v_knee_y, float v_knee_z, float v_hip_x, float v_hip_y, 
                        float v_hip_z, float v_IMU_x, float v_IMU_y, float v_IMU_z,  float dt) {

    // Use direct IMU angle for shank and thigh
    float theta_shank_x = data.angle_x; //same as theta_IMU by property of z-corner
    float theta_shank_y = data.angle_y;
    float theta_shank_z = data.angle_z;
    float theta_thigh_x = data.angle_x;  // CHANGE THIS
    float theta_thigh_y = data.angle_y; // CHANGE THIS
    float theta_thigh_z = data.angle_z; // CHANGE THIS

    // Calculate length vectors for shank and thigh
    float L_shank_x, L_shank_z, L_thigh_x, L_thigh_z;
    calculate_length_vectors(theta_shank_x, theta_thigh_x, L_shank_x, L_shank_z, L_thigh_x, L_thigh_z);

    // Integrate acceleration to get v of the IMU
    integrate(v_IMU_x, v_IMU_y, v_IMU_z, data, dt);

    // Function to calculate cross product of two 3D vectores
    void crossProduct(float vect_A[3], float vect_B[3], float cross_P[3]) {
        cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
        cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
        cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    }


    // Calculate the v of the shank, knee, thigh and hip with cross product approximation
    v_shank_x = v_IMU_x + crossProduct(data.gyro_y, L_shank_z * COM_shank, cross_P); //check axis for cross prod
    v_shank_y = v_IMU_y + crossProduct(data.gyro_x, L_shank_z * COM_shank, cross_P); //check axis for cross prod
    v_shank_z = v_IMU_z + crossProduct(data.gyro_y, L_shank_x * COM_shank, cross_P); //check axis for cross prod

    v_knee_x = v_IMU_x + crossProduct(data.gyro_z, L_shank_x, cross_P); //check axis for cross prod
    v_knee_y = v_IMU_y + crossProduct(data.gyro_z, L_shank_x, cross_P); //check axis for cross prod
    v_knee_z = v_IMU_z + crossProduct(data.gyro_z, L_shank_z, cross_P); //check axis for cross prod

    v_thigh_x = v_knee_x + crossProduct(w_shank_z, (1-COM_thigh) * L_thigh_x, cross_P); //check axis for cross prod
    v_thigh_y = v_knee_y + crossProduct(w_shank_z, (1-COM_thigh) * L_thigh_x, cross_P); //check axis for cross prod
    v_thigh_z = v_knee_z + crossProduct(w_shank_z, (1-COM_thigh) * L_thigh_z, cross_P); //check axis for cross prod
    
    v_hip_x = v_knee_x + crossProduct(w_thigh_z, L_thigh_x, cross_P); //check axis for cross prod
    v_hip_y = v_knee_y + crossProduct(w_thigh_z, L_thigh_x, cross_P); //check axis for cross prod
    v_hip_z = v_knee_z + crossProduct(w_thigh_z, L_thigh_z, cross_P); //check axis for cross prod
}

// Function to integrate acceleration to get v
void integrate(float &v_x, float &v_y, float &v_z, const SensorData &accel_data, float dt) {
    v_x += accel_data.accel_x * dt;
    v_y += accel_data.accel_y * dt;
    v_z += accel_data.accel_z * dt;
}

// Function to differentiate 
void differentiate(float x, float y, float z, float pre_x, float pre_y, float pre_z,
    float dt, vector<float> &acc) {
    
    acc[0] += (pre_x - x) / dt;
    acc[1] += (pre_y - y) / dt;
    acc[2] += (pre_z - z) / dt;
}

    // Function to calculate GRF
    void calculate_grf(float m_shank, float m_thigh, float a_shank, float a_thigh, 
        float a_hip, float g, float &F_shank, float &F_thigh, float &F_hip, float &F_ground) {
        F_shank = m_shank * (a_shank - g);
        F_thigh = m_thigh * (a_thigh - g);
        F_hip = m_hip * (a_hip - g);       
        F_ground = F_shank + F_thigh + F_hip;

    std::cout << "GRF: " << F_ground << " N" << std::endl;
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

    // Initialize v variables
    float v_shank_x = 0.0f, v_shank_y = 0.0f, v_shank_z = 0.0f;
    float v_thigh_x = 0.0f, v_thigh_y = 0.0f, v_thigh_z = 0.0f;
    float v_IMU_x = 0.0f, v_IMU_y = 0.0f, v_IMU_z = 0.0f;
    float v_knee_x = 0.0f, v_knee_y = 0.0f, v_knee_z = 0.0f;
    float v_hip_x = 0.0f, v_hip_y = 0.0f, v_hip_z = 0.0f;
    vector<float> w_thigh = {0,0,0};

    // Initialize acceleration variables
    vector<float> a_shank = {0,0,0};
    vector<float> a_hip = {0,0,0};
    vector<float> a_thigh = {0,0,0};
    vector <float> acc = {0,0,0};
    
    // Initialize pre variables (not the most efficient way but... :)
    float dt = 0.000004f;  // Time step for 250Hz (4 microseconds between calculations)
    float pre_v_shank_x = 0.00f;
    float pre_v_shank_y = 0.00f;
    float pre_v_shank_z = 0.00f;
    float pre_theta_thigh_x = 0.00f;
    float pre_theta_thigh_y = 0.00f;
    float pre_theta_thigh_z = 0.00f;
    float pre_v_thigh_x = 0.00f;
    float pre_v_thigh_y = 0.00f;
    float pre_v_thigh_z = 0.00f;
    float pre_v_hip_x = 0.00f;
    float pre_v_hip_y = 0.00f;
    float pre_v_hip_z = 0.00f;
    
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
            // Differentiate shank v to obtain a_shank 
            differentiate(v_shank_x, v_shank_y, v_shank_z, pre_v_shank_x,
                pre_v_shank_y, pre_v_shank_z, dt, a_shank);

            // Differentiate theta thigh to obtain w_thigh
            differentiate(theta_thigh_x, theta_thigh_y, theta_thigh_z, pre_theta_thigh_x,
                pre_theta_thigh_y, pre_theta_thigh_z, dt, w_thigh);
                
            // Differentiate v thigh to obtain a_thigh
            differentiate(v_thigh_x, v_thigh_y, v_thigh_z, pre_v_thigh_x,
                pre_v_thigh_y, pre_v_thigh_z, dt, a_thigh);

            // Differentiate v hip to obtain a_hip
            differentiate(v_hip_x, v_hip_y, v_hip_z, pre_v_hip_x,
                pre_v_hip_y, pre_v_hip_z, dt, a_hip);    

            pre_v_shank_x= v_shank_x;
            pre_v_shank_y= v_shank_y;
            pre_v_shank_z= v_shank_z;

            pre_theta_thigh_x= theta_thigh_x;
            pre_theta_thigh_y= theta_thigh_y;
            pre_theta_thigh_z= theta_thigh_z;

            pre_v_thigh_x= v_thigh_x;
            pre_v_thigh_y= v_thigh_y;
            pre_v_thigh_z= v_thigh_z;

            pre_v_hip_x= v_hip_x;
            pre_v_hip_y= v_hip_y;
            pre_v_hip_z= v_hip_z;    
        }

        // Calculate velocities for shank and thigh
        calculate_kinematics(data, v_shank_x, v_shank_y, v_shank_z, 
                           v_thigh_x, v_thigh_y, v_thigh_z, v_IMU_x, 
                           v_IMU_y, v_IMU_z, dt);
    
        // Calculate GRF
        calculate_grf(m * COM_shank, m * COM_thigh, a_shank, a_thigh, a_hip, 9.81f);  // Using 9.81m/s^2 for gravity

        sleep_ms(4);  // Sleep 4 mili sec until next sample to be taken
    }

    return 0;
}

//Make sure pins match datasheet. Vlad
//Calibration?
//Axis in cross product and variables in general