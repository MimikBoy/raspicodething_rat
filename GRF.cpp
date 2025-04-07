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
float L_shank_ini, L_thigh_ini, m; // Defined by user (lower in the code)

// Function to compute the length vectors for shank and thigh
vector<float> calculate_length(vector<float> angle, float length){
    float L_x = length * sin(angle[0]);
    float L_y = 0.00f;
    float L_z = length * cos(angle[2]);
    return {L_x, L_y, L_z};
}

// Function enrtywise multiplication
vector<float> entrywise_mul(vector<float> L, float COM){
    float L_x = L[0] * COM;
    float L_y = L[1] * COM;
    float L_z = L[2] * COM;
    return{L_x, L_y, L_z};
}

// Function enrtywise addition
vector<float> entrywise_add(vector<float> vect_A, vector<float> vect_B){
    float vect_Cx = vect_A[0] + vect_B[0];
    float vect_Cy = vect_A[1] + vect_B[1];
    float vect_Cz = vect_A[2] + vect_B[2];
    return{vect_Cx, vect_Cy, vect_Cz};
}

// NOTE THIS IS A PLACEHOLDER:
// Function to estimate the thigh angle
vector<float> estimate_angle_thigh(vector<float> angle){   
    float theta_thigh_x = angle[0];
    float theta_thigh_y = angle[1];
    float theta_thigh_z = angle[2];
    return{theta_thigh_x, theta_thigh_y, theta_thigh_z};
}

// Function to calculate cross product of two 3D vectores
vector <float> crossProduct(vector <float> vect_A, vector <float> vect_B){
    float cross_Px = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    float cross_Py = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    float cross_Pz = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    return {cross_Px, cross_Py, cross_Pz};
}

    // Function to integrate
    // This does have a problem that it does not now the initial bx, by, bz
    vector <float> integrate(vector <float> a, vector <float> b, float dt){
        float bx = b[0] + (a[0] * dt);
        float by = b[1] + (a[1] * dt);
        float bz = b[2] + (a[2] * dt);
    return {bx, by, bz};
    }

    // This does have a problem that it does not now the initial bx, by, bz
    // Function to differentiate
    vector <float> differentiate(vector <float> a, vector <float> pre_a, vector <float> b, float dt){
        float bx = b[0] + ((pre_a[0] - a[0]) / dt);
        float by = b[1] + ((pre_a[1] - a[1]) / dt);
        float bz = b[2] + ((pre_a[2] - a[2]) / dt);
    return {bx, by, bz};
    }

    // Function to calculate GRF
    vector <float> calculate_grf(vector <float> a_shank, vector <float> a_thigh, vector <float> a_hip, float m){
        float m_shank = 0.057*m;
        float m_thigh = 0.1416*m;
        float m_hip = 0.5*0.6028*m;
        float g = 9.81;
        float grf_x = m_shank*(a_shank[0]-g) + m_thigh*(a_thigh[0]-g) + m_hip*(a_hip[0]-g);
        float grf_y = m_shank*(a_shank[1]-g) + m_thigh*(a_thigh[1]-g) + m_hip*(a_hip[1]-g);
        float grf_z = m_shank*(a_shank[2]-g) + m_thigh*(a_thigh[2]-g) + m_hip*(a_hip[2]-g);
        return {grf_x, grf_y, grf_z};
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
    std::cin >> L_shank_ini;
    std::cout << "Enter the Length of Thigh (L_thigh) in meters: ";
    std::cin >> L_thigh_ini;
    std::cout << "Enter the Total Mass of the Body (m) in kg: ";
    std::cin >> m;

    float pitch = 0.0f, yaw = 0.0f, roll= 0.0f; 
    uint8_t accAccuracy = 0; 
    float accX = 0.0f, accY = 0.0f, accZ = 0.0f; // Accelerometer values
    float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f; // Gyroscope values
    uint8_t gyroAccuracy = 0.0f; // Gyroscope accuracy
    float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // Angle values
    uint8_t angleAccuracy = 0.0f; // Angle accuracy

    // // Initialize v, dt and w variables
    vector <float> v_IMU= {0,0,0};
    vector <float> v_shank= {0,0,0};
    vector <float> v_thigh= {0,0,0};
    vector <float> v_knee= {0,0,0};
    vector <float> v_hip= {0,0,0};
    vector <float> pre_v_shank= {0,0,0};
    vector <float> pre_v_thigh= {0,0,0};
    vector <float> pre_v_knee= {0,0,0};
    vector <float> pre_v_hip= {0,0,0};
    float dt = 0.000004f;  // Time step for 250Hz (4 microseconds between calculations)
    vector <float> w_thigh= {0,0,0};
    vector <float> w_IMU= {0,0,0};

    // Initialize acceleration variables
    vector <float> a_IMU = {0,0,0};
    vector<float> a_shank = {0,0,0};
    vector<float> a_thigh = {0,0,0};
    vector<float> a_hip = {0,0,0};

    //Initialize angles
    vector <float> pre_angle_thigh = {0,0,0};
    
    
    while (true) {
        // Read sensor data
        if (IMU.getSensorEvent() == true){
            if (IMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                pitch = IMU.getPitch(); // y-axis
                yaw = IMU.getYaw(); // z-axis
                roll = IMU.getRoll(); // x-axis
            }
    
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
                IMU.getAccel(accX, accY, accZ, accAccuracy);
            }

            if (IMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
                IMU.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
            }

            // Store sensor data in vector
            vector<float> a_IMU = {accX, accY, accZ}; 
            vector<float> w_IMU = {gyroX, gyroY, gyroZ};   
            vector<float> angle_IMU = {roll, pitch, yaw};

            vector<float> angle_thigh = estimate_angle_thigh(angle_IMU);

            // Calculate Length vectors
            vector <float> L_shank= calculate_length(angle_IMU, L_shank_ini);
            vector <float> L_shank_COM = entrywise_mul(L_shank, COM_shank);
            vector <float> L_thigh= calculate_length(angle_thigh, L_thigh_ini);
            vector <float> L_thigh_COM = entrywise_mul(L_thigh, (1-COM_thigh));

            // Integrate a_IMU to obtain v_IMU
            vector <float> v_IMU= integrate(a_IMU, v_IMU, dt);

            // Calculate velocity and acceleration shank
            vector <float> v_shank = entrywise_add(v_IMU, crossProduct(w_IMU,L_shank_COM));
            vector <float> a_shank = differentiate(v_shank, pre_v_shank, a_shank, dt);

            // Calculate velocity knee
            vector <float> v_knee= entrywise_add(v_IMU, crossProduct(w_IMU,L_shank));

            // Calculate velocity and acceleration thigh
            vector <float> w_thigh = differentiate(angle_thigh, pre_angle_thigh, w_thigh, dt);
            vector <float> v_thigh= entrywise_add(v_knee, crossProduct(w_thigh,L_thigh_COM));
            vector <float> a_thigh = differentiate(v_thigh, pre_v_thigh, a_thigh, dt);
            
            // Calculate velocity and acceleration hip
            vector <float> v_hip= entrywise_add(v_knee, crossProduct(w_thigh,L_thigh));
            vector <float> a_hip = differentiate(v_hip, pre_v_hip, a_hip, dt);

            // Assign current value to previous value
            vector <float> pre_angle_thigh={angle_thigh};
            vector <float> pre_v_thigh={v_thigh};
            vector <float> pre_v_hip={v_hip};
            vector <float> pre_v_shank={v_shank};


            // Calculate GRF
            vector <float> GRF = calculate_grf(a_IMU, a_thigh, a_hip, m);
        }

        sleep_ms(4);  // Sleep 4 mili sec until next sample to be taken
    }

    return 0;
}

//Make sure pins match datasheet. Vlad
//Calibration?
//Axis in cross product and variables in general