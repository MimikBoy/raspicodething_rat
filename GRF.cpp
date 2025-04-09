#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "hardware/i2c.h"
using namespace std;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>
#include <iostream>
#include <bno08x.h>
#include "utils.h"
// Test 
#include "GRF.h"

// I2C Settings
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define BNO085_ADDR 0x4C

// Constants 
float L_shank_ini, L_thigh_ini, m; // Defined by user (lower in the code)

// Function to compute the length vectors for shank and thigh
vector<float> calculate_length(vector<float> angle, float length){
    float L_x = (length/100) * sin(angle[0]);
    float L_y = 0.00f;
    float L_z = (length/100) * cos(angle[2]);
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

    //  // Function to integrate
    //  vector <float> integrate(vector <float> a, vector <float> b, float dt){
    //      float bx = b[0] + (a[0] * dt);
    //      float by = b[1] + (a[1] * dt);
    //      float bz = b[2] + (a[2] * dt);
    //     return {bx, by, bz};
    //  }

    // Function to integrate 
    vector<float> integrate(vector<float> a, vector<float> pre_a, vector<float> v, float dt) {
         float vx = v[0] + 0.5f * (a[0] + pre_a[0]) * dt;
         float vy = v[1] + 0.5f * (a[1] + pre_a[1]) * dt;
         float vz = v[2] + 0.5f * (a[2] + pre_a[2]) * dt;
         return {vx, vy, vz};
    }

    // Function to differentiate
    vector <float> differentiate(vector <float> a, vector <float> pre_a, float dt){
        float bx = (a[0] - pre_a[0]) / dt;
        float by = (a[1] - pre_a[1]) / dt;
        float bz = (a[2] - pre_a[2]) / dt;
    return {bx, by, bz};
    }

    // Function to calculate GRF
    vector <float> calculate_grf(vector <float> a_shank, vector <float> a_thigh, vector <float> a_hip, float m){
        float m_shank = 0.057*m;
        float m_thigh = 0.1416*m;
        float m_hip = 0.5*0.6028*m;
        float g = 9.81;
        float grf_x = m_shank*(a_shank[0]) + m_thigh*(a_thigh[0]) + m_hip*(a_hip[0]);
        float grf_y = m_shank*(a_shank[1]) + m_thigh*(a_thigh[1]) + m_hip*(a_hip[1]);
        float grf_z = m_shank*(a_shank[2]-g) + m_thigh*(a_thigh[2]-g) + m_hip*(a_hip[2]-g);
        return {grf_x, grf_y, grf_z};
    }

int main() {
    stdio_init_all();
    
    i2c_inst_t* i2c_port0 = i2c0;
    initI2C(i2c_port0, false);

    BNO08x IMU;

    while (!IMU.begin(0x4A, i2c_port0)) {
        printf("BNO08x not detected. Freezing...\n");
        scan_i2c_bus();
        sleep_ms(1000);
    }

    if (!IMU.enableGameRotationVector(100)) {
        printf("Failed to enable Game Rotation Vector!\n");
    }

    //const float RAD_TO_DEG = 180.0f / M_PI;
    //float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    // User input
    // std::cin >> L_shank_ini;
    // std::cin >> L_thigh_ini;
    // std::cin >> m;
    const float L_shank_ini=40; const float L_thigh_ini=50; const float m=75.0;
    //printf("L_shank_ini %f\n", L_shank_ini);
    float pitch = 0.0f, yaw = 0.0f, roll= 0.0f; 
    uint8_t accAccuracy = 0; 
    float accX = 0.0f, accY = 0.0f, accZ = 0.0f; // Accelerometer values
    float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f; // Gyroscope values
    uint8_t gyroAccuracy = 0.0f; // Gyroscope accuracy
    float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f; // Angle values
    uint8_t angleAccuracy = 0.0f; // Angle accuracy

    // // Initialize velocities
    vector <float> v_IMU= {0,0,0};
    vector <float> v_shank= {0,0,0};
    vector <float> v_thigh= {0,0,0};
    vector <float> v_knee= {0,0,0};
    vector <float> v_hip= {0,0,0};
    vector <float> pre_v_shank= {0,0,0};
    vector <float> pre_v_thigh= {0,0,0};
    vector <float> pre_v_knee= {0,0,0};
    vector <float> pre_v_hip= {0,0,0};
    vector <float> pre_v_IMU= {0,0,0};
    vector <float> w_thigh= {0,0,0};
    vector <float> w_IMU= {0,0,0};

    //Initialize sensor data
    vector<float> prev_accel = {0, 0, 0};
    vector<float> prev_gyro = {0, 0, 0};

    // Initialize acceleration variables
    vector <float> a_IMU = {0,0,0};
    vector<float> a_shank = {0,0,0};
    vector<float> a_thigh = {0,0,0};
    vector<float> a_hip = {0,0,0};
    vector<float> pre_a_IMU = {0,0,0};

    //Initialize angles
    vector <float> pre_angle_thigh = {0,0,0};
    vector<float> angle_IMU;

    //Initialize length vectors
    vector <float> L_shank;
    vector <float> L_shank_COM;
    vector <float> L_thigh;
    vector <float> L_thigh_COM;

    // Initialize GRF
    vector<float> GRF;

    // Constants
    const float COM_shank = 0.5726;  // Values taken from paper in Zotero
    const float COM_thigh = 0.4095;
    const float dt = 0.005f;  // Time step for 200Hz (5 miliseconds between calculations)


    while (true) 
    {
        // Read sensor data
            if (IMU.getSensorEvent() == true){
        
            yaw = IMU.getGameYaw();
            pitch = IMU.getGamePitch();
            roll = IMU.getRoll();
            //printf("Roll X %f\n Pitch Y %f\n Yaw Z %f\n", roll, pitch, yaw);

            // // Optional: print quaternions
            // float qi = IMU.getGameQuatI();
            // float qj = IMU.getGameQuatJ();
            // float qk = IMU.getGameQuatK();
            // float qr = IMU.getGameQuatReal();
            // //printf("Quat: i=%.2f, j=%.2f, k=%.2f, r=%.2f\n", qi, qj, qk, qr);

            accX = IMU.getLinAccelX();
            accY = IMU.getLinAccelY();
            accZ = IMU.getLinAccelZ();
            //printf("Acc X IMU %f\n Acc Y IMU %f\n Acc Z IMU %f\n", accX, accY, accZ);

            gyroX = IMU.getGyroX();
            gyroY = IMU.getGyroY();
            gyroZ = IMU.getGyroZ();
            //printf("w X IMU %f\n w Y IMU %f\n w Z IMU %f\n", gyroX, gyroY, gyroZ);

            float timeStamp = IMU.getTimeStamp();
            //printf("timeStamp %f\n", timeStamp);
            }

           // Store sensor data in vector
            a_IMU = {accX, accY, accZ};
            //printf("a IMU X %f\n a IMU Y %f\n a IMU Z %f\n", a_IMU[0], a_IMU[1], a_IMU[2]);
            w_IMU = {gyroX, gyroY, gyroZ};
            //printf("w IMU X %f\n w IMU Y %f\n w IMU Z %f\n", w_IMU[0], w_IMU[1], w_IMU[2]);
            angle_IMU = {roll, pitch, yaw};
            //printf("Angle IMU X %f\n Angle IMU Y %f\n Angle IMU Z %f\n", angle_IMU[0], angle_IMU[1], angle_IMU[2]);
            
            vector<float> angle_thigh = estimate_angle_thigh(angle_IMU);
            //printf("Angle thigh X %f\n Angle thigh Y %f\n Angle thigh Z %f\n", angle_thigh[0], angle_thigh[1], angle_thigh[2]);

            // Calculate Length vectors
            L_shank= calculate_length(angle_IMU, L_shank_ini);
            //printf("L shank X %f\n L shank Y %f\n L shank Z %f\n", L_shank[0], L_shank[1], L_shank[2]);
            L_shank_COM = entrywise_mul(L_shank, COM_shank);
            //printf("L shank X COM %f\n L shank Y COM %f\n L shank Z COM %f\n", L_shank_COM[0], L_shank_COM[1], L_shank_COM[2]);
            L_thigh= calculate_length(angle_thigh, L_thigh_ini);
            L_thigh_COM = entrywise_mul(L_thigh, (1-COM_thigh));

            // Integrate a_IMU to obtain v_IMU
            v_IMU= integrate(a_IMU, pre_a_IMU, v_IMU, dt);
            //v_IMU = integrate(a_IMU,v_IMU,dt);
            printf("v IMU X %f\n v IMU Y %f\n v IMU Z %f\n", v_IMU[0], v_IMU[1], v_IMU[2]);
            
            // Calculate velocity and acceleration shank
            v_shank = entrywise_add(v_IMU, crossProduct(w_IMU,L_shank_COM));
            //printf("v shank X %f\n v shank Y %f\n v shank Z %f\n", v_shank[0], v_shank[1], v_shank[2]);
            a_shank = differentiate(v_shank, pre_v_shank, dt);
            printf("a shank X %f\n a shank Y %f\n a shank Z %f\n", a_shank[0], a_shank[1], a_shank[2]);

            // Calculate velocity knee
            v_knee= entrywise_add(v_IMU, crossProduct(w_IMU,L_shank));
            printf("v knee X %f\n v knee Y %f\n v knee Z %f\n", v_knee[0], v_knee[1], v_knee[2]);
            // Calculate velocity and acceleration thigh
            w_thigh = differentiate(angle_thigh, pre_angle_thigh, dt);
            printf("w thigh X %f\n w thigh Y %f\n w thigh Z %f\n", w_thigh[0], w_thigh[1], w_thigh[2]);
            v_thigh= entrywise_add(v_knee, crossProduct(w_thigh,L_thigh_COM));
            printf("v thigh X %f\n v thigh Y %f\n v thigh Z %f\n", v_thigh[0], v_thigh[1], v_thigh[2]);
            a_thigh = differentiate(v_thigh, pre_v_thigh, dt);
            printf("a thigh X %f\n a thigh Y %f\n a thigh Z %f\n", a_thigh[0], a_thigh[1], a_thigh[2]);
            // Calculate velocity and acceleration hip
            v_hip= entrywise_add(v_knee, crossProduct(w_thigh,L_thigh));
            a_hip = differentiate(v_hip, pre_v_hip, dt);
            printf("a hip X %f\n a hip Y %f\n a hip Z %f\n", a_hip[0], a_hip[1], a_hip[2]);
            printf("v hip X %f\n v hip Y %f\n v hip Z %f\n", v_hip[0], v_hip[1], v_hip[2]);
            // Assign current value to previous value
            pre_angle_thigh = angle_thigh;
            pre_v_thigh = v_thigh;
            pre_v_hip = v_hip;
            pre_v_IMU = v_IMU;
            pre_v_shank = v_shank;
            pre_a_IMU = a_IMU;
           //vector <float> pre_v_IMU2= {v_IMU};
            // Calculate GRF
            GRF = calculate_grf(a_shank, a_thigh, a_hip, m);
            // Return GRF and timestamp here
            printf("GRF X %f\n GRF Y %f\n GRF Z %f\n", GRF[0], GRF[1], GRF[2]);
        sleep_ms(5);  // Sleep 5 mili sec until next sample to be taken
        } 

    return 0;
}
