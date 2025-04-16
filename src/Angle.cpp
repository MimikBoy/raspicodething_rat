#include <stdio.h>
#include "pico/stdlib.h"
#include "bno08x2.h"
#include "utils2.h"
#include "bno08x.h"
#include "utils.h"
#include <math.h> 
#include <vector>
#include <numeric>
#include <fstream>
#include <iostream>
#include <string>
#include "Angle.h"

extern BNO08x IMU;
extern BNO08x2 IMU2;
extern float start_time;
float pitch1 = 0.0f;
float pitch2 = 0.0f;
float diff = 0.0f;
float diff_init = 0.0f;
float conversion = 180/(2*M_PI);
float time = 0.0f;
const int SIZE = 30;
float sum = 0.0f;
float ref;


struct AngleData {
    float angle;
    float elapsed_time_ms;
};

AngleData AngleRead(float ref, BNO08x& IMU, BNO08x2& IMU2){
    float pitch1 = 0.0f;
    float pitch2 = 0.0f;
    float angle = 0.0f;
    float conversion = 180/(2*M_PI);
    float elapsed_time_ms = 0.0f;
    if (IMU.getSensorEvent() == true) {
        if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
            pitch2 = IMU.getPitch()*conversion;    
        }
    }
    if (IMU2.getSensorEvent() == true) {
        if (IMU2.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER_2) {
            pitch1 = IMU2.getPitch()*conversion;
            angle = 180 -(pitch1+pitch2+ref);
            elapsed_time_ms  = time_us_64() /1000.0f - start_time;
            return  AngleData{ angle, elapsed_time_ms} ;
        }
    }
    //return AngleData{0.0f, 0.0f}; // Return default values if no data is available
}    

void init_Angle(){
    for (int i = 0; i < SIZE; ++i) {
        if (IMU.getSensorEvent() == true) {
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
                pitch1 = IMU.getPitch()*conversion;
            }
        }

        if (IMU2.getSensorEvent() == true) {
            if (IMU2.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER_2) {
                pitch2 = IMU2.getPitch()*conversion;
            }
        }
        diff_init = -pitch2-pitch1+90;
        sum += diff_init;
        sleep_ms(100);
    }
}


void get_Angle(float* result)
{
    ref = sum / SIZE;

    AngleData data = AngleRead(ref, IMU, IMU2);
    //printf("Time: %.2f ms | Angle: %.2f deg\n", data.elapsed_time_ms, data.angle);
    //sleep_ms(100);
    result[0] = data.angle;
    result[1] = data.elapsed_time_ms;
}

