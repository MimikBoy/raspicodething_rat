/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** 
 * @file sh2_SensorValue.h 
 * @author David Wheeler
 * @date 10 Nov 2015
 * @brief Support for converting sensor events (messages) into natural data structures.
 *
 */

#ifndef SH2_SENSORVALUE_2_H
#define SH2_SENSORVALUE_2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sh22.h"

/* Note on quaternion naming conventions:
 * Quaternions are values with four real components that are usually
 * interpreted as coefficients in the complex quantity, Q.
 *
 * As in, Q = W + Xi + Yj + Zk
 *
 * Where i, j and k represent the three imaginary dimensions.
 *
 * So W represents the Real components and X, Y and Z the Imaginary ones.
 *
 * In the Hillcrest datasheets and in this code, however, the four components
 * are named real, i, j and k, to make it explicit which is which.  If you 
 * need to translate these names into the "wxyz" or "xyzw" convention, then, the
 * appropriate mapping is this:
 *     w = real
 *     x = i
 *     y = j
 *     z = k
 */
	
/**
 * @brief Raw Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawAccelerometer_2 {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC counts] */
    int16_t y;  /**< @brief [ADC counts] */
    int16_t z;  /**< @brief [ADC counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawAccelerometer_t_2;

/**
 * @brief Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Accelerometer_2 {
    float x;
    float y;
    float z;
} sh2_Accelerometer_t_2;

/**
 * @brief Raw gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawGyroscope_2 {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC Counts] */
    int16_t y;  /**< @brief [ADC Counts] */
    int16_t z;  /**< @brief [ADC Counts] */
    int16_t temperature;  /**< @brief [ADC Counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawGyroscope_t_2;

/**
 * @brief Gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Gyroscope_2 {
    /* Units are rad/s */
    float x;
    float y;
    float z;
} sh2_Gyroscope_t_2;

/**
 * @brief Uncalibrated gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_GyroscopeUncalibrated_2 {
    /* Units are rad/s */
    float x;  /**< @brief [rad/s] */
    float y;  /**< @brief [rad/s] */
    float z;  /**< @brief [rad/s] */
    float biasX;  /**< @brief [rad/s] */
    float biasY;  /**< @brief [rad/s] */
    float biasZ;  /**< @brief [rad/s] */
} sh2_GyroscopeUncalibrated_t_2;

/**
 * @brief Raw Magnetometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawMagnetometer_2 {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC Counts] */
    int16_t y;  /**< @brief [ADC Counts] */
    int16_t z;  /**< @brief [ADC Counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawMagnetometer_t_2;

/**
 * @brief Magnetic field
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_MagneticField_2 {
    /* Units are uTesla */
    float x;  /**< @brief [uTesla] */
    float y;  /**< @brief [uTesla] */
    float z;  /**< @brief [uTesla] */
} sh2_MagneticField_t_2;

/**
 * @brief Uncalibrated magnetic field
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_MagneticFieldUncalibrated_2 {
    /* Units are uTesla */
    float x;  /**< @brief [uTesla] */
    float y;  /**< @brief [uTesla] */
    float z;  /**< @brief [uTesla] */
    float biasX;  /**< @brief [uTesla] */
    float biasY;  /**< @brief [uTesla] */
    float biasZ;  /**< @brief [uTesla] */
} sh2_MagneticFieldUncalibrated_t_2;

/**
 * @brief Rotation Vector with Accuracy
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RotationVectorWAcc_2 {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component, real */
    float accuracy;  /**< @brief Accuracy estimate [radians] */
} sh2_RotationVectorWAcc_t_2;

/**
 * @brief Rotation Vector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RotationVector_2 {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component real */
} sh2_RotationVector_t_2;

/**
 * @brief Atmospheric Pressure
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Pressure_2 {
    float value;  /**< @brief Atmospheric Pressure.  [hectopascals] */
} sh2_Pressure_t_2;

/**
 * @brief Ambient Light
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_AmbientLight_2 {
    float value;  /**< @brief Ambient Light.  [lux] */
} sh2_AmbientLight_t_2;

/**
 * @brief Humidity
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Humidity_2 {
    float value;  /**< @brief Relative Humidity.  [percent] */
} sh2_Humidity_t_2;

/**
 * @brief Proximity
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Proximity_2 {
    float value;  /**< @brief Proximity.  [cm] */
} sh2_Proximity_t_2;

/**
 * @brief Temperature
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Temperature_2 {
    float value;  /**< @brief Temperature.  [C] */
} sh2_Temperature_t_2;

/**
 * @brief Reserved
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Reserved_2 {
    float tbd;  /**< @brief Reserved */
} sh2_Reserved_t_2;

/**
 * @brief TapDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define TAPDET_X_2      (1)
#define TAPDET_X_POS_2  (2)
#define TAPDET_Y_2      (4)
#define TAPDET_Y_POS_2  (8)
#define TAPDET_Z_2      (16)
#define TAPDET_Z_POS_2  (32)
#define TAPDET_DOUBLE_2 (64)
typedef struct sh2_TapDetector_2 {
    uint8_t flags;  /**< @brief TapDetector.  */
} sh2_TapDetector_t_2;

/**
 * @brief StepDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_StepDetector_2 {
    uint32_t latency;  /**< @brief Step detect latency [uS].  */
} sh2_StepDetector_t_2;

/**
 * @brief StepCounter
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_StepCounter_2 {
    uint32_t latency;  /**< @brief Step counter latency [uS].  */
    uint16_t steps;    /**< @brief Steps counted. */
} sh2_StepCounter_t_2;

/**
 * @brief SigMotion
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SigMotion_2 {
    uint16_t motion;
} sh2_SigMotion_t_2;

/**
 * @brief StabilityClassifier
 *
 * See the SH-2 Reference Manual for more detail.
 */

typedef struct sh2_StabilityClassifier_2 {
    uint8_t classification;
} sh2_StabilityClassifier_t_2;

/**
 * @brief ShakeDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define SHAKE_X_2 (1)
#define SHAKE_Y_2 (2)
#define SHAKE_Z_2 (4)
typedef struct sh2_ShakeDetector_2 {
    uint16_t shake;
} sh2_ShakeDetector_t_2;

/**
 * @brief flipDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_FlipDetector_2 {
    uint16_t flip;
} sh2_FlipDetector_t_2;

/**
 * @brief pickupDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define PICKUP_LEVEL_TO_NOT_LEVEL_2 (1)
#define PICKUP_STOP_WITHIN_REGION_2 (2)
typedef struct sh2_PickupDetector_2 {
    uint16_t pickup;   /**< flag field with bits defined above. */
} sh2_PickupDetector_t_2;

/**
 * @brief stabilityDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define STABILITY_ENTERED_2 (1)
#define STABILITY_EXITED_2  (2)
typedef struct sh2_StabilityDetector_2 {
    uint16_t stability;  /**< flag field with bits defined above. */
} sh2_StabilityDetector_t_2;

/**
 * @brief Personal Activity Classifier
 *
 * See the SH-2 Reference Manual for more detail.
 */

typedef struct sh2_PersonalActivityClassifier_2 {
    uint8_t page;
    bool lastPage;
    uint8_t mostLikelyState;
    uint8_t confidence[10];
} sh2_PersonalActivityClassifier_t_2;

/**
 * @brief sleepDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SleepDetector_2 {
    uint8_t sleepState;
} sh2_SleepDetector_t_2;

/**
 * @brief tiltDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_TiltDetector_2 {
    uint16_t tilt;
} sh2_TiltDetector_t_2;

/**
 * @brief pocketDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_PocketDetector_2 {
    uint16_t pocket;
} sh2_PocketDetector_t_2;

/**
 * @brief circleDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_CircleDetector_2 {
    uint16_t circle;
} sh2_CircleDetector_t_2;

/**
 * @brief heartRateMonitor
 *
 * See SH-2 Reference Manual for details.
 */
typedef struct sh2_HeartRateMonitor_2 {
    uint16_t heartRate; /**< heart rate in beats per minute. */
} sh2_HeartRateMonitor_t_2;

/**
 * @brief Gyro Integrated Rotation Vector
 *
 * See SH-2 Reference Manual for details.
 */
typedef struct sh2_GyroIntegratedRV_2 {
    float i;        /**< @brief Quaternion component i */
    float j;        /**< @brief Quaternion component j */
    float k;        /**< @brief Quaternion component k */
    float real;     /**< @brief Quaternion component real */
    float angVelX;  /**< @brief Angular velocity about x [rad/s] */
    float angVelY;  /**< @brief Angular velocity about y [rad/s] */
    float angVelZ;  /**< @brief Angular velocity about z [rad/s] */
} sh2_GyroIntegratedRV_t_2;

typedef struct sh2_IZroRequest_2 {
    sh2_IZroMotionIntent_t_2 intent;
    sh2_IZroMotionRequest_t_2 request;
} sh2_IZroRequest_t_2;

typedef struct sh2_SensorValue_2 {
    
    /** Which sensor produced this event. */
    uint8_t sensorId;

    /** @brief 8-bit unsigned integer used to track reports.
     *
     * The sequence number increments once for each report sent.  Gaps
     * in the sequence numbers indicate missing or dropped reports.
     */
    uint8_t sequence;

    /* Status of a sensor
     *   0 - Unreliable
     *   1 - Accuracy low
     *   2 - Accuracy medium
     *   3 - Accuracy high
     */
    uint8_t status; /**< @brief bits 7-5: reserved, 4-2: exponent delay, 1-0: Accuracy */

    uint64_t timestamp;  /**< [uS] */

    uint32_t delay; /**< @brief [uS] value is delay * 2^exponent (see status) */

    /** @brief Sensor Data
     *
     * Use the structure based on the value of the sensor
     * field.
     */
    union {
        sh2_RawAccelerometer_t_2 rawAccelerometer;
        sh2_Accelerometer_t_2 accelerometer; 
        sh2_Accelerometer_t_2 linearAcceleration; 
        sh2_Accelerometer_t_2 gravity; 
        sh2_RawGyroscope_t_2 rawGyroscope; 
        sh2_Gyroscope_t_2 gyroscope; 
        sh2_GyroscopeUncalibrated_t_2 gyroscopeUncal; 
        sh2_RawMagnetometer_t_2 rawMagnetometer; 
        sh2_MagneticField_t_2 magneticField; 
        sh2_MagneticFieldUncalibrated_t_2 magneticFieldUncal; 
        sh2_RotationVectorWAcc_t_2 rotationVector; 
        sh2_RotationVector_t_2 gameRotationVector; 
        sh2_RotationVectorWAcc_t_2 geoMagRotationVector;
        sh2_Pressure_t_2 pressure;
        sh2_AmbientLight_t_2 ambientLight;
        sh2_Humidity_t_2 humidity;
        sh2_Proximity_t_2 proximity;
        sh2_Temperature_t_2 temperature;
        sh2_Reserved_t_2 reserved;
        sh2_TapDetector_t_2 tapDetector;
        sh2_StepDetector_t_2 stepDetector;
        sh2_StepCounter_t_2 stepCounter;
        sh2_SigMotion_t_2 sigMotion;
        sh2_StabilityClassifier_t_2 stabilityClassifier;
        sh2_ShakeDetector_t_2 shakeDetector;
        sh2_FlipDetector_t_2 flipDetector;
        sh2_PickupDetector_t_2 pickupDetector;
        sh2_StabilityDetector_t_2 stabilityDetector;
        sh2_PersonalActivityClassifier_t_2 personalActivityClassifier;
        sh2_SleepDetector_t_2 sleepDetector;
        sh2_TiltDetector_t_2 tiltDetector;
        sh2_PocketDetector_t_2 pocketDetector;
        sh2_CircleDetector_t_2 circleDetector;
        sh2_HeartRateMonitor_t_2 heartRateMonitor;
        sh2_RotationVectorWAcc_t_2 arvrStabilizedRV;
        sh2_RotationVector_t_2 arvrStabilizedGRV;
        sh2_GyroIntegratedRV_t_2 gyroIntegratedRV;
        sh2_IZroRequest_t_2 izroRequest;
    } un;
} sh2_SensorValue_t_2;

int sh2_decodeSensorEvent_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
