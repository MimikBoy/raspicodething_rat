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

/*
 * BNO08x Sensor Event decoding
 */

#include "sh2_SensorValue2.h"
#include "sh2_err2.h"
#include "sh2_util2.h"

#define SCALE_Q_2(n) (1.0f / (1 << n))

const float scaleRadToDeg_2 = 180.0 / 3.14159265358;

// ------------------------------------------------------------------------
// Forward declarations

static int decodeRawAccelerometer_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeAccelerometer_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeLinearAcceleration_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeGravity_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeRawGyroscope_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeGyroscopeCalibrated_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeGyroscopeUncal_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeRawMagnetometer_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeMagneticFieldCalibrated_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeMagneticFieldUncal_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeRotationVector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeGameRotationVector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeGeomagneticRotationVector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodePressure_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeAmbientLight_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeHumidity_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeProximity_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeTemperature_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeReserved_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeTapDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeStepDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeStepCounter_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeSignificantMotion_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeStabilityClassifier_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeShakeDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeFlipDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodePickupDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeStabilityDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodePersonalActivityClassifier_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeSleepDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeTiltDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodePocketDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeCircleDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeHeartRateMonitor_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeArvrStabilizedRV_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeArvrStabilizedGRV_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeGyroIntegratedRV_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);
static int decodeIZroRequest_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event);

// ------------------------------------------------------------------------
// Public API

int sh2_decodeSensorEvent_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    // Fill out fields of *value based on *event, converting data from message representation
    // to natural representation.

    int rc = SH2_OK_2;

    value->sensorId = event->reportId;
    value->timestamp = event->timestamp_uS;

    if (value->sensorId != SH2_GYRO_INTEGRATED_RV_2) {
        value->sequence = event->report[1];
        value->status = event->report[2] & 0x03;
    }
    else {
        value->sequence = 0;
        value->status = 0;
    }

    // extract delay field (100uS units)
    
    
    switch (value->sensorId) {
        case SH2_RAW_ACCELEROMETER_2:
            rc = decodeRawAccelerometer_2(value, event);
            break;
        case SH2_ACCELEROMETER_2:
            rc = decodeAccelerometer_2(value, event);
            break;
        case SH2_LINEAR_ACCELERATION_2:
            rc = decodeLinearAcceleration_2(value, event);
            break;
        case SH2_GRAVITY_2:
            rc = decodeGravity_2(value, event);
            break;
        case SH2_RAW_GYROSCOPE_2:
            rc = decodeRawGyroscope_2(value, event);
            break;
        case SH2_GYROSCOPE_CALIBRATED_2:
            rc = decodeGyroscopeCalibrated_2(value, event);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED_2:
            rc = decodeGyroscopeUncal_2(value, event);
            break;
        case SH2_RAW_MAGNETOMETER_2:
            rc = decodeRawMagnetometer_2(value, event);
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED_2:
            rc = decodeMagneticFieldCalibrated_2(value, event);
            break;
        case SH2_MAGNETIC_FIELD_UNCALIBRATED_2:
            rc = decodeMagneticFieldUncal_2(value, event);
            break;
        case SH2_ROTATION_VECTOR_2:
            rc = decodeRotationVector_2(value, event);
            break;
        case SH2_GAME_ROTATION_VECTOR_2:
            rc = decodeGameRotationVector_2(value, event);
            break;
        case SH2_GEOMAGNETIC_ROTATION_VECTOR_2:
            rc = decodeGeomagneticRotationVector_2(value, event);
            break;
        case SH2_PRESSURE_2:
            rc = decodePressure_2(value, event);
            break;
        case SH2_AMBIENT_LIGHT_2:
            rc = decodeAmbientLight_2(value, event);
            break;
        case SH2_HUMIDITY_2:
            rc = decodeHumidity_2(value, event);
            break;
        case SH2_PROXIMITY_2:
            rc = decodeProximity_2(value, event);
            break;
        case SH2_TEMPERATURE_2:
            rc = decodeTemperature_2(value, event);
            break;
        case SH2_RESERVED_2:
            rc = decodeReserved_2(value, event);
            break;
        case SH2_TAP_DETECTOR_2:
            rc = decodeTapDetector_2(value, event);
            break;
        case SH2_STEP_DETECTOR_2:
            rc = decodeStepDetector_2(value, event);
            break;
        case SH2_STEP_COUNTER_2:
            rc = decodeStepCounter_2(value, event);
            break;
        case SH2_SIGNIFICANT_MOTION_2:
            rc = decodeSignificantMotion_2(value, event);
            break;
        case SH2_STABILITY_CLASSIFIER_2:
            rc = decodeStabilityClassifier_2(value, event);
            break;
        case SH2_SHAKE_DETECTOR_2:
            rc = decodeShakeDetector_2(value, event);
            break;
        case SH2_FLIP_DETECTOR_2:
            rc = decodeFlipDetector_2(value, event);
            break;
        case SH2_PICKUP_DETECTOR_2:
            rc = decodePickupDetector_2(value, event);
            break;
        case SH2_STABILITY_DETECTOR_2:
            rc = decodeStabilityDetector_2(value, event);
            break;
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER_2:
            rc = decodePersonalActivityClassifier_2(value, event);
            break;
        case SH2_SLEEP_DETECTOR_2:
            rc = decodeSleepDetector_2(value, event);
            break;
        case SH2_TILT_DETECTOR_2:
            rc = decodeTiltDetector_2(value, event);
            break;
        case SH2_POCKET_DETECTOR_2:
            rc = decodePocketDetector_2(value, event);
            break;
        case SH2_CIRCLE_DETECTOR_2:
            rc = decodeCircleDetector_2(value, event);
            break;
        case SH2_HEART_RATE_MONITOR_2:
            rc = decodeHeartRateMonitor_2(value, event);
            break;
        case SH2_ARVR_STABILIZED_RV_2:
            rc = decodeArvrStabilizedRV_2(value, event);
            break;
        case SH2_ARVR_STABILIZED_GRV_2:
            rc = decodeArvrStabilizedGRV_2(value, event);
            break;
        case SH2_GYRO_INTEGRATED_RV_2:
            rc = decodeGyroIntegratedRV_2(value, event);
            break;
        case SH2_IZRO_MOTION_REQUEST_2:
            rc = decodeIZroRequest_2(value, event);
            break;
        default:
            // Unknown report id
            rc = SH2_ERR_2;
            break;
    }

    return rc;
}

// ------------------------------------------------------------------------
// Private utility functions

static int decodeRawAccelerometer_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.rawAccelerometer.x = read16(&event->report[4]);
    value->un.rawAccelerometer.y = read16(&event->report[6]);
    value->un.rawAccelerometer.z = read16(&event->report[8]);
    value->un.rawAccelerometer.timestamp = read32(&event->report[12]);

    return SH2_OK_2;
}

static int decodeAccelerometer_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.accelerometer.x = read16(&event->report[4]) * SCALE_Q_2(8);
    value->un.accelerometer.y = read16(&event->report[6]) * SCALE_Q_2(8);
    value->un.accelerometer.z = read16(&event->report[8]) * SCALE_Q_2(8);

    return SH2_OK_2;
}

static int decodeLinearAcceleration_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.linearAcceleration.x = read16(&event->report[4]) * SCALE_Q_2(8);
    value->un.linearAcceleration.y = read16(&event->report[6]) * SCALE_Q_2(8);
    value->un.linearAcceleration.z = read16(&event->report[8]) * SCALE_Q_2(8);

    return SH2_OK_2;
}

static int decodeGravity_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.gravity.x = read16(&event->report[4]) * SCALE_Q_2(8);
    value->un.gravity.y = read16(&event->report[6]) * SCALE_Q_2(8);
    value->un.gravity.z = read16(&event->report[8]) * SCALE_Q_2(8);

    return SH2_OK_2;
}

static int decodeRawGyroscope_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.rawGyroscope.x = read16(&event->report[4]);
    value->un.rawGyroscope.y = read16(&event->report[6]);
    value->un.rawGyroscope.z = read16(&event->report[8]);
    value->un.rawGyroscope.temperature = read16(&event->report[10]);
    value->un.rawGyroscope.timestamp = read32(&event->report[12]);

    return SH2_OK_2;
}

static int decodeGyroscopeCalibrated_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.gyroscope.x = read16(&event->report[4]) * SCALE_Q_2(9);
    value->un.gyroscope.y = read16(&event->report[6]) * SCALE_Q_2(9);
    value->un.gyroscope.z = read16(&event->report[8]) * SCALE_Q_2(9);

    return SH2_OK_2;
}

static int decodeGyroscopeUncal_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.gyroscopeUncal.x = read16(&event->report[4]) * SCALE_Q_2(9);
    value->un.gyroscopeUncal.y = read16(&event->report[6]) * SCALE_Q_2(9);
    value->un.gyroscopeUncal.z = read16(&event->report[8]) * SCALE_Q_2(9);

    value->un.gyroscopeUncal.biasX = read16(&event->report[10]) * SCALE_Q_2(9);
    value->un.gyroscopeUncal.biasY = read16(&event->report[12]) * SCALE_Q_2(9);
    value->un.gyroscopeUncal.biasZ = read16(&event->report[14]) * SCALE_Q_2(9);

    return SH2_OK_2;
}

static int decodeRawMagnetometer_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.rawMagnetometer.x = read16(&event->report[4]);
    value->un.rawMagnetometer.y = read16(&event->report[6]);
    value->un.rawMagnetometer.z = read16(&event->report[8]);
    value->un.rawMagnetometer.timestamp = read32(&event->report[12]);

    return SH2_OK_2;
}

static int decodeMagneticFieldCalibrated_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.magneticField.x = read16(&event->report[4]) * SCALE_Q_2(4);
    value->un.magneticField.y = read16(&event->report[6]) * SCALE_Q_2(4);
    value->un.magneticField.z = read16(&event->report[8]) * SCALE_Q_2(4);

    return SH2_OK_2;
}

static int decodeMagneticFieldUncal_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.magneticFieldUncal.x = read16(&event->report[4]) * SCALE_Q_2(4);
    value->un.magneticFieldUncal.y = read16(&event->report[6]) * SCALE_Q_2(4);
    value->un.magneticFieldUncal.z = read16(&event->report[8]) * SCALE_Q_2(4);

    value->un.magneticFieldUncal.biasX = read16(&event->report[10]) * SCALE_Q_2(4);
    value->un.magneticFieldUncal.biasY = read16(&event->report[12]) * SCALE_Q_2(4);
    value->un.magneticFieldUncal.biasZ = read16(&event->report[14]) * SCALE_Q_2(4);

    return SH2_OK_2;
}

static int decodeRotationVector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.rotationVector.i = read16(&event->report[4]) * SCALE_Q_2(14);
    value->un.rotationVector.j = read16(&event->report[6]) * SCALE_Q_2(14);
    value->un.rotationVector.k = read16(&event->report[8]) * SCALE_Q_2(14);
    value->un.rotationVector.real = read16(&event->report[10]) * SCALE_Q_2(14);
    value->un.rotationVector.accuracy = read16(&event->report[12]) * SCALE_Q_2(12);

    return SH2_OK_2;
}

static int decodeGameRotationVector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.gameRotationVector.i = read16(&event->report[4]) * SCALE_Q_2(14);
    value->un.gameRotationVector.j = read16(&event->report[6]) * SCALE_Q_2(14);
    value->un.gameRotationVector.k = read16(&event->report[8]) * SCALE_Q_2(14);
    value->un.gameRotationVector.real = read16(&event->report[10]) * SCALE_Q_2(14);

    return SH2_OK_2;
}

static int decodeGeomagneticRotationVector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.geoMagRotationVector.i = read16(&event->report[4]) * SCALE_Q_2(14);
    value->un.geoMagRotationVector.j = read16(&event->report[6]) * SCALE_Q_2(14);
    value->un.geoMagRotationVector.k = read16(&event->report[8]) * SCALE_Q_2(14);
    value->un.geoMagRotationVector.real = read16(&event->report[10]) * SCALE_Q_2(14);
    value->un.geoMagRotationVector.accuracy = read16(&event->report[12]) * SCALE_Q_2(12);

    return SH2_OK_2;
}

static int decodePressure_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.pressure.value = read32(&event->report[4]) * SCALE_Q_2(20);

    return SH2_OK_2;
}

static int decodeAmbientLight_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.ambientLight.value = read32(&event->report[4]) * SCALE_Q_2(8);

    return SH2_OK_2;
}

static int decodeHumidity_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.humidity.value = read16(&event->report[4]) * SCALE_Q_2(8);

    return SH2_OK_2;
}

static int decodeProximity_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.proximity.value = read16(&event->report[4]) * SCALE_Q_2(4);

    return SH2_OK_2;
}

static int decodeTemperature_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.temperature.value = read16(&event->report[4]) * SCALE_Q_2(7);

    return SH2_OK_2;
}

static int decodeReserved_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.reserved.tbd = read16(&event->report[4]) * SCALE_Q_2(7);

    return SH2_OK_2;
}

static int decodeTapDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.tapDetector.flags = event->report[4];

    return SH2_OK_2;
}

static int decodeStepDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.stepDetector.latency = readu32(&event->report[4]);

    return SH2_OK_2;
}

static int decodeStepCounter_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.stepCounter.latency = readu32(&event->report[4]);
    value->un.stepCounter.steps = readu32(&event->report[8]);

    return SH2_OK_2;
}

static int decodeSignificantMotion_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.sigMotion.motion = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodeStabilityClassifier_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.stabilityClassifier.classification = event->report[4];

    return SH2_OK_2;
}

static int decodeShakeDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.shakeDetector.shake = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodeFlipDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.flipDetector.flip = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodePickupDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.pickupDetector.pickup = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodeStabilityDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.stabilityDetector.stability = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodePersonalActivityClassifier_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.personalActivityClassifier.page = event->report[4] & 0x7F;
    value->un.personalActivityClassifier.lastPage = ((event->report[4] & 0x80) != 0);
    value->un.personalActivityClassifier.mostLikelyState = event->report[5];
    for (int n = 0; n < 10; n++) {
        value->un.personalActivityClassifier.confidence[n] = event->report[6+n];
    }
    
    return SH2_OK_2;
}

static int decodeSleepDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.sleepDetector.sleepState = event->report[4];

    return SH2_OK_2;
}

static int decodeTiltDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.tiltDetector.tilt = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodePocketDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.pocketDetector.pocket = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodeCircleDetector_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.circleDetector.circle = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodeHeartRateMonitor_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.heartRateMonitor.heartRate = readu16(&event->report[4]);

    return SH2_OK_2;
}

static int decodeArvrStabilizedRV_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.arvrStabilizedRV.i = read16(&event->report[4]) * SCALE_Q_2(14);
    value->un.arvrStabilizedRV.j = read16(&event->report[6]) * SCALE_Q_2(14);
    value->un.arvrStabilizedRV.k = read16(&event->report[8]) * SCALE_Q_2(14);
    value->un.arvrStabilizedRV.real = read16(&event->report[10]) * SCALE_Q_2(14);
    value->un.arvrStabilizedRV.accuracy = read16(&event->report[12]) * SCALE_Q_2(12);

    return SH2_OK_2;
}

static int decodeArvrStabilizedGRV_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.arvrStabilizedGRV.i = read16(&event->report[4]) * SCALE_Q_2(14);
    value->un.arvrStabilizedGRV.j = read16(&event->report[6]) * SCALE_Q_2(14);
    value->un.arvrStabilizedGRV.k = read16(&event->report[8]) * SCALE_Q_2(14);
    value->un.arvrStabilizedGRV.real = read16(&event->report[10]) * SCALE_Q_2(14);

    return SH2_OK_2;
}

static int decodeGyroIntegratedRV_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.gyroIntegratedRV.i = read16(&event->report[0]) * SCALE_Q_2(14);
    value->un.gyroIntegratedRV.j = read16(&event->report[2]) * SCALE_Q_2(14);
    value->un.gyroIntegratedRV.k = read16(&event->report[4]) * SCALE_Q_2(14);
    value->un.gyroIntegratedRV.real = read16(&event->report[6]) * SCALE_Q_2(14);
    value->un.gyroIntegratedRV.angVelX = read16(&event->report[8]) * SCALE_Q_2(10);
    value->un.gyroIntegratedRV.angVelY = read16(&event->report[10]) * SCALE_Q_2(10);
    value->un.gyroIntegratedRV.angVelZ = read16(&event->report[12]) * SCALE_Q_2(10);

    return SH2_OK_2;
}

static int decodeIZroRequest_2(sh2_SensorValue_t_2 *value, const sh2_SensorEvent_t_2 *event)
{
    value->un.izroRequest.intent = (sh2_IZroMotionIntent_t_2)event->report[4];
    value->un.izroRequest.request = (sh2_IZroMotionRequest_t_2)event->report[5];

    return SH2_OK_2;
}
