/*
 * Copyright 2015-2018 Hillcrest Laboratories, Inc.
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
 * @file sh2.h
 * @author David Wheeler
 * @date 22 Sept 2015
 * @brief API Definition for Hillcrest SH-2 Sensor Hub.
 *
 * The sh2 API provides an interface to the Hillcrest Labs sensor hub devices.
 */

#ifndef SH2_2_H
#define SH2_2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal2.h"

/***************************************************************************************
 * Public type definitions
 ***************************************************************************************/

/**
 * @brief Sensor Event
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define SH2_MAX_SENSOR_EVENT_LEN_2 (16)
typedef struct sh2_SensorEvent_2 {
    uint64_t timestamp_uS;
    uint8_t len;
    uint8_t reportId;
    uint8_t report[SH2_MAX_SENSOR_EVENT_LEN_2];
} sh2_SensorEvent_t_2;

typedef void (sh2_SensorCallback_t_2)(void * cookie, sh2_SensorEvent_t_2 *pEvent);

/**
 * @brief Product Id value
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_ProductId_s_2 {
    uint8_t resetCause;
    uint8_t swVersionMajor;
    uint8_t swVersionMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVersionPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} sh2_ProductId_t_2;

#define SH2_MAX_PROD_ID_ENTRIES_2 (5)
typedef struct sh2_ProductIds_s_2 {
    sh2_ProductId_t_2 entry[SH2_MAX_PROD_ID_ENTRIES_2];
    uint8_t numEntries;
} sh2_ProductIds_t_2;

/**
 * @brief List of sensor types supported by the hub
 *
 * See the SH-2 Reference Manual for more information on each type.
 */
enum sh2_SensorId_e_2 {
    SH2_RAW_ACCELEROMETER_2 = 0x14,
    SH2_ACCELEROMETER_2 = 0x01,
    SH2_LINEAR_ACCELERATION_2 = 0x04,
    SH2_GRAVITY_2 = 0x06,
    SH2_RAW_GYROSCOPE_2 = 0x15,
    SH2_GYROSCOPE_CALIBRATED_2 = 0x02,
    SH2_GYROSCOPE_UNCALIBRATED_2 = 0x07,
    SH2_RAW_MAGNETOMETER_2 = 0x16,
    SH2_MAGNETIC_FIELD_CALIBRATED_2 = 0x03,
    SH2_MAGNETIC_FIELD_UNCALIBRATED_2 = 0x0f,
    SH2_ROTATION_VECTOR_2 = 0x05,
    SH2_GAME_ROTATION_VECTOR_2 = 0x08,
    SH2_GEOMAGNETIC_ROTATION_VECTOR_2 = 0x09,
    SH2_PRESSURE_2 = 0x0a,
    SH2_AMBIENT_LIGHT_2 = 0x0b,
    SH2_HUMIDITY_2 = 0x0c,
    SH2_PROXIMITY_2 = 0x0d,
    SH2_TEMPERATURE_2 = 0x0e,
    SH2_RESERVED_2 = 0x17,
    SH2_TAP_DETECTOR_2 = 0x10,
    SH2_STEP_DETECTOR_2 = 0x18,
    SH2_STEP_COUNTER_2 = 0x11,
    SH2_SIGNIFICANT_MOTION_2 = 0x12,
    SH2_STABILITY_CLASSIFIER_2 = 0x13,
    SH2_SHAKE_DETECTOR_2 = 0x19,
    SH2_FLIP_DETECTOR_2 = 0x1a,
    SH2_PICKUP_DETECTOR_2 = 0x1b,
    SH2_STABILITY_DETECTOR_2 = 0x1c,
    SH2_PERSONAL_ACTIVITY_CLASSIFIER_2 = 0x1e,
    SH2_SLEEP_DETECTOR_2 = 0x1f,
    SH2_TILT_DETECTOR_2 = 0x20,
    SH2_POCKET_DETECTOR_2 = 0x21,
    SH2_CIRCLE_DETECTOR_2 = 0x22,
    SH2_HEART_RATE_MONITOR_2 = 0x23,
    SH2_ARVR_STABILIZED_RV_2 = 0x28,
    SH2_ARVR_STABILIZED_GRV_2 = 0x29,
    SH2_GYRO_INTEGRATED_RV_2 = 0x2A,
    SH2_IZRO_MOTION_REQUEST_2 = 0x2B,

    // UPDATE to reflect greatest sensor id
    SH2_MAX_SENSOR_ID_2 = 0x2B,
};
typedef uint8_t sh2_SensorId_t_2;

/**
 * @brief Sensor Configuration settings
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SensorConfig_2 {
    /* Change sensitivity enabled */
    bool changeSensitivityEnabled;  /**< @brief Enable reports on change */

    /* Change sensitivity - true if relative; false if absolute */
    bool changeSensitivityRelative;  /**< @brief Change reports relative (vs absolute) */

    /* Wake-up enabled */
    bool wakeupEnabled;  /**< @brief Wake host on event */

    /* Always on enabled */
    bool alwaysOnEnabled;  /**< @brief Sensor remains on in sleep state */
    /* 16-bit signed fixed point integer representing the value a
     * sensor output must exceed in order to trigger another input
     * report. A setting of 0 causes all reports to be sent.
     */
    uint16_t changeSensitivity;  /**< @brief Report-on-change threshold */

    /* Interval in microseconds between asynchronous input reports. */
    uint32_t reportInterval_us;  /**< @brief [uS] Report interval */

    /* Reserved field, not used. */
    uint32_t batchInterval_us;  /**< @brief [uS] Batch interval */

    /* Meaning is sensor specific */
    uint32_t sensorSpecific;  /**< @brief See SH-2 Reference Manual for details. */
} sh2_SensorConfig_t_2;

/**
 * @brief Sensor Metadata Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SensorMetadata_2 {
    uint8_t meVersion;   /**< @brief Motion Engine Version */
    uint8_t mhVersion;  /**< @brief Motion Hub Version */
    uint8_t shVersion;  /**< @brief SensorHub Version */
    uint32_t range;  /**< @brief Same units as sensor reports */
    uint32_t resolution;  /**< @brief Same units as sensor reports */
    uint16_t revision;  /**< @brief Metadata record format revision */
    uint16_t power_mA;    /**< @brief [mA] Fixed point 16Q10 format */
    uint32_t minPeriod_uS;  /**< @brief [uS] */
    uint32_t maxPeriod_uS;  /**< @brief [uS] */
    uint32_t fifoReserved;  /**< @brief (Unused) */
    uint32_t fifoMax;  /**< @brief (Unused) */
    uint32_t batchBufferBytes;  /**< @brief (Unused) */
    uint16_t qPoint1;     /**< @brief q point for sensor values */
    uint16_t qPoint2;     /**< @brief q point for accuracy or bias fields */
    uint16_t qPoint3;     /**< @brief q point for sensor data change sensitivity */
    uint32_t vendorIdLen; /**< @brief [bytes] */
    char vendorId[48];  /**< @brief Vendor name and part number */
    uint32_t sensorSpecificLen;  /**< @brief [bytes] */
    uint8_t sensorSpecific[48];  /**< @brief See SH-2 Reference Manual */
} sh2_SensorMetadata_t_2;

/**
 * @brief SensorHub Error Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_ErrorRecord_2 {
    uint8_t severity;   /**< @brief Error severity, 0: most severe. */
    uint8_t sequence;   /**< @brief Sequence number (by severity) */
    uint8_t source;     /**< @brief 1-MotionEngine, 2-MotionHub, 3-SensorHub, 4-Chip  */
    uint8_t error;      /**< @brief See SH-2 Reference Manual */
    uint8_t module;     /**< @brief See SH-2 Reference Manual */
    uint8_t code;       /**< @brief See SH-2 Reference Manual */
} sh2_ErrorRecord_t_2;

/**
 * @brief SensorHub Counter Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Counts_2 {
    uint32_t offered;   /**< @brief [events] */
    uint32_t accepted;  /**< @brief [events] */
    uint32_t on;        /**< @brief [events] */
    uint32_t attempted; /**< @brief [events] */
} sh2_Counts_t_2;

/**
 * @brief Values for specifying tare basis
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_TareBasis_2 {
    SH2_TARE_BASIS_ROTATION_VECTOR_2 = 0,             /**< @brief Use Rotation Vector */
    SH2_TARE_BASIS_GAMING_ROTATION_VECTOR_2 = 1,      /**< @brief Use Game Rotation Vector */
    SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR_2 = 2, /**< @brief Use Geomagnetic R.V. */
} sh2_TareBasis_t_2;

/**
 * @brief Bit Fields for specifying tare axes.
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_TareAxis_2 {
    SH2_TARE_X_2 = 1,  /**< @brief sh2_tareNow() axes bit field */
    SH2_TARE_Y_2 = 2,  /**< @brief sh2_tareNow() axes bit field */
    SH2_TARE_Z_2 = 4,  /**< @brief sh2_tareNow() axes bit field */
} sh2_TareAxis_t_2;

/**
 * @brief Quaternion (double precision floating point representation.)
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Quaternion_2 {
    double x;
    double y;
    double z;
    double w;
} sh2_Quaternion_t_2;

/**
 * @brief Oscillator type: Internal or External
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_OscType_2 {
    SH2_OSC_INTERNAL_2    = 0,
    SH2_OSC_EXT_CRYSTAL_2 = 1,
    SH2_OSC_EXT_CLOCK_2   = 2,
} sh2_OscType_t_2;

/**
 * @brief Calibration result
 *
 * See the SH-2 Reference Manual, Finish Calibration Response.
 */
typedef enum sh2_CalStatus_2 {
    SH2_CAL_SUCCESS_2 = 0,
    SH2_CAL_NO_ZRO_2,
    SH2_CAL_NO_STATIONARY_DETECTION_2,
    SH2_CAL_ROTATION_OUTSIDE_SPEC_2,
    SH2_CAL_ZRO_OUTSIDE_SPEC_2,
    SH2_CAL_ZGO_OUTSIDE_SPEC_2,
    SH2_CAL_GYRO_GAIN_OUTSIDE_SPEC_2,
    SH2_CAL_GYRO_PERIOD_OUTSIDE_SPEC_2,
    SH2_CAL_GYRO_DROPS_OUTSIDE_SPEC_2,
} sh2_CalStatus_t_2;

// FRS Record Ids
#define STATIC_CALIBRATION_AGM_2                   (0x7979)
#define NOMINAL_CALIBRATION_2                      (0x4D4D)
#define STATIC_CALIBRATION_SRA_2                   (0x8A8A)
#define NOMINAL_CALIBRATION_SRA_2                  (0x4E4E)
#define DYNAMIC_CALIBRATION_2                      (0x1F1F)
#define ME_POWER_MGMT_2                            (0xD3E2)
#define SYSTEM_ORIENTATION_2                       (0x2D3E)
#define ACCEL_ORIENTATION_2                        (0x2D41)
#define SCREEN_ACCEL_ORIENTATION_2                 (0x2D43)
#define GYROSCOPE_ORIENTATION_2                    (0x2D46)
#define MAGNETOMETER_ORIENTATION_2                 (0x2D4C)
#define ARVR_STABILIZATION_RV_2                    (0x3E2D)
#define ARVR_STABILIZATION_GRV_2                   (0x3E2E)
#define TAP_DETECT_CONFIG_2                        (0xC269)
#define SIG_MOTION_DETECT_CONFIG_2                 (0xC274)
#define SHAKE_DETECT_CONFIG_2                      (0x7D7D)
#define MAX_FUSION_PERIOD_2                        (0xD7D7)
#define SERIAL_NUMBER_2                            (0x4B4B)
#define ES_PRESSURE_CAL_2                          (0x39AF)
#define ES_TEMPERATURE_CAL_2                       (0x4D20)
#define ES_HUMIDITY_CAL_2                          (0x1AC9)
#define ES_AMBIENT_LIGHT_CAL_2                     (0x39B1)
#define ES_PROXIMITY_CAL_2                         (0x4DA2)
#define ALS_CAL_2                                  (0xD401)
#define PROXIMITY_SENSOR_CAL_2                     (0xD402)
#define PICKUP_DETECTOR_CONFIG_2                   (0x1B2A)
#define FLIP_DETECTOR_CONFIG_2                     (0xFC94)
#define STABILITY_DETECTOR_CONFIG_2                (0xED85)
#define ACTIVITY_TRACKER_CONFIG_2                  (0xED88)
#define SLEEP_DETECTOR_CONFIG_2                    (0xED87)
#define TILT_DETECTOR_CONFIG_2                     (0xED89)
#define POCKET_DETECTOR_CONFIG_2                   (0xEF27)
#define CIRCLE_DETECTOR_CONFIG_2                   (0xEE51)
#define USER_RECORD_2                              (0x74B4)
#define ME_TIME_SOURCE_SELECT_2                    (0xD403)
#define UART_FORMAT_2                              (0xA1A1)
#define GYRO_INTEGRATED_RV_CONFIG_2                (0xA1A2)
#define FRS_ID_META_RAW_ACCELEROMETER_2            (0xE301)
#define FRS_ID_META_ACCELEROMETER_2                (0xE302)
#define FRS_ID_META_LINEAR_ACCELERATION_2          (0xE303)
#define FRS_ID_META_GRAVITY_2                      (0xE304)
#define FRS_ID_META_RAW_GYROSCOPE_2                (0xE305)
#define FRS_ID_META_GYROSCOPE_CALIBRATED_2         (0xE306)
#define FRS_ID_META_GYROSCOPE_UNCALIBRATED_2       (0xE307)
#define FRS_ID_META_RAW_MAGNETOMETER_2             (0xE308)
#define FRS_ID_META_MAGNETIC_FIELD_CALIBRATED_2    (0xE309)
#define FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED_2  (0xE30A)
#define FRS_ID_META_ROTATION_VECTOR_2              (0xE30B)
#define FRS_ID_META_GAME_ROTATION_VECTOR_2         (0xE30C)
#define FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR_2  (0xE30D)
#define FRS_ID_META_PRESSURE_2                     (0xE30E)
#define FRS_ID_META_AMBIENT_LIGHT_2                (0xE30F)
#define FRS_ID_META_HUMIDITY_2                     (0xE310)
#define FRS_ID_META_PROXIMITY_2                    (0xE311)
#define FRS_ID_META_TEMPERATURE_2                  (0xE312)
#define FRS_ID_META_TAP_DETECTOR_2                 (0xE313)
#define FRS_ID_META_STEP_DETECTOR_2                (0xE314)
#define FRS_ID_META_STEP_COUNTER_2                 (0xE315)
#define FRS_ID_META_SIGNIFICANT_MOTION_2           (0xE316)
#define FRS_ID_META_STABILITY_CLASSIFIER_2         (0xE317)
#define FRS_ID_META_SHAKE_DETECTOR_2               (0xE318)
#define FRS_ID_META_FLIP_DETECTOR_2                (0xE319)
#define FRS_ID_META_PICKUP_DETECTOR_2              (0xE31A)
#define FRS_ID_META_STABILITY_DETECTOR_2           (0xE31B)
#define FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER_2 (0xE31C)
#define FRS_ID_META_SLEEP_DETECTOR_2               (0xE31D)
#define FRS_ID_META_TILT_DETECTOR_2                (0xE31E)
#define FRS_ID_META_POCKET_DETECTOR_2              (0xE31F)
#define FRS_ID_META_CIRCLE_DETECTOR_2              (0xE320)
#define FRS_ID_META_HEART_RATE_MONITOR_2           (0xE321)
#define FRS_ID_META_ARVR_STABILIZED_RV_2           (0xE322)
#define FRS_ID_META_ARVR_STABILIZED_GRV_2          (0xE323)
#define FRS_ID_META_GYRO_INTEGRATED_RV_2           (0xE324)

/**
 * @brief Interactive ZRO Motion Intent
 *
 * See the SH-2 Reference Manual, 6.4.13
 */
typedef enum {
    SH2_IZRO_MI_UNKNOWN_2 = 0,
    SH2_IZRO_MI_STATIONARY_NO_VIBRATION_2,
    SH2_IZRO_MI_STATIONARY_WITH_VIBRATION_2,
    SH2_IZRO_MI_IN_MOTION_2,
} sh2_IZroMotionIntent_t_2;

/**
 * @brief Interactive ZRO Motion Intent
 *
 * See the SH-2 Reference Manual, 6.4.13
 */
typedef enum {
    SH2_IZRO_MR_NO_REQUEST_2 = 0,
    SH2_IZRO_MR_STAY_STATIONARY_2,
    SH2_IZRO_MR_STATIONARY_NON_URGENT_2,
    SH2_IZRO_MR_STATIONARY_URGENT_2,
} sh2_IZroMotionRequest_t_2;


/**
* @brief Asynchronous Event
*
* Represents reset events and other non-sensor events received from SH-2 sensor hub.
*/

enum sh2_AsyncEventId_e_2 {
    SH2_RESET_2,
    SH2_SHTP_EVENT_2,
    SH2_GET_FEATURE_RESP_2,
};
typedef enum sh2_AsyncEventId_e_2 sh2_AsyncEventId_t_2;

enum sh2_ShtpEvent_e_2 {
    SH2_SHTP_TX_DISCARD_2 = 0,
    SH2_SHTP_SHORT_FRAGMENT_2 = 1,
    SH2_SHTP_TOO_LARGE_PAYLOADS_2 = 2,
    SH2_SHTP_BAD_RX_CHAN_2 = 3,
    SH2_SHTP_BAD_TX_CHAN_2 = 4,
};
typedef uint8_t sh2_ShtpEvent_t_2;

typedef struct sh2_SensorConfigResp_e_2 {
    sh2_SensorId_t_2 sensorId;
    sh2_SensorConfig_t_2 sensorConfig;
} sh2_SensorConfigResp_t_2;

typedef struct sh2_AsyncEvent_2 {
    uint32_t eventId;
    union {
        sh2_ShtpEvent_t_2 shtpEvent;
        sh2_SensorConfigResp_t_2 sh2SensorConfigResp;
    };
} sh2_AsyncEvent_t_2;

typedef void (sh2_EventCallback_t_2)(void * cookie, sh2_AsyncEvent_t_2 *pEvent);


/***************************************************************************************
 * Public API
 **************************************************************************************/

int sh2_open_2(sh2_Hal_t_2 *pHal,
               sh2_EventCallback_t_2 *eventCallback, void *eventCookie);

void sh2_close_2(void);

void sh2_service_2(void);

int sh2_setSensorCallback_2(sh2_SensorCallback_t_2 *callback, void *cookie);

/**
 * @brief Reset the sensor hub device by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devReset_2(void);

/**
 * @brief Turn sensor hub on by sending ON (2) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devOn_2(void);

/**
 * @brief Put sensor hub in sleep state by sending SLEEP (3) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devSleep_2(void);

/**
 * @brief Get Product ID information from Sensorhub.
 *
 * @param  prodIds Pointer to structure that will receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getProdIds_2(sh2_ProductIds_t_2 *prodIds);

/**
 * @brief Get sensor configuration.
 *
 * @param  sensorId Which sensor to query.
 * @param  config SensorConfig structure to store results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getSensorConfig_2(sh2_SensorId_t_2 sensorId, sh2_SensorConfig_t_2 *config);

/**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig_2(sh2_SensorId_t_2 sensorId, const sh2_SensorConfig_t_2 *pConfig);

/**
 * @brief Get metadata related to a sensor.
 *
 * @param  sensorId Which sensor to query.
 * @param  pData Pointer to structure to receive the results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getMetadata_2(sh2_SensorId_t_2 sensorId, sh2_SensorMetadata_t_2 *pData);

/**
 * @brief Get an FRS record.
 *
 * @param  recordId Which FRS Record to retrieve.
 * @param  pData pointer to buffer to receive the results
 * @param[in] words Size of pData buffer, in 32-bit words.
 * @param[out] words Number of 32-bit words retrieved.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getFrs_2(uint16_t recordId, uint32_t *pData, uint16_t *words);

/**
 * @brief Set an FRS record
 *
 * @param  recordId Which FRS Record to set.
 * @param  pData pointer to buffer containing the new data.
 * @param  words number of 32-bit words to write.  (0 to delete record.)
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setFrs_2(uint16_t recordId, uint32_t *pData, uint16_t words);

/**
 * @brief Get error counts.
 *
 * @param  severity Only errors of this severity or greater are returned.
 * @param  pErrors Buffer to receive error codes.
 * @param  numErrors size of pErrors array
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getErrors_2(uint8_t severity, sh2_ErrorRecord_t_2 *pErrors, uint16_t *numErrors);

/**
 * @brief Read counters related to a sensor.
 *
 * @param  sensorId Which sensor to operate on.
 * @param  pCounts Pointer to Counts structure that will receive data.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCounts_2(sh2_SensorId_t_2 sensorId, sh2_Counts_t_2 *pCounts);

/**
 * @brief Clear counters related to a sensor.
 *
 * @param  sensorId which sensor to operate on.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearCounts_2(sh2_SensorId_t_2 sensorId);

/**
 * @brief Perform a tare operation on one or more axes.
 *
 * @param  axes Bit mask specifying which axes should be tared.
 * @param  basis Which rotation vector to use as the basis for Tare adjustment.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setTareNow_2(uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t_2 basis);

/**
 * @brief Clears the previously applied tare operation.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearTare_2(void);

/**
 * @brief Persist the results of last tare operation to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_persistTare_2(void);

/**
 * @brief Set the current run-time sensor reorientation. (Set to zero to clear tare.)
 *
 * @param  orientation Quaternion rotation vector to apply as new tare.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setReorientation_2(sh2_Quaternion_t_2 *orientation);

/**
 * @brief Command the sensorhub to reset.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_reinitialize_2(void);

/**
 * @brief Save Dynamic Calibration Data to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_saveDcdNow_2(void);

/**
 * @brief Get Oscillator type.
 *
 * @param  pOscType pointer to data structure to receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getOscType_2(sh2_OscType_t_2 *pOscType);

// Flags for sensors field of sh_calConfig
#define SH2_CAL_ACCEL_2 (0x01)
#define SH2_CAL_GYRO_2  (0x02)
#define SH2_CAL_MAG_2   (0x04)
#define SH2_CAL_PLANAR_2 (0x08)

/**
 * @brief Enable/Disable dynamic calibration for certain sensors
 *
 * @param  sensors Bit mask to configure which sensors are affected.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setCalConfig_2(uint8_t sensors);

/**
 * @brief Get dynamic calibration configuration settings.
 *
 * @param  pSensors pointer to Bit mask, set on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCalConfig_2(uint8_t *pSensors);

/**
 * @brief Configure automatic saving of dynamic calibration data.
 *
 * @param  enabled Enable or Disable DCD auto-save.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setDcdAutoSave_2(bool enabled);

/**
 * @brief Immediately issue all buffered sensor reports from a given sensor.
 *
 * @param  sensorId Which sensor reports to flush.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_flush_2(sh2_SensorId_t_2 sensorId);

/**
 * @brief Command clear DCD in RAM, then reset sensor hub.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearDcdAndReset_2(void);

/**
 * @brief Start simple self-calibration procedure.
 *
 * @parameter interval_us sensor report interval, uS.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_startCal_2(uint32_t interval_us);

/**
 * @brief Finish simple self-calibration procedure.
 *
 * @parameter status contains calibration status code on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_finishCal_2(sh2_CalStatus_t_2 *status);

/**
 * @brief send Interactive ZRO Request.
 *
 * @parameter intent Inform the sensor hub what sort of motion should be in progress.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setIZro_2(sh2_IZroMotionIntent_t_2 intent);

#ifdef __cplusplus
} // extern "C"
#endif

#endif 
