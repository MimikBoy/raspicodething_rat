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
 * @file sh2.c
 * @author David Wheeler
 * @date 22 Sept 2015
 * @brief API Definition for Hillcrest SH-2 Sensor Hub.
 *
 * The sh2 API provides functions for opening a session with
 * the sensor hub and performing all supported operations with it.
 * This includes enabling sensors and reading events as well as
 * other housekeeping functions.
 *
 */

#include "sh22.h"
#include "sh2_err2.h"
#include "shtp2.h"
#include "sh2_util2.h"

#include <string.h>

// ------------------------------------------------------------------------
// Private type definitions

#define GUID_EXECUTABLE_2 (1)
#define GUID_SENSORHUB_2 (2)

// executable/device channel responses
#define EXECUTABLE_DEVICE_CMD_RESET_2 (1)
#define EXECUTABLE_DEVICE_CMD_ON_2    (2)
#define EXECUTABLE_DEVICE_CMD_SLEEP_2 (3)

// executable/device channel responses
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE_2 (1)

// Tags for sensorhub app advertisements.
#define TAG_SH2_VERSION_2 (0x80)
#define TAG_SH2_REPORT_LENGTHS_2 (0x81)

// Max length of sensorhub version string.
#define MAX_VER_LEN_2 (16)

// Max number of report ids supported
#define SH2_MAX_REPORT_IDS_2 (64)

#if defined(_MSC_VER)
#define PACKED_STRUCT_2 struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT_2 struct __attribute__((packed))
#else 
#define PACKED_STRUCT_2 __packed struct
#endif

#define ADVERT_TIMEOUT_US_2 (200000)

// Command and Subcommand values
#define SH2_CMD_ERRORS_2                 1
#define SH2_CMD_COUNTS_2                 2
#define     SH2_COUNTS_GET_COUNTS_2          0
#define     SH2_COUNTS_CLEAR_COUNTS_2        1
#define SH2_CMD_TARE_2                   3
#define     SH2_TARE_TARE_NOW_2              0
#define     SH2_TARE_PERSIST_TARE_2          1
#define     SH2_TARE_SET_REORIENTATION_2     2
#define SH2_CMD_INITIALIZE_2             4
#define     SH2_INIT_SYSTEM_2                1
#define     SH2_INIT_UNSOLICITED_2           0x80
// #define SH2_CMD_FRS                    5 /* Depreciated */
#define SH2_CMD_DCD_2                    6
#define SH2_CMD_ME_CAL_2                 7
#define SH2_CMD_DCD_SAVE_2               9
#define SH2_CMD_GET_OSC_TYPE_2           0x0A
#define SH2_CMD_CLEAR_DCD_AND_RESET_2    0x0B
#define SH2_CMD_CAL_2                    0x0C
#define     SH2_CAL_START_2                   0
#define     SH2_CAL_FINISH_2                  1
#define SH2_CMD_BOOTLOADER_2             0x0D     /* SH-2 Reference Manual 6.4.12 */
#define     SH2_BL_MODE_REQ_2                 0
#define     SH2_BL_STATUS_REQ_2               1
#define SH2_CMD_INTERACTIVE_ZRO_2        0x0E     /* SH-2 Reference Manual 6.4.13 */

// SENSORHUB_COMMAND_REQ
#define SENSORHUB_COMMAND_REQ_2        (0xF2)
#define COMMAND_PARAMS_2 (9)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t p[COMMAND_PARAMS_2];
} CommandReq_t_2;

// SENSORHUB_COMMAND_RESP
#define SENSORHUB_COMMAND_RESP_2       (0xF1)
#define RESPONSE_VALUES_2 (11)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t commandSeq;
    uint8_t respSeq;
    uint8_t r[RESPONSE_VALUES_2];
} CommandResp_t_2;

// SENSORHUB_PROD_ID_REQ
#define SENSORHUB_PROD_ID_REQ_2        (0xF9)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;  
    uint8_t reserved;
} ProdIdReq_t_2;

// SENSORHUB_PROD_ID_RESP
#define SENSORHUB_PROD_ID_RESP_2       (0xF8)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t resetCause;
    uint8_t swVerMajor;
    uint8_t swVerMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVerPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} ProdIdResp_t_2;

// Report definitions
// Bit fields for Feature Report flags
#define FEAT_CHANGE_SENSITIVITY_RELATIVE_2 (1)
#define FEAT_CHANGE_SENSITIVITY_ABSOLUTE_2 (0)
#define FEAT_CHANGE_SENSITIVITY_ENABLED_2  (2)
#define FEAT_CHANGE_SENSITIVITY_DISABLED_2 (0)
#define FEAT_WAKE_ENABLED_2                (4)
#define FEAT_WAKE_DISABLED_2               (0)
#define FEAT_ALWAYS_ON_ENABLED_2           (8)
#define FEAT_ALWAYS_ON_DISABLED_2          (0)

// GET_FEATURE_REQ
#define SENSORHUB_GET_FEATURE_REQ_2    (0xFE)
typedef PACKED_STRUCT_2{
    uint8_t reportId;
    uint8_t featureReportId;
} GetFeatureReq_t_2;

// SENSORHUB_GET_FEATURE_RESP
#define SENSORHUB_GET_FEATURE_RESP_2   (0xFC)
typedef PACKED_STRUCT_2{
    uint8_t reportId;
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} GetFeatureResp_t_2;


typedef struct sh2_s_2 sh2_t_2;

typedef int (sh2_OpStart_t_2)(sh2_t_2 *pSh2);
typedef void (sh2_OpRx_t_2)(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len);

typedef struct sh2_Op_s_2 {
    uint32_t timeout_us;
    sh2_OpStart_t_2 *start;
    sh2_OpRx_t_2 *rx;
} sh2_Op_t_2;

// Parameters and state information for the operation in progress
typedef union {
    struct {
        CommandReq_t_2 req;
    } sendCmd_2;
    struct {
        sh2_ProductIds_t_2 *pProdIds;
        uint8_t nextEntry;
        uint8_t expectedEntries;
    } getProdIds_2;
    struct {
        sh2_SensorConfig_t_2 *pConfig;
        sh2_SensorId_t_2 sensorId;
    } getSensorConfig_2;
    struct {
        const sh2_SensorConfig_t_2 *pConfig;
        sh2_SensorId_t_2 sensorId;
    } setSensorConfig_2;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t *pWords;
        uint16_t nextOffset;
    } getFrs_2;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t words;
        uint16_t offset;
    } setFrs_2;
    struct {
        uint8_t severity;
        sh2_ErrorRecord_t_2 *pErrors;
        uint16_t *pNumErrors;
        uint16_t errsRead;
    } getErrors_2;
    struct {
        sh2_SensorId_t_2 sensorId;
        sh2_Counts_t_2 *pCounts;
    } getCounts_2;
    struct {
        uint8_t sensors;
    } calConfig_2;
    struct {
        uint8_t *pSensors;
    } getCalConfig_2;
    struct {
        sh2_SensorId_t_2 sensorId;
    } forceFlush_2;
    struct {
        sh2_OscType_t_2 *pOscType;
    } getOscType_2;
    struct {
        uint32_t interval_us;
    } startCal_2;
    struct {
        sh2_CalStatus_t_2 status;
    } finishCal_2;
} sh2_OpData_t_2;

// Max length of an FRS record, words.
#define MAX_FRS_WORDS_2 (72)

struct sh2_s_2 {
    // Pointer to the SHTP HAL
    sh2_Hal_t_2 *pHal;

    // associated SHTP instance
    void *pShtp;
    
    volatile bool resetComplete;
    bool advertDone;
    uint8_t executableChan;
    uint8_t controlChan;
    char version[MAX_VER_LEN_2+1];

    // Report lengths
    struct {
        uint8_t id;
        uint8_t len;
    } report_2[SH2_MAX_REPORT_IDS_2];

    // Multi-step operation support
    const sh2_Op_t_2 *pOp;
    int opStatus;
    sh2_OpData_t_2 opData;
    uint8_t lastCmdId;
    uint8_t cmdSeq;
    uint8_t nextCmdSeq;
    
    // Event callback and it's cookie
    sh2_EventCallback_t_2 *eventCallback;
    void * eventCookie;

    // Sensor callback and it's cookie
    sh2_SensorCallback_t_2 *sensorCallback;
    void * sensorCookie;

    // Storage space for reading sensor metadata
    uint32_t frsData[MAX_FRS_WORDS_2];
    uint16_t frsDataLen;

    // Stats
    uint32_t execBadPayload;
    uint32_t emptyPayloads;
    uint32_t unknownReportIds;

};

#define SENSORHUB_BASE_TIMESTAMP_REF_2 (0xFB)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint32_t timebase;
} BaseTimestampRef_t_2;

#define SENSORHUB_TIMESTAMP_REBASE_2   (0xFA)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    int32_t timebase;
} TimestampRebase_t_2;

// SENSORHUB_FORCE_SENSOR_FLUSH
#define SENSORHUB_FORCE_SENSOR_FLUSH_2 (0xF0)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushReq_t_2;

// SENSORHUB_FLUSH_COMPLETED    
#define SENSORHUB_FLUSH_COMPLETED_2    (0xEF)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushResp_t_2;

// ------------------------------------------------------------------------
// Private data

// SH2 state
sh2_t_2 _sh2_2;

// SH2 Async Event Message
static sh2_AsyncEvent_t_2 sh2AsyncEvent;

// ------------------------------------------------------------------------
// Private functions

// SH-2 transaction phases
static int opStart_2(sh2_t_2 *pSh2, const sh2_Op_t_2 *pOp)
{
    // return error if another operation already in progress
    if (pSh2->pOp) return SH2_ERR_OP_IN_PROGRESS_2;

    // Establish this operation as the new operation in progress
    pSh2->pOp = pOp;
    pSh2->opStatus = SH2_OK_2;
    int rc = pOp->start(pSh2);  // Call start method
    if (rc != SH2_OK_2) {
        // Unregister this operation
        pSh2->opStatus = rc;
        pSh2->pOp = 0;
    }

    return rc;
}

static void opRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{ 
    if ((pSh2->pOp != 0) &&                      // An operation is in progress
        (pSh2->pOp->rx != 0)) {                  // and it has an rx method
        pSh2->pOp->rx(pSh2, payload, len);  // Call receive method
    }
}

static void sensorhubAdvertHdlr_2(void *cookie, uint8_t tag, uint8_t len, uint8_t *value)
{
    sh2_t_2 *pSh2 = (sh2_t_2 *)cookie;
    
    switch (tag) {
        case TAG_SH2_VERSION_2:
            strcpy(pSh2->version, (const char *)value);
            break;

        case TAG_SH2_REPORT_LENGTHS_2:
        {
            uint8_t reports = len/2;
            if (reports > SH2_MAX_REPORT_IDS_2) {
                // Hub gave us more report lengths than we can store!
                reports = SH2_MAX_REPORT_IDS_2;
            }
        
            for (int n = 0; n < reports; n++) {
                pSh2->report_2[n].id = value[n*2];
                pSh2->report_2[n].len = value[n*2 + 1];
            }
            break;
        }
    
        case 0:
        {
            // 0 tag indicates end of advertisements for this app
            // At this time, the SHTP layer can give us our channel numbers
            pSh2->executableChan = shtp_chanNo_2(pSh2->pShtp, "executable", "device");
            pSh2->controlChan = shtp_chanNo_2(pSh2->pShtp, "sensorhub", "control");

            pSh2->advertDone = true;
            break;
        }
        
        default:
            break;
    }
}

static void sensorhubControlHdlr_2(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t_2 *pSh2 = (sh2_t_2 *)cookie;

    uint16_t cursor = 0;
    uint32_t count = 0;
    CommandResp_t_2 * pResp = 0;
    
    if (len == 0) {
        pSh2->emptyPayloads++;
        return;
    }

    while (cursor < len) {
        // Get next report id
        count++;
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = 0;
        for (int n = 0; n < SH2_MAX_REPORT_IDS_2; n++) {
            if (pSh2->report_2[n].id == reportId) {
                reportLen = pSh2->report_2[n].len;
                break;
            }
        }
        if (reportLen == 0) {
            // An unrecognized report id
            pSh2->unknownReportIds++;
            return;
        }
        else {
            // Check for unsolicited initialize response
            if (reportId == SENSORHUB_COMMAND_RESP_2) {
                pResp = (CommandResp_t_2 *)(payload+cursor);
                if ((pResp->command == (SH2_CMD_INITIALIZE_2 | SH2_INIT_UNSOLICITED_2)) &&
                    (pResp->r[1] == SH2_INIT_SYSTEM_2)) {
                    // This is an unsolicited INIT message.
                    // Is it time to call reset callback?
                }

            } // Check for Get Feature Response
            else if (reportId == SENSORHUB_GET_FEATURE_RESP_2) {
                if (pSh2->eventCallback) {
                    GetFeatureResp_t_2 * pGetFeatureResp;
                    pGetFeatureResp = (GetFeatureResp_t_2 *)(payload + cursor);

                    sh2AsyncEvent.eventId = SH2_GET_FEATURE_RESP_2;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorId = pGetFeatureResp->featureReportId;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivityEnabled = ((pGetFeatureResp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED_2) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivityRelative = ((pGetFeatureResp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE_2) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.wakeupEnabled = ((pGetFeatureResp->flags & FEAT_WAKE_ENABLED_2) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.alwaysOnEnabled = ((pGetFeatureResp->flags & FEAT_ALWAYS_ON_ENABLED_2) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivity = pGetFeatureResp->changeSensitivity;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.reportInterval_us = pGetFeatureResp->reportInterval_uS;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.batchInterval_us = pGetFeatureResp->batchInterval_uS;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.sensorSpecific = pGetFeatureResp->sensorSpecific;

                    pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
                }
            }

            // Hand off to operation in progress, if any
            opRx_2(pSh2, payload+cursor, reportLen);
            cursor += reportLen;
        }
    }
}

static int opCompleted_2(sh2_t_2 *pSh2, int status)
{
    // Record status
    pSh2->opStatus = status;

    // Signal that op is done.
    pSh2->pOp = 0;

    return SH2_OK_2;
}

static int opProcess_2(sh2_t_2 *pSh2, const sh2_Op_t_2 *pOp)
{
    int status = SH2_OK_2;
    uint32_t start_us = 0;

    start_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    
    status = opStart_2(&_sh2_2, pOp);
    if (status != SH2_OK_2) {
        return status;
    }

    uint32_t now_us = start_us;
    
    // While op not complete and not timed out.
    while ((pSh2->pOp != 0) &&
           ((pOp->timeout_us == 0) ||
            ((now_us-start_us) < pOp->timeout_us))) {
        // Service SHTP to poll the device.
        shtp_service_2(pSh2->pShtp);

        // Update the time
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }

    if (pSh2->pOp != 0) {
        // Operation has timed out.  Clean up.
        pSh2->pOp = 0;
        pSh2->opStatus = SH2_ERR_TIMEOUT_2;
    }

    return pSh2->opStatus;
}

static uint8_t getReportLen_2(sh2_t_2 *pSh2, uint8_t reportId)
{
    for (int n = 0; n < SH2_MAX_REPORT_IDS_2; n++) {
        if (pSh2->report_2[n].id == reportId) {
            return pSh2->report_2[n].len;
        }
    }

    return 0;
}

// Produce 64-bit microsecond timestamp for a sensor event
static uint64_t touSTimestamp_2(uint32_t hostInt, int32_t referenceDelta, uint16_t delay)
{
    static uint32_t lastHostInt = 0;
    static uint32_t rollovers = 0;
    uint64_t timestamp;

    // Count times hostInt timestamps rolled over to produce upper bits
    if (hostInt < lastHostInt) {
        rollovers++;
    }
    lastHostInt = hostInt;
    
    timestamp = ((uint64_t)rollovers << 32);
    timestamp += hostInt + (referenceDelta + delay) * 100;

    return timestamp;
}

static void sensorhubInputHdlr_2(sh2_t_2 *pSh2, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_SensorEvent_t_2 event;
    uint16_t cursor = 0;

    uint32_t referenceDelta;

    referenceDelta = 0;

    while (cursor < len) {
        // Get next report id
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = getReportLen_2(pSh2, reportId);
        if (reportLen == 0) {
            // An unrecognized report id
            pSh2->unknownReportIds++;
            return;
        }
        else {
            if (reportId == SENSORHUB_BASE_TIMESTAMP_REF_2) {
                const BaseTimestampRef_t_2 *rpt = (const BaseTimestampRef_t_2 *)(payload+cursor);
                
                // store base timestamp reference
                referenceDelta = -rpt->timebase;
            }
            else if (reportId == SENSORHUB_TIMESTAMP_REBASE_2) {
                const TimestampRebase_t_2 *rpt = (const TimestampRebase_t_2 *)(payload+cursor);

                referenceDelta += rpt->timebase;
            }
            else if (reportId == SENSORHUB_FLUSH_COMPLETED_2) {
                // Route this as if it arrived on command channel.
                opRx_2(pSh2, payload+cursor, reportLen);
            }
            else {
                uint8_t *pReport = payload+cursor;
                uint16_t delay = ((pReport[2] & 0xFC) << 6) + pReport[3];
                event.timestamp_uS = touSTimestamp_2(timestamp, referenceDelta, delay);
                event.reportId = reportId;
                memcpy(event.report, pReport, reportLen);
                event.len = reportLen;
                if (pSh2->sensorCallback != 0) {
                    pSh2->sensorCallback(pSh2->sensorCookie, &event);
                }
            }
            cursor += reportLen;
        }
    }
}

static void sensorhubInputNormalHdlr_2(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t_2 *pSh2 = (sh2_t_2 *)cookie;

    sensorhubInputHdlr_2(pSh2, payload, len, timestamp);
}

static void sensorhubInputWakeHdlr_2(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t_2 *pSh2 = (sh2_t_2 *)cookie;
    
    sensorhubInputHdlr_2(pSh2, payload, len, timestamp);
}

static void sensorhubInputGyroRvHdlr_2(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t_2 *pSh2 = (sh2_t_2 *)cookie;
    sh2_SensorEvent_t_2 event;
    uint16_t cursor = 0;

    uint8_t reportId = SH2_GYRO_INTEGRATED_RV_2;
    uint8_t reportLen = getReportLen_2(pSh2, reportId);

    while (cursor < len) {
        event.timestamp_uS = timestamp;
        event.reportId = reportId;
        memcpy(event.report, payload+cursor, reportLen);
        event.len = reportLen;

        if (pSh2->sensorCallback != 0) {
            pSh2->sensorCallback(pSh2->sensorCookie, &event);
        }

        cursor += reportLen;
    }
}

static void executableAdvertHdlr_2(void *cookie, uint8_t tag, uint8_t len, uint8_t *value)
{
    // Ignore.  No known TLV tags for this app.
}

static void executableDeviceHdlr_2(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t_2 *pSh2 = (sh2_t_2 *)cookie;

    // Discard if length is bad
    if (len != 1) {
        pSh2->execBadPayload++;
        return;
    }
    
    switch (payload[0]) {
        case EXECUTABLE_DEVICE_RESP_RESET_COMPLETE_2:
            // reset process is now done.
            pSh2->resetComplete = true;
            
            // Notify client that reset is complete.
            sh2AsyncEvent.eventId = SH2_RESET_2;
            if (pSh2->eventCallback) {
                pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
            }
            break;
        default:
            pSh2->execBadPayload++;
            break;
    }
}

static int sendExecutable_2(sh2_t_2 *pSh2, uint8_t cmd)
{
    return shtp_send_2(pSh2->pShtp, pSh2->executableChan, &cmd, 1);
}

static int sendCtrl_2(sh2_t_2 *pSh2, const uint8_t *data, uint16_t len)
{
    return shtp_send_2(pSh2->pShtp, pSh2->controlChan, data, len);
}

static int16_t toQ14_2(double x)
{
    int16_t retval = (int16_t)(x * (1<<14));
    
    return retval;
}

// ------------------------------------------------------------------------
// Get Product ID support

// Get Product ID Op handler
static int getProdIdStart_2(sh2_t_2 *pSh2)
{
    int rc = SH2_OK_2;
    ProdIdReq_t_2 req;
    
    pSh2->opData.getProdIds_2.nextEntry = 0;
    pSh2->opData.getProdIds_2.expectedEntries = 4;  // Most products supply 4 product ids.
                                                // When the first arrives, we'll know if
                                                // we need to adjust this.
    
    // Set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_PROD_ID_REQ_2;
    rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getProdIdRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    ProdIdResp_t_2 *resp = (ProdIdResp_t_2 *)payload;
    
    // skip this if it isn't the product id response.
    if (resp->reportId != SENSORHUB_PROD_ID_RESP_2) return;

    // Store this product id, if we can
    sh2_ProductIds_t_2 *pProdIds = pSh2->opData.getProdIds_2.pProdIds;
    
    if (pProdIds) {
        // Store the product id response
        if (pSh2->opData.getProdIds_2.nextEntry < pSh2->opData.getProdIds_2.expectedEntries) {
            sh2_ProductId_t_2 *pProdId = &pProdIds->entry[pSh2->opData.getProdIds_2.nextEntry];
            
            pProdId->resetCause = resp->resetCause;
            pProdId->swVersionMajor = resp->swVerMajor;
            pProdId->swVersionMinor = resp->swVerMinor;
            pProdId->swPartNumber = resp->swPartNumber;
            pProdId->swBuildNumber = resp->swBuildNumber;
            pProdId->swVersionPatch = resp->swVerPatch;
            pProdId->reserved0 = resp->reserved0;
            pProdId->reserved1 = resp->reserved1;

            if (pProdId->swPartNumber == 10004095) {
                // FSP200 has 5 product id entries
                pSh2->opData.getProdIds_2.expectedEntries = 5;
            }


            pSh2->opData.getProdIds_2.nextEntry++;
        }
    }

    // Complete this operation if there is no storage for more product ids
    if ((pSh2->opData.getProdIds_2.pProdIds == 0) ||
        (pSh2->opData.getProdIds_2.nextEntry >= pSh2->opData.getProdIds_2.expectedEntries)) {
        
        pSh2->opData.getProdIds_2.pProdIds->numEntries = pSh2->opData.getProdIds_2.nextEntry;
        opCompleted_2(pSh2, SH2_OK_2);
    }

    return;
}

const sh2_Op_t_2 getProdIdOp_2 = {
    .start = getProdIdStart_2,
    .rx = getProdIdRx_2,
};

// ------------------------------------------------------------------------
// Set Sensor Config

static int getSensorConfigStart_2(sh2_t_2 *pSh2)
{
    int rc = SH2_OK_2;
    GetFeatureReq_t_2 req;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_GET_FEATURE_REQ_2;
    req.featureReportId = pSh2->opData.getSensorConfig_2.sensorId;
    rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getSensorConfigRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    GetFeatureResp_t_2 *resp = (GetFeatureResp_t_2 *)payload;
    sh2_SensorConfig_t_2 *pConfig;
    
    // skip this if it isn't the response we're waiting for.
    if (resp->reportId != SENSORHUB_GET_FEATURE_RESP_2) return;
    if (resp->featureReportId != pSh2->opData.getSensorConfig_2.sensorId) return;

    // Copy out data
    pConfig = pSh2->opData.getSensorConfig_2.pConfig;
    
    pConfig->changeSensitivityEnabled = ((resp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED_2) != 0);
    pConfig->changeSensitivityRelative = ((resp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE_2) != 0);
    pConfig->wakeupEnabled = ((resp->flags & FEAT_WAKE_ENABLED_2) != 0);
    pConfig->alwaysOnEnabled = ((resp->flags & FEAT_ALWAYS_ON_ENABLED_2) != 0);
    pConfig->changeSensitivity = resp->changeSensitivity;
    pConfig->reportInterval_us = resp->reportInterval_uS;
    pConfig->batchInterval_us = resp->batchInterval_uS;
    pConfig->sensorSpecific = resp->sensorSpecific;

    // Complete this operation
    opCompleted_2(pSh2, SH2_OK_2);

    return;
}

const sh2_Op_t_2 getSensorConfigOp_2 = {
    .start = getSensorConfigStart_2,
    .rx = getSensorConfigRx_2,
};

// ------------------------------------------------------------------------
// Set Sensor Config

// SENSORHUB_SET_FEATURE_CMD
#define SENSORHUB_SET_FEATURE_CMD_2    (0xFD)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;             // 0xFD
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} SetFeatureReport_t_2;

static int setSensorConfigStart_2(sh2_t_2 *pSh2)
{
    SetFeatureReport_t_2 req;
    uint8_t flags = 0;
    int rc;
    sh2_SensorConfig_t_2 *pConfig = pSh2->opData.getSensorConfig_2.pConfig;
    
    if (pConfig->changeSensitivityEnabled)  flags |= FEAT_CHANGE_SENSITIVITY_ENABLED_2;
    if (pConfig->changeSensitivityRelative) flags |= FEAT_CHANGE_SENSITIVITY_RELATIVE_2;
    if (pConfig->wakeupEnabled)             flags |= FEAT_WAKE_ENABLED_2;
    if (pConfig->alwaysOnEnabled)           flags |= FEAT_ALWAYS_ON_ENABLED_2;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_SET_FEATURE_CMD_2;
    req.featureReportId = pSh2->opData.setSensorConfig_2.sensorId;
    req.flags = flags;
    req.changeSensitivity = pConfig->changeSensitivity;
    req.reportInterval_uS = pConfig->reportInterval_us;
    req.batchInterval_uS = pConfig->batchInterval_us;
    req.sensorSpecific = pConfig->sensorSpecific;

    rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));
    opCompleted_2(pSh2, rc);

    return rc;
}

const sh2_Op_t_2 setSensorConfigOp_2 = {
    .start = setSensorConfigStart_2,
};

// ------------------------------------------------------------------------
// Get FRS.

// SENSORHUB_FRS_WRITE_REQ
#define SENSORHUB_FRS_WRITE_REQ_2      (0xF7)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t length;
    uint16_t frsType;
} FrsWriteReq_t_2;

// SENSORHUB_FRS_WRITE_DATA_REQ
#define SENSORHUB_FRS_WRITE_DATA_REQ_2 (0xF6)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t offset;
    uint32_t data0;
    uint32_t data1;
} FrsWriteDataReq_t_2;

// FRS write status values
#define FRS_WRITE_STATUS_RECEIVED_2 (0)
#define FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE_2 (1)
#define FRS_WRITE_STATUS_BUSY_2 (2)
#define FRS_WRITE_STATUS_WRITE_COMPLETED_2 (3)
#define FRS_WRITE_STATUS_READY_2 (4)
#define FRS_WRITE_STATUS_FAILED_2 (5)
#define FRS_WRITE_STATUS_NOT_READY_2 (6) // data received when not in write mode
#define FRS_WRITE_STATUS_INVALID_LENGTH_2 (7)
#define FRS_WRITE_STATUS_RECORD_VALID_2 (8)
#define FRS_WRITE_STATUS_INVALID_RECORD_2 (9)
#define FRS_WRITE_STATUS_DEVICE_ERROR_2 (10)
#define FRS_WRITE_STATUS_READ_ONLY_2 (11)

// SENSORHUB_FRS_WRITE_RESP
#define SENSORHUB_FRS_WRITE_RESP_2     (0xF5)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t status;
    uint16_t wordOffset;
} FrsWriteResp_t_2;

// RESP_FRS_READ_REQ
#define SENSORHUB_FRS_READ_REQ_2       (0xF4)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t readOffset;
    uint16_t frsType;
    uint16_t blockSize;
} FrsReadReq_t_2;

// Get Datalen portion of len_status field
#define FRS_READ_DATALEN_2(x) ((x >> 4) & 0x0F)

// Get status portion of len_status field
#define FRS_READ_STATUS_2(x) ((x) & 0x0F)

// Status values
#define FRS_READ_STATUS_NO_ERROR_2                        0
#define FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE_2           1
#define FRS_READ_STATUS_BUSY_2                            2
#define FRS_READ_STATUS_READ_RECORD_COMPLETED_2           3
#define FRS_READ_STATUS_OFFSET_OUT_OF_RANGE_2             4
#define FRS_READ_STATUS_RECORD_EMPTY_2                    5
#define FRS_READ_STATUS_READ_BLOCK_COMPLETED_2            6
#define FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED_2 7
#define FRS_READ_STATUS_DEVICE_ERROR_2                    8

// SENSORHUB_FRS_READ_RESP
#define SENSORHUB_FRS_READ_RESP_2      (0xF3)
typedef PACKED_STRUCT_2 {
    uint8_t reportId;
    uint8_t len_status;  // See FRS_READ... macros above
    uint16_t wordOffset;
    uint32_t data0;
    uint32_t data1;
    uint16_t frsType;
    uint8_t reserved0;
    uint8_t reserved1;
} FrsReadResp_t_2;

static int getFrsStart_2(sh2_t_2 *pSh2)
{
    int rc = SH2_OK_2;
    FrsReadReq_t_2 req;

    pSh2->opData.getFrs_2.nextOffset = 0;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FRS_READ_REQ_2;
    req.reserved = 0;
    req.readOffset = 0; // read from start
    req.frsType = pSh2->opData.getFrs_2.frsType;
    req.blockSize = 0;  // read all avail data

    rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getFrsRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    FrsReadResp_t_2 *resp = (FrsReadResp_t_2 *)payload;
    uint8_t status;

    // skip this if it isn't the response we're looking for
    if (resp->reportId != SENSORHUB_FRS_READ_RESP_2) return;

    // Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
    status = FRS_READ_STATUS_2(resp->len_status);
    if ((status == FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE_2) ||
        (status == FRS_READ_STATUS_BUSY_2) ||
        (status == FRS_READ_STATUS_OFFSET_OUT_OF_RANGE_2) ||
        (status == FRS_READ_STATUS_DEVICE_ERROR_2)
        ) {
        // Operation failed
        opCompleted_2(pSh2, SH2_ERR_HUB_2);
        return;
    }

    if (status == FRS_READ_STATUS_RECORD_EMPTY_2) {
        // Empty record, return zero length.
        *(pSh2->opData.getFrs_2.pWords) = 0;
        opCompleted_2(pSh2, SH2_OK_2);
    }

    // Store the contents from this response
    uint16_t offset = resp->wordOffset;

    // check for missed offsets, resulting in error.
    if (offset != pSh2->opData.getFrs_2.nextOffset) {
        // Some data was dropped.
        *(pSh2->opData.getFrs_2.pWords) = 0;
        opCompleted_2(pSh2, SH2_ERR_IO_2);
    }
    
    // store first word, if we have room
    if ((*(pSh2->opData.getFrs_2.pWords) == 0) ||
        (offset <= *(pSh2->opData.getFrs_2.pWords))) {
        pSh2->opData.getFrs_2.pData[offset] = resp->data0;
        pSh2->opData.getFrs_2.nextOffset = offset+1;
    }

    // store second word if there is one and we have room
    if ((FRS_READ_DATALEN_2(resp->len_status) == 2)  &&
        ((*(pSh2->opData.getFrs_2.pWords) == 0) ||
         (offset <= *(pSh2->opData.getFrs_2.pWords)))) {
        pSh2->opData.getFrs_2.pData[offset+1] = resp->data1;
        pSh2->opData.getFrs_2.nextOffset = offset+2;
    }

    // If read is done, complete the operation
    if ((status == FRS_READ_STATUS_READ_RECORD_COMPLETED_2) ||
        (status == FRS_READ_STATUS_READ_BLOCK_COMPLETED_2) ||
        (status == FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED_2)) {
        *(pSh2->opData.getFrs_2.pWords) = pSh2->opData.getFrs_2.nextOffset;

        opCompleted_2(pSh2, SH2_OK_2);
    }

    return;
}

const sh2_Op_t_2 getFrsOp_2 = {
    .start = getFrsStart_2,
    .rx = getFrsRx_2,
};

// ------------------------------------------------------------------------
// Support for sh2_getMetadata

const static struct {
    sh2_SensorId_t_2 sensorId;
    uint16_t recordId;
} sensorToRecordMap_2[] = {
    { SH2_RAW_ACCELEROMETER_2,            FRS_ID_META_RAW_ACCELEROMETER_2 },
    { SH2_ACCELEROMETER_2,                FRS_ID_META_ACCELEROMETER_2 },
    { SH2_LINEAR_ACCELERATION_2,          FRS_ID_META_LINEAR_ACCELERATION_2 },
    { SH2_GRAVITY_2,                      FRS_ID_META_GRAVITY_2 },
    { SH2_RAW_GYROSCOPE_2,                FRS_ID_META_RAW_GYROSCOPE_2 },
    { SH2_GYROSCOPE_CALIBRATED_2,         FRS_ID_META_GYROSCOPE_CALIBRATED_2 },
    { SH2_GYROSCOPE_UNCALIBRATED_2,       FRS_ID_META_GYROSCOPE_UNCALIBRATED_2 },
    { SH2_RAW_MAGNETOMETER_2,             FRS_ID_META_RAW_MAGNETOMETER_2 },
    { SH2_MAGNETIC_FIELD_CALIBRATED_2,    FRS_ID_META_MAGNETIC_FIELD_CALIBRATED_2 },
    { SH2_MAGNETIC_FIELD_UNCALIBRATED_2,  FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED_2 },
    { SH2_ROTATION_VECTOR_2,              FRS_ID_META_ROTATION_VECTOR_2 },
    { SH2_GAME_ROTATION_VECTOR_2,         FRS_ID_META_GAME_ROTATION_VECTOR_2 },
    { SH2_GEOMAGNETIC_ROTATION_VECTOR_2,  FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR_2 },
    { SH2_PRESSURE_2,                     FRS_ID_META_PRESSURE_2 },
    { SH2_AMBIENT_LIGHT_2,                FRS_ID_META_AMBIENT_LIGHT_2 },
    { SH2_HUMIDITY_2,                     FRS_ID_META_HUMIDITY_2 },
    { SH2_PROXIMITY_2,                    FRS_ID_META_PROXIMITY_2 },
    { SH2_TEMPERATURE_2,                  FRS_ID_META_TEMPERATURE_2 },
    { SH2_TAP_DETECTOR_2,                 FRS_ID_META_TAP_DETECTOR_2 },
    { SH2_STEP_DETECTOR_2,                FRS_ID_META_STEP_DETECTOR_2 },
    { SH2_STEP_COUNTER_2,                 FRS_ID_META_STEP_COUNTER_2 },
    { SH2_SIGNIFICANT_MOTION_2,           FRS_ID_META_SIGNIFICANT_MOTION_2 },
    { SH2_STABILITY_CLASSIFIER_2,         FRS_ID_META_STABILITY_CLASSIFIER_2 },
    { SH2_SHAKE_DETECTOR_2,               FRS_ID_META_SHAKE_DETECTOR_2 },
    { SH2_FLIP_DETECTOR_2,                FRS_ID_META_FLIP_DETECTOR_2 },
    { SH2_PICKUP_DETECTOR_2,              FRS_ID_META_PICKUP_DETECTOR_2 },
    { SH2_STABILITY_DETECTOR_2,           FRS_ID_META_STABILITY_DETECTOR_2 },
    { SH2_PERSONAL_ACTIVITY_CLASSIFIER_2, FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER_2 },
    { SH2_SLEEP_DETECTOR_2,               FRS_ID_META_SLEEP_DETECTOR_2 },
    { SH2_TILT_DETECTOR_2,                FRS_ID_META_TILT_DETECTOR_2 },
    { SH2_POCKET_DETECTOR_2,              FRS_ID_META_POCKET_DETECTOR_2 },
    { SH2_CIRCLE_DETECTOR_2,              FRS_ID_META_CIRCLE_DETECTOR_2 },
};

static void stuffMetadata_2(sh2_SensorMetadata_t_2 *pData, uint32_t *frsData)
{
    // Populate the sensorMetadata structure with results
    pData->meVersion        = (frsData[0] >> 0) & 0xFF;
    pData->mhVersion        = (frsData[0] >> 8) & 0xFF;
    pData->shVersion        = (frsData[0] >> 16) & 0xFF;
    pData->range            = frsData[1];
    pData->resolution       = frsData[2];
    pData->power_mA         = (frsData[3] >> 0) & 0xFFFF;    // 16.10 format
    pData->revision         = (frsData[3] >> 16) & 0xFFFF;
    pData->minPeriod_uS     = frsData[4];
    pData->maxPeriod_uS     = 0;  // ...unless reading format 4 metadata below
    pData->fifoMax          = (frsData[5] >> 0) & 0xFFFF;
    pData->fifoReserved     = (frsData[5] >> 16) & 0xFFFF;
    pData->batchBufferBytes = (frsData[6] >> 0) & 0xFFFF;;
    pData->vendorIdLen      = (frsData[6] >> 16) & 0xFFFF;

    // Init fields that may not be present, depending on metadata revision
    pData->qPoint1           = 0;
    pData->qPoint2           = 0;
    pData->qPoint3           = 0;
    pData->sensorSpecificLen = 0;
    strcpy(pData->vendorId, ""); // init with empty string in case vendorIdLen == 0

    int vendorIdOffset = 8;
    // Get revision-specific fields
    if (pData->revision == 0) {
        // No fixed fields, vendor id copied after if-else block
    }
    else if (pData->revision == 1) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
    }
    else if (pData->revision == 2) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
        vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else if (pData->revision == 3) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        pData->qPoint3        = (frsData[8] >> 16) & 0xFFFF;
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
        vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else if (pData->revision == 4) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        pData->qPoint3        = (frsData[8] >> 16) & 0xFFFF;
        pData->maxPeriod_uS   = frsData[9];
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[10], pData->sensorSpecificLen);
        vendorIdOffset = 10 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else {
        // Unrecognized revision!
    }

    // Copy vendor id
    memcpy(pData->vendorId, (uint8_t *)&frsData[vendorIdOffset],
           pData->vendorIdLen);
}

// ------------------------------------------------------------------------
// Set FRS.

static int setFrsStart_2(sh2_t_2 *pSh2)
{
    int rc = SH2_OK_2;
    FrsWriteReq_t_2 req;

    pSh2->opData.setFrs_2.offset = 0;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FRS_WRITE_REQ_2;
    req.reserved = 0;
    req.length = pSh2->opData.setFrs_2.words;
    req.frsType = pSh2->opData.getFrs_2.frsType;

    rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void setFrsRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    FrsWriteResp_t_2 *resp = (FrsWriteResp_t_2 *)payload;
    FrsWriteDataReq_t_2 req;
    uint8_t status;
    bool sendMoreData = false;
    bool completed = false;
    int rc = SH2_OK_2;

    // skip this if it isn't the response we're looking for.
    if (resp->reportId != SENSORHUB_FRS_WRITE_RESP_2) return;

    // Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
    status = resp->status;
    switch(status) {
        case FRS_WRITE_STATUS_RECEIVED_2:
        case FRS_WRITE_STATUS_READY_2:
            sendMoreData = true;
            break;
        case FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE_2:
        case FRS_WRITE_STATUS_BUSY_2:
        case FRS_WRITE_STATUS_FAILED_2:
        case FRS_WRITE_STATUS_NOT_READY_2:
        case FRS_WRITE_STATUS_INVALID_LENGTH_2:
        case FRS_WRITE_STATUS_INVALID_RECORD_2:
        case FRS_WRITE_STATUS_DEVICE_ERROR_2:
        case FRS_WRITE_STATUS_READ_ONLY_2:
            rc = SH2_ERR_HUB_2;
            completed = true;
            break;
        case FRS_WRITE_STATUS_WRITE_COMPLETED_2:
            // Successful completion
            rc = SH2_OK_2;
            completed = true;
            break;
        case FRS_WRITE_STATUS_RECORD_VALID_2:
            // That's nice, keep waiting
            break;
    }

    // if we should send more data, do it.
    if (sendMoreData &&
        (pSh2->opData.setFrs_2.offset < pSh2->opData.setFrs_2.words)) {
        uint16_t offset = pSh2->opData.setFrs_2.offset;
        
        memset(&req, 0, sizeof(req));
        req.reportId = SENSORHUB_FRS_WRITE_DATA_REQ_2;
        req.reserved = 0;
        req.offset = offset;
        req.data0 = pSh2->opData.setFrs_2.pData[offset++];
        if (offset < pSh2->opData.setFrs_2.words) {
            req.data1 = pSh2->opData.setFrs_2.pData[offset++];
        } else {
            req.data1 = 0;
        }
        pSh2->opData.setFrs_2.offset = offset;
        
        rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));
    }

    // if the operation is done or has to be aborted, complete it
    if (completed) {
        opCompleted_2(pSh2, rc);
    }

    return;
}

const sh2_Op_t_2 setFrsOp_2 = {
    .start = setFrsStart_2,
    .rx = setFrsRx_2,
};

// ------------------------------------------------------------------------
// Support for sending commands

static int sendCmd_2(sh2_t_2 *pSh2, uint8_t cmd, uint8_t p[COMMAND_PARAMS_2])
{
    int rc = SH2_OK_2;
    CommandReq_t_2 req;

    // Clear request structure
    memset(&req, 0, sizeof(req));
    
    // Create a command sequence number for this command
    pSh2->lastCmdId = cmd;
    pSh2->cmdSeq = pSh2->nextCmdSeq++;
    
    // set up request to issue
    req.reportId = SENSORHUB_COMMAND_REQ_2;
    req.seq = pSh2->cmdSeq;
    req.command = cmd;
    for (int n = 0; n < COMMAND_PARAMS_2; n++) {
        req.p[n] = p[n];
    }
    
    rc = sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));
    
    return rc;
}

// Send a command with 0 parameters
static int sendCmd0_2(sh2_t_2 *pSh2, uint8_t cmd)
{
    uint8_t p[COMMAND_PARAMS_2];

    memset(p, 0, COMMAND_PARAMS_2);

    return sendCmd_2(pSh2, cmd, p);
}

// Send a command with 1 parameter
static int sendCmd1_2(sh2_t_2 *pSh2, uint8_t cmd, uint8_t p0)
{
    uint8_t p[COMMAND_PARAMS_2];

    memset(p, 0, COMMAND_PARAMS_2);

    p[0] = p0;
    return sendCmd_2(pSh2, cmd, p);
}

// Send a command with 2 parameters
static int sendCmd2_2(sh2_t_2 *pSh2, uint8_t cmd, uint8_t p0, uint8_t p1)
{
    uint8_t p[COMMAND_PARAMS_2];

    memset(p, 0, COMMAND_PARAMS_2);
    
    p[0] = p0;
    p[1] = p1;
    return sendCmd_2(pSh2, cmd, p);
}

static bool wrongResponse_2(sh2_t_2 *pSh2, CommandResp_t_2 *resp)
{
    if (resp->reportId != SENSORHUB_COMMAND_RESP_2) return true;
    if (resp->command != pSh2->lastCmdId) return true;
    if (resp->commandSeq != pSh2->cmdSeq) return true;

    return false;
}

// ------------------------------------------------------------------------
// Get Errors

static int getErrorsStart_2(sh2_t_2 *pSh2)
{
    // Initialize state
    pSh2->opData.getErrors_2.errsRead = 0;

    return sendCmd1_2(pSh2, SH2_CMD_ERRORS_2, pSh2->opData.getErrors_2.severity);
}

static void getErrorsRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;
    
    // skip this if it isn't the right response
    if (wrongResponse_2(pSh2, resp)) return;

    if (resp->r[2] == 255) {
        // No error to report, operation is complete
        *(pSh2->opData.getErrors_2.pNumErrors) = pSh2->opData.getErrors_2.errsRead;
        opCompleted_2(pSh2, SH2_OK_2);
    } else {
        // Copy data for invoker.
        unsigned int index = pSh2->opData.getErrors_2.errsRead;
        if (index < *(pSh2->opData.getErrors_2.pNumErrors)) {
            // We have room for this one.
            pSh2->opData.getErrors_2.pErrors[index].severity = resp->r[0];
            pSh2->opData.getErrors_2.pErrors[index].sequence = resp->r[1];
            pSh2->opData.getErrors_2.pErrors[index].source = resp->r[2];
            pSh2->opData.getErrors_2.pErrors[index].error = resp->r[3];
            pSh2->opData.getErrors_2.pErrors[index].module = resp->r[4];
            pSh2->opData.getErrors_2.pErrors[index].code = resp->r[5];

            pSh2->opData.getErrors_2.errsRead++;
        }
    }

    return;
}

const sh2_Op_t_2 getErrorsOp_2 = {
    .start = getErrorsStart_2,
    .rx = getErrorsRx_2,
};

// ------------------------------------------------------------------------
// Get Counts

static int getCountsStart_2(sh2_t_2 *pSh2)
{
    return sendCmd2_2(pSh2, SH2_CMD_COUNTS_2, SH2_COUNTS_GET_COUNTS_2, pSh2->opData.getCounts_2.sensorId);
}

static void getCountsRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;

    if (wrongResponse_2(pSh2, resp)) return;
    
    // Store results
    if (resp->respSeq == 0) {
        pSh2->opData.getCounts_2.pCounts->offered = readu32(&resp->r[3]);
        pSh2->opData.getCounts_2.pCounts->accepted = readu32(&resp->r[7]);
    }
    else {
        pSh2->opData.getCounts_2.pCounts->on = readu32(&resp->r[3]);
        pSh2->opData.getCounts_2.pCounts->attempted = readu32(&resp->r[7]);
    }
    
    // Complete this operation if we've received last response
    if (resp->respSeq == 1) {
        opCompleted_2(pSh2, SH2_OK_2);
    }

    return;
}

const sh2_Op_t_2 getCountsOp_2 = {
    .start = getCountsStart_2,
    .rx = getCountsRx_2,
};

// ------------------------------------------------------------------------
// Generic Send Command

static int sendCmdStart_2(sh2_t_2 *pSh2)
{
    int status = sendCmd_2(pSh2, pSh2->opData.sendCmd_2.req.command,
                     pSh2->opData.sendCmd_2.req.p);

    opCompleted_2(pSh2, status);

    return status;
}

const sh2_Op_t_2 sendCmdOp_2 = {
    .start = sendCmdStart_2,
};

// ------------------------------------------------------------------------
// Reinit

static int reinitStart_2(sh2_t_2 *pSh2)
{
    int status = sendCmd1_2(pSh2, SH2_CMD_INITIALIZE_2, SH2_INIT_SYSTEM_2);

    return status;
}

static void reinitRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;

    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    // Get return status
    int status = SH2_OK_2;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB_2;
    }

    opCompleted_2(pSh2, status);
}

const sh2_Op_t_2 reinitOp_2 = {
    .start = reinitStart_2,
    .rx = reinitRx_2,
};

// ------------------------------------------------------------------------
// Save DCD Now

static int saveDcdNowStart_2(sh2_t_2 *pSh2)
{
    int status = sendCmd0_2(pSh2, SH2_CMD_DCD_2);

    return status;
}

static void saveDcdNowRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;

    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    // Get return status
    int status = SH2_OK_2;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB_2;
    }

    opCompleted_2(pSh2, status);
}

const sh2_Op_t_2 saveDcdNowOp_2 = {
    .start = saveDcdNowStart_2,
    .rx = saveDcdNowRx_2,
};

// ------------------------------------------------------------------------
// Get Osc Type

static int getOscTypeStart_2(sh2_t_2 *pSh2)
{
    return sendCmd0_2(pSh2, SH2_CMD_GET_OSC_TYPE_2);
}

static void getOscTypeRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;
    sh2_OscType_t_2 *pOscType;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    // Read out data
    pOscType = pSh2->opData.getOscType_2.pOscType;
    *pOscType = (sh2_OscType_t_2)resp->r[0];

    // Complete this operation
    opCompleted_2(pSh2, SH2_OK_2);
}

const sh2_Op_t_2 getOscTypeOp_2 = {
    .start = getOscTypeStart_2,
    .rx = getOscTypeRx_2,
};

// ------------------------------------------------------------------------
// Set Cal Config

static int setCalConfigStart_2(sh2_t_2 *pSh2)
{
    uint8_t p[COMMAND_PARAMS_2];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS_2);
    
    // Which cal processes to enable/disable
    p[0] = (pSh2->opData.calConfig_2.sensors & SH2_CAL_ACCEL_2) ? 1 : 0; // accel cal
    p[1] = (pSh2->opData.calConfig_2.sensors & SH2_CAL_GYRO_2)  ? 1 : 0; // gyro cal
    p[2] = (pSh2->opData.calConfig_2.sensors & SH2_CAL_MAG_2)   ? 1 : 0; // mag cal
    p[4] = (pSh2->opData.calConfig_2.sensors & SH2_CAL_PLANAR_2) ? 1 : 0; // planar cal
    
    return sendCmd_2(pSh2, SH2_CMD_ME_CAL_2, p);
}

static void setCalConfigRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    // Read out data
    int status = SH2_OK_2;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB_2;
    }

    // Complete this operation
    opCompleted_2(pSh2, status);
}

const sh2_Op_t_2 setCalConfigOp_2 = {
    .start = setCalConfigStart_2,
    .rx = setCalConfigRx_2,
};

// ------------------------------------------------------------------------
// Get Cal Config

static int getCalConfigStart_2(sh2_t_2 *pSh2)
{
    uint8_t p[COMMAND_PARAMS_2];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS_2);
    
    // Subcommand: Get ME Calibration
    p[3] = 0x01;
    
    return sendCmd_2(pSh2, SH2_CMD_ME_CAL_2, p);
}

static void getCalConfigRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    // Read out data
    int status = SH2_OK_2;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB_2;
    }
    else {
        // Unload results into pSensors
        uint8_t sensors = 0;
        if (resp->r[1]) sensors |= SH2_CAL_ACCEL_2;
        if (resp->r[2]) sensors |= SH2_CAL_GYRO_2;
        if (resp->r[3]) sensors |= SH2_CAL_MAG_2;
        if (resp->r[4]) sensors |= SH2_CAL_PLANAR_2;
        *(pSh2->opData.getCalConfig_2.pSensors) = sensors;
    }
    
    // Complete this operation
    opCompleted_2(pSh2, status);
}


const sh2_Op_t_2 getCalConfigOp_2 = {
    .start = getCalConfigStart_2,
    .rx = getCalConfigRx_2,
};

// ------------------------------------------------------------------------
// Force Flush

static int forceFlushStart_2(sh2_t_2 *pSh2)
{
    ForceFlushReq_t_2 req;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FORCE_SENSOR_FLUSH_2;
    req.sensorId = pSh2->opData.forceFlush_2.sensorId;
    
    return sendCtrl_2(pSh2, (uint8_t *)&req, sizeof(req));
}

static void forceFlushRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    ForceFlushResp_t_2 *resp = (ForceFlushResp_t_2 *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (resp->reportId != SENSORHUB_FLUSH_COMPLETED_2) return;
    if (resp->sensorId != pSh2->opData.forceFlush_2.sensorId) return;

    // Complete this operation
    opCompleted_2(pSh2, SH2_OK_2);
}

const sh2_Op_t_2 forceFlushOp_2 = {
    .start = forceFlushStart_2,
    .rx = forceFlushRx_2,
};

// ------------------------------------------------------------------------
// Start Cal

static int startCalStart_2(sh2_t_2 *pSh2)
{
    uint8_t p[COMMAND_PARAMS_2];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS_2);
    
    // Subcommand: Get ME Calibration
    p[0] = SH2_CAL_START_2;
    p[1] = pSh2->opData.startCal_2.interval_us & 0xFF;          // LSB
    p[2] = (pSh2->opData.startCal_2.interval_us >> 8) & 0xFF;
    p[3] = (pSh2->opData.startCal_2.interval_us >> 16) & 0xFF;
    p[4] = (pSh2->opData.startCal_2.interval_us >> 24) & 0xFF;  // MSB
    
    return sendCmd_2(pSh2, SH2_CMD_CAL_2, p);
}

static void startCalRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    // Complete this operation
    opCompleted_2(pSh2, SH2_OK_2);
}

const sh2_Op_t_2 startCalOp_2 = {
    .start = startCalStart_2,
    .rx = startCalRx_2,
};

// ------------------------------------------------------------------------
// Start Cal

static int finishCalStart_2(sh2_t_2 *pSh2)
{
    return sendCmd1_2(pSh2, SH2_CMD_CAL_2, SH2_CAL_FINISH_2);
}

static void finishCalRx_2(sh2_t_2 *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t_2 *resp = (CommandResp_t_2 *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse_2(pSh2, resp)) return;

    pSh2->opData.finishCal_2.status = (sh2_CalStatus_t_2)resp->r[1];

    // Complete this operation
    if (pSh2->opData.finishCal_2.status == SH2_CAL_SUCCESS_2) {
        opCompleted_2(pSh2, SH2_OK_2);
    }
    else {
        opCompleted_2(pSh2, SH2_ERR_HUB_2);
    }
}

const sh2_Op_t_2 finishCalOp_2 = {
    .start = finishCalStart_2,
    .rx = finishCalRx_2,
};


// ------------------------------------------------------------------------
// SHTP Event Callback

static void shtpEventCallback_2(void *cookie, shtp_Event_t_2 shtpEvent) {
    sh2_t_2 *pSh2 = &_sh2_2;

    sh2AsyncEvent.eventId = SH2_SHTP_EVENT_2;
    sh2AsyncEvent.shtpEvent = shtpEvent;
    if (pSh2->eventCallback) {
        pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
    }
}

// ------------------------------------------------------------------------
// Public functions

/**
 * @brief Open a session with a sensor hub.
 *
 * This function should be called before others in this API.
 * An instance of an SH2 HAL should be passed in.
 * This call will result in the open() function of the HAL being called.
 *
 * As part of the initialization process, a callback function is registered that will
 * be invoked when the device generates certain events.  (See sh2_AsyncEventId)
 *
 * @param pHal Pointer to an SH2 HAL instance, provided by the target system.
 * @param  eventCallback Will be called when events, such as reset complete, occur.
 * @param  eventCookie Will be passed to eventCallback.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_open_2(sh2_Hal_t_2 *pHal,
             sh2_EventCallback_t_2 *eventCallback, void *eventCookie)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // Validate parameters
    if (pHal == 0) return SH2_ERR_BAD_PARAM_2;

    // Clear everything in sh2 structure.
    memset(&_sh2_2, 0, sizeof(_sh2_2));
        
    pSh2->resetComplete = false;  // will go true after reset response from SH.
    pSh2->controlChan = 0xFF;  // An invalid value since we don't know yet.
    
    // Store reference to HAL for future use.
    pSh2->pHal = pHal;
    pSh2->eventCallback = eventCallback;
    pSh2->eventCookie = eventCookie;
    pSh2->sensorCallback = 0;
    pSh2->sensorCookie = 0;

    // Open SHTP layer
    pSh2->pShtp = shtp_open_2(pSh2->pHal);
    if (pSh2->pShtp == 0) {
        // Error opening SHTP
        return SH2_ERR_2;
    }

    // Register SHTP event callback
    shtp_setEventCallback_2(pSh2->pShtp, shtpEventCallback_2, &_sh2_2);

    // Register with SHTP
    // Register SH2 handlers
    shtp_listenAdvert_2(pSh2->pShtp, GUID_SENSORHUB_2, sensorhubAdvertHdlr_2, &_sh2_2);
    shtp_listenChan_2(pSh2->pShtp, GUID_SENSORHUB_2, "control", sensorhubControlHdlr_2, &_sh2_2);
    shtp_listenChan_2(pSh2->pShtp, GUID_SENSORHUB_2, "inputNormal", sensorhubInputNormalHdlr_2, &_sh2_2);
    shtp_listenChan_2(pSh2->pShtp, GUID_SENSORHUB_2, "inputWake", sensorhubInputWakeHdlr_2, &_sh2_2);
    shtp_listenChan_2(pSh2->pShtp, GUID_SENSORHUB_2, "inputGyroRv", sensorhubInputGyroRvHdlr_2, &_sh2_2);

    // Register EXECUTABLE handlers
    shtp_listenAdvert_2(pSh2->pShtp, GUID_EXECUTABLE_2, executableAdvertHdlr_2, &_sh2_2);
    shtp_listenChan_2(pSh2->pShtp, GUID_EXECUTABLE_2, "device", executableDeviceHdlr_2, &_sh2_2);

    // Wait for reset notifications to arrive.
    // The client can't talk to the sensor hub until that happens.
    uint32_t start_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    uint32_t now_us = start_us;
    while (((now_us - start_us) < ADVERT_TIMEOUT_US_2) &&
           (!pSh2->resetComplete))
    {
        shtp_service_2(pSh2->pShtp);
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }
    
    // No errors.
    return SH2_OK_2;
}

/**
 * @brief Close a session with a sensor hub.
 *
 * This should be called at the end of a sensor hub session.  
 * The underlying SHTP and HAL instances will be closed.
 */
void sh2_close_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    shtp_close_2(pSh2->pShtp);

    // Clear everything in sh2 structure.
    memset(pSh2, 0, sizeof(sh2_t_2));
}

/**
 * @brief Service the SH2 device, reading any data that is available and dispatching callbacks.
 *
 * This function should be called periodically by the host system to service an open sensor hub.
 */
void sh2_service_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    shtp_service_2(pSh2->pShtp);
}

/**
 * @brief Register a function to receive sensor events.
 *
 * @param  callback A function that will be called each time a sensor event is received.
 * @param  cookie  A value that will be passed to the sensor callback function.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorCallback_2(sh2_SensorCallback_t_2 *callback, void *cookie)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    pSh2->sensorCallback = callback;
    pSh2->sensorCookie = cookie;

    return SH2_OK_2;
}

/**
 * @brief Reset the sensor hub device by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devReset_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    return sendExecutable_2(pSh2, EXECUTABLE_DEVICE_CMD_RESET_2);
}

/**
 * @brief Turn sensor hub on by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devOn_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    return sendExecutable_2(pSh2, EXECUTABLE_DEVICE_CMD_ON_2);
}

/**
 * @brief Put sensor hub in sleep state by sending SLEEP (2) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devSleep_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    return sendExecutable_2(pSh2, EXECUTABLE_DEVICE_CMD_SLEEP_2);
}

/**
 * @brief Get Product ID information from Sensorhub.
 *
 * @param  prodIds Pointer to structure that will receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getProdIds_2(sh2_ProductIds_t_2 *prodIds)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.getProdIds_2.pProdIds = prodIds;

    return opProcess_2(pSh2, &getProdIdOp_2);
}

/**
 * @brief Get sensor configuration.
 *
 * @param  sensorId Which sensor to query.
 * @param  config SensorConfig structure to store results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getSensorConfig_2(sh2_SensorId_t_2 sensorId, sh2_SensorConfig_t_2 *pConfig)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    // Set up operation
    pSh2->opData.getSensorConfig_2.sensorId = sensorId;
    pSh2->opData.getSensorConfig_2.pConfig = pConfig;

    return opProcess_2(pSh2, &getSensorConfigOp_2);
}

/**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig_2(sh2_SensorId_t_2 sensorId, const sh2_SensorConfig_t_2 *pConfig)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    // Set up operation
    pSh2->opData.setSensorConfig_2.sensorId = sensorId;
    pSh2->opData.setSensorConfig_2.pConfig = pConfig;

    return opProcess_2(pSh2, &setSensorConfigOp_2);
}

/**
 * @brief Get metadata related to a sensor.
 *
 * @param  sensorId Which sensor to query.
 * @param  pData Pointer to structure to receive the results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getMetadata_2(sh2_SensorId_t_2 sensorId, sh2_SensorMetadata_t_2 *pData)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // pData must be non-null
    if (pData == 0) return SH2_ERR_BAD_PARAM_2;
  
    // Convert sensorId to metadata recordId
    int i;
    for (i = 0; i < ARRAY_LEN(sensorToRecordMap_2); i++) {
        if (sensorToRecordMap_2[i].sensorId == sensorId) {
            break;
        }
    }
    if (i >= ARRAY_LEN(sensorToRecordMap_2)) {
        // no match was found
        return SH2_ERR_BAD_PARAM_2;
    }
    uint16_t recordId = sensorToRecordMap_2[i].recordId;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    // Set up an FRS read operation
    pSh2->opData.getFrs_2.frsType = recordId;
    pSh2->opData.getFrs_2.pData = pSh2->frsData;
    pSh2->frsDataLen = ARRAY_LEN(pSh2->frsData);
    pSh2->opData.getFrs_2.pWords = &pSh2->frsDataLen;

    // Read an FRS record
    int status = opProcess_2(pSh2, &getFrsOp_2);
    
    // Copy the results into pData
    if (status == SH2_OK_2) {
        stuffMetadata_2(pData, pSh2->frsData);
    }

    return status;
}

/**
 * @brief Get an FRS record.
 *
 * @param  recordId Which FRS Record to retrieve.
 * @param  pData pointer to buffer to receive the results
 * @param[in] words Size of pData buffer, in 32-bit words.
 * @param[out] words Number of 32-bit words retrieved.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getFrs_2(uint16_t recordId, uint32_t *pData, uint16_t *words)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    if ((pData == 0) || (words == 0)) {
        return SH2_ERR_BAD_PARAM_2;
    }
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    // Store params for this op
    pSh2->opData.getFrs_2.frsType = recordId;
    pSh2->opData.getFrs_2.pData = pData;
    pSh2->opData.getFrs_2.pWords = words;

    return opProcess_2(pSh2, &getFrsOp_2);
}

/**
 * @brief Set an FRS record
 *
 * @param  recordId Which FRS Record to set.
 * @param  pData pointer to buffer containing the new data.
 * @param  words number of 32-bit words to write.  (0 to delete record.)
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setFrs_2(uint16_t recordId, uint32_t *pData, uint16_t words)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    if ((pData == 0) && (words != 0)) {
        return SH2_ERR_BAD_PARAM_2;
    }
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.setFrs_2.frsType = recordId;
    pSh2->opData.setFrs_2.pData = pData;
    pSh2->opData.setFrs_2.words = words;

    return opProcess_2(pSh2, &setFrsOp_2);
}

/**
 * @brief Get error counts.
 *
 * @param  severity Only errors of this severity or greater are returned.
 * @param  pErrors Buffer to receive error codes.
 * @param  numErrors size of pErrors array
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getErrors_2(uint8_t severity, sh2_ErrorRecord_t_2 *pErrors, uint16_t *numErrors)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.getErrors_2.severity = severity;
    pSh2->opData.getErrors_2.pErrors = pErrors;
    pSh2->opData.getErrors_2.pNumErrors = numErrors;
    
    return opProcess_2(pSh2, &getErrorsOp_2);
}

/**
 * @brief Read counters related to a sensor.
 *
 * @param  sensorId Which sensor to operate on.
 * @param  pCounts Pointer to Counts structure that will receive data.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCounts_2(sh2_SensorId_t_2 sensorId, sh2_Counts_t_2 *pCounts)
{
    sh2_t_2 *pSh2 = &_sh2_2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.getCounts_2.sensorId = sensorId;
    pSh2->opData.getCounts_2.pCounts = pCounts;
    
    return opProcess_2(pSh2, &getCountsOp_2);
}

/**
 * @brief Clear counters related to a sensor.
 *
 * @param  sensorId which sensor to operate on.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearCounts_2(sh2_SensorId_t_2 sensorId)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_COUNTS_2;
    pSh2->opData.sendCmd_2.req.p[0] = SH2_COUNTS_CLEAR_COUNTS_2;
    pSh2->opData.sendCmd_2.req.p[1] = sensorId;

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Perform a tare operation on one or more axes.
 *
 * @param  axes Bit mask specifying which axes should be tared.
 * @param  basis Which rotation vector to use as the basis for Tare adjustment.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setTareNow_2(uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t_2 basis)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_TARE_2;
    pSh2->opData.sendCmd_2.req.p[0] = SH2_TARE_TARE_NOW_2;
    pSh2->opData.sendCmd_2.req.p[1] = axes;
    pSh2->opData.sendCmd_2.req.p[2] = basis;

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Clears the previously applied tare operation.
 *
 * @return SH2_OK \n");
 */
int sh2_clearTare_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_TARE_2;
    pSh2->opData.sendCmd_2.req.p[0] = SH2_TARE_SET_REORIENTATION_2;

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Persist the results of last tare operation to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_persistTare_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_TARE_2;
    pSh2->opData.sendCmd_2.req.p[0] = SH2_TARE_PERSIST_TARE_2;

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Set the current run-time sensor reorientation. (Set to zero to clear tare.)
 *
 * @param  orie_2ntation Quaternion rotation vector to apply as new tare.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setReorientation_2(sh2_Quaternion_t_2 *orientation)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_TARE_2;
    pSh2->opData.sendCmd_2.req.p[0] = SH2_TARE_SET_REORIENTATION_2;

    // save me a lot of typing and you a lot of reading
    uint8_t *p = pSh2->opData.sendCmd_2.req.p;

    // Put new orientation in command parameters
    writeu16(&p[1], toQ14_2(orientation->x));
    writeu16(&p[3], toQ14_2(orientation->y));
    writeu16(&p[5], toQ14_2(orientation->z));
    writeu16(&p[7], toQ14_2(orientation->w));

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Command the sensorhub to reset.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_reinitialize_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    return opProcess_2(pSh2, &reinitOp_2);
}

/**
 * @brief Save Dynamic Calibration Data to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_saveDcdNow_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    return opProcess_2(pSh2, &saveDcdNowOp_2);
}

/**
 * @brief Get Oscillator type.
 *
 * @param  pOscType pointer to data structure to receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getOscType_2(sh2_OscType_t_2 *pOscType)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    pSh2->opData.getOscType_2.pOscType = pOscType;

    return opProcess_2(pSh2, &getOscTypeOp_2);
}

/**
 * @brief Enable/Disable dynamic calibration for certain sensors
 *
 * @param  sensors Bit mask to configure which sensors are affected.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setCalConfig_2(uint8_t sensors)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    pSh2->opData.calConfig_2.sensors = sensors;

    return opProcess_2(pSh2, &setCalConfigOp_2);
}

/**
 * @brief Get dynamic calibration configuration settings.
 *
 * @param  pSensors pointer to Bit mask, set on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCalConfig_2(uint8_t *pSensors)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    pSh2->opData.getCalConfig_2.pSensors = pSensors;

    return opProcess_2(pSh2, &getCalConfigOp_2);
}

/**
 * @brief Configure automatic saving of dynamic calibration data.
 *
 * @param  enabled Enable or Disable DCD auto-save.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setDcdAutoSave_2(bool enabled)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_DCD_SAVE_2;
    pSh2->opData.sendCmd_2.req.p[0] = enabled ? 0 : 1;

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Immediately issue all buffered sensor reports from a given sensor.
 *
 * @param  sensorId Which sensor reports to flush.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_flush_2(sh2_SensorId_t_2 sensorId)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.forceFlush_2.sensorId = sensorId;

    return opProcess_2(pSh2, &forceFlushOp_2);
}

/**
 * @brief Command clear DCD in RAM, then reset sensor hub.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearDcdAndReset_2(void)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_CLEAR_DCD_AND_RESET_2;

    return opProcess_2(pSh2, &sendCmdOp_2);
}

/**
 * @brief Start simple self-calibration procedure.
 *
 * @parameter interval_us sensor report interval, uS.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_startCal_2(uint32_t interval_us)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    pSh2->opData.startCal_2.interval_us = interval_us;

    return opProcess_2(pSh2, &startCalOp_2);
}

/**
 * @brief Finish simple self-calibration procedure.
 *
 * @parameter status contains calibration status code on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_finishCal_2(sh2_CalStatus_t_2 *status)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));
    
    return opProcess_2(pSh2, &finishCalOp_2);
}

/**
 * @brief send Interactive ZRO Request.
 *
 * @parameter intent Inform the sensor hub what sort of motion should be in progress.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setIZro_2(sh2_IZroMotionIntent_t_2 intent)
{
    sh2_t_2 *pSh2 = &_sh2_2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t_2));

    // set up opData for iZRO request
    pSh2->opData.sendCmd_2.req.command = SH2_CMD_INTERACTIVE_ZRO_2;
    pSh2->opData.sendCmd_2.req.p[0] = intent;

    // Send command
    return opProcess_2(pSh2, &sendCmdOp_2);
}
