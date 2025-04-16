/*
 * Copyright 2015-18 Hillcrest Laboratories, Inc.
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
 * Hillcrest Sensor Hub Transport Protocol (SHTP) API
 */

#ifndef SHTP_2_H
#define SHTP_2_H

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal2.h"

// Advertisement TLV tags
#define TAG_NULL_2 0
#define TAG_GUID_2 1
#define TAG_MAX_CARGO_PLUS_HEADER_WRITE_2 2
#define TAG_MAX_CARGO_PLUS_HEADER_READ_2 3
#define TAG_MAX_TRANSFER_WRITE_2 4
#define TAG_MAX_TRANSFER_READ_2 5
#define TAG_NORMAL_CHANNEL_2 6
#define TAG_WAKE_CHANNEL_2 7
#define TAG_APP_NAME_2 8
#define TAG_CHANNEL_NAME_2 9
#define TAG_ADV_COUNT_2 10
#define TAG_APP_SPECIFIC_2 0x80

typedef enum shtp_Event_e_2 {
    SHTP_TX_DISCARD_2 = 0,
    SHTP_SHORT_FRAGMENT_2 = 1,
    SHTP_TOO_LARGE_PAYLOADS_2 = 2,
    SHTP_BAD_RX_CHAN_2 = 3,
    SHTP_BAD_TX_CHAN_2 = 4,
} shtp_Event_t_2;

typedef void shtp_Callback_t_2(void * cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
typedef void shtp_AdvertCallback_t_2(void * cookie, uint8_t tag, uint8_t len, uint8_t *value);
typedef void shtp_SendCallback_t_2(void *cookie);
typedef void shtp_EventCallback_t_2(void *cookie, shtp_Event_t_2 shtpEvent);

// Takes HAL pointer, returns shtp ID for use in future calls.
// HAL will be opened by this call.
void * shtp_open_2(sh2_Hal_t_2 *pHal);

// Releases resources associated with this SHTP instance.
// HAL will not be closed.
void shtp_close_2(void *pShtp);

// Provide the point of the callback function for reporting SHTP asynchronous events
void shtp_setEventCallback_2(void *pInstance,
                           shtp_EventCallback_t_2 * eventCallback, 
                           void *eventCookie);

// Register a listener for an SHTP channel
int shtp_listenChan_2(void *pShtp,
                    uint16_t guid, const char * chan,
                    shtp_Callback_t_2 *callback, void * cookie);

// Register a listener for SHTP advertisements 
int shtp_listenAdvert_2(void *pShtp,
                      uint16_t guid,
                      shtp_AdvertCallback_t_2 *advertCallback, void * cookie);

// Look up the channel number for a particular app, channel.
uint8_t shtp_chanNo_2(void *pShtp,
                    const char * appName, const char * chanName);

// Send an SHTP payload on a particular channel
int shtp_send_2(void *pShtp,
              uint8_t channel, const uint8_t *payload, uint16_t len);

// Check for received data and process it.
void shtp_service_2(void *pShtp);

// #ifdef SHTP_H
#endif
