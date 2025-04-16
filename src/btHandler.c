/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

 #define BTSTACK_FILE__ "spp_counter.c"

 // *****************************************************************************
 /* EXAMPLE_START(spp_counter): SPP Server - Heartbeat Counter over RFCOMM
  *
  * @text The Serial port profile (SPP) is widely used as it provides a serial
  * port over Bluetooth. The SPP counter example demonstrates how to setup an SPP
  * service, and provide a periodic timer over RFCOMM.   
  *
  * @text Note: To test, please run the spp_counter example, and then pair from 
  * a remote device, and open the Virtual Serial Port.
  */
 // *****************************************************************************
 
 #include <inttypes.h>
 #include <stdint.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "pico/mutex.h"
 #include <GRF.h>
#include <Angle.h>
  
 #include "btstack.h"
 
 #define RFCOMM_SERVER_CHANNEL 1
 #define HEARTBEAT_PERIOD_MS 0
 
 static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
 static void SENDDATA(char data[]);

 extern float start_time;
 static uint16_t rfcomm_channel_id;
 static uint8_t  spp_service_buffer[150];
 static btstack_packet_callback_registration_t hci_event_callback_registration;
 float grf_value, angle_value;
 
 static mutex_t grf_mutex;

void init_mutex() {
    mutex_init(&grf_mutex);
}
 
 /* LISTING_START(SPPSetup): SPP service setup */ 
 static void spp_service_setup(void){
 
     // register for HCI events
     hci_event_callback_registration.callback = &packet_handler;
     hci_add_event_handler(&hci_event_callback_registration);
 
     l2cap_init();
 
 #ifdef ENABLE_BLE
     // Initialize LE Security Manager. Needed for cross-transport key derivation
     sm_init();
 #endif
 
     rfcomm_init();
     rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap
 
     // init SDP, create record for SPP and register with SDP
     sdp_init();
     memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
     spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Counter");
     sdp_register_service(spp_service_buffer);
     printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
 }
 /* LISTING_END */

 
 /* LISTING_START(PeriodicCounter): Periodic Counter */ 
 static btstack_timer_source_t heartbeat;
 #define VARIABLE_BUFFER 1
 static char lineBuffer[420];
 static char tempgrf[60];
 static char temptimegrf[60];
 static char tempground[60];
 static char temptimeground[60];
 static char tempangle[60];
 static char temptimeangle[60];
 static char temp[60];
 static char result[420];
 bool left = true;
 bool sendData = false;
 float grf[VARIABLE_BUFFER] = {0.0f};
 float timegrf[VARIABLE_BUFFER] = {0.0f};
 float angle[VARIABLE_BUFFER] = {0.0f};
 float timeAngle[VARIABLE_BUFFER] = {0.0f};
 float ground[VARIABLE_BUFFER] = {0.0f};
 float timeground[VARIABLE_BUFFER] = {0.0f};
 //extern uint64_t start_time_us;
 int counter = 0;
 float grf_result[4];
 float angle_result[2];

 static uint64_t start_time_2 = 0; // Start time in milliseconds
    static uint64_t last_print_time = 0; // Last time the counter was printed
    static int send_counter = 0; // Counter for sends

 float kneehipINPUT, weightINPUT, imukneeINPUT;
 static void  heartbeat_handler(struct btstack_timer_source *ts){

    // if(rfcomm_channel_id && sendData){
    //     SENDDATA("d");
    //     send_counter++;
    // }
     
     if (rfcomm_channel_id && counter == VARIABLE_BUFFER && sendData) {
        memset(tempgrf, 0, sizeof(tempgrf));
        memset(temptimegrf, 0, sizeof(temptimegrf));
        memset(tempground, 0, sizeof(tempground));
        memset(temptimeground, 0, sizeof(temptimeground));
        memset(tempangle, 0, sizeof(tempangle));
        memset(temptimeangle, 0, sizeof(temptimeangle));
        memset(result, 0, sizeof(result));

        for (int i = 0; i < VARIABLE_BUFFER; i++) {
            /////GRF
            if (i == 0 && grf[i] != 0.0f) {
                snprintf(tempgrf, sizeof(tempgrf), "%s", (left ? "|grfLeft" : "|grfRight"));
                snprintf(temptimegrf, sizeof(temptimegrf), "%s", (left ? "|timeGrfLeft" : "|timeGrfRight"));
            }
            if (i<VARIABLE_BUFFER-1 && (grf[i]) != 0.0f && grf[i+1]!=0.0f) {

                snprintf(temp, sizeof(temp), "%.2f ", grf[i]);
                strcat(tempgrf, temp);

                snprintf(temp, sizeof(temp), "%.2f ", timegrf[i]);
                strcat(temptimegrf, temp);
            }
            else if (grf[i] != 0.0f) {
                snprintf(temp, sizeof(temp), "%.2f", grf[i]);
                strcat(tempgrf, temp);
                snprintf(temp, sizeof(temp), "%.2f", timegrf[i]);
                strcat(temptimegrf, temp);
            }
            ///ground
            if (i == 0 && timeground[i] != 0.0f) {
                snprintf(temptimeground, sizeof(temptimeground), "%s", (left ? "|timeGroundLeft" : "|timeGroundRight"));
            }
            if (i<VARIABLE_BUFFER-1 && (timeground[i]) != 0.0f && timeground[i+1]!=0.0f) {

                snprintf(temp, sizeof(temp), "%.2f ", timeground[i]);
                strcat(temptimeground, temp);
            } else if (timeground[i] != 0.0f) {
                snprintf(temp, sizeof(temp), "%.2f", timeground[i]);
                strcat(temptimeground, temp);
            }

            if (i == 0 && ground[i] != -1.0f) {
                snprintf(tempground, sizeof(tempground), "%s", (left ? "|groundContactLeft" : "|groundContactRight"));
            }
            if (i<VARIABLE_BUFFER-1 && (timeground[i]) != -1.0f && timeground[i+1]!=0.0f) {
                snprintf(temp, sizeof(temp), "%.2f ", ground[i]);
                strcat(tempground, temp);
            } else if (timeground[i] != -1.0f) {
                snprintf(temp, sizeof(temp), "%.2f", ground[i]);
                strcat(tempground, temp);
            }

            ///angle
            if (i == 0 && angle[i] != 0.0f) {
                snprintf(tempangle, sizeof(tempangle), "%s", (left ? "|angleLeft" : "|angleRight"));
                snprintf(temptimeangle, sizeof(temptimeangle), "%s", (left ? "|timeAngleLeft" : "|timeAngleRight"));
            }
            if (i<VARIABLE_BUFFER-1 && (angle[i]) != 0.0f && angle[i+1]!=0.0f) {

                snprintf(temp, sizeof(temp), "%.2f ", angle[i]);
                strcat(tempangle, temp);

                snprintf(temp, sizeof(temp), "%.2f ", timeAngle[i]);
                strcat(temptimeangle, temp);
            } else if (angle[i] != 0.0f) {
                snprintf(temp, sizeof(temp), "%.2f", angle[i]);
                strcat(tempangle, temp);
                snprintf(temp, sizeof(temp), "%.2f", timeAngle[i]);
                strcat(temptimeangle, temp);
            }
            
        }
        // Add GRF data to result
        strcat(result, tempgrf);
        strcat(result, temptimegrf);     
        strcat(result, temptimeground);
        strcat(result, tempground);
        strcat(result, tempangle);
        strcat(result, temptimeangle);

        SENDDATA(result);
        send_counter++;
        counter = 0;
     } else if (rfcomm_channel_id && sendData) {
        get_GRF(grf_result, weightINPUT, imukneeINPUT, kneehipINPUT);
        grf[counter] = grf_result[0];         // GRF magnitude
        timegrf[counter] = grf_result[1];     // Timestamp
        ground[counter] = grf_result[2];       // Step detector
        timeground[counter] = grf_result[3]; 

        get_Angle(angle_result);
        angle[counter] = angle_result[0];     // Angle
        timeAngle[counter] = angle_result[1]; // Timestamp
        counter++;
     }

      // Check if one second has passed
    uint64_t current_time = time_us_64() / 1000; // Current time in milliseconds
    if (current_time - last_print_time >= 1000) {
        printf("Sends per second: %d\n", send_counter);
        send_counter = 0; // Reset the counter
        last_print_time = current_time; // Update the last print time
    }

     btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
     btstack_run_loop_add_timer(ts);
    }

static void SENDDATA(char data[]) {
    snprintf(lineBuffer, sizeof(lineBuffer),"%s\n", data);
    rfcomm_request_can_send_now_event(rfcomm_channel_id);
    //printf("SENT: %s\n", lineBuffer);
}
 
 static void one_shot_timer_setup(void){
     // set one-shot timer
     heartbeat.process = &heartbeat_handler;
     btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
     btstack_run_loop_add_timer(&heartbeat);
 }
 /* LISTING_END */
 
 /* LISTING_START(SppServerPacketHandler): SPP Server - Heartbeat Counter over RFCOMM */
 static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
     UNUSED(channel);
 
 /* LISTING_PAUSE */ 
     bd_addr_t event_addr;
     uint8_t   rfcomm_channel_nr;
     uint16_t  mtu;
     int i;
 
     switch (packet_type) {
         case HCI_EVENT_PACKET:
             switch (hci_event_packet_get_type(packet)) {
 /* LISTING_RESUME */ 
                 case HCI_EVENT_PIN_CODE_REQUEST:
                     // inform about pin code request
                     printf("Pin code request - using '0000'\n");
                     hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                     gap_pin_code_response(event_addr, "0000");
                     break;
 
                 case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                     // ssp: inform about user confirmation request
                     printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                     printf("SSP User Confirmation Auto accept\n");
                     break;
 
                 case RFCOMM_EVENT_INCOMING_CONNECTION:
                     rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                     rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                     rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                     printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                     rfcomm_accept_connection(rfcomm_channel_id);
                     break;
                
                 case RFCOMM_EVENT_CHANNEL_OPENED:
                     if (rfcomm_event_channel_opened_get_status(packet)) {
                         printf("RFCOMM channel open failed, status %u\n", rfcomm_event_channel_opened_get_status(packet));
                     } else {
                         rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                         mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                         printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                     }
                     break;
                 case RFCOMM_EVENT_CAN_SEND_NOW:
                     rfcomm_send(rfcomm_channel_id, (uint8_t*) lineBuffer, (uint16_t) strlen(lineBuffer));  
                     break;
 
 /* LISTING_PAUSE */                 
                 case RFCOMM_EVENT_CHANNEL_CLOSED:
                     printf("RFCOMM channel closed\n");
                     rfcomm_channel_id = 0;
                     break;
                 
                 default:
                     break;
             }
             break;
 
             case RFCOMM_DATA_PACKET:

             // Create a buffer big enough for the packet plus a null-terminator
             char packet_str[64]; // Use a size bigger than expected packets
 
             // Make sure not to overflow it
             size_t len = (size < sizeof(packet_str) - 1) ? size : (sizeof(packet_str) - 1);
 
             // Copy the packet data into the buffer
             memcpy(packet_str, packet, len);
 
             //trim received data
             for (int i = len - 1; i >= 0; i--) {
                 if (packet_str[i] == '\r' || packet_str[i] == '\n' || packet_str[i] == ' ') {
                     packet_str[i] = '\0';
                 } else {
                     break;
                 }
             }
 
             const char s[2] = " "; //we use space separation
             char *token;
             
             /* get the first token */
             token = strtok(packet_str, s);
 
             // Null-terminate the string
             packet_str[len] = '\0';
             printf("RECEIVED: ");
             printf(packet_str);
             printf("\n");
             
             if (strcmp(token, "stop") == 0) 
             { //if equal to "stop" then do:
                 sendData = false;
             }
             else if (strcmp(token, "start") == 0) 
             {
                 
                 sendData = true;
                 int counter = 0;
                 /* walk through other tokens */
                 while(token != NULL) {
                     token = strtok(NULL, s);
                     counter++;
                     switch (counter) {
                         case 1:
                             weightINPUT = atof(token); 
                             break;
                         case 2:
                             kneehipINPUT = atof(token); 
                             break;
                         case 3:
                             imukneeINPUT = atof(token); 
                             break;
                         default:
                             break;    
                     }
                 }
 
                 start_time = time_us_64() / 1000.0f;
                 start_time_2 = start_time;
                 last_print_time = start_time_2;
                 send_counter = 0;
                 //sleep_ms(1000);
             }
            //  else if (strcmp(token, "battery") == 0) 
            //  {
            //      adc_select_input(0);
            //      float battery_volt = get_battery_percentage();
            //      int battery_percent = estimate_battery_percentage(battery_volt);
            //      snprintf(result, sizeof(result), "alert battery percentage: %d", battery_percent);
            //      SENDDATA(result);
            //  }
             
             break;
 
         default:
             break;
     }
 /* LISTING_RESUME */ 
 }
 /* LISTING_END */
 
 int btstack_main(int argc, const char * argv[]);
 int btstack_main(int argc, const char * argv[]){
     (void)argc;
     (void)argv;
    
     //init_mutex();
     one_shot_timer_setup();
     spp_service_setup();
 
     gap_discoverable_control(1);
     gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
     gap_set_local_name("Test_PICO");
 
     // turn on!
     hci_power_control(HCI_POWER_ON);
     
     return 0;
 }
 /* EXAMPLE_END */