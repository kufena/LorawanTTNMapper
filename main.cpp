/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "mbed.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"
#include "gps/gps.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        9000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * GPS sensor using GT-U7 MakerHawk sensor. Neo-6M based.
 */
MyGPSClass myGPS;

/**
 * Entry point for application
 */
int main(void)
{
    // setup gps.
    // switches off all but two UBX binary commands, NavPosllh and NavStatus.
    myGPS.init();
    
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }
    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * A couple of methods to write long and unsigned long values to
 * a byte array.  Both needed, because of types, although we do
 * some stuff to get around it, although we don't use it.
 */
static void writeToBuffL(long value, char* buffer, int pos) {
    for (int i = 0; i < 4; i++) {
        buffer[pos+i] = ((value >> (8 * i)) & 0XFF);
    }
}

static void writeToBuffUL(unsigned long value, char* buffer, int pos) {
    for (int i = 0; i < 4; i++) {
        buffer[pos+i] = ((value >> (8 * i)) & 0XFF);
    }
}

static void writeToBuffUS(unsigned short value, char* buffer, int pos) {
    for (int i = 0; i < 2; i++) {
        buffer[pos+i] = ((value >> (8 * i)) & 0XFF);
    }
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
   
    // NOTE THESE ARE BLOCKING METHOD CALLS - IF IT CAN'T FIND THE MESSAGE
    // IT IS LOOKING FOR IT WILL WAIT UNTIL IT DOES!
    
    // read the gps status message. 
    myGPS.getStatus();
    char fix = myGPS.ubxMessage.navStatus.gpsFix;
    char fixok = (myGPS.ubxMessage.navStatus.flags & 0x01);
    
    printf("\r\nfix:: %d, fix2:: %d\r\n",(int)fix,(int)fixok);
    
    if (fixok == 0) {
        printf("No Fix - no send.\r\n");
        myled2 = 1;
        ev_queue.call_in(3000, send_message);
        return;
    } else {
        myled2 = 0;
    }
    
    // read the gps posllh message to get lat/lon etc.
    myGPS.getLatLon();
    long lon = myGPS.ubxMessage.navPosllh.lon;
    long lat = myGPS.ubxMessage.navPosllh.lat;
    long height = myGPS.ubxMessage.navPosllh.height;
    long msl = myGPS.ubxMessage.navPosllh.hMSL;
    unsigned long hacc = myGPS.ubxMessage.navPosllh.hAcc;
    unsigned long vacc = myGPS.ubxMessage.navPosllh.vAcc;
    unsigned long ulon = 0;
    char neglon = 0;
    if (lon < 0) {
        ulon = (unsigned long) (-lon);
        neglon = 1;
    }
    else {
        ulon = (unsigned long) (lon);
        neglon = 0;
    }
    unsigned long ulat = 0;
    char neglat = 0;
    if (lat < 0) {
        ulat = (unsigned long) (-lat);
        neglat = 1;
    }
    else {
        ulat = (unsigned long) (lat);
        neglat = 0;
    }
    
    printf("\r\nSize of DOP msg is 4 + %d + 2 I guess\r\n",sizeof(NAV_DOP));
    printf("\r\nDoing DOP stuff!\r\n");
    
    myGPS.getDOP();
    unsigned short hdop = myGPS.ubxMessage.navDOP.hDOP;
    
    printf("\r\n %ld, %lu, %d, %lu, %d, %ld, %lu, %lu, %d", lon, ulon,(int)neglon,ulat,(int)neglat,height,hacc,vacc,(int)fix);
    
    // write values to tx_buffer for transmission.
    tx_buffer[0] = fix;
    writeToBuffL(lon, (char*)tx_buffer, 1);
    writeToBuffL(lat, (char*)tx_buffer, 5);
    writeToBuffL(height, (char*)tx_buffer, 9);
    writeToBuffUL(hacc, (char*)tx_buffer, 13);
    writeToBuffUL(vacc, (char*)tx_buffer, 17);
    tx_buffer[21] = neglon;
    tx_buffer[22] = neglat;
    writeToBuffUS(hdop, (char*)tx_buffer, 23);
    writeToBuffL(msl, (char*)tx_buffer, 25);
    tx_buffer[29] = fixok;
    packet_len = 31;

    // send it.
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_CONFIRMED_FLAG);

    // get status of send - do some stuff if not right.
    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
             myled1 = 1;
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            myled1 = 0;
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
