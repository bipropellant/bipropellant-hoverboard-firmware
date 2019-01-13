/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "sensorcoms.h"
#include "protocol.h"
#include "hallinterrupts.h"
#include "softwareserial.h"
#include "bldc.h"

#include "flashcontent.h"
#include "flashaccess.h"
#include "comms.h"

#include <string.h>
#include <stdlib.h>

#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)


typedef struct tag_PROTOCOL_STAT {
    char allow_ascii;
    unsigned long last_send_time;
    unsigned long last_tick_time;

    char state;
    unsigned long last_char_time;
    unsigned char CS;
    unsigned char count;
    unsigned int nonsync;
    PROTOCOL_MSG2 curr_msg;
    unsigned char lastRXCI;

    unsigned int unwantedacks;
    unsigned int unwantednacks;

    char send_state;
    PROTOCOL_MSG2 curr_send_msg;
    char retries;

    int timeout1;
    int timeout2;

} PROTOCOL_STAT;
PROTOCOL_STAT s;

#define PROTOCOL_STATE_IDLE 0
#define PROTOCOL_STATE_WAIT_LEN 1
#define PROTOCOL_STATE_WAIT_CI 2
#define PROTOCOL_STATE_WAIT_END 3
#define PROTOCOL_STATE_BADCHAR 4

#define PROTOCOL_TX_IDLE 0
#define PROTOCOL_TX_WAITING 1


/////////////////////////////////////////////////////////////
// specify where to send data out of with a function pointer.
#ifdef SOFTWARE_SERIAL
static int (*send_serial_data)( unsigned char *data, int len ) = softwareserial_Send;
static int (*send_serial_data_wait)( unsigned char *data, int len ) = softwareserial_Send_Wait;
#endif

// TODO: Method to select which output is used for Protocol when both are active
#if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)
extern int USART2_IT_send(unsigned char *data, int len);

static int (*send_serial_data)( unsigned char *data, int len ) = USART2_IT_send;
static int (*send_serial_data_wait)( unsigned char *data, int len ) = USART2_IT_send;
#elif defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)
extern int USART3_IT_send(unsigned char *data, int len);

static int (*send_serial_data)( unsigned char *data, int len ) = USART3_IT_send;
static int (*send_serial_data_wait)( unsigned char *data, int len ) = USART3_IT_send;
#endif


extern void ascii_byte( unsigned char byte );
extern void protocol_process_message(PROTOCOL_MSG2 *msg);

static void protocol_send_nack(unsigned char CI);
static void protocol_send_ack(unsigned char CI);

static int protocol_send(PROTOCOL_MSG2 *msg);
static void protocol_send_raw(PROTOCOL_MSG2 *msg);


void protocol_init(){
    memset(&s, 0, sizeof(s));
    s.timeout1 = 500;
    s.timeout2 = 100;
    s.allow_ascii = 1;
}

void protocol_byte( unsigned char byte ){

    switch(s.state){
        case PROTOCOL_STATE_BADCHAR:
        case PROTOCOL_STATE_IDLE:
            if (byte == PROTOCOL_SOM){
                s.curr_msg.SOM = byte;
                s.state = PROTOCOL_STATE_WAIT_CI;
                s.CS = 0;
            } else {
                if (s.allow_ascii){
                    //////////////////////////////////////////////////////
                    // if the byte was NOT SOM (02), then treat it as an 
                    // ascii protocol byte.  BOTH protocol can co-exist
                    ascii_byte( byte );
                    //////////////////////////////////////////////////////
                } else {
                    s.last_char_time = HAL_GetTick();
                    s.state = PROTOCOL_STATE_BADCHAR;
                }
            }
            break;
        case PROTOCOL_STATE_WAIT_CI:
            s.last_char_time = HAL_GetTick();
            s.curr_msg.CI = byte;
            s.CS += byte;
            s.state = PROTOCOL_STATE_WAIT_LEN;
            break;

        case PROTOCOL_STATE_WAIT_LEN:
            s.last_char_time = HAL_GetTick();
            s.curr_msg.len = byte;
            s.count = 0;
            s.CS += byte;
            s.state = PROTOCOL_STATE_WAIT_END;
            break;
        case PROTOCOL_STATE_WAIT_END:
            s.last_char_time = HAL_GetTick();
            s.curr_msg.bytes[s.count++] = byte;
            s.CS += byte;
            if (s.count == s.curr_msg.len){
                s.last_char_time = 0;
                switch(s.curr_msg.bytes[0]){
                    case PROTOCOL_CMD_ACK:
                        if (s.send_state == PROTOCOL_TX_WAITING){
                            if (s.curr_msg.CI == s.curr_send_msg.CI){
                                s.last_send_time = 0;
                                s.send_state = PROTOCOL_TX_IDLE; 
                            } else {
                                // ignore
                                s.unwantedacks++;
                                // 'if an ACK is received which contains a CI different to the last sent message, the ACK will be ignored. (?? implications??)'
                            }
                        } else {
                            // ignore
                            // sort of:
                            // 'if an ACK is received which contains the same CI as the last ACK, the ACK will be ignored.'
                            s.unwantedacks++;
                        }
                        break;
                    case PROTOCOL_CMD_NACK:
                        // 'If an end receives a NACK, it should resend the last message with the same CI, up to 2 retries'
                        if (s.send_state == PROTOCOL_TX_WAITING){
                            // ignore CI
                            if (s.retries > 0){
                                protocol_send_raw(&s.curr_send_msg);
                                s.last_send_time = HAL_GetTick();
                                s.retries--;
                            } else {
                                s.send_state = PROTOCOL_TX_IDLE; 
                            }
                        } else {
                            // unsolicited NACK received?
                            s.unwantednacks++;
                            // ignore
                        }
                        break;
                    default:
                        if (s.CS != 0){
                            protocol_send_nack(s.curr_msg.CI);
                        } else {
                            protocol_send_ack(s.curr_msg.CI);
                            // 'if a message is received with the same CI as the last received message, ACK will be sent, but the message discarded.'
                            if (s.lastRXCI != s.curr_msg.CI){
                                protocol_process_message(&s.curr_msg);
                            }
                            s.lastRXCI = s.curr_msg.CI;
                        }
                        break;
                }
                s.state = PROTOCOL_STATE_IDLE;
            }
            break;
    }
}


void protocol_send_nack(unsigned char CI){
    char tmp[] = { PROTOCOL_SOM, CI, 2, PROTOCOL_CMD_NACK, 0 };
    protocol_send_raw((PROTOCOL_MSG2 *)tmp);
}

void protocol_send_ack(unsigned char CI){
    char tmp[] = { PROTOCOL_SOM, CI, 2, PROTOCOL_CMD_ACK, 0 };
    protocol_send_raw((PROTOCOL_MSG2 *)tmp);
}

// do we need queueing here?
int protocol_send(PROTOCOL_MSG2 *msg){
    if (s.send_state == PROTOCOL_TX_WAITING){
        // 'If an end sends a message, it should not send another until an ACK has been received or all retries sent'
        return -1;
    }
    msg->CI = s.curr_send_msg.CI+1;
    memcpy(&s.curr_send_msg, msg, sizeof(s.curr_send_msg));
    protocol_send_raw(&s.curr_send_msg);
    s.last_send_time = HAL_GetTick();
    s.retries = 2;
    return 0;
}

void protocol_send_raw(PROTOCOL_MSG2 *msg){
    unsigned char CS = 0;
    unsigned char *src = &msg->CI;
    int i;
    for (i = 0; i < msg->len+1; i++){
        CS -= *(src++);
    }
    msg->bytes[i-1] = CS;
    send_serial_data((unsigned char *) msg, msg->len+3);
}


// call regularly
void protocol_tick(){
    s.last_tick_time = HAL_GetTick();
    switch(s.send_state){
        case PROTOCOL_TX_IDLE:
            break;
        case PROTOCOL_TX_WAITING:
            if ((s.last_tick_time - s.last_send_time) > s.timeout1){
                // 'If an end does not receive an ACK response within (TIMEOUT1), it should resend the last message with the same CI, up to 2 retries'
                if (s.retries > 0){
                    protocol_send_raw(&s.curr_send_msg);
                    s.last_send_time = HAL_GetTick();
                    s.retries--;
                } else {
                    s.send_state = PROTOCOL_TX_IDLE; 
                }
            }
            break;
    }

    switch(s.state){
        case PROTOCOL_STATE_IDLE:
            break;
        case PROTOCOL_STATE_BADCHAR:
            // 'normally, characters received BETWEEN messages which are not SOM should be treated as ASCII commands.'
            // 'implement a mode where non SOM characters between messages cause (TIMEOUT2) to be started, 
            // resulting in a _NACK with CI of last received message + 1_.'
            if ((s.last_tick_time - s.last_char_time) > s.timeout2){
                protocol_send_nack(s.curr_msg.CI+1);
                s.last_char_time = 0;
                s.state = PROTOCOL_STATE_IDLE;
            }
            break;
        default:
            // in a message
            // 'In receive, if in a message (SOM has been received) and the time since the last character 
            // exceeds (TIMEOUT2), the incomming message should be discarded, 
            // and a NACK should be sent with the CI of the message in progress or zero if no CI received yet'
            if ((s.last_tick_time - s.last_char_time) > s.timeout2){
                protocol_send_nack(s.curr_msg.CI);
                s.last_char_time = 0;
                s.state = PROTOCOL_STATE_IDLE;
            }
            break;
    }
}




#endif // INCLUDE_PROTOCOL2