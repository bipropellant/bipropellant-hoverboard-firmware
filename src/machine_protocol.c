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

/*
* usage:
* call void protocol_byte( unsigned char byte ); with incoming bytes from main.call
* will call protocol_process_message(PROTOCOL_LEN_ONWARDS *) when message received (protocol.c)
* call protocol_post(PROTOCOL_LEN_ONWARDS *) to send a message
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

// from protocol.c
extern void ascii_byte( unsigned char byte );
extern void protocol_process_message(PROTOCOL_LEN_ONWARDS *msg);






//////////////////////////////////////////////////////////////////
// protocol_post() uses this structure to store outgoing messages 
// until they can be sent.
// messages are stored only as len|data
// SOM, CI, and CS are not included.
#define MACHINE_PROTOCOL_TX_BUFFER_SIZE 1024
typedef struct tag_MACHINE_PROTOCOL_TX_BUFFER {
    volatile unsigned char buff[MACHINE_PROTOCOL_TX_BUFFER_SIZE];
    volatile int head; 
    volatile int tail; 
    
    // count of buffer overflows
    volatile unsigned int overflow;

} MACHINE_PROTOCOL_TX_BUFFER;

// buffer to hold waiting messages in
MACHINE_PROTOCOL_TX_BUFFER TxBuffer;
static int mpTxQueued();
static unsigned char mpGetTxByte();
static char mpGetTxMsg(unsigned char *dest);
static void mpPutTx(unsigned char value);
//
//////////////////////////////////////////////////////////////////



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
//static int (*send_serial_data_wait)( unsigned char *data, int len ) = softwareserial_Send_Wait;
#endif

// TODO: Method to select which output is used for Protocol when both are active
#if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)
extern int USART2_IT_send(unsigned char *data, int len);

static int (*send_serial_data)( unsigned char *data, int len ) = USART2_IT_send;
//static int (*send_serial_data_wait)( unsigned char *data, int len ) = USART2_IT_send;
#elif defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)
extern int USART3_IT_send(unsigned char *data, int len);

static int (*send_serial_data)( unsigned char *data, int len ) = USART3_IT_send;
//static int (*send_serial_data_wait)( unsigned char *data, int len ) = USART3_IT_send;
#endif



// private to us
static void protocol_send_nack(unsigned char CI);
static void protocol_send_ack(unsigned char CI);
static int protocol_send(PROTOCOL_LEN_ONWARDS *len_bytes);
static void protocol_send_raw(PROTOCOL_MSG2 *msg);


// called from main.c
// externed in protocol.h
void protocol_init(){
    memset(&s, 0, sizeof(s));
    s.timeout1 = 500;
    s.timeout2 = 100;
    s.allow_ascii = 1;
}

// called from main.c
// externed in protocol.h
void protocol_byte( unsigned char byte ){

    switch(s.state){
        case PROTOCOL_STATE_BADCHAR:
        case PROTOCOL_STATE_IDLE:
            if (byte == PROTOCOL_SOM){
                s.curr_msg.SOM = byte;
                s.last_char_time = HAL_GetTick();
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

            if (s.count == s.curr_msg.len+1){
                //send_serial_data((unsigned char *) " end ", 5);
                //send_serial_data((unsigned char *) &s.CS, 1);
                s.last_char_time = 0;
                switch(s.curr_msg.bytes[0]){
                    case PROTOCOL_CMD_ACK:
                        if (s.send_state == PROTOCOL_TX_WAITING){
                            if (s.curr_msg.CI == s.curr_send_msg.CI){
                                s.last_send_time = 0;
                                s.send_state = PROTOCOL_TX_IDLE; 
                                // if we got ack, then try to send a next message
                                int txcount = mpTxQueued();
                                if (txcount){
                                    // send from tx queue
                                    protocol_send(NULL);
                                }
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
                                // if we run out of retries, then try to send a next message
                                int txcount = mpTxQueued();
                                if (txcount){
                                    // send from tx queue
                                    protocol_send(NULL);
                                }
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
                                // protocol_process_message now takes len onwards
                                protocol_process_message((PROTOCOL_LEN_ONWARDS*)&s.curr_msg.len);
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


// private
void protocol_send_nack(unsigned char CI){
    char tmp[] = { PROTOCOL_SOM, CI, 1, PROTOCOL_CMD_NACK, 0 };
    protocol_send_raw((PROTOCOL_MSG2 *)tmp);
}

// private
void protocol_send_ack(unsigned char CI){
    char tmp[] = { PROTOCOL_SOM, CI, 1, PROTOCOL_CMD_ACK, 0 };
    protocol_send_raw((PROTOCOL_MSG2 *)tmp);
}


// called to send a message.
// just supply len|bytes - no SOM, CI, or CS
// returns:
//  -1 - cannot even queue
//  0 sent immediately
//  1 queued for later TX
int protocol_post(PROTOCOL_LEN_ONWARDS *len_bytes){
    int txcount = mpTxQueued();
    if ((s.send_state != PROTOCOL_TX_WAITING) && !txcount){

        return protocol_send(len_bytes);
    }

    // add to tx queue
    int total = len_bytes->len + 1; // +1 len

    if (txcount + total >= MACHINE_PROTOCOL_TX_BUFFER_SIZE-2) {
        TxBuffer.overflow++;
        return -1;
    }

    char *src = (char *) len_bytes;
    for (int i = 0; i < total; i++) {
        mpPutTx(*(src++));
    }

    return 1; // added to queue
}


// private
// note: if NULL in, send from queue
int protocol_send(PROTOCOL_LEN_ONWARDS *len_bytes){
    if (s.send_state == PROTOCOL_TX_WAITING){
        // 'If an end sends a message, it should not send another until an ACK has been received or all retries sent'
        return -1;
    }
    unsigned char CI = s.curr_send_msg.CI+1;
    // if message input
    if (len_bytes) {
        s.curr_send_msg.SOM = PROTOCOL_SOM;
        s.curr_send_msg.CI = CI;
        memcpy(&s.curr_send_msg.len, len_bytes, len_bytes->len + 1);
    } else {
        // else try to send from queue
        int ismsg = mpGetTxMsg(&s.curr_send_msg.len);
        if (ismsg){
            s.curr_send_msg.SOM = PROTOCOL_SOM;
            s.curr_send_msg.CI = CI;
        } else {
            return -1; // nothing to send
        }
    }
    protocol_send_raw(&s.curr_send_msg);
    s.send_state = PROTOCOL_TX_WAITING;
    s.last_send_time = HAL_GetTick();
    s.retries = 2;
    return 0;
}


// private
void protocol_send_raw(PROTOCOL_MSG2 *msg){
    unsigned char CS = 0;
    int i;
    CS -= msg->CI;
    CS -= msg->len;

    for (i = 0; i < msg->len; i++){
        CS -= msg->bytes[i];
    }
    msg->bytes[i] = CS;
    send_serial_data((unsigned char *) msg, msg->len+4);
}


// called regularly from main.c
// externed from protocol.h
void protocol_tick(){
    s.last_tick_time = HAL_GetTick();
    switch(s.send_state){
        case PROTOCOL_TX_IDLE:{
                int txcount = mpTxQueued();
                if (txcount){
                    // send from tx queue
                    protocol_send(NULL);
                }
            }
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
                    // if we run out of retries, then try to send a next message
                    int txcount = mpTxQueued();
                    if (txcount){
                        // send from tx queue
                        protocol_send(NULL);
                    }
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


int mpTxQueued(){
    if (TxBuffer.head != TxBuffer.tail){
        int count = TxBuffer.head - TxBuffer.tail;
        if (count < 0){
            count += MACHINE_PROTOCOL_TX_BUFFER_SIZE;
        }
        return count;
    }
    return 0;
}

unsigned char mpGetTxByte(){
    short t = -1;
    if (TxBuffer.head != TxBuffer.tail){
        t = TxBuffer.buff[TxBuffer.tail];
        TxBuffer.tail = ((TxBuffer.tail + 1 ) % MACHINE_PROTOCOL_TX_BUFFER_SIZE);
    }
    return t;
}

char mpGetTxMsg(unsigned char *dest){
    if (mpTxQueued()) {
        unsigned char len = *(dest++) = mpGetTxByte(); // len of bytes to follow
        for (int i = 0; i < len; i++) {
            *(dest++) = mpGetTxByte(); // data
        }
        return 1; // we got a message
    }
    return 0;
}

void mpPutTx(unsigned char value){
    TxBuffer.buff[TxBuffer.head] = value;
    TxBuffer.head = ((TxBuffer.head + 1 ) % MACHINE_PROTOCOL_TX_BUFFER_SIZE);
}


#endif // INCLUDE_PROTOCOL2