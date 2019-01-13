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
#include "softwareserial.h"
#include "protocol.h"
#include "comms.h"

#include <string.h>
#include <stdlib.h>

#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL1)


// from protocol.c
extern void ascii_byte( unsigned char byte );
extern void protocol_process_message(PROTOCOL_LEN_ONWARDS *msg);


//////////////////////////////////////////////////////////
// two new protocols are created, and simultaneously active
// 1. simple ascii protocol
//  press ?<CR> for a list of commands
//  this is very suitable for development and playing
// 2. a protocol with length, checksum, ACK/NACK etc.
//  this is more suitable for machine control.
//////////////////////////////////////////////////////////
//
// ASCII protocol:
// this accepts command sup to 10 bytes long terminated with CR.
// one of these commands (I) can enable an 'immediate' mode.
// In 'immediate' mode, keypresses cause immediate action;
// for example, controlling speed, or getting real-time feedback.
//
//////////////////////////////////////////////////////////
//
// Machine protocol:
// a very simple protocol, starting 02 (SOM), with length and checksum
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
// e.g. for test:
// 02 - SOM
// 06 - length = 6
// 54 - byte0 - 'cmd' 'T'
// 54 - byte1 - payload for text command - 'T'est
// 65 - byte2 - 'e'
// 73 - byte3 - 's'
// 74 - byte4 - 't'
// 06 - checksum = (00 - (06+54+54+65+73+74))&0xff = 06,
// or  can be stated as : (06+54+54+65+73+74+06)&0xff = 0
//
// if a message is received with invalid checksum, then nack will be sent.
// if a message is received complete, it will with be responded to with a 
// return message, or with the ack message
//
// for simplicities sake, we will treat the hoverboard controller as a 
// slave unit always - i.e. not ask it to send *unsolicited* messages.
// in this way, it does not need to wait for ack, etc. from the host.
// if the host gets a bad message, or no response, it can retry.
//
//////////////////////////////////////////////////////////



///////////////////////////////////////////////
// extern variables you want to read/write here


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

#ifdef DEBUG_SERIAL_USART3
// need to implement a buffering function here.
// current DMA method needs attention...
static int nosend( unsigned char *data, int len ){ return 0; };
static int (*send_serial_data)( unsigned char *data, int len ) = nosend;
static int (*send_serial_data_wait)( unsigned char *data, int len ) = nosend;
#endif
/////////////////////////////////////////////////////////////


//////////////////////////////////////////////
// variables and functions in support of parameters here
//
static void protocol_send_nack();
static void protocol_send_ack();


///////////////////////////////////////////////////
// local functions, not really for external usage
static void protocol_send(PROTOCOL_MSG *msg);
extern void protocol_process_message(PROTOCOL_LEN_ONWARDS *msg);
extern void ascii_byte( unsigned char byte );


///////////////////////////////////////////////////
// local variables for handling the machine protocol, 
// not really for external usage
//
typedef struct tag_PROTOCOL_STAT {
    char state;
    unsigned char CS;
    unsigned char count;
    unsigned int nonsync;
    PROTOCOL_MSG curr_msg;
} PROTOCOL_STAT;
PROTOCOL_STAT s;

#define PROTOCOL_STATE_IDLE 0
#define PROTOCOL_STATE_WAIT_LEN 1
#define PROTOCOL_STATE_WAIT_END 2
///////////////////////////////////////////////////




///////////////////////////////////////////////////
// process incomming serial a byte at a time
// and only when a complete, valid message is received,
// process it.
// msgs with invalid CS will get NACK response.
//
// called from main.c
// externed in protocol.h
void protocol_byte( unsigned char byte ){
    switch(s.state){
        case PROTOCOL_STATE_IDLE:
            if (byte == PROTOCOL_SOM){
                s.curr_msg.SOM = byte;
                s.state = PROTOCOL_STATE_WAIT_LEN;
                s.CS = 0;
            } else {
                //////////////////////////////////////////////////////
                // if the byte was NOT SOM (02), then treat it as an 
                // ascii protocol byte.  BOTH protocol can co-exist
                ascii_byte( byte );
                //////////////////////////////////////////////////////
            }
            break;
        case PROTOCOL_STATE_WAIT_LEN:
            s.curr_msg.len = byte;
            s.count = 0;
            s.CS += byte;
            s.state = PROTOCOL_STATE_WAIT_END;
            break;
        case PROTOCOL_STATE_WAIT_END:
            s.curr_msg.bytes[s.count++] = byte;
            s.CS += byte;
            if (s.count == s.curr_msg.len){
                if (s.CS != 0){
                    protocol_send_nack();
                } else {
                    s.curr_msg.len --; // remove CS from len
                    protocol_process_message((PROTOCOL_LEN_ONWARDS*)&s.curr_msg.len);  // this should ack or return a message
                }
                s.state = PROTOCOL_STATE_IDLE;
            }
            break;
    }
}





/////////////////////////////////////////////
// MACHINE PROTOCOL
// functions in support of the operation of the machine protocol
//
// private
void protocol_send_nack(){
    char tmp[] = { PROTOCOL_SOM, 2, PROTOCOL_CMD_NACK, 0 };
    protocol_send((PROTOCOL_MSG *)tmp);
}

// private
void protocol_send_ack(){
    char tmp[] = { PROTOCOL_SOM, 2, PROTOCOL_CMD_ACK, 0 };
    protocol_send((PROTOCOL_MSG *)tmp);
}

// called to send a message.
// just supply len|bytes - no SOM, CI, or CS
int protocol_post(PROTOCOL_LEN_ONWARDS *len_bytes){
    PROTOCOL_MSG msg;
    msg.SOM = 0x02;
    msg.len = len_bytes->len + 1;
    memcpy(msg.bytes, len_bytes->bytes, len_bytes->len);
    protocol_send(&msg);
    return 1;
}

// private
void protocol_send(PROTOCOL_MSG *msg){
    unsigned char CS = 0;
    unsigned char *src = &msg->len;
    for (int i = 0; i < msg->len; i++){
        CS -= *(src++);
    }
    msg->bytes[msg->len-1] = CS;
    send_serial_data((unsigned char *) msg, msg->len+2);
}

// called from main.c
void protocol_tick(){
    // do nothing in this protocol
}

#endif