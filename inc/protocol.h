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
#ifndef PROCOTOL_H
#define PROCOTOL_H

#include "config.h"



/////////////////////////////////////////////////////////////////
// call this with received bytes; normally from main loop
extern void protocol_byte( unsigned char byte );
// call this regularly from main.c
extern void protocol_tick();
extern void protocol_init();
/////////////////////////////////////////////////////////////////


//// control structures used in firmware
typedef struct tag_POSN_DATA {
    // these get set
    long wanted_posn_mm[2];

    // configurations/constants
    int posn_max_speed; // max speed in this mode
    int posn_min_speed; // minimum speed (to get wheels moving)



    // just so it can be read back
    long posn_diff_mm[2];
    long posn_speed_demand[2];
} POSN_DATA;

extern POSN_DATA PosnData;

typedef struct tag_SPEED_DATA {
    // these get set
    long wanted_speed_mm_per_sec[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_speed; // below this, we don't ask it to do anything


    // just so it can be read back
    long speed_diff_mm_per_sec[2];
    long speed_power_demand[2];
} SPEED_DATA;

extern SPEED_DATA SpeedData;

extern int control_type;
#define CONTROL_TYPE_NONE 0
#define CONTROL_TYPE_POSITION 1
#define CONTROL_TYPE_SPEED 2
#define CONTROL_TYPE_PWM 3
#define CONTROL_TYPE_MAX 4


/////////////////////////////////////
// the rest only if we have a protocol.
#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL1) || (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)

/////////////////////////////////////////////////////////////////
// 'machine' protocol structures and definitions
//
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
/////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct tag_PROTOCOL_MSG {
    unsigned char SOM; // 0x02
    unsigned char len; // len is len of ALL bytes to follow, including CS
    unsigned char bytes[254];  // variable number of data bytes, with a checksum on the end
    // checksum such that sum of bytes len to CS is zero     
} PROTOCOL_MSG;

typedef struct tag_PROTOCOL_MSG2 {
    unsigned char SOM; // 0x02
    unsigned char CI; // continuity counter
    unsigned char len; // len is len of bytes to follow, NOT including CS
    unsigned char bytes[255];  // variable number of data bytes, with a checksum on the end, cmd is first
    // checksum such that sum of bytes CI to CS is zero     
} PROTOCOL_MSG2;

typedef struct tag_PROTOCOL_LEN_ONWARDS {
    unsigned char len; // len is len of ALL bytes to follow, including CS
    unsigned char bytes[253];  // variable number of data bytes, with a checksum on the end, cmd is first
} PROTOCOL_LEN_ONWARDS;

// content of 'bytes' above, for single byte commands
typedef struct tag_PROTOCOL_BYTES {
    unsigned char cmd; //
    unsigned char bytes[253];
} PROTOCOL_BYTES;


// content of 'bytes' above, for single byte commands
#define PROTOCOL_CMD_READVAL 'R'
typedef struct tag_PROTOCOL_BYTES_READVALS {
    unsigned char cmd; // 'R'
    unsigned char code; // code of value to read
} PROTOCOL_BYTES_READVALS;

#define PROTOCOL_CMD_WRITEVAL 'W'
typedef struct tag_PROTOCOL_BYTES_WRITEVALS {
    unsigned char cmd; // 'W'
    unsigned char code; // code of value to write
    unsigned char content[252]; // value to write
} PROTOCOL_BYTES_WRITEVALS;
#pragma pack(pop)

/////////////////////////////////////////////////////////
// command definitions
// ack - no payload
#define PROTOCOL_CMD_ACK 'A'
// nack - no payload
#define PROTOCOL_CMD_NACK 'N'

// a test command - normal payload - 'Test'
#define PROTOCOL_CMD_TEST 'T'
#define PROTOCOL_CMD_TESTRESPONSE 't'

// cause unit to restart - no payload
#define PROTOCOL_CMD_REBOOT 'B'

// response to an unkonwn command - maybe payload
#define PROTOCOL_CMD_UNKNOWN '?'

#define PROTOCOL_SOM 2
//
/////////////////////////////////////////////////////////////////




#endif

#endif