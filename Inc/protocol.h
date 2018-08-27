#ifndef PROCOTOL_H
#define PROCOTOL_H

#include "config.h"

#ifdef INCLUDE_PROTOCOL

typedef struct tag_PROTOCOL_MSG {
    unsigned char SOM; // 0x02
    unsigned char len; // len is len of ALL bytes to follow, including CS
    unsigned char bytes[254];  // variable number of data bytes, with a checksum on the end
    // checksum such that sum of byts len to CS is zero     
} PROTOCOL_MSG;

typedef struct tag_PROTOCOL_BYTES {
    unsigned char cmd; //
    unsigned char bytes[253];
} PROTOCOL_BYTES;


#define PROTOCOL_CMD_READVAL 'R'
typedef struct tag_PROTOCOL_BYTES_READVALS {
    unsigned char cmd; // 0x01
    unsigned char code; // code of value to read
} PROTOCOL_BYTES_READVALS;

#define PROTOCOL_CMD_WRITEVAL 'W'
typedef struct tag_PROTOCOL_BYTES_WRITEVALS {
    unsigned char cmd; // 0x02
    unsigned char code; // code of value to write
    unsigned char content[252]; // value to write
} PROTOCOL_BYTES_WRITEVALS;

#define PROTOCOL_CMD_ACK 'A'
#define PROTOCOL_CMD_NACK 'N'

// cause unit to restart
#define PROTOCOL_CMD_REBOOT 'B'

// response to an unkonwn command
#define PROTOCOL_CMD_UNKNOWN '?'

#define PROTOCOL_SOM 2

typedef struct tag_PROTOCOL_STAT {
    char state;
    unsigned char CS;
    unsigned char count;
    unsigned int nonsync;
    PROTOCOL_MSG curr_msg;
} PROTOCOL_STAT;

#define PROTOCOL_STATE_IDLE 0
#define PROTOCOL_STATE_WAIT_LEN 1
#define PROTOCOL_STATE_WAIT_END 2


void protocol_byte( unsigned char byte );


#endif

#endif