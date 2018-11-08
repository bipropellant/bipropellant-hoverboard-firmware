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

#include <string.h>
#include <stdlib.h>

#ifdef INCLUDE_PROTOCOL

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
#ifdef CONTROL_SENSOR
extern SENSOR_DATA sensor_data[2];
extern int sensor_control;
extern int sensor_stabilise;
#endif

extern uint8_t enable; // global variable for motor enable
extern volatile uint32_t timeout; // global variable for timeout
extern int dspeeds[2];
extern int pwms[2];



extern int debug_out;
extern int disablepoweroff;
extern int powerofftimer;
extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
extern int buzzerLen;
extern int enablescope; // enable scope on values

int speedB = 0;
int steerB = 0;

int control_type = 0;
char *control_types[]={
    "none",
    "Position",
    "Speed (disabled)",
    "PWM Direct"
};

POSN_DATA PosnData = {
    {0, 0},

    200, // max pwm in posn mode
    70, // min pwm in posn mode
};


SPEED_DATA SpeedData = {
    {0, 0},

    600, // max power (PWM)
    -600,  // min power 
    40 // minimum mm/s which we can ask for
};


int speed_control = 0; // incicates protocol driven

///////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// specify where to send data out of with a function pointer.
#ifdef SOFTWARE_SERIAL
int (*send_serial_data)( unsigned char *data, int len ) = softwareserial_Send;
int (*send_serial_data_wait)( unsigned char *data, int len ) = softwareserial_Send_Wait;
#endif

#ifdef DEBUG_SERIAL_USART3
// need to implement a buffering function here.
// current DMA method needs attention...
int nosend( unsigned char *data, int len ){ return 0; };
int (*send_serial_data)( unsigned char *data, int len ) = nosend;
int (*send_serial_data_wait)( unsigned char *data, int len ) = nosend;
#endif
/////////////////////////////////////////////////////////////


//////////////////////////////////////////////
// variables and functions in support of parameters here
//
void protocol_send_nack();
void protocol_send_ack();
void protocol_send_test();


// e.g. to gather two separate speed variables togther,
typedef struct tag_SPEEDS{
    int speedl;
    int speedr;
} SPEEDS;
SPEEDS speedsx = {0,0};

// before read we call this...
void PreRead_getspeeds(void){
    speedsx.speedl = SpeedData.wanted_speed_mm_per_sec[0];
    speedsx.speedr = SpeedData.wanted_speed_mm_per_sec[1];
}

// after write we call this...
void PostWrite_setspeeds(void){
   // SpeedData.wanted_speed_mm_per_sec[0] = speedsx.speedl;
   // SpeedData.wanted_speed_mm_per_sec[1] = speedsx.speedr;
    control_type = CONTROL_TYPE_PWM;
    timeout = 0;
}

typedef struct tag_POSN {
    long LeftAbsolute;
    long RightAbsolute;
    long LeftOffset;
    long RightOffset;
} POSN;

POSN Position;

void PreRead_getposnupdate(){
    Position.LeftAbsolute = HallData[0].HallPosn_mm;
    Position.LeftOffset = HallData[0].HallPosn_mm - HallData[0].HallPosn_mm_lastread;
    Position.RightAbsolute = HallData[1].HallPosn_mm;
    Position.RightOffset = HallData[1].HallPosn_mm - HallData[1].HallPosn_mm_lastread;
}

void PostWrite_setposnupdate(){
    HallData[0].HallPosn_mm_lastread = Position.LeftAbsolute;
    HallData[1].HallPosn_mm_lastread = Position.RightAbsolute; 
}

typedef struct tag_POSN_INCR {
    long Left;
    long Right;
} POSN_INCR;

POSN_INCR PositionIncr;

void PostWrite_incrementposition(){
    PosnData.wanted_posn_mm[0] += PositionIncr.Left; 
    PosnData.wanted_posn_mm[1] += PositionIncr.Right; 
}


void PostWrite_writeflash(){
    if (FlashContent.magic != CURRENT_MAGIC){
        consoleLog("incorrect magic %d, should be %d\r\nFlash not written\r\n", FlashContent.magic, CURRENT_MAGIC);
        FlashContent.magic = CURRENT_MAGIC;
        return;
    }
    writeFlash( &FlashContent, sizeof(FlashContent) );
    consoleLog("wrote flash\r\n");
}

void PostWrite_PID(){
    change_PID_constants();
}

///////////////////////////////////////////////////
// structure used to gather variables we want to read/write.
#define PARAM_R     1
#define PARAM_RW    3

#pragma pack(push, 1)
typedef struct tag_PARAMSTAT {
    unsigned char code;     // code in protocol to refer to this
    char *description;          // if non-null, description
    char *uistr;          // if non-null, used in ascii protocol to adjust with f<str>num<cr>
    char ui_type;           // only UI_NONE or UI_SHORT
    void *ptr;              // pointer to value
    char len;               // length of value
    char rw;                // PARAM_R or PARAM_RW

    void (*preread)(void);                // function to call after write
    void (*postread)(void);                // function to call after write
    void (*prewrite)(void);                // function to call after write
    void (*postwrite)(void);                // function to call after write
} PARAMSTAT;
#pragma pack(pop)
///////////////////////////////////////////////////

#define UI_NONE 0
#define UI_SHORT 1

int version = 1;

// NOTE: Don't start uistr with 'a'
PARAMSTAT params[] = {
    { 0x00, NULL, NULL, UI_NONE, &version,           sizeof(version),        PARAM_R,    NULL, NULL, NULL, NULL },
#ifdef CONTROL_SENSOR
    { 0x01, NULL, NULL, UI_NONE, &sensor_data,       sizeof(sensor_data),    PARAM_R,    NULL, NULL, NULL, NULL },
#endif
#ifdef HALL_INTERRUPTS
    { 0x02, NULL, NULL, UI_NONE, (void *)&HallData,          sizeof(HallData),       PARAM_R,    NULL, NULL, NULL, NULL },
#endif
    { 0x03, NULL, NULL, UI_NONE, &SpeedData,         sizeof(SpeedData),      PARAM_RW,   PreRead_getspeeds, NULL, NULL, PostWrite_setspeeds },
    { 0x04, NULL, NULL, UI_NONE, &Position,          sizeof(Position),       PARAM_RW,   PreRead_getposnupdate, NULL, NULL, PostWrite_setposnupdate },
    { 0x05, NULL, NULL, UI_NONE, &PositionIncr,      sizeof(PositionIncr),   PARAM_RW,    NULL, NULL, NULL, PostWrite_incrementposition },
    { 0x06, NULL, NULL, UI_NONE, &PosnData,          sizeof(PosnData),       PARAM_RW,    NULL, NULL, NULL, NULL },

    { 0x80, "flash magic", "m", UI_SHORT, &FlashContent.magic, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_writeflash },  // write this with CURRENT_MAGIC to commit to flash

    { 0x82, "posn kp x 100", "pkp", UI_SHORT, &FlashContent.PositionKpx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x81, "posn ki x 100", "pki", UI_SHORT, &FlashContent.PositionKix100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // pid params for Position
    { 0x83, "posn kd x 100", "pkd", UI_SHORT, &FlashContent.PositionKdx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x84, "posn pwm lim", "pl", UI_SHORT, &FlashContent.PositionPWMLimit, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // e.g. 200

    { 0x86, "speed kp x 100", "skp", UI_SHORT, &FlashContent.SpeedKpx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x85, "speed ki x 100", "ski", UI_SHORT, &FlashContent.SpeedKix100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // pid params for Speed
    { 0x87, "speed kd x 100", "skd", UI_SHORT, &FlashContent.SpeedKdx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x88, "speed pwm incr lim", "sl", UI_SHORT, &FlashContent.SpeedPWMIncrementLimit, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // e.g. 20

    { 0xA0, "hoverboard enable", "he", UI_SHORT, &FlashContent.HoverboardEnable, sizeof(short), PARAM_RW, NULL, NULL, NULL, NULL } // e.g. 20
};



///////////////////////////////////////////////////
// local functions, not really for external usage
void protocol_send_nack();
void protocol_send(PROTOCOL_MSG *msg);
void process_message(PROTOCOL_MSG *msg);
int ascii_process_immediate(unsigned char byte);
void ascii_process_msg(char *cmd, int len);
void ascii_byte( unsigned char byte );



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
                    process_message(&s.curr_msg);  // this should ack or return a message
                }
                s.state = PROTOCOL_STATE_IDLE;
            }
            break;
    }
}



///////////////////////////////////////////////////
// local variables for handling the 'human' protocol, 
// not really for external usage
//
char ascii_cmd[20];
char ascii_out[512];
int ascii_posn = 0;
int enable_immediate = 0;

void ascii_byte( unsigned char byte ){
    int skipchar = 0;
    // only if no characters buffered, process single keystorkes
    if (enable_immediate && (ascii_posn == 0)){
        // returns 1 if char should not be kept in command buffer
        skipchar = ascii_process_immediate(byte);
    }

    if (!skipchar){
        // on CR or LF, process gathered messages
        if ((byte == '\r') || (byte == '\n')){
            send_serial_data((unsigned char *) &byte, 1);
            ascii_cmd[ascii_posn] = 0;
            ascii_process_msg(ascii_cmd, ascii_posn);
            ascii_posn = 0;
            // send prompt
            byte = '>';
        } else {
            if (ascii_posn < 20){
                ascii_cmd[ascii_posn++] = byte;
            } else {
                //byte = '#';
            }
        }
    } else {
        // no echo for immediate.
        // send prompt after immediate
        byte = '>';
    }
    // echo or prompt after processing
    send_serial_data((unsigned char *) &byte, 1);
}


/////////////////////////////////////////////
// single byte commands at start of command 
// - i.e. only after CR of LF and ascii buffer empty
int ascii_process_immediate(unsigned char byte){
    int processed = 0;
    ascii_out[0] = 0;

    int dir = 1;
    switch(byte){
        case 'S':
        case 's':
            dir = -1;
        case 'W':
        case 'w':
            processed = 1;
            if (!enable) { speedB = 0; steerB = 0; }
            enable = 1;
            timeout = 0;

            switch (control_type){
                case CONTROL_TYPE_POSITION:
                    PosnData.wanted_posn_mm[0] += dir * 100;
                    PosnData.wanted_posn_mm[1] += dir * 100;
                    sprintf(ascii_out, "wanted_posn now %ldmm %ldmm\r\n", PosnData.wanted_posn_mm[0], PosnData.wanted_posn_mm[1]);
                    break;
                case CONTROL_TYPE_SPEED:
                case CONTROL_TYPE_PWM:
                    speedB += 10*dir;
                    SpeedData.wanted_speed_mm_per_sec[1] = CLAMP(speedB * SPEED_COEFFICIENT -  steerB * STEER_COEFFICIENT, -1000, 1000);
                    SpeedData.wanted_speed_mm_per_sec[0] = CLAMP(speedB * SPEED_COEFFICIENT +  steerB * STEER_COEFFICIENT, -1000, 1000);
                    sprintf(ascii_out, "speed now %d, steer now %d, speedL %ld, speedR %ld\r\n", speedB, steerB, SpeedData.wanted_speed_mm_per_sec[0], SpeedData.wanted_speed_mm_per_sec[1]);
                    break;
            }
            break;

        case 'A':
        case 'a':
            dir = -1;
        case 'D':
        case 'd':
            processed = 1;
            if (!enable) { speedB = 0; steerB = 0; }
            enable = 1;
            timeout = 0;
            switch (control_type){
                case CONTROL_TYPE_POSITION:
                    PosnData.wanted_posn_mm[0] += dir * 100;
                    PosnData.wanted_posn_mm[1] -= dir * 100;
                    sprintf(ascii_out, "wanted_posn now %ldmm %ldmm\r\n", PosnData.wanted_posn_mm[0], PosnData.wanted_posn_mm[1]);
                    break;
                case CONTROL_TYPE_SPEED:
                case CONTROL_TYPE_PWM:
                    steerB += 10*dir;
                    SpeedData.wanted_speed_mm_per_sec[1] = CLAMP(speedB * SPEED_COEFFICIENT -  steerB * STEER_COEFFICIENT, -1000, 1000);
                    SpeedData.wanted_speed_mm_per_sec[0] = CLAMP(speedB * SPEED_COEFFICIENT +  steerB * STEER_COEFFICIENT, -1000, 1000);
                    sprintf(ascii_out, "speed now %d, steer now %d, speedL %ld, speedR %ld\r\n", speedB, steerB, SpeedData.wanted_speed_mm_per_sec[0], SpeedData.wanted_speed_mm_per_sec[1]);
                    break;
            }
            break;

        case 'X':
        case 'x':
            processed = 1;
            speedB = 0; 
            steerB = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
            HallData[0].HallSpeed_mm_per_s = HallData[1].HallSpeed_mm_per_s = 0;
            dspeeds[0] = dspeeds[1] = speedB;
            pwms[0] = pwms[1] = speedB;
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
#ifdef CONTROL_SENSOR
            sensor_control = 0;
#endif
            enable = 0;
            sprintf(ascii_out, "Stop set\r\n");
            break;

        case 'Q':
        case 'q':
            processed = 1;
            enable_immediate = 0;
            speedB = 0; 
            steerB = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
            HallData[0].HallSpeed_mm_per_s = HallData[1].HallSpeed_mm_per_s = 0;
            dspeeds[0] = dspeeds[1] = speedB;
            pwms[0] = pwms[1] = speedB;
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
#ifdef CONTROL_SENSOR
            sensor_control = 0;
#endif
            control_type = 0;
            enable = 0;
            sprintf(ascii_out, "Immediate commands disabled\r\n");
            break;

#ifdef CONTROL_SENSOR
        case 'R':
        case 'r':
            processed = 1;
            sensor_stabilise ^= 1;
            sprintf(ascii_out, "Sensor Stabilisation is now %d\r\n", sensor_stabilise);
            break;
#endif

        case 'H':
        case 'h':
#ifdef HALL_INTERRUPTS
            processed = 1;
            sprintf(ascii_out, 
                "L: P:%ld(%ldmm) S:%ld(%ldmm/s) dT:%lu Skip:%lu Dma:%d\r\n"\
                "R: P:%ld(%ldmm) S:%ld(%ldmm/s) dT:%lu Skip:%lu Dma:%d\r\n",
                HallData[0].HallPosn, HallData[0].HallPosn_mm, HallData[0].HallSpeed, HallData[0].HallSpeed_mm_per_s, HallData[0].HallTimeDiff, HallData[0].HallSkipped, local_hall_params[0].dmacount,
                HallData[1].HallPosn, HallData[1].HallPosn_mm, HallData[1].HallSpeed, HallData[1].HallSpeed_mm_per_s, HallData[1].HallTimeDiff, HallData[1].HallSkipped, local_hall_params[1].dmacount
            );
#else
            sprintf(ascii_out, "Hall Data not available\r\n");
#endif
            break;

        case 'N':
        case 'n':
#ifdef CONTROL_SENSOR
            processed = 1;
            sprintf(ascii_out, 
                "L: OK:%d Foot:%d Angle:%d Roll:%d Accel:%d\r\n"\
                "R: OK:%d Foot:%d Angle:%d Roll:%d Accel:%d\r\n",
                sensor_data[0].sensor_ok, (sensor_data[0].AA_55 == 0x55)?1:0, sensor_data[0].Angle, sensor_data[0].Roll, sensor_data[0].Accelleration,
                sensor_data[1].sensor_ok, (sensor_data[1].AA_55 == 0x55)?1:0, sensor_data[1].Angle, sensor_data[1].Roll, sensor_data[1].Accelleration
            );
#else
            sprintf(ascii_out, "Sensor Data not available\r\n");
#endif
            break;

        case 'C':
        case 'c':
            processed = 1;
            sprintf(ascii_out, 
                "Bat: %dmV(%d) Temp:%dC(%d)\r\n"
                "L: Current:%dmA Avg:%dmA r1:%d r2:%d\r\n"\
                "R: Current:%dmA Avg:%dmA r1:%d r2:%d\r\n",
                (int)(electrical_measurements.batteryVoltage*1000), electrical_measurements.bat_raw, 
                (int)electrical_measurements.board_temp_deg_c, electrical_measurements.board_temp_raw,
                (int)(electrical_measurements.motors[0].dcAmps*1000.0), (int)(electrical_measurements.motors[0].dcAmpsAvg*1000.0), electrical_measurements.motors[0].r1, electrical_measurements.motors[0].r2,
                (int)(electrical_measurements.motors[1].dcAmps*1000.0), (int)(electrical_measurements.motors[1].dcAmpsAvg*1000.0), electrical_measurements.motors[1].r1, electrical_measurements.motors[1].r2
            );
            break;

        case 'G':
        case 'g':
            processed = 1;
            sprintf(ascii_out, 
                "A:%04X B:%04X C:%04X D:%04X E:%04X\r\n"\
                "Button: %d Charge:%d\r\n",
                (int)GPIOA->IDR, (int)GPIOB->IDR, (int)GPIOC->IDR, (int)GPIOD->IDR, (int)GPIOE->IDR,
                (int)(BUTTON_PORT->IDR & BUTTON_PIN)?1:0,
                (int)(CHARGER_PORT->IDR & CHARGER_PIN)?1:0
            );
            break;

        case 'O':
        case 'o':{
            int control_old = control_type;
            //stop all
            ascii_process_immediate('x');
            processed = 1;
            control_type = (control_old+1) % CONTROL_TYPE_MAX;
            sprintf(ascii_out, "control type now %d (%s)\r\n", control_type, control_types[control_type]);
            }
            break;

        default:
            break;
    }
    send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));

    return processed;
}
/////////////////////////////////////////////



/////////////////////////////////////////////
// process commands which ended CR or LF
void ascii_process_msg(char *cmd, int len){
    ascii_out[0] = 0;

    // skip nuls, observed at startup
    while (((*cmd) == 0) && (len > 0)){
        cmd++;
        len--;
    }

    if (len == 0){ // makes double prompt if /r/n is sent by terminal
        //sprintf(ascii_out, "\r\n>");
        //send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));
        return;
    }

    switch(cmd[0]){
        case '?':
            // split, else too big for buffer
            snprintf(ascii_out, sizeof(ascii_out)-1, 
                "Hoverboard Mk1\r\n"\
                "Cmds (press return after):\r\n"\
                " A n m l -set buzzer (freq, patt, len_ms)\r\n");
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

#ifdef CONTROL_SENSOR            
            snprintf(ascii_out, sizeof(ascii_out)-1, 
                " B -toggle sensor Board control\r\n");
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
#endif
            snprintf(ascii_out, sizeof(ascii_out)-1, 
                " E - dEbug 'E'-disable all, EC-enable consoleLog, ES enable Scope\r\n"\
                " P -power control\r\n"\
                "  P -disablepoweroff\r\n");
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

            snprintf(ascii_out, sizeof(ascii_out)-1, 
                "  PE enable poweroff\r\n"\
                "  Pn power off in n seconds\r\n" \
                "  Pr software reset\r\n" \
                " I -enable Immediate commands:\r\n"\
                "   W/S/A/D/X -Faster/Slower/Lefter/Righter/DisableDrive\r\n");
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

            snprintf(ascii_out, sizeof(ascii_out)-1, 
                "   H/C/G/Q -read Hall posn,speed/read Currents/read GPIOs/Quit immediate mode\r\n");
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
            snprintf(ascii_out, sizeof(ascii_out)-1, 
#ifdef CONTROL_SENSOR
                "   N/O/R - read seNsor data/toggle pOsitional control/dangeR\r\n");
#else
                "   O - toggle pOsitional control\r\n");
#endif
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
            snprintf(ascii_out, sizeof(ascii_out)-1, 
                "  Ip/Is/Iw - direct to posn/speed/pwm control\r\n"\
                " T -send a test message A-ack N-nack T-test\r\n"\
                " F - print/set a flash constant (Fa to print all, Fi to default all):\r\n"
                "  Fss - print, Fss<n> - set\r\n"
                );
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
            
            for (int i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].uistr){
                    snprintf(ascii_out, sizeof(ascii_out)-1, 
                        "  %s - F%s<n>\r\n", 
                            (params[i].description)?params[i].description:"", 
                            params[i].uistr
                        );
                    send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
                }
            }
            snprintf(ascii_out, sizeof(ascii_out)-1, 
                " ? -show this\r\n"
                );
            send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

            ascii_out[0] = 0;
            break;

        case 'A':
        case 'a':{
            int a = 0;
            int b = 0;
            int c = 0;
            if (len > 1){
                sscanf(cmd+1, "%d %d %d", &a, &b, &c);
            }
            if (a && (0==c)){
                c = 1000;
            }

            buzzerFreq = a;
            buzzerPattern = b;
            buzzerLen = c/5; // roughly 5ms per main loop, so 1s default
            sprintf(ascii_out, "Alarm set to %d %d %d\r\n", a, b, c);
            break;
        }

#ifdef CONTROL_SENSOR
        case 'B':
        case 'b':
            sensor_control ^= 1;
            control_type = 0;
            speedB = 0; 
            steerB = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
            dspeeds[0] = dspeeds[1] = speedB;
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
            sprintf(ascii_out, "Sensor control now %d\r\n", sensor_control);
            break;
#endif
        case 'C':
        case 'c':
            ascii_process_immediate('c');
            // already sent
            ascii_out[0] = 0;
            break;
        case 'E':
        case 'e':
            if (len == 1){
                debug_out = 0;
                enablescope = 0;
            } else {
                if ((cmd[1] | 0x20) == 's'){
                    enablescope = 1;
                    debug_out = 1;
                }
                if ((cmd[1] | 0x20) == 'c'){
                    debug_out = 1;
                }
            }
            sprintf(ascii_out, "debug_out now %d\r\nenablescope now %d\r\n", debug_out, enablescope);
            break;
        case 'F':
        case 'f': // setting any parameter marked with uistr
            if (len == 1){
                sprintf(ascii_out, "no flash var given\r\n");
            } else {
                if ((cmd[1] | 0x20) == 'i'){ // initilaise
                    memset(&FlashContent, 0, sizeof(FlashContent));
                    memcpy(&FlashContent, &FlashDefaults, (sizeof(FlashContent) < sizeof(FlashDefaults))?sizeof(FlashContent) : sizeof(FlashDefaults)) ;
                    writeFlash( (unsigned char *)&FlashContent, sizeof(FlashContent) );
                    sprintf(ascii_out, "Flash initiailised\r\n");
                } else {
                    if ((cmd[1] | 0x20) == 'a'){
                        // read all
                        for (int i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                            if (params[i].uistr){
                                switch (params[i].ui_type){
                                    case UI_SHORT:
                                        // read it
                                        if (params[i].preread) params[i].preread();
                                        sprintf(ascii_out, "%s(%s): %d\r\n",
                                                (params[i].description)?params[i].description:"", 
                                                params[i].uistr,
                                                (int)*(short *)params[i].ptr);
                                        send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
                                        ascii_out[0] = 0; // don't print last one twice
                                        if (params[i].postread) params[i].postread();
                                        break;
                                    default:
                                        break;
                                }
                            }
                        }
                    } else {
                        int i = 0;
                        int count = sizeof(params)/sizeof(params[0]);
                        for (i = 0; i < count; i++){
                            if (params[i].uistr){
                                if (!strncmp(params[i].uistr, &cmd[1], strlen(params[i].uistr))){
                                    switch (params[i].ui_type){
                                        case UI_SHORT:
                                            // if number supplied, write
                                            if ((cmd[1+strlen(params[i].uistr)] >= '0') && (cmd[1+strlen(params[i].uistr)] <= '9')){
                                                if (params[i].prewrite) params[i].prewrite();
                                                *((short *)params[i].ptr) = atoi(&cmd[1+strlen(params[i].uistr)]);
                                                if (params[i].postwrite) params[i].postwrite();
                                                sprintf(ascii_out, "flash var %s(%s) now %d\r\n", 
                                                    (params[i].description)?params[i].description:"", 
                                                    params[i].uistr,
                                                    (int)*(short *)params[i].ptr);
                                            } else {
                                                // read it
                                                if (params[i].preread) params[i].preread();
                                                sprintf(ascii_out, "%s(%s): %d\r\n",
                                                        (params[i].description)?params[i].description:"", 
                                                        params[i].uistr,
                                                        (int)*(short *)params[i].ptr
                                                );
                                                send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
                                                if (params[i].postread) params[i].postread();
                                            }
                                            break;
                                        default:
                                            sprintf(ascii_out, "flash var %s(%s) unsupported type\r\n",
                                                    (params[i].description)?params[i].description:"", 
                                                    params[i].uistr
                                            );
                                            break;
                                    }
                                    break; // found our param, now leave
                                }
                            }
                        }
                        if (i == count){
                            sprintf(ascii_out, "unknown flash data %s\r\n", cmd);
                        }
                    }
                }
            }
            break; // end generic read of flash or other variable
        case 'G':
        case 'g':
            ascii_process_immediate('g');
            // already sent
            ascii_out[0] = 0;
            break;

        case 'H':
        case 'h':
            ascii_process_immediate('h');
            // already sent
            ascii_out[0] = 0;
            break;

        case 'I':
        case 'i':
            speedB = 0; 
            steerB = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
            dspeeds[0] = dspeeds[1] = speedB;
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
            if (len == 1){
                enable_immediate = 1;
                sprintf(ascii_out, "Immediate commands enabled - WASDXHCGQ\r\n>");
            } else {
                switch (cmd[1] | 0x20){
                    case 's':
                        enable_immediate = 1;
                        control_type = CONTROL_TYPE_SPEED;
                        sprintf(ascii_out, "Immediate commands enabled - WASDXHCGQ - Speed control\r\n>");
                        break;
                    case 'p':
                        enable_immediate = 1;
                        control_type = CONTROL_TYPE_POSITION;
                        sprintf(ascii_out, "Immediate commands enabled - WASDXHCGQ - Position control\r\n>");
                        break;
                    case 'w':
                        enable_immediate = 1;
                        control_type = CONTROL_TYPE_PWM;
                        sprintf(ascii_out, "Immediate commands enabled - WASDXHCGQ - Power (pWm) control\r\n>");
                        break;
                }
            }
            break;

        case 'N':
        case 'n':
            ascii_process_immediate('n');
            // already sent
            ascii_out[0] = 0;
            break;

        case 'P':
        case 'p':
            if (len == 1){
                disablepoweroff = 1;
                powerofftimer = 0;
            } else {
                if ((cmd[1] | 0x20) == 'r'){
                    sprintf(ascii_out, "Reset in 500ms\r\n");
                    send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
                    HAL_Delay(500);
                    HAL_NVIC_SystemReset();
                }

                if ((cmd[1] | 0x20) == 'e'){
                    disablepoweroff = 0;
                    powerofftimer = 0;
                } else {
                    int s = -1;
                    sscanf(cmd+1, "%d", &s);
                    if (s >= 0){
                        if (s == 0){
                            powerofftimer = 1; // immediate
                        } else {
                            powerofftimer = ((s*1000)/DELAY_IN_MAIN_LOOP);
                        }
                    }
                }
            }
            sprintf(ascii_out, 
                "disablepoweroff now %d\r\n"\
                "powerofftimer now %d\r\n",
                disablepoweroff,
                powerofftimer);
            break;

        case 'T':
        case 't':
            if (len < 2){
                sprintf(ascii_out, "Test command needs A N or T qualifier\r\n");
            } else {
                // send a test message in machine protocol
                switch (cmd[1]){
                    case 'A':
                    case 'a':
                        protocol_send_ack();
                        break;
                    case 'N':
                    case 'n':
                        protocol_send_nack();
                        break;
                    case 'T':
                    case 't':
                        protocol_send_test();
                        break;
                }
                // CR before prompt.... after message
                sprintf(ascii_out, "\r\n");
            }
            break;

        default:
            sprintf(ascii_out, "Unknown cmd %c\r\n", cmd[0]);
            break;
    }
    send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));
    // prompt
    sprintf(ascii_out, ">");
    send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));


}
/////////////////////////////////////////////




/////////////////////////////////////////////
// MACHINE PROTOCOL
// functions in support of the operation of the machine protocol
//
void protocol_send_nack(){
    char tmp[] = { PROTOCOL_SOM, 2, PROTOCOL_CMD_NACK, 0 };
    protocol_send((PROTOCOL_MSG *)tmp);
}

void protocol_send_ack(){
    char tmp[] = { PROTOCOL_SOM, 2, PROTOCOL_CMD_ACK, 0 };
    protocol_send((PROTOCOL_MSG *)tmp);
}

void protocol_send_test(){
    char tmp[] = { PROTOCOL_SOM, 6, PROTOCOL_CMD_TEST, 'T', 'e', 's', 't', 0 };
    protocol_send((PROTOCOL_MSG *)tmp);
}


void protocol_send(PROTOCOL_MSG *msg){
    unsigned char CS = 0;
    unsigned char *src = &msg->len;
    for (int i = 0; i < msg->len; i++){
        CS -= *(src++);
    }
    msg->bytes[msg->len-1] = CS;
    send_serial_data((unsigned char *) msg, msg->len+2);
}


/////////////////////////////////////////////
// a complete machineprotocl message has been 
// received without error
void process_message(PROTOCOL_MSG *msg){
    PROTOCOL_BYTES *bytes = (PROTOCOL_BYTES *)msg->bytes; 
    switch (bytes->cmd){
        case PROTOCOL_CMD_READVAL:{

            PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) msg->bytes;
            PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == readvals->code){
                    if (params[i].preread) params[i].preread();
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *src = params[i].ptr;
                    for (int j = 0; j < params[i].len; j++){
                        writevals->content[j] = *(src++);
                    }
                    msg->len = 1+1+1+params[i].len+1;
                    // send back with 'read' command plus data like write.
                    protocol_send(msg);
                    if (params[i].postread) params[i].postread();
                    break;
                }
            }
            // nothing read
            if (i == sizeof(params)/sizeof(params[0])){
                msg->len = 1+1+1+0+1;
                // send back with 'read' command plus data like write.
                protocol_send(msg);
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVAL:{
            PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    if (params[i].prewrite) params[i].prewrite();
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *dest = params[i].ptr;
                    for (int j = 0; j < params[i].len; j++){
                        *(dest++) = writevals->content[j];
                    }
                    msg->len = 1+1+0+1;
                    // send back with 'write' command with no data.
                    protocol_send(msg);
                    if (params[i].postwrite) params[i].postwrite();
                }
            }
            // nothing written
            if (i == sizeof(params)/sizeof(params[0])){
                msg->len = 1+1+1+0+1;
                // send back with 'write' command plus data like write.
                protocol_send(msg);
            }
            break;
        }

        case PROTOCOL_CMD_REBOOT:
            protocol_send_ack();
            HAL_Delay(500);
            HAL_NVIC_SystemReset();
            break;

        case PROTOCOL_CMD_TEST:
            // just send it back!
            msg->bytes[0] = PROTOCOL_CMD_TESTRESPONSE;
            protocol_send(msg);
            break;

        default:
            msg->bytes[0] = PROTOCOL_CMD_UNKNOWN;
            msg->len = 2;
            protocol_send(msg);
            break;
    }
}



#endif