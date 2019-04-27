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
#ifdef CONTROL_SENSOR
    #include "sensorcoms.h"
#endif
#include "protocol.h"
#ifdef HALL_INTERRUPTS
    #include "hallinterrupts.h"
#endif
#ifdef SOFTWARE_SERIAL
    #include "softwareserial.h"
#endif
#ifndef SKIP_ELECTRICAL_MEASUREMENTS
    #include "bldc.h"
#endif
#ifdef FLASH_STORAGE
    #include "flashcontent.h"
    #include "flashaccess.h"
#endif
#include "comms.h"

#include <string.h>
#include <stdlib.h>


//////////////////////////////////////////////////////////
// from protocol.c
extern int control_type;
extern POSN_DATA PosnData;
extern SPEED_DATA SpeedData;
//////////////////////////////////////////////////////////


#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)

//////////////////////////////////////////////////////////
//
// ASCII protocol:
// this accepts command sup to 10 bytes long terminated with CR.
// one of these commands (I) can enable an 'immediate' mode.
// In 'immediate' mode, keypresses cause immediate action;
// for example, controlling speed, or getting real-time feedback.
//
//////////////////////////////////////////////////////////


///////////////////////////////////////////////
// extern variables you want to read/write here
#ifdef CONTROL_SENSOR
extern SENSOR_DATA sensor_data[2];
extern int sensor_control;
extern int sensor_stabilise;
#endif

#ifdef FLASH_STORAGE
// from main.c
extern void change_PID_constants();
#endif

extern uint8_t enable; // global variable for motor enable
extern volatile uint32_t timeout; // global variable for timeout
extern int dspeeds[2];
extern int pwms[2];



extern uint8_t debug_out;
extern uint8_t disablepoweroff;
extern int powerofftimer;
extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
extern uint16_t buzzerLen;
extern uint8_t enablescope; // enable scope on values

static int speedB = 0;
static int steerB = 0;

static char *control_types[]={
    "none",
    "Position",
    "Speed (disabled)",
    "PWM Direct"
};

///////////////////////////////////////////////


extern int protocol_post(PROTOCOL_STAT *s, PROTOCOL_LEN_ONWARDS *len_bytes);

// from protocol.c
extern POSN Position;
extern POSN RawPosition;
extern POSN_INCR PositionIncr;

///////////////////////////////////////////////////
// from protocol.c
extern PARAMSTAT params[];
extern int paramcount;


///////////////////////////////////////////////////
// used in machine_protocol.c
void ascii_byte(PROTOCOL_STAT *s, unsigned char byte );

///////////////////////////////////////////////////
// local functions, not really for external usage
static int ascii_process_immediate(PROTOCOL_STAT *s, unsigned char byte);
static void ascii_process_msg(PROTOCOL_STAT *s, char *cmd, int len);


///////////////////////////////////////////////////
// local variables for handling the 'human' protocol,
// not really for external usage
//
static char ascii_cmd[20];
static char ascii_out[512];
static int ascii_posn = 0;
static int enable_immediate = 0;

void ascii_byte(PROTOCOL_STAT *s, unsigned char byte ){
    int skipchar = 0;
    // only if no characters buffered, process single keystorkes
    if (enable_immediate && (ascii_posn == 0)){
        // returns 1 if char should not be kept in command buffer
        skipchar = ascii_process_immediate(s, byte);
    }

    if (!skipchar){
        // on CR or LF, process gathered messages
        if ((byte == '\r') || (byte == '\n')){
            s->send_serial_data((unsigned char *) &byte, 1);
            ascii_cmd[ascii_posn] = 0;
            ascii_process_msg(s, ascii_cmd, ascii_posn);
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
    s->send_serial_data((unsigned char *) &byte, 1);
}


/////////////////////////////////////////////
// single byte commands at start of command
// - i.e. only after CR of LF and ascii buffer empty
int ascii_process_immediate(PROTOCOL_STAT *s, unsigned char byte){
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
            if (!enable) { speedB = 0; steerB = 0; PwmSteerCmd.base_pwm = 0; PwmSteerCmd.steer = 0; }
            enable = 1;
            timeout = 0;

            switch (control_type){
#ifdef FLASH_STORAGE
                case CONTROL_TYPE_POSITION:
    #ifdef HALL_INTERRUPTS
                    PosnData.wanted_posn_mm[0] += dir * 100;
                    PosnData.wanted_posn_mm[1] += dir * 100;
                    sprintf(ascii_out, "wanted_posn now %ldmm %ldmm\r\n", PosnData.wanted_posn_mm[0], PosnData.wanted_posn_mm[1]);
    #endif
                    break;
                case CONTROL_TYPE_SPEED:
#endif
                case CONTROL_TYPE_PWM:
                    speedB += 10*dir;
                    PwmSteerCmd.base_pwm += 10*dir;
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
            if (!enable) { speedB = 0; steerB = 0; PwmSteerCmd.base_pwm = 0; PwmSteerCmd.steer = 0; }
            enable = 1;
            timeout = 0;
            switch (control_type){
#ifdef FLASH_STORAGE
                case CONTROL_TYPE_POSITION:
    #ifdef HALL_INTERRUPTS
                    PosnData.wanted_posn_mm[0] += dir * 100;
                    PosnData.wanted_posn_mm[1] -= dir * 100;
                    sprintf(ascii_out, "wanted_posn now %ldmm %ldmm\r\n", PosnData.wanted_posn_mm[0], PosnData.wanted_posn_mm[1]);
    #endif
                    break;
                case CONTROL_TYPE_SPEED:
#endif
                case CONTROL_TYPE_PWM:
                    steerB += 10*dir;
                    PwmSteerCmd.steer += 10*dir;
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
            PwmSteerCmd.base_pwm = 0;
            PwmSteerCmd.steer = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
#ifdef HALL_INTERRUPTS
            HallData[0].HallSpeed_mm_per_s = HallData[1].HallSpeed_mm_per_s = 0;
#endif
            dspeeds[0] = dspeeds[1] = speedB;
            pwms[0] = pwms[1] = speedB;
#ifdef HALL_INTERRUPTS
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
#endif
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
            PwmSteerCmd.base_pwm = 0;
            PwmSteerCmd.steer = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
#ifdef HALL_INTERRUPTS
            HallData[0].HallSpeed_mm_per_s = HallData[1].HallSpeed_mm_per_s = 0;
#endif
            dspeeds[0] = dspeeds[1] = speedB;
            pwms[0] = pwms[1] = speedB;
#ifdef HALL_INTERRUPTS
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
#endif
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

#ifndef SKIP_ELECTRICAL_MEASUREMENTS
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
#endif

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
            ascii_process_immediate(s, 'x');
            processed = 1;
            control_type = (control_old+1) % CONTROL_TYPE_MAX;
            sprintf(ascii_out, "control type now %d (%s)\r\n", control_type, control_types[control_type]);
            }
            break;

        default:
            break;
    }
    s->send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));

    return processed;
}
/////////////////////////////////////////////



/////////////////////////////////////////////
// process commands which ended CR or LF
void ascii_process_msg(PROTOCOL_STAT *s, char *cmd, int len){
    ascii_out[0] = 0;

    // skip nuls, observed at startup
    while (((*cmd) == 0) && (len > 0)){
        cmd++;
        len--;
    }

    if (len == 0){ // makes double prompt if /r/n is sent by terminal
        //sprintf(ascii_out, "\r\n>");
        //s->send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));
        return;
    }

    switch(cmd[0]){
        case '?':
            // split, else too big for buffer
            snprintf(ascii_out, sizeof(ascii_out)-1,
                "Hoverboard Mk1\r\n"\
                "Cmds (press return after):\r\n"\
                " A n m l -set buzzer (freq, patt, len_ms)\r\n");
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

#ifdef CONTROL_SENSOR
            snprintf(ascii_out, sizeof(ascii_out)-1,
                " B -toggle sensor Board control\r\n");
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
#endif
            snprintf(ascii_out, sizeof(ascii_out)-1,
                " E - dEbug 'E'-disable all, EC-enable consoleLog, ES enable Scope\r\n"\
                " P -power control\r\n"\
                "  P -disablepoweroff\r\n");
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

            snprintf(ascii_out, sizeof(ascii_out)-1,
                "  PE enable poweroff\r\n"\
                "  Pn power off in n seconds\r\n" \
                "  Pr software reset\r\n" \
                " I -enable Immediate commands:\r\n"\
                "   W/S/A/D/X -Faster/Slower/Lefter/Righter/DisableDrive\r\n");
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

#ifdef HALL_INTERRUPTS
            snprintf(ascii_out, sizeof(ascii_out)-1,
                "   H/C/G/Q -read Hall posn,speed/read Currents/read GPIOs/Quit immediate mode\r\n");
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
#endif

            snprintf(ascii_out, sizeof(ascii_out)-1,

#ifdef CONTROL_SENSOR
                "   N/O/R - read seNsor data/toggle pOsitional control/dangeR\r\n");
#else
                "   O - toggle pOsitional control\r\n");
#endif
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
            snprintf(ascii_out, sizeof(ascii_out)-1,
                "  Ip/Is/Iw - direct to posn/speed/pwm control\r\n"\
                " T -send a test message A-ack N-nack T-test\r\n"\
                " F - print/set a flash constant (Fa to print all, Fi to default all):\r\n"
                "  Fss - print, Fss<n> - set\r\n"
                );
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

            for (int i = 0; i < paramcount; i++){
                if (params[i].uistr){
                    snprintf(ascii_out, sizeof(ascii_out)-1,
                        "  %s - F%s<n>\r\n",
                            (params[i].description)?params[i].description:"",
                            params[i].uistr
                        );
                    s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
                }
            }
            snprintf(ascii_out, sizeof(ascii_out)-1,
                " ? -show this\r\n"
                );
            s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));

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
    #ifdef HALL_INTERRUPTS
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
    #endif
            sprintf(ascii_out, "Sensor control now %d\r\n", sensor_control);
            break;
#endif
        case 'C':
        case 'c':
            ascii_process_immediate(s, 'c');
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
#ifdef FLASH_STORAGE
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
                        for (int i = 0; i < paramcount; i++){
                            if (params[i].uistr){
                                switch (params[i].ui_type){
                                    case UI_SHORT:
                                        // read it
                                        if (params[i].preread) params[i].preread();
                                        sprintf(ascii_out, "%s(%s): %d\r\n",
                                                (params[i].description)?params[i].description:"",
                                                params[i].uistr,
                                                (int)*(short *)params[i].ptr);
                                        s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
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
                        int count = paramcount;
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
                                                s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
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
#endif
        case 'G':
        case 'g':
            ascii_process_immediate(s, 'g');
            // already sent
            ascii_out[0] = 0;
            break;

        case 'H':
        case 'h':
            ascii_process_immediate(s, 'h');
            // already sent
            ascii_out[0] = 0;
            break;

        case 'I':
        case 'i':
            speedB = 0;
            steerB = 0;
            PwmSteerCmd.base_pwm = 0;
            PwmSteerCmd.steer = 0;
            SpeedData.wanted_speed_mm_per_sec[0] = SpeedData.wanted_speed_mm_per_sec[1] = speedB;
            dspeeds[0] = dspeeds[1] = speedB;
#ifdef HALL_INTERRUPTS
            PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
            PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
#endif
            if (len == 1){
                enable_immediate = 1;
                sprintf(ascii_out, "Immediate commands enabled - WASDXHCGQ\r\n>");
            } else {
                switch (cmd[1] | 0x20){
#ifdef FLASH_STORAGE
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
#endif
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
            ascii_process_immediate(s, 'n');
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
                    s->send_serial_data_wait((unsigned char *)ascii_out, strlen(ascii_out));
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
                        //protocol_send_ack();
                        break;
                    case 'N':
                    case 'n':
                        //protocol_send_nack();
                        break;
                    case 'T':
                    case 't':{
                            char tmp[] = { 5, PROTOCOL_CMD_TEST, 'T', 'e', 's', 't' };
                            protocol_post(s, (PROTOCOL_LEN_ONWARDS*)tmp);
                        }
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
    s->send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));
    // prompt
    sprintf(ascii_out, ">");
    s->send_serial_data((unsigned char *) ascii_out, strlen(ascii_out));


}
/////////////////////////////////////////////


#endif