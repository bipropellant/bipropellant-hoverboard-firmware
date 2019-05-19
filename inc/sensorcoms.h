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
#pragma once

#include "config.h"

/////////////////////////////////////////////////////////////////////////////////////
// this file encapsulates coms with the original sensor boards
// these use the 9 bit USART mode, sending 0x100 to signal the start of a comms frame
/////////////////////////////////////////////////////////////////////////////////////


#ifdef READ_SENSOR

/////////////////////////////////////////////////////////
// functions to use.
void sensor_init();
void sensor_read_data();
int sensor_get_speeds(int16_t *speedL, int16_t *speedR);
void sensor_set_flash(int side, int count);
void sensor_set_colour(int side, int colour);
void sensor_send_lights();

// the IRQ.
void USART_sensor_IRQ(int port, USART_TypeDef *us);



// bytes send from sensor board
#pragma pack(push, 1)
typedef struct tag_sensor_data{
  short Angle;
  short Angle_duplicate;
  unsigned char AA_55;
  unsigned char Accelleration;
  unsigned char Accelleration_duplicate;
  short Roll;
  unsigned char header_00; // this byte gets 0x100 (9 bit serial)

  // not included in message:
  int sensor_ok; // set to 10 when 55, decremented if not
  int read_timeout;
  short Center;

  unsigned int foottime_ms; // last time we transitions this foot.
  int doubletap; // indicates user tapped foot sensor twice in 2s
} SENSOR_DATA;
#pragma pack(pop)


// bytes send to sensor.
// must be sent twice to take effect if other serial is on the same line
#pragma pack(push, 1)
typedef struct tag_sensor_lights {
  unsigned char unknown;
  unsigned char flashcount; // non zero-> red flash number of times with pause
  unsigned char unknown1;
  unsigned char unknown2;
  unsigned char unknown3;
  unsigned char colour; // this byte gets bit 8 set (on 9 bit serial);
} SENSOR_LIGHTS;
#pragma pack(pop)

SENSOR_DATA sensor_data[2];


// bit masks for colour
#define SENSOR_COLOUR_OFF 0x0 // headlamp is on always?
#define SENSOR_COLOUR_HEADLAMP_FLASH 0x1 // IR foot detectors must be closed
#define SENSOR_COLOUR_ORANGE 0x2
#define SENSOR_COLOUR_GREEN 0x4
#define SENSOR_COLOUR_RED 0x8
#define SENSOR_COLOUR_NO_HEADLIGHTS 0x10 // suppress headlamp.
#define SENSOR_COLOUR_YELLOW (SENSOR_COLOUR_RED | SENSOR_COLOUR_GREEN)




int  USART_sensorSend(int port, unsigned char *data, int len, int startframe);
SERIAL_USART_IT_BUFFERTYPE USART_sensor_getrx(int port);
int USART_sensor_rxcount(int port);
int USART_sensor_txcount(int port);
/////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////
// internal functions
int  USART_sensor_starttx(int port);
void USART_sensor_addTXshort(int port, SERIAL_USART_IT_BUFFERTYPE value);

#endif