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
#ifndef SOFTWARESERIAL_H
#define SOFTWARESERIAL_H

#include "config.h"

#ifdef SOFTWARE_SERIAL


//////////////////////////////////////////////////////////
// init8ilaise timers and GPIOs
void SoftwareSerialInit(void);

//////////////////////////////////////////////////////////
// get a character, -1 if none.
short softwareserial_getrx(); 

// get a character, -1 if none.  But don't move buffer
short softwareserial_peekrx();

//////////////////////////////////////////////////////////
// flush receive
void softwareserial_flushRX();

//////////////////////////////////////////////////////////
// flush tx
void softwareserial_flushTX();

//////////////////////////////////////////////////////////
// return 1 if chars are available
int softwareserial_available();

//////////////////////////////////////////////////////////
// copy a buffer of data to the output buffer, fail if no room
int softwareserial_Send(unsigned char *data, int len);

//////////////////////////////////////////////////////////
// copy a buffer of data to the output buffer in chunks
// waiting if necessary for the data to leave so there is room
// use sparingly as this will pause main loop!!!
// used in 'protocol' for long strings, like '?' response
int softwareserial_Send_Wait(unsigned char *data, int len);




void softwareserialRXInterrupt(void);

#endif //SOFTWARE_SERIAL


#endif //SOFTWARESERIAL_H