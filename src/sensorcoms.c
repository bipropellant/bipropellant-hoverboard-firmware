#include "stm32f1xx_hal.h"
#include "sensorcoms.h"
#include "comms.h"
#include "config.h"
#include "stdio.h"
#include "string.h"

/////////////////////////////////////////////////////////////////////////////////////
// this file encapsulates coms with the original sensor boards
// these use the 9 bit USART mode, sending 0x100 to signal the END of a comms frame
// it uses CONTROL_SENSOR, CONTROL_BAUD (52177 for GD32 based YST board) and DEBUG_SERIAL_SENSOR 
// Implements Interrupt driven USART2 & USART3 with buffers, 
// and uses 9 bit serial.
/////////////////////////////////////////////////////////////////////////////////////

#ifdef READ_SENSOR

////////////////////////////////////////////////////////////////
// code to read and interpret sensors
SENSOR_DATA sensor_data[2];
SENSOR_LIGHTS sensorlights[2];

SENSOR_LIGHTS last_sensorlights[2];


///////////////////////////
// sends data on sensor port.
// set startframe=1 to add 0x100 before data.
//
int USART_sensorSend(int port, unsigned char *data, int len, int endframe){

    int count = USART_sensor_txcount(port);
    // overflow
    if (count + len + 1 > SERIAL_USART_BUFFER_SIZE-3){
        return -1;
    }

    for (int i = 0; i < len; i++){
        unsigned short c = data[i];
        if(endframe && (i == len-1)){
            c |= 0x100;
        }
        USART_sensor_addTXshort( port, (unsigned short) c );
    }
    
    USART_sensor_starttx(port);
    return 1;
}


int USART_sensor_rxcount(int port){
    if (port == 0){
        return  serial_usart_buffer_count(&usart2_it_RXbuffer);
    }
    return  serial_usart_buffer_count(&usart3_it_RXbuffer);
}


int USART_sensor_txcount(int port){
    if (port == 0){
        return  serial_usart_buffer_count(&usart2_it_TXbuffer);
    }
    return  serial_usart_buffer_count(&usart3_it_TXbuffer);
}

void USART_sensor_addTXshort(int port, SERIAL_USART_IT_BUFFERTYPE value) {
    if (port == 0){
        serial_usart_buffer_push(&usart2_it_TXbuffer, value);
    }
    serial_usart_buffer_push(&usart3_it_TXbuffer, value);
}

SERIAL_USART_IT_BUFFERTYPE USART_sensor_getrx(int port) {
    if (port == 0){
        return serial_usart_buffer_pop(&usart2_it_RXbuffer);
    }
    return serial_usart_buffer_pop(&usart3_it_RXbuffer);
}

void sensor_init(){
    memset((void *)sensorlights, 0, sizeof(sensorlights));
    memset((void *)sensor_data, 0, sizeof(sensor_data));
}

///////////////////////////
// starts transmit from buffer on specific port, if data present
int USART_sensor_starttx(int port){
    if (port == 0){
        return USART2_IT_starttx();
    }
    return USART3_IT_starttx();
}


short rx2[2][20];
int rx2posn[2];

void sensor_read_data(){
    // read the last sensor message in the buffer
    unsigned int time_ms = HAL_GetTick();

    int counts[2];
    unsigned char rx[2][20];
    for (int side = 0; side < 2; side++){
        counts[side] = USART_sensor_rxcount(side);
        int toflush = counts[side] - 20;
        // flush data up to last 20 bytes
        for (int i = 0; i < toflush; i++){
            USART_sensor_getrx(side);
            counts[side]--;
        }

        short c = 0;
        int i = 0;
        if (counts[side] >= 20){
            // read bytes until 0x100 is found with at least 10 bytes read
            do {
                c = USART_sensor_getrx(side);
                if (c < 0) break;
                rx2[side][i] = c;
                rx[side][i++] = c & 0xff;
            } while ((!(c & 0x100) || (i < 10)) && (i < 20));
            rx2posn[side] = i;
        }


        // if we got the end of a frame, copy into data
        if ((c & 0x100) && (i >= 10)){
            unsigned char orgsw = sensor_data[side].AA_55;
            memcpy(&sensor_data[side], &rx[side][i-10], 10);
            sensor_data[side].read_timeout = 10;
            
            // if we just stepped on
            if ((sensor_data[side].AA_55 == 0x55) && (orgsw == 0xAA)){
                sensor_data[side].Center = sensor_data[side].Angle;
                sensor_data[side].sensor_ok = 10;
                if (sensor_data[side].foottime_ms){
                    int diff = time_ms - sensor_data[side].foottime_ms;
                    if ((diff < 2000) && (diff > 250)){
                        sensor_data[side].doubletap = 1;
                    } else {
                        sensor_data[side].doubletap = 0;
                    }
                }
                sensor_data[side].foottime_ms = time_ms;
            }
            if (sensor_data[side].AA_55 == 0xAA){
                if (sensor_data[side].sensor_ok > 0){
                    sensor_data[side].sensor_ok--;
                }
            }
        } else {
            if (sensor_data[side].read_timeout > 0){
                sensor_data[side].read_timeout--;
            }
            if (sensor_data[side].sensor_ok > 0){
                sensor_data[side].sensor_ok--;
            }
        }
    }
}


int sensor_get_speeds(int16_t *speedL, int16_t *speedR){
	if (sensor_data[0].read_timeout && sensor_data[1].read_timeout){
		if ((sensor_data[0].AA_55 == 0x55) && (sensor_data[0].AA_55 == 0x55)){
            if (speedL){
                int angle = (sensor_data[0].Angle - sensor_data[0].Center)/100;
                *speedL = CLAMP( angle , -10, 10);
            }
            if (speedR){
                int angle = (sensor_data[0].Angle - sensor_data[0].Center)/100;
                *speedR = CLAMP( angle , -10, 10);
            }
            return 1;
        }
	}
    if (speedL){
        *speedL = 0;
    }
    if (speedR){
        *speedR = 0;
    }
    return 0;
}


void sensor_set_flash(int side, int count){
    sensorlights[side].flashcount = count;
}
void sensor_set_colour(int side, int colour){
    sensorlights[side].colour = colour;
}


void sensor_send_lights(){
    if (memcmp(last_sensorlights, sensorlights, sizeof(sensorlights))){
        // send twice to make sure each side gets it.
        // if we sent diagnositc data, it seems to need this.
        USART_sensorSend(0, (unsigned char *)&sensorlights[0], 6, 1);
        USART_sensorSend(0, (unsigned char *)&sensorlights[0], 6, 1);
        USART_sensorSend(1, (unsigned char *)&sensorlights[1], 6, 1);
        USART_sensorSend(1, (unsigned char *)&sensorlights[1], 6, 1);
        memcpy(last_sensorlights, sensorlights, sizeof(sensorlights));
    }
}


#endif