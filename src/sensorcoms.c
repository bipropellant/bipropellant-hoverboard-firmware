#include "stm32f1xx_hal.h"
#include "sensorcoms.h"
#include "comms.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "flashcontent.h"
#include "defines.h"
#include "stdlib.h"

/////////////////////////////////////////////////////////////////////////////////////
// this file encapsulates coms with the original sensor boards
// these use the 9 bit USART mode, sending 0x100 to signal the END of a comms frame
// it uses CONTROL_SENSOR, CONTROL_BAUD (52177 for GD32 based YST board) and DEBUG_SERIAL_SENSOR
// Implements Interrupt driven USART2 & USART3 with buffers,
// and uses 9 bit serial.
/////////////////////////////////////////////////////////////////////////////////////

#ifdef CONTROL_SENSOR

////////////////////////////////////////////////////////////////
// code to read and interpret sensors
SENSOR_DATA sensor_data[2];
SENSOR_LIGHTS sensorlights[2];

///////////////////////////
// sends data on sensor port.
// set startframe=1 to add 0x100 to first byte of data.
// 2019-05-22 - rework based on assumption that 0x1xx
// represents the START of the data, not the end.
int USART_sensorSend(int port, unsigned char *data, int len, int endframe){

    int count = USART_sensor_txcount(port);
    // overflow
    if (count + len + 1 > SERIAL_USART_BUFFER_SIZE-3){
        return -1;
    }

    for (int i = 0; i < len; i++){
        unsigned short c = data[i];
        if(endframe && (i == 0)){
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
        return serial_usart_buffer_push(&usart2_it_TXbuffer, value);
    }
    return serial_usart_buffer_push(&usart3_it_TXbuffer, value);
}

SERIAL_USART_IT_BUFFERTYPE USART_sensor_getrx(int port) {
    if (port == 0){
        return serial_usart_buffer_pop(&usart2_it_RXbuffer);
    }
    return serial_usart_buffer_pop(&usart3_it_RXbuffer);
}

// inializes sensor calibration, must be called after flash init
void sensor_init(){
    memset((void *)sensorlights, 0, sizeof(sensorlights));
    memset((void *)sensor_data, 0, sizeof(sensor_data));

    sensor_data[0].Center_calibration = FlashContent.calibration_0;
    sensor_data[1].Center_calibration = FlashContent.calibration_1;
    sensor_data[0].side = 0;
    sensor_data[1].side = 1;
}

///////////////////////////
// starts transmit from buffer on specific port, if data present
int USART_sensor_starttx(int port){
    if (port == 0){
        return USART2_IT_starttx();
    }
    return USART3_IT_starttx();
}

// copy what data we have onto our sensor data structure
void sensor_copy_buffer(SENSOR_DATA *s){
    int count = s->bytecount;
    if (count > sizeof(s->complete)) {
        count = sizeof(s->complete);
    }

    int copy = 1;

    SENSOR_FRAME *f = (SENSOR_FRAME *)s->buffer;

    if ((s->buffer[5] != 0xAA) && (s->buffer[5] != 0x55)){
        char t[10];
        sprintf(t, "side %d:", s->side);
        consoleLog(t);
        consoleLog("sensor data not 55 or AA\r\n");
        for (int i = 0; i < s->bytecount; i++) {
            sprintf(t, "%2.2X ", s->buffer[i]);
            consoleLog(t);
        }
        consoleLog("\r\n");
        copy = 0;
    }

    if (copy && (f->Angle != f->Angle_duplicate)) {
        char t[50];
        sprintf(t, "angle mismatch %d != %d:\r\n", f->Angle, f->Angle_duplicate);
        consoleLog(t);
        for (int i = 0; i < s->bytecount; i++) {
            sprintf(t, "%2.2X ", s->buffer[i]);
            consoleLog(t);
        }
        consoleLog("\r\n");
        copy = 0;
    }

    if (copy && (ABS(s->last_angle - f->Angle) > 200)) {
        char t[50];
        sprintf(t, "angle change > 200 %d -> %d:\r\n", s->last_angle, f->Angle);
        consoleLog(t);
    }

    if (copy) {
        if (count != sizeof(s->complete)){
            memset(&s->complete, 0, sizeof(s->complete));
        }
        memcpy(&s->complete, s->buffer, count);
        s->last_angle = f->Angle;

        if ((ABS(s->complete.Roll) > 2000) ||
            (ABS(s->complete.Angle) > 9000)){
            if (s->rollhigh > 0) {
                s->rollhigh--;
            }
        } else {
            s->rollhigh = 5;
        }
    }
    s->framecopied = 1;
}


void sensor_read_data(){
    // read the last sensor message in the buffer
    unsigned int time_ms = HAL_GetTick();

    // 2019-05-22 - rework based on assumption that 0x1xx
    // represents the START of the data, not the end.
    // so when we get 0x1xx, we then read bytes until we have enough, then prcoess.
    for (int side = 0; side < 2; side++){
        // flush data up to last 20 bytes
        int process = 0;
        SENSOR_DATA *s = &sensor_data[side];
        unsigned char orgsw = s->complete.AA_55;
        int remaining = USART_sensor_rxcount(side);
        unsigned char *dest = s->buffer;
        while (remaining > 0) {
            short c;
            c = USART_sensor_getrx(side);
            if (c & 0x100) {
                if ((s->bytecount >= MIN_SENSOR_WORDS) && !s->framecopied){
                    sensor_copy_buffer(s);
                    if (s->framecopied)
                        process++;
                }
                // note how many words were in a frame
                s->last_sensor_words = s->bytecount;
                s->bytecount = 0;
                memset(s->buffer, 0, sizeof(s->buffer));
                s->framecopied = 0;
            }

            if ((s->bytecount >= 0) && (s->bytecount < MAX_SENSOR_WORDS)) {
                dest[s->bytecount] = (c & 0xff);
                s->bytecount ++;
            }

            // do an early read if we know how many words
            // we want - set in config.h
            // else it will wait for 0x1xx
            if ((s->bytecount >= SENSOR_WORDS) && (!s->framecopied)) {
                sensor_copy_buffer(s);
                // stop extra bytes coming in
                if (s->framecopied)
                    process++;
            }
            remaining--;
        };

        // we may have read more than one message here (process>1)
        if (process) {
            if (s->read_timeout == 0) {
                consoleLog("Sensor RX received\r\n");
            }
            s->read_timeout = 10;

            // if we just stepped on
            if (s->complete.AA_55 == 0x55) {
                s->sensor_ok = 10;
                if (orgsw == 0xAA) {
                    consoleLog("Stepped On\r\n");
                    s->Center = s->complete.Angle;
                    if (s->foottime_ms){
                        int diff = time_ms - s->foottime_ms;
                        if ((diff < 2000) && (diff > 250)){
                            s->doubletap = 1;
                        } else {
                            s->doubletap = 0;
                        }
                    }
                    s->foottime_ms = time_ms;
                } else if (s->Center !=  s->Center_calibration) {
                    if ((s->Center > s->Center_calibration) && (s->complete.Angle < s->Center))
                        s->Center = (s->Center_calibration > s->complete.Angle ? s->Center_calibration : s->complete.Angle); // max

                    if ((s->Center < s->Center_calibration) && (s->complete.Angle > s->Center))
                        s->Center = (s->Center_calibration < s->complete.Angle ? s->Center_calibration : s->complete.Angle); // min
                }
            }
            if (s->complete.AA_55 == 0xAA){
                 if (s->sensor_ok > 0) s->sensor_ok--;
                if (orgsw == 0x55)
                    consoleLog("Stepped Off\r\n");
            }

        } else {
            //if (sensor_data[side].read_timeout==10)
            //    consoleLog("\r\nSensor SOF not found");

            if (s->read_timeout > 0){
                s->read_timeout--;
                if (s->read_timeout == 0){
                    consoleLog("Sensor RX timeout\r\n");
                }

            }

            if (s->sensor_ok > 0){
                s->sensor_ok--;
                if (s->sensor_ok == 0){
                    consoleLog("SensorOK -> 0 in unprocessed poll\r\n");
                }
            }
        }
    }
}

// *** NOT USED.
int sensor_get_speeds(int16_t *speedL, int16_t *speedR){
	if (sensor_data[0].read_timeout && sensor_data[1].read_timeout){
		if ((sensor_data[0].complete.AA_55 == 0x55) && (sensor_data[0].complete.AA_55 == 0x55)){
            if (speedL){
                int angle = (sensor_data[0].complete.Angle - sensor_data[0].Center)/100;
                *speedL = CLAMP( angle , -10, 10);
            }
            if (speedR){
                int angle = (sensor_data[1].complete.Angle - sensor_data[1].Center)/100;
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

int diag_count = 0;
void sensor_set_colour(int side, int colour){
    if (sensorlights[side].colour != colour) {
        char tmp[40];
        sprintf(tmp, "colour %d %x -> %x (%d)\r\n", side, sensorlights[side].colour, colour, diag_count++);
        consoleLog(tmp);
    }
    sensorlights[side].colour = colour;
}

void sensor_send_lights(){
    // send twice - we don;t send very often, and sensor board seems
    // to need a complete frame betwen 0x1xx words to trigger
    int count = USART_sensor_txcount(0);
    // don't send unless we are empty
    if (!count) {
    USART_sensorSend(0, (unsigned char *)&sensorlights[0], 6, 1);
    USART_sensorSend(0, (unsigned char *)&sensorlights[0], 6, 1);
    }
    count = USART_sensor_txcount(1);
    // don't send unless we are empty
    if (!count) {
    USART_sensorSend(1, (unsigned char *)&sensorlights[1], 6, 1);
        USART_sensorSend(1, (unsigned char *)&sensorlights[1], 6, 1);
    }
}


#define MAX_SAMPLES 100

int getSensorBaudRate(int side) {
    uint32_t *cycle_store = NULL;
    volatile int cycle_len = 0;
    cycle_store = malloc(sizeof(uint32_t) * MAX_SAMPLES);
    cycle_len = 0;

    float rate = 0;
    uint16_t pin;
    GPIO_TypeDef *gpioport;

    if (!side) {
        gpioport = GPIOA;
        pin = GPIO_PIN_3;
    } else {
        gpioport = GPIOB;
        pin = GPIO_PIN_11;
    }

    char t[100];

    __disable_irq(); // but we want all values at the same time, without interferance
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t cycles = start_cycles;
    char value = (gpioport->IDR & pin)?1:0;
    do {
        cycles = DWT->CYCCNT;
        cycle_store[cycle_len] = cycles;

        char newvalue = (gpioport->IDR & pin)?1:0;
        if (newvalue != value) {
            cycle_len ++;
            value = newvalue;
        }
    } while ((cycle_len < MAX_SAMPLES) && ((cycles - start_cycles) < 400000));
    __enable_irq(); // but we want all values at the same time, without interferance

    if (cycle_len > 10) {
        uint32_t first_time = cycle_store[0];
        uint32_t last_time = cycle_store[cycle_len-1];
        uint32_t total_time = last_time - first_time;

        // convert to difference
        int min_cycles = 100000;
        for (int i = 0; i < cycle_len-1; i++) {
            cycle_store[i] = cycle_store[i+1] - cycle_store[i];
            if (cycle_store[i] < min_cycles) {
                min_cycles = cycle_store[i];
            }
        }


        int total_cycles = 0;
        int num_cycles = 0;
        for (int i = 0; i < cycle_len-1; i++) {
            if (cycle_store[i] < min_cycles + 20) {
                total_cycles += cycle_store[i];
                num_cycles++;
            }
        }

        float avg_cycles = 0;
        if (num_cycles) {
            avg_cycles = (float)total_cycles/(float)num_cycles;
        }
        int freq = HAL_RCC_GetHCLKFreq();
        if (avg_cycles) {
            rate = (float)freq/avg_cycles;
        }

        float bits = (float)total_time/avg_cycles;
        sprintf(t, "usart %d\r\n", side+2 );
        consoleLog(t);
        sprintf(t, "min time %d avg short time x 100 %d clk %d baud %d\r\n", min_cycles, (int)(avg_cycles*100), freq, (int)rate);
        consoleLog(t);
        sprintf(t, "total cycles %d, bits x 100 %d\r\n", (int)total_time, (int)(bits*100.0));
        consoleLog(t);
    } else {
        sprintf(t, "usart %d\r\nno input?\r\n", side+2 );
        consoleLog(t);
    }

    free(cycle_store);
    cycle_store = NULL;

    // may be zero
    return (int)rate;
}


#endif
