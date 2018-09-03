#include "stm32f1xx_hal.h"
#include "sensorcoms.h"
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

UART_HandleTypeDef sensoruart2;
UART_HandleTypeDef sensoruart3;

////////////////////////////////////////////////////////////////
// code to read and interpret sensors
SENSOR_DATA sensor_data[2];
SENSOR_LIGHTS sensorlights[2];

SENSOR_LIGHTS last_sensorlights[2];

// un comment to run 8 bit with non original sensor boards ?
//#define SENSOR8BIT
#define SENSOR_BUFFER_SIZE 256
typedef struct tag_sensor_buffer {
#ifdef SENSOR8BIT    
    unsigned char buff[SENSOR_BUFFER_SIZE];
#else
    unsigned short buff[SENSOR_BUFFER_SIZE];
#endif
    int head; 
    int tail; 
    
    // count of buffer overflows
    unsigned int overflow;

} SENSOR_BUFFER;

volatile SENSOR_BUFFER sensorTXbuffers[2];
volatile SENSOR_BUFFER sensorRXbuffers[2];

void USART_init_sensor_port_USART2(){
    memset(&sensoruart2, 0, sizeof(sensoruart2));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    //__HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    sensoruart2.Instance = USART2;
    sensoruart2.Init.BaudRate = CONTROL_SENSOR_BAUD;
#ifdef SENSOR8BIT    
    sensoruart2.Init.WordLength = UART_WORDLENGTH_8B;
#else
    sensoruart2.Init.WordLength = UART_WORDLENGTH_9B;
#endif    
    sensoruart2.Init.StopBits = UART_STOPBITS_1;
    sensoruart2.Init.Parity = UART_PARITY_NONE;
    sensoruart2.Init.Mode = UART_MODE_TX_RX;
    sensoruart2.Init.OverSampling = UART_OVERSAMPLING_16;
    sensoruart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    //sensoruart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&sensoruart2);

    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    // start interrupt receive?
    __HAL_UART_ENABLE_IT(&sensoruart2, UART_IT_RXNE);

}

void USART_init_sensor_port_USART3(){
    memset(&sensoruart3, 0, sizeof(sensoruart3));
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    sensoruart3.Instance = USART3;
    sensoruart3.Init.BaudRate = CONTROL_SENSOR_BAUD;
#ifdef SENSOR8BIT    
    sensoruart3.Init.WordLength = UART_WORDLENGTH_8B;
#else
    sensoruart3.Init.WordLength = UART_WORDLENGTH_9B;
#endif    
    sensoruart3.Init.StopBits = UART_STOPBITS_1;
    sensoruart3.Init.Parity = UART_PARITY_NONE;
    sensoruart3.Init.Mode = UART_MODE_TX_RX;
    sensoruart3.Init.OverSampling = UART_OVERSAMPLING_16;
    sensoruart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&sensoruart3);

    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // start interrupt receive?
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    __HAL_UART_ENABLE_IT(&sensoruart3, UART_IT_RXNE);
}

void sensor_USART_init(){
  memset((void *)sensorTXbuffers, 0, sizeof(sensorTXbuffers));
  memset((void *)sensorRXbuffers, 0, sizeof(sensorRXbuffers));
  memset((void *)sensorlights, 0, sizeof(sensorlights));
  memset((void *)sensor_data, 0, sizeof(sensor_data));


  USART_init_sensor_port_USART2();
  USART_init_sensor_port_USART3();

}

///////////////////////////
// starts transmit from buffer on specific port, if data present
int USART_sensor_starttx(int port){
    switch(port){
        case 0:
            __HAL_UART_ENABLE_IT(&sensoruart2, UART_IT_TXE);
            break;
        case 1:
            __HAL_UART_ENABLE_IT(&sensoruart3, UART_IT_TXE);
            break;
    }

    return 1;
}


///////////////////////////
// sends data on sensor port.
// set startframe=1 to add 0x100 before data.
//
int USART_sensorSend(int port, unsigned char *data, int len, int endframe){

    int count = USART_sensor_txcount(port);
    // overflow
    if (count + len + 1 > SENSOR_BUFFER_SIZE-3){
        sensorTXbuffers[port].overflow++;
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
    int count = sensorRXbuffers[port].head - sensorRXbuffers[port].tail;
    if (count < 0) count += SENSOR_BUFFER_SIZE;
    return count;
}

int USART_sensor_txcount(int port){
    int count = sensorTXbuffers[port].head - sensorTXbuffers[port].tail;
    if (count < 0) count += SENSOR_BUFFER_SIZE;
    return count;
}


short USART_sensor_getrx(int port){
    short t = -1;
    if (sensorRXbuffers[port].head != sensorRXbuffers[port].tail){
        t = sensorRXbuffers[port].buff[sensorRXbuffers[port].tail];
        sensorRXbuffers[port].tail = ((sensorRXbuffers[port].tail + 1 ) % SENSOR_BUFFER_SIZE);
    }
    return t;
}


// RX, use our own IRQ routines
void USART_sensor_addRXshort(int port, unsigned short value){
    int count = sensorRXbuffers[port].head - sensorRXbuffers[port].tail;
    if (count < 0) count += SENSOR_BUFFER_SIZE;
    if (count >=  SENSOR_BUFFER_SIZE-2){
        sensorRXbuffers[port].overflow++;
        return;
    }

    sensorRXbuffers[port].buff[sensorRXbuffers[port].head] = value;
    sensorRXbuffers[port].head = ((sensorRXbuffers[port].head + 1 ) % SENSOR_BUFFER_SIZE);
}

short USART_sensor_getTXshort(int port){
    short t = -1;
    if (sensorTXbuffers[port].head != sensorTXbuffers[port].tail){
        t = sensorTXbuffers[port].buff[sensorTXbuffers[port].tail];
        sensorTXbuffers[port].tail = ((sensorTXbuffers[port].tail + 1 ) % SENSOR_BUFFER_SIZE);
    }
    return t;
}

void USART_sensor_addTXshort(int port, unsigned short value){
    int count = sensorTXbuffers[port].head - sensorTXbuffers[port].tail;
    if (count < 0) 
        count += SENSOR_BUFFER_SIZE;

    if (count >= SENSOR_BUFFER_SIZE-2){
        sensorTXbuffers[port].overflow++;
        return;
    }
    sensorTXbuffers[port].buff[sensorTXbuffers[port].head] = value;
    sensorTXbuffers[port].head = ((sensorTXbuffers[port].head + 1 ) % SENSOR_BUFFER_SIZE);
}

//////////////////////////////////////////////////////
// called from actual IRQ routines
void USART_sensor_IRQ(int port, USART_TypeDef *us)
{
    volatile uint32_t *SR = &us->SR;
    volatile uint32_t *SRread = &us->SR;
    volatile uint32_t *DR = &us->DR;
    volatile uint32_t *DRread = &us->DR;
    volatile uint32_t *CR1 = &us->CR1;
    short t = -1;

    if ((*SR) & UART_FLAG_TXE) {
        t = USART_sensor_getTXshort(port);

        if (t < 0){
            *CR1 = (*CR1 & ~(USART_CR1_TXEIE | USART_CR1_TCIE));
        } else {
            *DR = (t & 0x1ff);    
        }
    }

    if (((*SRread) & UART_FLAG_RXNE)) {
        short rword = (*DRread) & 0x01FF;
        USART_sensor_addRXshort(port, rword);
    }

    return;
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