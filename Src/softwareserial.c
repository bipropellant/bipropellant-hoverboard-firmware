#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"

#ifdef SOFTWARE_SERIAL

#define SOFTWARE_SERIAL_RX_TIMER_FREQ (SOFTWARE_SERIAL_BAUD*4)
#define SOFTWARE_SERIAL_TX_TIMER_FREQ (SOFTWARE_SERIAL_BAUD)

TIM_HandleTypeDef softwareserialtimer;
TIM_HandleTypeDef softwareserialtimerTX;

volatile unsigned int timerval = 0;
volatile unsigned int ssbits = 0;

#define DOTX

#define SOFTWARE_SERIAL_BUFFER_SIZE 256
typedef struct tag_SOFTWARE_SERIAL_BUFFER {
    unsigned char buff[SOFTWARE_SERIAL_BUFFER_SIZE];
    int head; 
    int tail; 
    int bit;

    unsigned long lasttime;
    char lastvalue;
    
    // count of buffer overflows
    unsigned int overflow;

} SOFTWARE_SERIAL_BUFFER;

volatile SOFTWARE_SERIAL_BUFFER softwareserialRXbuffer;
volatile SOFTWARE_SERIAL_BUFFER softwareserialTXbuffer;

void SoftwareSerialReadTimer(void){
  unsigned int time = softwareserialtimer.Instance->CNT;
  timerval = time;
}

void SoftwareSerialInit(void){
  memset(&softwareserialRXbuffer, 0, sizeof(softwareserialRXbuffer));
  memset(&softwareserialTXbuffer, 0, sizeof(softwareserialTXbuffer));

  softwareserialRXbuffer.bit = -1; // awaiting start bit
  softwareserialTXbuffer.bit = -1; // awaiting start bit
  
  // setup our GPIO pin for rising and falling interrupts
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = SOFTWARE_SERIAL_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
  HAL_GPIO_Init(SOFTWARE_SERIAL_PORT, &GPIO_InitStruct);


  // setup TIM2:
  __HAL_RCC_TIM2_CLK_ENABLE();
  softwareserialtimer.Instance = TIM2;
  softwareserialtimer.Init.Prescaler         = 64000000 / 2 / SOFTWARE_SERIAL_RX_TIMER_FREQ;
  softwareserialtimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
  softwareserialtimer.Init.Period            = 0xFFFF;
  softwareserialtimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  softwareserialtimer.Init.RepetitionCounter = 0;
  softwareserialtimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&softwareserialtimer);
  HAL_TIM_Base_Start(&softwareserialtimer);

#ifdef DOTX

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = SOFTWARE_SERIAL_TX_PIN;
  HAL_GPIO_Init(SOFTWARE_SERIAL_TX_PORT, &GPIO_InitStruct);

  // setup TIM3:
  __HAL_RCC_TIM3_CLK_ENABLE();
  softwareserialtimerTX.Instance = TIM3;
  softwareserialtimerTX.Init.Prescaler         = 64000000 / 2 / SOFTWARE_SERIAL_TX_TIMER_FREQ;
  softwareserialtimerTX.Init.CounterMode       = TIM_COUNTERMODE_UP;
  softwareserialtimerTX.Init.Period            = 1;
  softwareserialtimerTX.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  softwareserialtimerTX.Init.RepetitionCounter = 0;
  softwareserialtimerTX.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&softwareserialtimerTX);
  HAL_TIM_Base_Start(&softwareserialtimerTX);
  __HAL_TIM_ENABLE_IT(&softwareserialtimerTX, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
#endif

  // and enable the interrupts.
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);


}

short softwareserial_getrx(int port){
    short t = -1;
    if (softwareserialRXbuffer.head != softwareserialRXbuffer.tail){
        t = softwareserialRXbuffer.buff[softwareserialRXbuffer.tail];
        softwareserialRXbuffer.tail = ((softwareserialRXbuffer.tail + 1 ) % SOFTWARE_SERIAL_BUFFER_SIZE);
    }
    return t;
}

void softwareserial_puttx(unsigned char value){
    int count = softwareserialTXbuffer.head - softwareserialTXbuffer.tail;
    if (count < 0) 
        count += SOFTWARE_SERIAL_BUFFER_SIZE;

    if (count >= SOFTWARE_SERIAL_BUFFER_SIZE-2){
        softwareserialTXbuffer.overflow++;
        return;
    }
    softwareserialTXbuffer.buff[softwareserialTXbuffer.head] = value;
    softwareserialTXbuffer.head = ((softwareserialTXbuffer.head + 1 ) % SOFTWARE_SERIAL_BUFFER_SIZE);
}

int softwareserial_Send(unsigned char *data, int len){
    int count = softwareserialTXbuffer.head - softwareserialTXbuffer.tail;
    if (count < 0) 
        count += SOFTWARE_SERIAL_BUFFER_SIZE;

    if (count >= SOFTWARE_SERIAL_BUFFER_SIZE-2){
        softwareserialTXbuffer.overflow++;
        return;
    }

    for (int i = 0; i < len; i++){
        softwareserial_puttx( data[i] );
    }
    return 1;
}


// interrupt on rising or falling edge of serial....
void EXTI2_IRQHandler(void){
    char value = (SOFTWARE_SERIAL_PORT->IDR & SOFTWARE_SERIAL_PIN)?1:0;
    unsigned int time = softwareserialtimer.Instance->CNT;

    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET) {
        /* Clear the EXTI line 8 pending bit */
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    }

    timerval = time;

    // time is in terms of baud *8
    int tdiff = time - softwareserialRXbuffer.lasttime;
    if (tdiff < 0) tdiff += 0x10000;

    int bits = (tdiff+2)/8;

    if (bits > 10) {
        bits = 10;
        softwareserialRXbuffer.bit = -2;
    }

    if ((softwareserialRXbuffer.bit == -2) && (softwareserialRXbuffer.lastvalue == 0)) {
        softwareserialRXbuffer.bit = -1;
        softwareserialRXbuffer.buff[softwareserialRXbuffer.head] = 0;
    }

    if (softwareserialRXbuffer.bit > -2){
        for (int i = 0; i < bits; i++){
            if (softwareserialRXbuffer.bit >= 0){
                softwareserialRXbuffer.buff[softwareserialRXbuffer.head] >>= 1;
                softwareserialRXbuffer.buff[softwareserialRXbuffer.head] |= (softwareserialRXbuffer.lastvalue?0x80:0);
            }
            softwareserialRXbuffer.bit++;
            if (softwareserialRXbuffer.bit >= 8){
                int count = softwareserialRXbuffer.head - softwareserialRXbuffer.tail;
                if (count < 0) count += SOFTWARE_SERIAL_BUFFER_SIZE;
                if (count >=  SOFTWARE_SERIAL_BUFFER_SIZE-2){
                    softwareserialRXbuffer.overflow++;
                    break;
                } else {
                    softwareserialRXbuffer.head = (softwareserialRXbuffer.head + 1) % SOFTWARE_SERIAL_BUFFER_SIZE;
                    softwareserialRXbuffer.bit = -2;
                    break;
                }
            }
        }
    }

    softwareserialRXbuffer.lastvalue = value;
    softwareserialRXbuffer.lasttime = time;
}



#ifdef DOTX
// transmit interrupt
void TIM3_IRQHandler(void){
    short t = -1;


    if (__HAL_TIM_GET_FLAG(&softwareserialtimerTX, TIM_IT_UPDATE) != RESET){
        __HAL_TIM_CLEAR_FLAG(&softwareserialtimerTX, TIM_IT_UPDATE);
    }

    if (softwareserialTXbuffer.head != softwareserialTXbuffer.tail){
        t = softwareserialTXbuffer.buff[softwareserialTXbuffer.tail];
        switch (softwareserialTXbuffer.bit){
            case -1: // send start bit
                HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_RESET);
                softwareserialTXbuffer.bit++;
                break;
            case 8: // send stop bit and next
                HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_SET);
                softwareserialTXbuffer.tail = ((softwareserialTXbuffer.tail + 1 ) % SOFTWARE_SERIAL_BUFFER_SIZE);
                softwareserialTXbuffer.bit = -1;
                ssbits++;
                break;
            default:
                if (t & (0x1 << softwareserialTXbuffer.bit)){
                    HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_SET);
                } else {
                    HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_RESET);
                }
                softwareserialTXbuffer.bit++;
                break;
        }
    } else {
        // send high
        HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_SET);
    }

}
#endif


#endif