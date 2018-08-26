#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"


// indicates that the hall sensors were not read in order - 
// i.e. we missed an edge?
volatile unsigned long skippedhall = 0;
// the real position of the wheels since startup
volatile int HallPosn[2] = {0, 0};

#ifdef HALL_INTERRUPTS

// intialisation for interrupts from hall sensor edges
void HallInterruptinit(void){
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


static const int increments[7][7] =
{
    {  0,  0,  0,  0,  0,  0,  0 },
    {  0,  0,  0, -1,  0,  1,  0 },
    {  0,  0,  0,  1,  0,  0, -1 },
    {  0,  1, -1,  0,  0,  0,  0 },
    {  0,  0,  0,  0,  0, -1,  1 },
    {  0, -1,  0,  0,  1,  0,  0 },
    {  0,  0,  1,  0, -1,  0,  0 },
};

// local data
static int last_halll = 0;
static int last_hallr = 0;


// called from edge interrups off hall GPIO pins.
void readHall(){
  uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
  uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
  uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

  uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
  uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
  uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

  uint8_t halll = hall_ul * 1 + hall_vl * 2 + hall_wl * 4;
  uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;
  
  

  int incl = increments[last_halll][halll];
  int incr = increments[last_hallr][hallr];

  HallPosn[0] = HallPosn[0] + incl;
  HallPosn[1] = HallPosn[1] + incr;

  last_halll = halll;
  last_hallr = hallr;

  // if we skipped one?
  if ((!incl) && (last_halll) && (last_halll != halll))
    skippedhall++;
  // if we skipped one?
  if ((!incr) && (last_hallr) && (last_hallr != hallr))
    skippedhall++;
}

void EXTI9_5_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
  {
    /* Clear the EXTI line 8 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
  }
  
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
  {
    /* Clear the EXTI line 9 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
  }

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
  }
 
  readHall();
} 

void EXTI15_10_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
  {
    /* Clear the EXTI line 8 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
  }
  
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
  {
    /* Clear the EXTI line 9 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
  }

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
  }

  readHall();
} 
#endif