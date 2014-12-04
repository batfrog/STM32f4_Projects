/*
 *  Holy sheet. It works. Nice little ramp.
 *
 **/

#include "system_stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_dbgmcu.h"


#define ITM_Port8(n) (*((volatile unsigned char *) (0xE0000000 + 4 * n)))

/* private variables------------------------------------------------------------------------------------------*/

volatile uint16_t pulse_update;                                      // use to write updated value to CCR2
volatile uint16_t period = 16000;                                    // value of ARR
volatile uint16_t prescaler_value = 0;                               // TIM prescaler 0 = 16MHz clock

/* Calculations-----------------------------------------------------------------------------------------------*/

volatile uint16_t ramp_time = 32000000;                                       //  time (sec) / 62.5ns -> 2 seconds / 62.5ns = 32,000,000
volatile uint16_t max_pulse = 14400;                                          // 900 us /62.5ns = 14400
volatile uint16_t number_steps = 2000;	                 			          // ramp_time / period (WHOLE ramp)
volatile uint16_t steps_up = 1000;                                            // number_steps / 2
volatile uint16_t steps_down;	                                              // won't use this just yet
volatile uint16_t pulse_step = 14;                    	                      // max_pulse / steps_up (increase/decrease pulse by)

volatile uint16_t count_up = 1;
volatile uint16_t count_down = 1000;


/* Initialize Interrupt on PC8 (connect to PC7) -------------------*/

void initExtInterrupt() {
    EXTI_InitTypeDef   EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);           // pin
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;                              // must match pin doesn't work on 8. not sure why.
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
/* interrupt handler-----------------------------------------------*/

void EXTI0_IRQHandler() {


    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {

        while(ITM_Port8(0) == 0);
        ITM_Port8(0) = 'X';

    	if (count_up <= steps_up){

            EXTI_ClearITPendingBit(EXTI_Line0);						       // clear interrupt flag
            pulse_update = count_up * pulse_step;                          // Get current counter value
            count_up += 1;               							       // increment count
            TIM3 -> CCR2 = pulse_update;                                   // write to CCR2

            // TIM_SetCompare2(TIM3, current_count + pulse_step);        // Set Output Compare 1 to the new value

            while(ITM_Port8(0) == 0);
            ITM_Port8(0) = 'H';
            while(ITM_Port8(0) == 0);
            ITM_Port8(0) = 'i';

    	}
    	if (count_up  >= steps_up && count_down >= 2) {

        	EXTI_ClearITPendingBit(EXTI_Line0);
        	pulse_update = count_down * pulse_step;
        	// switch to down counter here instead of resetting main counter. use up/down
        	count_down -= 1;
        	TIM3 -> CCR2 = pulse_update;

        	while(ITM_Port8(0) == 0);
        	ITM_Port8(0) = 'D';
    	}
    	if (count_down <= 1 && count_up >= steps_up){
    		count_up = 1;
    		count_down = steps_up;
    	}

    }
}

/*-----------------------------------------------------------------*/
/* initialize LEDs attached to port C                              */
/* We'll use PC7 because it has AF2 of TIM3_CH2
/*-----------------------------------------------------------------*/

void InitializeLEDs()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Port C
    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_7 ;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpioStructure);

    // Port B PB5 pin for oscilloscope monitoring
    GPIO_InitTypeDef gpio2Structure;
    gpio2Structure.GPIO_Pin = GPIO_Pin_5 ;
    gpio2Structure.GPIO_Mode = GPIO_Mode_AF;
    gpio2Structure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio2Structure);


    GPIO_InitTypeDef interrupt;
    interrupt.GPIO_Pin = GPIO_Pin_8;
    interrupt.GPIO_Mode = GPIO_Mode_IN;
    interrupt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &interrupt);
}

/* ----------------------------------------------------------------*/
/* Set up timer to start counting                                  */
/* count upwards with a period of 500                              */
/* set the divider to 1 and the prescaler to 40000                 */
/* TIM clk = 16MHz / TIM_Prescaler                                 */
/* Period = TIM clk * period
/*-----------------------------------------------------------------*/



void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);


    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = prescaler_value; /*divide timer clock by 40000  to get 400Hz clk */
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;  // value Auto Reload Register ARR  count to 500 at 400Hz (2.5ms per count) 1.25sec period
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0; //only valid for TIM1 and TIM8 generates update event
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

/*--------------------------------------------------------------- */
/* Initialize PWM                                                 */
/* We will try for ONE led. PC7                    */
/*----------------------------------------------------------------*/

void InitializePWMChannel()
{
	TIM_OCInitTypeDef outputChannelInit = {1,};                             // what does it DO? -> selects channel to initialize
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 1;                                       // sets pulse value * TIM clk of 400Hz/2.5ms
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &outputChannelInit);                                 //MAKE SURE you're refering to the channel you want to use!
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
}

/* ------------------------------------------------------------------*/
/*                                                                   */
/* ------------------------------------------------------------------*/
