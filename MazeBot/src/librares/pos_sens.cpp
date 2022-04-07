/*#include "pos_sens.h"
#include "pinsStr.h"
#include "pins.h"

namespace posSensors
{
    static void init();
    static bool sost[4];
    static bool flChange[4];
    static uint32_t time[4];
    static int32_t period[4];
    
    static void init()
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10 , ENABLE);
        
        pins::initFunc::pinInit(FrRiMotPins::posSens);
        pins::initFunc::pinInit(FrLeMotPins::posSens);
        pins::initFunc::pinInit(BoRiMotPins::posSens);
        pins::initFunc::pinInit(BoLeMotPins::posSens);
        
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        
        TIM_TimeBaseStructure.TIM_Period = 1-1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_Prescaler = 1000-1;
        TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);    
    
        NVIC_InitTypeDef NVIC_InitStructure;

        TIM_ARRPreloadConfig(TIM10,DISABLE);

        TIM_ClearITPendingBit(TIM10, TIM_IT_Update);

        TIM_Cmd(TIM10, ENABLE);

        NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource1);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource0);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, GPIO_PinSource7);
        EXTI_InitTypeDef extiInitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        extiInitStruct.EXTI_Line = EXTI_Line1;
        extiInitStruct.EXTI_LineCmd = ENABLE;
        extiInitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        extiInitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_Init(&extiInitStruct);
        extiInitStruct.EXTI_Line = EXTI_Line0;
        EXTI_Init(&extiInitStruct);
        extiInitStruct.EXTI_Line = EXTI_Line7;
        EXTI_Init(&extiInitStruct);
        extiInitStruct.EXTI_Line = EXTI_Line8;
        EXTI_Init(&extiInitStruct);

        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
        NVIC_Init(&NVIC_InitStruct);
        NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
        NVIC_Init(&NVIC_InitStruct);
        NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
        NVIC_Init(&NVIC_InitStruct);
    
        __enable_irq();
    }
    extern "C"
    {
        void EXTI9_5_IRQHandler()
        {  
            
            if(EXTI_GetFlagStatus(EXTI_Line8))
            {
                sost[0] = GPIOB->IDR & GPIO_Pin_8;
                if(sost[0] == 0)
                {
                    if(flChange[0])
                    {
                        period[0] = TIM11->CNT + time[0] - 1000;
                        flChange[0] = 0;
                    }
                    else
                        period[0] = TIM11->CNT - time[0];            
                }
                time[0] = TIM11->CNT;
                EXTI_ClearITPendingBit(EXTI_Line8);
            }
            else if(EXTI_GetFlagStatus(EXTI_Line7))
            {
                sost[1] = GPIOF->IDR & GPIO_Pin_7;
                if(sost[1] == 0)
                {
                    if( flChange[1])
                    {
                        period[0] = TIM11->CNT + time[1] - 1000;
                        flChange[1] = 0;
                    }
                    else
                        period[1] = TIM11->CNT - time[1];            
                }
                time[1] = TIM11->CNT;
                EXTI_ClearITPendingBit(EXTI_Line7);            
            }
        }
        void EXTI0_IRQHandler()
        {
            sost[2] = GPIOB->IDR & GPIO_Pin_0;
            if(sost[2] == 0)
                {
                    if( flChange[2])
                    {
                        period[2] = TIM11->CNT + time[2] - 1000;
                        flChange[2] = 0;
                    }
                    else
                        period[2] = TIM11->CNT - time[2];            
                }
            time[2] = TIM11->CNT;
            EXTI_ClearITPendingBit(EXTI_Line0);
        }
        void EXTI1_IRQHandler()
        {
            sost[3] = GPIOB->IDR & GPIO_Pin_0;
            if(sost[3] == 0)
                {
                    if( flChange[3])
                    {
                        period[3] = TIM11->CNT + time[3] - 1000;
                        flChange[3] = 0;
                    }
                    else
                        period[0] = TIM11->CNT - time[3];            
                }            
            time[3] = TIM11->CNT;
            EXTI_ClearITPendingBit(EXTI_Line1);
        }
        void TIM1_UP_TIM10_IRQHandler()
        {
            if(sost[0])flChange[0] = 1;
            if(sost[1])flChange[1] = 1;
            if(sost[2])flChange[2] = 1;
            if(sost[3])flChange[3] = 1;            
            TIM_ClearITPendingBit(TIM10,TIM_IT_Update);
        }
    };
}*/
