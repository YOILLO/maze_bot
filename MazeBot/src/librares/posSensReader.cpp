#include "posSensReader.h"

namespace posSensor
{
    /*
    Инициализация
    */
    void Reader::m_fullInit()
    {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10 , ENABLE);
            
            pins::initFunc::pinInit(FrRiMotPins::posSens);
            pins::initFunc::pinInit(FrLeMotPins::posSens);
            pins::initFunc::pinInit(BoRiMotPins::posSens);
            pins::initFunc::pinInit(BoLeMotPins::posSens);
            
            TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
            
            TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
            TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
            TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
            TIM_TimeBaseStructure.TIM_Prescaler = 4200;
            TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);    
        
            NVIC_InitTypeDef NVIC_InitStructure;

            TIM_ARRPreloadConfig(TIM10,DISABLE);

            TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
            
            TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);

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
    /*
    Прерывания
    */
    extern "C"
    {
        void EXTI9_5_IRQHandler()
        {  
            
            if(EXTI_GetFlagStatus(EXTI_Line8))
            {
                Reader::getIsnance(posSensorE::FRONT_RIGHT).period(TIM10->CNT,GPIOB->IDR & GPIO_Pin_8);
                EXTI_ClearITPendingBit(EXTI_Line8);
            }
            else if(EXTI_GetFlagStatus(EXTI_Line7))
            {
                Reader::getIsnance(posSensorE::BACK_LEFT).period(TIM10->CNT,GPIOF->IDR & GPIO_Pin_7);
                EXTI_ClearITPendingBit(EXTI_Line7);            
            }
        }
        void EXTI0_IRQHandler()
        {
            Reader::getIsnance(posSensorE::BACK_RIGHT).period(TIM10->CNT, GPIOB->IDR & GPIO_Pin_0);
            EXTI_ClearITPendingBit(EXTI_Line0);
        }
        void EXTI1_IRQHandler()
        {
            Reader::getIsnance(posSensorE::FRONT_LEFT).period(TIM10->CNT, GPIOB->IDR & GPIO_Pin_1);
            EXTI_ClearITPendingBit(EXTI_Line1);
        }
        void TIM1_UP_TIM10_IRQHandler()
        {
            Reader::getIsnance(posSensorE::BACK_LEFT).add();
            Reader::getIsnance(posSensorE::BACK_RIGHT).add();
            Reader::getIsnance(posSensorE::FRONT_RIGHT).add();
            Reader::getIsnance(posSensorE::FRONT_LEFT).add();
            TIM_ClearITPendingBit(TIM10,TIM_IT_Update);
        }
    };
}
