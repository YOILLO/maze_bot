#pragma once
#include "project_config.h"

static int32_t constrain(int32_t val, int32_t min, int32_t max)
{
    if(val < min)return min;
    if(val > max)return max;
    return val;
}

static int32_t constrainMin(int32_t val, int32_t min, int32_t max)
{
    if(val > min && val < max)
    {
        if(abs(val - min) > abs(val - max))
            return max;
        else 
            return min;
    }
    return val;
}

static int32_t min(int32_t val1, int32_t val2)
{
    if(val1 < val2)return val1;
    else return val2;
}
static int32_t max(int32_t val1, int32_t val2)
{
    if(val1 > val2)return val1;
    else return val2;
}
namespace pins
{
/*
	Структура для описания любого пина:
	uint16_t pinNum: номер пина
	GPIO_InitTypeDef GPIOx: порт
	GPIOMode_TypeDef mode: режим работы
	TIM_TypeDef * TIMx: таймер, привязанный к пину
	uint16_t TIM_Channel: канал таймера, привязанного к пину
	uint8_t pinSource1: источник нина(я не знаю что это)
    uint8_t AFMode: режим в случае если пин AF
*/
	struct pinStructure
    {
		uint16_t pinNum;
		GPIO_TypeDef * GPIOx;
		GPIOMode_TypeDef mode;
        GPIOPuPd_TypeDef PuPd;
        GPIOOType_TypeDef OType;
		TIM_TypeDef * TIMx;
		uint16_t TIM_Channel;
		uint8_t pinSource;
        uint8_t AFMode;
	};
	namespace initFunc
	{        
        /*
        Тактирует определенный таймер, плучаемый во входные данные
        */
		static void GPIOClock(GPIO_TypeDef * GPIOx)
        {
            if(GPIOx == GPIOA)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
				//GPIOA->ODR=0;
			}
			else if(GPIOx == GPIOB)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
				//GPIOB->ODR=0;
			}
			else if(GPIOx == GPIOC)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
				//GPIOC->ODR=0;
			}
			else if(GPIOx == GPIOD)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
				//GPIOD->ODR=0;
			}
			else if(GPIOx == GPIOE)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
				//GPIOE->ODR=0;
			}
            else if(GPIOx == GPIOF)
            {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
            }
            else if(GPIOx == GPIOG)
            {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
            }
            else if(GPIOx == GPIOH)
            {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
            }
            else if(GPIOx == GPIOI)
            {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
            }
            else if(GPIOx == GPIOJ)
            {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ,ENABLE);
            }
            else if(GPIOx == GPIOK)
            {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK,ENABLE);
            }
        }
/*
	Инициализация пива:
	Входные данные: pinStructure pin - данные о пине (подробнее см. pinStructure)
	Выходные данные:
*/
		static void pinInit(pinStructure pin)
		{
            GPIOClock(pin.GPIOx);
			GPIO_InitTypeDef GPIO_InitStructure;
			GPIO_InitStructure.GPIO_Mode = pin.mode;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_Pin = pin.pinNum;
            GPIO_InitStructure.GPIO_PuPd = pin.PuPd;
            GPIO_InitStructure.GPIO_OType = pin.OType;
			GPIO_Init(pin.GPIOx, &GPIO_InitStructure );
			GPIO_SetBits(pin.GPIOx, pin.pinNum);
		}
/*
        Тактирование таймера
        входные данные:TIM_TypeDef * TIMx - таймер, который нужно затактировать
*/
		static void timerClock(TIM_TypeDef * TIMx)
        {
            if(TIMx == TIM1)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
			}
			else if(TIMx == TIM2)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
			}
			else if(TIMx == TIM3)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
			}
			else if(TIMx == TIM4)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
			}
			else if(TIMx == TIM5)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
			}
			else if(TIMx == TIM6)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
			}
			else if(TIMx == TIM7)
            {
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
			}
			else if(TIMx == TIM8)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 , ENABLE);
			}
			else if(TIMx == TIM9)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9 , ENABLE);
			}
			else if(TIMx == TIM10)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10 , ENABLE);
			}
			else if(TIMx == TIM11)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11 , ENABLE);	
			}
			else if(TIMx == TIM12)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12 , ENABLE);
			}
			else if(TIMx == TIM13)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13 , ENABLE);
			}
			else if(TIMx == TIM14)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);
			}
        }
/*
Инициализация пина для шима:
Входные данные: pinStructure pin - пин который должен работать как шим
Выходные данные:
*/
		static void initPwm(pinStructure pin)
		{
			pinInit(pin);
            
            GPIO_PinAFConfig( pin.GPIOx , pin.pinSource, pin.AFMode);
            
            timerClock(pin.TIMx);
			//GPIO_InitTypeDef GPIO_InitStructure;
			//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
			//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
			//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
			
			TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

			TIM_TimeBaseStructure.TIM_Prescaler = 100;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_Period = 3200-1;
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(pin.TIMx, &TIM_TimeBaseStructure);
            
            TIM_CCxCmd( pin.TIMx, pin.TIM_Channel, TIM_CCx_Enable );
			
			TIM_OCInitTypeDef TIM_OCInitStructure;

			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
			TIM_OCInitStructure.TIM_Pulse = 4200;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
			TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
            if(pin.TIM_Channel == TIM_Channel_1)
                TIM_OC1Init(pin.TIMx,&TIM_OCInitStructure);
			if(pin.TIM_Channel == TIM_Channel_2)
                TIM_OC2Init(pin.TIMx,&TIM_OCInitStructure);
            if(pin.TIM_Channel == TIM_Channel_3)
                TIM_OC3Init(pin.TIMx,&TIM_OCInitStructure);
            if(pin.TIM_Channel == TIM_Channel_4)
                TIM_OC4Init(pin.TIMx,&TIM_OCInitStructure);
            
            if(pin.TIMx == TIM1 || pin.TIMx == TIM8)
            {
                //TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
                //TIM_BDTRStructInit(&TIM_BDTRInitStruct);
                //TIM_BDTRConfig(pin.TIMx, &TIM_BDTRInitStruct);
                //TIM_CCPreloadControl(pin.TIMx, ENABLE);
                TIM_CtrlPWMOutputs(pin.TIMx, ENABLE);
            }
			//TIM_OCInitTypeDef TIM_OCInitStructure;

			//TIM_CtrlPWMOutputs(pin.TIMx, ENABLE);
            //TIM_PWMIConfig( pin.TIMx, &TIM_OCInitStructure);
			
			//NVIC_EnableIRQ(TIM1_UP_IRQn);

			__disable_irq();
	
			TIM_Cmd(pin.TIMx, ENABLE);

			__enable_irq();
			
			switch (pin.TIM_Channel){
				case (TIM_Channel_1):
					TIM_SetCompare1(pin.TIMx, 0);
					break;
				case (TIM_Channel_2):
					TIM_SetCompare2(pin.TIMx, 0);
					break;
				case (TIM_Channel_3):
					TIM_SetCompare3(pin.TIMx, 0);
					break;
				case (TIM_Channel_4):
					TIM_SetCompare4(pin.TIMx, 0);
					break;
			};
		}
	};
	namespace setFunc
    {

/*
Функция для установки значения ШИМа
Входные данные: pinStructure pin - пин, uint32_t value значение шима(0-1000)
Выходные данные:
*/
		static void setPWM(pinStructure pin,uint32_t value)
		{
			switch (pin.TIM_Channel)
            {
				case (TIM_Channel_1):
					TIM_SetCompare1(pin.TIMx, value);
					break;
				case (TIM_Channel_2):
					TIM_SetCompare2(pin.TIMx, value);
					break;
				case (TIM_Channel_3):
					TIM_SetCompare3(pin.TIMx, value);
					break;
				case (TIM_Channel_4):
					TIM_SetCompare4(pin.TIMx, value);
					break;
			};
		}
	};
    namespace readFunc
    {
    };
};
