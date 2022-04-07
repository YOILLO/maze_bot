#include "analog_input.h"
/*
Конструктор класса
инициализирует ацп и пдп
*/
Analog_input::Analog_input()
{
     // инициализация всяческих ацп
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Stream0);
    //DMA_DeInit(DMA2_Stream4);
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)m_adcData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 8;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream0, ENABLE);
    
    /*DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)m_adcData1;
    DMA_InitStructure.DMA_BufferSize = 1;
    
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream4, ENABLE);*/

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_NbrOfConversion = 8;

    ADC_Init(ADC3, &ADC_InitStructure);
    
    /*ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);*/

    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    //ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    ADC_RegularChannelConfig( ADC3, ADC_Channel_4,   1, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_5,   2, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_6,   3, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_7,   4, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_8,   5, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_9,   6, ADC_SampleTime_56Cycles );
    //ADC_RegularChannelConfig( ADC3, ADC_Channel_10,  9, ADC_SampleTime_56Cycles );
    //ADC_RegularChannelConfig( ADC3, ADC_Channel_11,  8, ADC_SampleTime_56Cycles );
    //ADC_RegularChannelConfig( ADC3, ADC_Channel_12,  9, ADC_SampleTime_56Cycles );
    //ADC_RegularChannelConfig( ADC3, ADC_Channel_13, 10, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_14, 7, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig( ADC3, ADC_Channel_15, 8, ADC_SampleTime_56Cycles );
    
    //ADC_RegularChannelConfig( ADC1, ADC_Channel_10, 1, ADC_SampleTime_56Cycles );

    ADC_DMACmd(ADC3, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
    
    /*ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);*/

    ADC_SoftwareStartConv( ADC3 );
    //ADC_SoftwareStartConv( ADC1 );
}
/*
Функция берет значения из массива
Входные данные: uint8_t adcNum - номер элемента массива
Выходнае данные: значение
*/
uint16_t Analog_input::getAdc(uint8_t adcNum)
{
    return m_adcData[adcNum];
}
