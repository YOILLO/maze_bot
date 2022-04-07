#pragma once
 
#include "project_config.h"

 
class Analog_input
{
    public:
        Analog_input();
        uint16_t getAdc(uint8_t adcNum);
        static const uint8_t lightSensor = 4;
        static const uint8_t currentFrontRight = 0;
        static const uint8_t currentFrontLeft = 7;
        static const uint8_t currentBackRight = 6;
        static const uint8_t currentBackLeft = 5;
    private:
        uint16_t m_adcData[16];
        uint16_t m_adcData1[16];
};
