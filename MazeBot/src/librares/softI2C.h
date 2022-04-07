#pragma once
#include "project_config.h"
#include "pins.h"
//using namespace pins;

class SoftI2C{
    public:
        SoftI2C(pins::pinStructure sclPin, pins::pinStructure sdaPin);
        bool generateStart();
        void generateStop();
        uint8_t getByte();
        void sendByte(uint8_t byte);
        void generateACK();
        void generateNACK();
        bool waitACK();
    private:
        void sclHigh();
        void sdaHigh();
        void sclLow();
        void sdaLow();
        bool sclRead();
        bool sdaRead();

        void m_delay();
    
        pins::pinStructure m_sclPin;
        pins::pinStructure m_sdaPin;
};
