#pragma once
#include "project_config.h"
#include "pins.h"
#include "pinsStr.h"
namespace posSensor
{
    STRONG_ENUM(posSensorE, 
        FRONT_RIGHT = 0, 
        BACK_RIGHT = 1, 
        FRONT_LEFT = 2, 
        BACK_LEFT = 3);
    class Reader
    {
        public:
            static Reader & getIsnance(posSensorE sensorP)
            {
                static Reader Psensors[4];
                return Psensors[sensorP.toInt()];
            }
            void add()
            {
                m_add += 65536;
            }
            void period(int32_t time, bool sost)
            {
                m_sost = sost;
                if(!m_sost)
                {
                    m_period = time - m_timePrev + m_add;
                    m_timePrev = time;
                }
                m_add = 0;
            }
            uint32_t getVal()
            {
                return m_period;
            }
        private:
            Reader( void ):
                m_sost (0)
            {
                m_timePrev = 0;
                m_fullInit();
            }
            void m_fullInit();
            uint32_t m_period;
            uint32_t m_add;
            uint32_t m_timePrev;
            bool m_sost;
    };
}
