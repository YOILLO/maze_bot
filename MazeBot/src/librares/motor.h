#pragma once 
#include "project_config.h"
#include "pins.h"
#include "i_task.h"
#include "stm32f4xx_exti.h"
//#include "i2c.h"
#include "analog_input.h"

using namespace pins;
using namespace	tasks;

class Motor : public ITask<uint32_t>
{
	public:
		Motor(pinStructure out1,
              pinStructure out2,
              uint8_t currentPin,
              Analog_input &analogSesnor,
              uint16_t currentLimitMax,
              uint16_t currentLimitMin,
              uint32_t accel,
              pinStructure encPin1,
              pinStructure encPin2,
              pinStructure posSens);

		void setPower(int16_t power);
        int32_t getEnc(bool makeZero = 0);
    
        void changeAccel(uint16_t newAccel);
        void returnOldAccel();
    
        void getPitch(int32_t pitch);
    
  		virtual  void worker(void);
        
        pinStructure m_posSens;
        
        bool flagState;
	private:
        Analog_input & m_analogSesnor;
    
        int32_t m_encValue;//переменная для хранения значений энкодера
        int32_t m_posValue;
        int32_t m_posRast;
        int32_t m_encValuePrev;
    
        int32_t m_pitch;
    
        uint32_t m_motorSaveTime;
    
        void m_rawPowerSet(int32_t power);
        int32_t m_calcControl();
        void m_speedChange();
    
        uint16_t m_adc;
        uint16_t m_adcPrev;
        uint16_t m_adcPrev2;
    
        uint32_t m_notWorkTime;
    
		double m_powerMot;
		int16_t m_powerAim;
		uint32_t m_accelMot;
        uint16_t m_startAccel;
        uint16_t m_currentLimMax;
        uint16_t m_currentLimMin;
    
		pinStructure m_out1Mot;
		pinStructure m_out2Mot;
		pinStructure m_encPin1;
        pinStructure m_encPin2;
        uint8_t m_currentPin;
        
        uint32_t m_timePrev;
        uint32_t m_posValue_prev;
        int32_t m_encPrev;
        int32_t m_errPrev;
        int32_t m_errSumm;
        
        uint32_t m_timePrev2;
};
