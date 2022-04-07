#pragma once 
#include "project_config.h"
#include "i_task.h"
using namespace tasks;

class Gyro: public ITask<uint32_t>{
	public:
        Gyro();
        virtual ~Gyro();
        virtual void worker();
        uint16_t getYaw();
        uint16_t getRoll();
        uint16_t getAccel();
        bool isHill();
        bool isCalibrated();
        bool ConcR();
        bool ConcL();
	private:
        uint8_t dat[8];
        uint8_t m_numByte;
        uint8_t datProv[8];
        uint16_t m_getRollPrev;
        uint16_t m_getYawPrev;
        uint16_t m_getAccelPrev;
        uint32_t m_calibrTime;
    
};
