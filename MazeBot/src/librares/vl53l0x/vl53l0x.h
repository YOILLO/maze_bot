#pragma once 
#include "project_config.h"
#include "i2c.h"
#include "pins.h"
#include "i_task.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "softI2C.h"
#include "time_service.h"
#include "vl53l0x_api.h"
#define stdAdr 0x52

using namespace pins;
using namespace tasks;

class vl53l0x : public i2c, public ITask<uint32_t>{
	public:
        //vl53l0x(SoftI2C* softI2C,pinStructure xshutPin, uint8_t adr);
        uint16_t takeSens();
        void reInit();
        virtual void worker();
        virtual ~vl53l0x();
        static const uint8_t sensor_default_address = 0x52;

        vl53l0x(pinStructure xshutPin, pinStructure enPin, uint8_t address, SoftI2C* softI2c, uint8_t id);


        VL53L0X_Error init();

        //Result< uint16_t > getDistance();

    private:

        pinStructure m_enPin;
        pinStructure m_xshutPin;
        uint8_t m_address;
        uint16_t m_data;
        VL53L0X_Dev_t m_dev;
        uint32_t m_numOfErrors;
        
};
