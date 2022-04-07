#pragma once 
#include "project_config.h"
#include "i2c.h"
#include "i_task.h"
using namespace tasks;
class mlx906014:public i2c, public ITask<uint32_t>{
    public:
        mlx906014(SoftI2C* softI2C, uint8_t adr);
        virtual ~mlx906014();
        virtual void worker();
        double takeTemp();
    private:
        uint32_t m_god;
        uint32_t m_bad;
        uint16_t m_strAdr;
        uint32_t m_rowData;
};
