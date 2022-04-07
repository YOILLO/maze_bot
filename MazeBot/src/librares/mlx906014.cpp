#include "mlx906014.h"
#define regObj 0x07
#define regAmb 0x06
/*
конструктор класса, инициализирует шину i2c
Входные данные:
        SoftI2C* softI2C - инициализированная шина softI2C
        uint8_t adr - адресс конкретного датчика
*/
mlx906014::mlx906014(SoftI2C *softI2C, uint8_t adr)
{
    m_strAdr=adr;
    i2c::init_I2C(*softI2C,adr);
    m_rowData=0;
    i2c::readReg16Bit(regObj);
    time_service::delay_ms(100);
    i2c::readReg16Bit(regAmb);

}
/*
метод worker для ITask
*/
void mlx906014::worker()
{
    uint16_t rowData = i2c::readReg16Bit(regObj);
    if(rowData == 0)
    {
        return;
    }
    m_rowData=rowData;
}
/*
Метод для получения температуры
Выходные данные:double температура в цельсиях
*/
double mlx906014::takeTemp()
{
    double retDat= double(m_rowData * 0.02) - 273.5;
    return retDat;
}
/*
диструктор чтоб ворнингов небыло
*/
mlx906014::~mlx906014()
{
}
