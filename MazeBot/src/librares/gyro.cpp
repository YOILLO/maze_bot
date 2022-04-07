#include "gyro.h"
static const uint32_t calibrConst = 2000;
static const uint8_t angleRampLimit = 17;
//static const uint8_t accelRampLimit = 3;
/*
конструктор класса, инициализирует uart и обнуляет переменные
*/
Gyro::Gyro()
{
    uart::uart3.init(uart::RxPins::UART3_PB11,uart::TxPins::UART3_PB10, 115200);
    m_numByte=0;
}
/*
метод work для itask
*/
void Gyro::worker()
{
    while(uart::uart3.isNewByte())
    {
        uint8_t data = uart::uart3.getByte();
        if(m_numByte==0)
        {
            if((data & 0xFC) == 0x78)
            {
                dat[0]=data;
                m_numByte++;
            }
        }
        else
        {
            dat[m_numByte++] = data;
            if(m_numByte == 7)
            {
                uint16_t summ=0;
                for(int i=0;i<7;i++)
                {
                    summ+=dat[i];
                }
                summ/=7;
                //if(summ==dat[7])
                //{
                datProv[0]=dat[0];
                datProv[1]=dat[1];
                datProv[2]=dat[2];
                datProv[3]=dat[3];
                datProv[4]=dat[4];
                datProv[5]=dat[5];    
                datProv[6]=dat[6];
                datProv[7]=dat[7];
                //}
                m_numByte=0;
            }
        }
    }
}

/*
возващает угол рысканья
Выходные данные:uint16_t сам угол
*/
uint16_t Gyro::getYaw()
{
    return (datProv[1] << 8) + datProv[2];
}
/*
возващает угол тангаж
Выходные данные:uint16_t сам угол
*/
uint16_t Gyro::getRoll()
{
    return (datProv[3]<<8)+datProv[4];
}
/*
возващает ускорение по оси робота
Выходные данные:uint16_t сама ускорение
*/
uint16_t Gyro::getAccel()
{
    return (datProv[5]<<8)+datProv[6];
}
/*
возвращяет на горке ли робот
Выходные данные:
1 - на горке
0 - не на горке
*/
bool Gyro::isHill()
{
    //return 0;
    if(getRoll() > angleRampLimit /*&& abs(getAccel() - 100) >= accelRampLimit*/)return 1;
    else return 0;
}
/*
Проверяет стабильность показаний гироскопа
Выходные данные:
1 - Откалиброван
0 - Не откалиброван
*/
bool Gyro::isCalibrated()
{
    if(getYaw() != m_getYawPrev ||
       /*getAccel() != m_getAccelPrev ||*/
       getRoll() != m_getRollPrev) 
            m_calibrTime = time_service::getCurTime();
    m_getYawPrev = getYaw();
    m_getAccelPrev = getAccel();
    m_getRollPrev = getRoll();
    if(time_service::getCurTime() - m_calibrTime > calibrConst)return 1;
    else return 0;
}

bool Gyro::ConcL()
{
    return datProv[0] & 0x1;
}
bool Gyro::ConcR()
{
    return datProv[0] & 0x2;
}
/*
Пустой диструктор
*/
Gyro::~Gyro()
{
    
}
