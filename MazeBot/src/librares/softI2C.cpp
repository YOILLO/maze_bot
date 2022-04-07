#include "softI2C.h"

using namespace pins;
/*
Инциализация шины softI2C:
Входные данные:
        pinStructure sclPin - пин scl(клоки)
        pinStructure sdaPin - пин sda(данные)
*/
SoftI2C::SoftI2C(pinStructure sclPin, pinStructure sdaPin)
{
    m_sclPin=sclPin;
    m_sdaPin=sdaPin;
    initFunc::pinInit(m_sclPin);
    initFunc::pinInit(m_sdaPin);
    //sclHigh();
    //sdaHigh(); 
    //m_delay();
    //sclLow();
    //sdaLow();
    //m_delay();
    sclHigh();
    sdaHigh();
}
/*
Функция генерирует начало пакета
Входные данные:
Выходные данные:
        bool - true - все хорошо, false - какая-то ошибка
*/
bool SoftI2C::generateStart()
{
    //sdaLow();
    //sclLow();
    //m_delay();
    sdaHigh();
    sclHigh();
    m_delay();
    if ( !sdaRead() )
        return false;
    sdaLow();
    m_delay();
    if ( sdaRead() )
        return false;
    sclLow();
    m_delay();
    
    return true;
}
/*
Функция генерирует конец пакета
Входные данные:
Выходные данные:
*/
void SoftI2C::generateStop()
{
    sclLow();
    m_delay();
    sdaLow();
    m_delay();
    sclHigh();
    m_delay();
    sdaHigh();
    m_delay();
}
/*
Функция генерирует проверяет на анличие ACK
Входные данные:
Выходные данные:
        bool - true - ACK, false - NACK
*/
bool SoftI2C::waitACK()
{
    sclLow();
    m_delay();
    //sdaHigh();
    //m_delay();
    //m_delay();
    sclHigh();
    m_delay();

    if(sdaRead())
    {
       sclLow();
       m_delay();
       return false;
    }

    sclLow();
    m_delay();
    return true;
}
/*
Функция отсылает один байт
Входные данные:uint8_t byte - данные на отправку
Выходные данные:
*/
void SoftI2C::sendByte(uint8_t byte)
{
    uint8_t i = 8;

    while(i--)
    {
        sclLow();
        m_delay();
        if( byte & 0x80 )
        {
            sdaHigh();
        }
        else
        {
            sdaLow();
        }
        byte <<= 1;
        m_delay();
        sclHigh();
        m_delay();
    }
    sclLow();
    m_delay();
    //sdaHigh();
}
/*
Функция принимает один байт
Входные данные:
Выходные данные:uint8_t - полученные данные
*/
uint8_t SoftI2C::getByte()
{
    uint8_t i = 8;
    uint8_t byte = 0;
        
    sdaHigh();
    while ( i-- )
    {       
        byte <<= 1;
        sclLow();
        m_delay();
        sclHigh();
        m_delay();
        if( sdaRead() )
        {
            byte |= 0x01;
        }
    }
    sclLow();
    return byte;
}
/*
Функция чтения состояние ноги scl
*/
bool SoftI2C::sclRead()
{
    return GPIO_Read(m_sclPin);
    //return (m_sclPin.GPIOx->IDR & m_sclPin.pinNum);
    //return GPIO_ReadInputData(m_sclPin.GPIOx) & m_sclPin.pinNum;
}
/*
Функция чтения состояние ноги sda
*/
bool SoftI2C::sdaRead()
{
    return GPIO_Read(m_sdaPin);
    //return GPIO_ReadInputData(m_sdaPin.GPIOx) & m_sdaPin.pinNum;
}
/*
Функция установки ноги scl в высокое положение
*/
void SoftI2C::sclHigh()
{
    GPIO_SetPin(m_sclPin);
}
/*
Функция установки ноги sda в высокое положение
*/
void SoftI2C::sdaHigh()
{
    GPIO_SetPin(m_sdaPin);
}
/*
Функция установки ноги scl в низкое положение
*/
void SoftI2C::sclLow()
{
    GPIO_ResetPin(m_sclPin);
}
/*
Функция установки ноги sda в низкое положение
*/
void SoftI2C::sdaLow()
{
    GPIO_ResetPin(m_sdaPin);
}
/*
Функция еденичного ожидания во время работы i2c
*/
void SoftI2C::m_delay()
{
    volatile unsigned int i = 25;

    while ( i )
    {
        i--;
    }
}
/*
Функция генерирует ACK
*/
void SoftI2C::generateACK()
{
    sclLow();
    m_delay();
    sdaLow();
    m_delay();
    sclHigh();
    m_delay();
    sclLow();
    m_delay();
}
/*
Функция генерирует NACK
*/
void SoftI2C::generateNACK()
{
    sclLow();
    m_delay();
    sdaHigh();
    m_delay();
    sclHigh();
    m_delay();
    sclLow();
    m_delay();
}

/*void SoftI2C::deInit()
{
    m_sclPin.OType = GPIO_OType_PP;
    
}*/
