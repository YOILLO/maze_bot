#include "i2c.h"
using namespace pins;

#define cycles 1000

/*
Функция для инициализации i2c:
Входные данные: uint16_t pin1 - SoftI2C* softI2C - инициализированная шина softI2C
								uint8_t adress - адрес i2c устройства
Выходные данные:
*/
void i2c::init_I2C(SoftI2C& softI2C, uint8_t adr)
{
    m_adr = adr;
    m_softI2C = &softI2C;
}

/*
Функция для смены адреса:
Входные двнные: uint8_t adr - новый адресс 
*/
void i2c::adrChange(uint8_t adr)
{
    m_adr=adr;
}

/*
Функция для записи 8-ми битного регистра:
Входные данные: uint8_t reg - адресс регистра, uint8_t value - значение для решистра
Выходные данные:
*/
void i2c::writeReg(uint8_t reg, uint8_t value)
{
    m_softI2C->generateStart();
    m_softI2C->sendByte((m_adr<<1)|1);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte(value);
    if(!m_softI2C->waitACK())return;
    m_softI2C->generateStop();
}

/*
Функция для записи 16-и битного регистра:
Входные данные: uint8_t reg - адресс регистра, uint16_t value - значение для решистра
Выходные данные:
*/
void i2c::writeReg16Bit(uint8_t reg, uint16_t value)
{
    m_softI2C->generateStart();
    m_softI2C->sendByte((m_adr<<1)|1);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte((value >> 8) & 0xFF);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte(value & 0xFF);
    if(!m_softI2C->waitACK())return;
    m_softI2C->generateStop();
}

/*
Функция для записи 32-х битного регистра:
Входные данные: uint8_t reg - адресс регистра, uint32_t value - значение для решистра
Выходные данные:
*/
void i2c::writeReg32Bit(uint8_t reg, uint32_t value)
{
    m_softI2C->generateStart();
    m_softI2C->sendByte((m_adr<<1)|1);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte((value >> 24) & 0xFF);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte((value >> 16) & 0xFF);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte((value >> 8) & 0xFF);
    if(!m_softI2C->waitACK())return;
    m_softI2C->sendByte(value & 0xFF);
    if(!m_softI2C->waitACK())return;
    m_softI2C->generateStop();
}

/*
Функция для чтения 8-ми битного регистра:
Входные данные: uint8_t reg - адресс регистра
Выходные данные: uint8_t значение регистра
*/
uint8_t i2c::readReg(uint8_t reg)
{
    m_softI2C->generateStart();
    m_softI2C->sendByte((m_adr<<1)&254);
    if(!m_softI2C->waitACK())return 0;
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return 0;
    m_softI2C->generateStart();
    //while(m_softI2C->waitACK());
    m_softI2C->sendByte((m_adr<<1)|1);
    if(!m_softI2C->waitACK())return 0;
    uint8_t dat = m_softI2C->getByte();
    m_softI2C->generateACK();
    volatile uint8_t pec;
    pec = m_softI2C->getByte();
    m_softI2C->generateStop();
    return dat;
}

/*
Функция для чтения 16-и битного регистра:
Входные данные: uint8_t reg - адресс регистра
Выходные данные: uint16_t значение регистра
*/
uint16_t i2c::readReg16Bit(uint8_t reg)
{
    m_softI2C->generateStart();
    m_softI2C->sendByte((m_adr<<1) & 254);
    if(!m_softI2C->waitACK())return 0;
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return 0;
    m_softI2C->generateStart();
    //while(m_softI2C->waitACK());
    m_softI2C->sendByte((m_adr<<1)|1);
    if(!m_softI2C->waitACK())return 0;
    uint16_t dat = m_softI2C->getByte();
    //if(dat == 255)return 0;
    m_softI2C->generateACK();
    dat |= m_softI2C->getByte() << 8;
    m_softI2C->generateACK();
    volatile uint8_t pec;
    pec = m_softI2C->getByte();
    m_softI2C->generateStop();
    return dat;
}
/*
Функция для чтения 24-х битного регистра:
Входные данные: uint8_t reg - адресс регистра
Выходные данные: uint32_t значение регистра
*/
uint32_t i2c::readReg24Bit(uint8_t reg)
{
    return 0;
}
/*
Функция для чтения 32-х битного регистра:
Входные данные: uint8_t reg - адресс регистра
Выходные данные: uint32_t значение регистра
*/
uint32_t i2c::readReg32Bit(uint8_t reg)
{
    m_softI2C->generateStart();
    m_softI2C->sendByte((m_adr<<1)&254);
    if(!m_softI2C->waitACK())return 0;
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return 0;
    m_softI2C->generateStart();
    //while(m_softI2C->waitACK());
    m_softI2C->sendByte((m_adr<<1)|1);
    if(!m_softI2C->waitACK())return 0;
    uint32_t dat = m_softI2C->getByte();
    m_softI2C->generateACK();
    dat+=m_softI2C->getByte()<<8;
    m_softI2C->generateACK();
    dat+=m_softI2C->getByte()<<8;
    m_softI2C->generateACK();
    dat+=m_softI2C->getByte()<<8;
    m_softI2C->generateACK();
    volatile uint8_t pec;
    pec = m_softI2C->getByte();
    m_softI2C->generateStop();
    return dat;
}
/*
Функция для записи count байтного регистра:
Входные данные: uint8_t reg - адресс регистра, uint8_t const * src - массив, uint8_t count - размер массива
Выходные данные:
*/
void i2c::writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
    m_softI2C->generateStart();

    m_softI2C->sendByte(m_adr);
    if(!m_softI2C->waitACK())return;

    // reg addr
    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return;

    while (count-- > 0)
    {
        m_softI2C->sendByte( *(src++) );
        if(!m_softI2C->waitACK())return;
    }

    m_softI2C->generateStop();
}
/*
Функция для чтения count байтного регистра:
Входные данные: uint8_t reg - адресс регистра, uint8_t * src - массив, uint8_t count - размер массива
Выходные данные: все уже в массиве
*/
void i2c::readMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{   
    m_softI2C->generateStart();

    m_softI2C->sendByte(m_adr);
    if(!m_softI2C->waitACK())return;

    m_softI2C->sendByte(reg);
    if(!m_softI2C->waitACK())return;

    m_softI2C->generateStart();
    
    m_softI2C->sendByte(m_adr | 1);
    if(!m_softI2C->waitACK())return;

    while (count-- > 0)
    {
      *(dst++) = m_softI2C->getByte();
      m_softI2C->generateACK();
    }


    m_softI2C->generateStop();
}
