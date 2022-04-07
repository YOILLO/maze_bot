#include "camera.h"

/*
Конструктор класса, инициализурет все пины
Входные данные: pinStructure pinL1 - первый пин с левой камеры
pinStructure pinL2 - второй пин с левой камеры
pinStructure pinR1 - первый пин с правой камеры
pinStructure pinR2 - второй пин м правой камеры
*/
Camera::Camera(pinStructure pinL1,pinStructure pinL2,pinStructure pinR1,pinStructure pinR2)
{
    initFunc::pinInit(pinL1);
    initFunc::pinInit(pinL2);
    initFunc::pinInit(pinR1);
    initFunc::pinInit(pinR2);
    m_pinL1=pinL1;
    m_pinL2=pinL2;
    m_pinR1=pinR1;
    m_pinR2=pinR2;
}

/*
Взятие значений с камер
Выходные данные: CameraResult показания камер:
Буква - направление выдачи
Цифра - количество наборов
*/
Camera::CameraResult Camera::takeSens()
{
    uint8_t datL1=GPIO_ReadInputDataBit(m_pinL1.GPIOx,m_pinL1.pinNum) & 0x01;
    uint8_t datL2=GPIO_ReadInputDataBit(m_pinL2.GPIOx,m_pinL2.pinNum) & 0x01;
    uint8_t datR1=GPIO_ReadInputDataBit(m_pinR1.GPIOx,m_pinR1.pinNum) & 0x01;
    uint8_t datR2=GPIO_ReadInputDataBit(m_pinR2.GPIOx,m_pinR2.pinNum) & 0x01;
    uint8_t bitResult = (datL1 << 3) + (datL2 << 2) + (datR1 << 1) + datR2;
    
    CameraResult result = CameraResult::NO_DATA;
    result.fromInt(bitResult);
    return result;
//    if(((datL1==0)&&(datL2==0))&&((datR1==0)&&(datR2==0)))return 0;
//    if((datL1==1)&&(datL2==1))return 1;
//    if((datL1==0)&&(datL2==1))return 2;
//    if((datL1==1)&&(datL2==0))return 3;
//    if((datR1==1)&&(datR2==1))return 1;
//    if((datR1==0)&&(datR2==1))return 4;
//    if((datR1==1)&&(datR2==0))return 6;
//    return 0;
}
