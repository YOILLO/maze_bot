#include "servo.h"
#include "pins.h"
using namespace pins;

#define SERVO_180 2100.0
#define SERVO_0 400.0
#define STEP (SERVO_180 - SERVO_0) / 180.0

/*
Конструктор класса servo:
Входные данные: pinStructure pin - пин, на котором висит серва
Выходные данные:
*/
Servo::Servo(pinStructure pin)
{
	initFunc::initPwm(pin);
	//uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
    setFunc::setPWM(pin,SERVO_0 + (STEP) * 90);
	m_ServPin=pin;
}

/*
Функция для установки угла:
Входные данные:	uint16_t angle - угол от 0 - 180, на который встает серва
Выходные данные:
*/
void Servo::setAngle(uint16_t angle)
{
	//uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
    setFunc::setPWM(m_ServPin,int(SERVO_0 + double(STEP) * angle));
}
