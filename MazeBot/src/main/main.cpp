#include "project_config.h"
#include "stm32f4xx.h"
#include "pinsStr.h"
#include "pins.h"
#include "motor.h"
#include "gyro.h"
#include "softI2C.h"
#include "display.h"
#include "analog_input.h"
#include "math.h"

//#define useDebug

static const uint32_t motorAccel = 90000;
static const uint32_t motorCurrentLimitMax = 2800;
static const uint32_t motorCurrentLimitMin = 1300;

int main()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    time_service::init();
    time_service::startTime();
    

    pins::initFunc::pinInit(FrRiMotPins::en);
    GPIO_SetPin(FrRiMotPins::en);
    pins::initFunc::pinInit(FrLeMotPins::en);
    GPIO_SetPin(FrLeMotPins::en);
    pins::initFunc::pinInit(BoRiMotPins::en);
    GPIO_SetPin(BoRiMotPins::en);
    pins::initFunc::pinInit(BoLeMotPins::en);
    GPIO_SetPin(BoLeMotPins::en);
    
    //Инверсия моторов
    pins::initFunc::pinInit(BoLeMotPins::inv);
    GPIO_SetPin(BoLeMotPins::inv);
    
    pins::initFunc::pinInit(otherPins::victimLed);
    GPIO_ResetPin(otherPins::victimLed);
    pins::initFunc::pinInit(otherPins::button1);
    pins::initFunc::pinInit(otherPins::button2);

    static Display disp;
    static OLED_GFX work_disp = OLED_GFX();
    work_disp.Device_Init();
    work_disp.Set_Color(RED);
    work_disp.Clear_Screen();
    work_disp.print_String(10, 0, "MK", FONT_8X16);
    work_disp.print_String(0, 20, "Periph init", FONT_5X8);
    
    pins::initFunc::pinInit(BoLeMotPins::current);
    pins::initFunc::pinInit(BoRiMotPins::current);
    pins::initFunc::pinInit(FrLeMotPins::current);
    pins::initFunc::pinInit(FrRiMotPins::current);
    pins::initFunc::pinInit(otherPins::lightSesor);
    pins::initFunc::pinInit(otherPins::SHARPPin);
    Analog_input analog_sensors;
    
    //инициализация гироскоопа
    static Gyro gyro;
    work_disp.print_String(0, 50, "Gyro init complete", FONT_5X8);
    
    //Моторы
    static Motor motorFrontLeft(BoLeMotPins::out1,BoLeMotPins::out2,
            analog_sensors.currentFrontLeft, analog_sensors,
            motorCurrentLimitMax, motorCurrentLimitMin, motorAccel,
            FrLeMotPins::encPin1,FrLeMotPins::encPin2,
            FrLeMotPins::posSens);
    static Motor motorFrontRight(BoRiMotPins::out1,BoRiMotPins::out2,
            analog_sensors.currentFrontRight, analog_sensors,
            motorCurrentLimitMax, motorCurrentLimitMin, motorAccel,
            FrRiMotPins::encPin1,FrRiMotPins::encPin2,
            FrRiMotPins::posSens);
    static Motor motorBackLeft(FrLeMotPins::out1,FrLeMotPins::out2,
            analog_sensors.currentBackLeft, analog_sensors,
            motorCurrentLimitMax, motorCurrentLimitMin, motorAccel,
            BoLeMotPins::encPin1,BoLeMotPins::encPin2,
            BoLeMotPins::posSens);
    static Motor motorBackRight(FrRiMotPins::out1,FrRiMotPins::out2,
            analog_sensors.currentBackRight, analog_sensors,
            motorCurrentLimitMax, motorCurrentLimitMin, motorAccel,
            BoRiMotPins::encPin1,BoRiMotPins::encPin2,
            BoRiMotPins::posSens);
    work_disp.print_String(0, 80, "Motor init complete", FONT_5X8);
    
    work_disp.print_String(0, 90, "Init complete", FONT_5X8);
    
    __enable_irq();
    work_disp.Clear_Screen();
    work_disp.print_String(0, 0, "press 1", FONT_5X8);

    uart::uart6.init(uart::RxPins::UART6_PG9,uart::TxPins::UART6_PG14, 115200);
       
    volatile static uint8_t readbyte;
    while(!GPIO_Read(otherPins::button2))
    {    
    }
    work_disp.print_String(0, 10, "start", FONT_5X8);
    while(1)
    {
        char str[15];
        if(uart::uart6.isNewByte())
        {
            readbyte = uart::uart6.getByte();
            sprintf(str, "distFr = %i   ", readbyte);
            work_disp.print_String(0, 20, str, FONT_5X8);            
        }
    }
}
