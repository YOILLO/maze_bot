#include "motor.h"
#include "math.h"
#define P_MOT_K 0.05 //пропорциональный кофициент
#define D_MOT_K 0.7 //деферинциальный кофициент
#define I_MOT_K 0.009 //интегральный кофициент
#define speedToEnc 0.7 //сколько тиков один тик енкодера
static const uint32_t saveTimeLimit = 1000;
static const uint32_t motorProtectionTimer = 1000;
using namespace pins;
using namespace std; 
//using namespace motor;

/*
	Конструктор класса для управления одним мотором:
	Входные данные:
	pinStructure out1 - пин выхода №1
	pinStructure out2 - пин выхода №2
	pinStructure currentPin - пин контроля тока для защиты моторов
	uint8_t currentLimit - моксимальное значение тока
	pinStructure encPin - пин энкодера
*/
Motor::Motor(pinStructure out1,
             pinStructure out2,
             uint8_t currentPin,
             Analog_input &analogSesnor,
             uint16_t currentLimitMax,
             uint16_t currentLimitMin,
             uint32_t accel,
             pinStructure encPin1,
             pinStructure encPin2,
             pinStructure posSens):
             m_analogSesnor(analogSesnor)
{
    m_currentPin = currentPin;
    m_posValue_prev = 0;
    m_timePrev2=0;
    m_encPrev = 0;
    m_errPrev = 0;
    m_errSumm = 0;
    m_timePrev = 0;
    m_encValue = 0;
    m_posValue = 0;
    initFunc::initPwm(out1);
	m_out1Mot=out1;
	initFunc::initPwm(out2);
    m_out2Mot = out2;
    //initFunc::initPwm(posSens); 
    
    m_out2Mot=out2;
	m_currentLimMax=currentLimitMax;
    m_currentLimMin=currentLimitMin;
	m_accelMot=accel;
    m_startAccel = accel;
	initFunc::pinInit(encPin1);
    initFunc::pinInit(encPin2);
    initFunc::timerClock(encPin1.TIMx);
    GPIO_PinAFConfig(encPin1.GPIOx, encPin1.pinSource, encPin1.AFMode);
    //GPIO_PinAFConfig(encPin2.GPIOx, encPin2.pinSource, encPin2.AFMode);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    TIM_TimeBaseStructure.TIM_Period = (5000*2) - 1;
    //TIM_TimeBaseStructure.TIM_ClockDyvysyom = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_Prescaler = 1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(encPin1.TIMx, &TIM_TimeBaseStructure);
    
    TIM_EncoderInterfaceConfig(encPin1.TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
    
    encPin1.TIMx->CNT = 0;
    TIM_Cmd(encPin1.TIMx, ENABLE);
    m_encPin1 = encPin1;
    m_encPin2 = encPin2;
    m_pitch = 0;
    /*TIM_TimeBaseInitTypeDef timer;
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 10;
    timer.TIM_Period = 256;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(encPin1.TIMx, &timer);
    TIM_EncoderInterfaceConfig(encPin1.TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    //TIM_Cmd(encPin1.TIMx, ENABLE);
    __disable_irq();
	
    TIM_Cmd(encPin1.TIMx, ENABLE);

    __enable_irq();
	m_encPin1=encPin1;
    m_encPin2=encPin2;*/
}

/*
	Функция для подачи на мотор скорости
	Входные данные: int16_t power - мощность со знаком (<0-назад, >0-вперед)
	Выходные данные:
*/
void Motor::setPower(int16_t power)
{
		m_powerAim=power;
}

/*
Функция, подающая определенную мощность на моторы
Входные данные:int32_t power - мощность, которую нужно подать на моторы от -1000 до 1000
*/
void Motor::m_rawPowerSet(int32_t power){
    if(power < -2500)power = -2500;
    if(power > 2500)power = 2500;
    if(power>0)
    {
		setFunc::setPWM(m_out1Mot,3200 - abs(power));
		setFunc::setPWM(m_out2Mot,3200);
	}
	else if(power<0)
    {
		setFunc::setPWM(m_out2Mot,3200 - abs(power));
		setFunc::setPWM(m_out1Mot,3200);
	}
	else 
    {
		setFunc::setPWM(m_out1Mot,3200);
		setFunc::setPWM(m_out2Mot,3200);
    }
}

/*
функция, считющая ошибку, для поддеожания определенной скорости:
Выходные данные: int32_t - нейденная ошибка
*/
int32_t Motor::m_calcControl(){
    uint32_t time = time_service::getCurTime();
	uint32_t delTime=time-m_timePrev;
    int16_t err=0;
	err=(abs(m_encValue-m_encPrev)/delTime * 1000) - m_powerMot * speedToEnc;
	m_errSumm+=err;
    m_errSumm = constrain(m_errSumm,-2500,2500);
    int32_t u = err*P_MOT_K + (abs(err-m_errPrev)/delTime) * D_MOT_K + m_errSumm * I_MOT_K;
	m_errPrev=err;
    m_encPrev=m_encValue;
    m_timePrev=time;
    return -u;
}
/*
Функция для плавного наростания и уменьшения скорости
*/
void Motor::m_speedChange(){
    uint32_t delTime=0;
    uint32_t time = time_service::getCurTime();
	delTime=time-m_timePrev2;
    if(delTime == 0) return;
	if(m_powerMot < m_powerAim)
    {
		
        m_powerMot+=double(m_accelMot)*double(delTime)/1000.0;
        if(m_powerMot > m_powerAim) m_powerMot = m_powerAim;	
    }
	if(m_powerMot > m_powerAim)
    {
		m_powerMot-=double(m_accelMot)*double(delTime)/1000.0;
        if(m_powerMot < m_powerAim) m_powerMot = m_powerAim;
	}
    //m_powerMot = m_powerAim;
    m_timePrev2=time;
}
/*
	Функция задачи, которя будет все время выполняться, держать скорость, отвечать за плавность:
	Входные данные:
	Выходные данные:
*/
void Motor::worker(void)
{
    m_speedChange();
    int32_t u = m_calcControl();
    m_adc = m_analogSesnor.getAdc(m_currentPin);
    volatile uint16_t adcMid = (m_adc + m_adcPrev + m_adcPrev2) / 3;
    if (adcMid < m_currentLimMax 
     && adcMid > m_currentLimMin)
        m_motorSaveTime = time_service::getCurTime();
    if(time_service::getCurTime() - m_motorSaveTime > saveTimeLimit)
    {
        m_notWorkTime = time_service::getCurTime();
    }
    if(time_service::getCurTime() - m_notWorkTime < motorProtectionTimer)
    {
        m_powerMot = 0;
    }
    if(m_powerMot == 0) m_rawPowerSet(0);
    else m_rawPowerSet(m_powerMot /*+ u*/);
    m_encValue = m_encPin1.TIMx->CNT/* * cos((double)m_pitch / (180.0 / 3.1415))*/;
    m_encValuePrev = m_encPin1.TIMx->CNT;
    m_posValue_prev = m_posValue;
    if(abs(int(m_posValue_prev - m_posValue))>1000)m_posRast += 3200 + (m_posValue_prev - m_posValue);
    else m_posRast += m_posValue_prev - m_posValue;
    m_adcPrev = m_adc;
    m_adcPrev2 = m_adcPrev;
}
/*
Функция, возвращающая значение энкодеров
Входные данные: 1 если нужно обнулить енкодер, 0 если не нужно
Выходные данные: значения энкодеров
*/
int32_t Motor::getEnc(bool makeZero){
    if(makeZero){
        //m_posRast = 0;
        m_encValue = 0;
        m_encPin1.TIMx->CNT = 0;
        return 0;
    }
    return m_encValue;
    //return m_posValue;
}
/*
функция, меняющая ускорение
Вхлдные данные: uint16_t newAccel - новое ускорение
*/
void Motor::changeAccel(uint16_t newAccel)
{
    m_accelMot = newAccel;
}
/*
Функция вернет указанное в конструкторе значение ускорения
*/
void Motor::returnOldAccel()
{
    m_accelMot = m_startAccel;
}

void Motor::getPitch(int32_t pitch)
{
    m_pitch = pitch;
}
