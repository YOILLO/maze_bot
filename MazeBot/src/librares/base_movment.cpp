#include "base_movment.h"
#include "math.h"
#define encP 15
#define distP 25
#define distD 300
#define distI (double)0.3
static const uint32_t Iconstrain = 5000;
#define speed 1300
#define rotateSpeed 1500
#define rast 720
#define angR 90
#define angL 90
#define angB 180
#define WallRast0 115
#define WallRast 270
#define levelingRastFr 50
#define levelingRastBa 55
#define levelingK 70
//static const double cofAddSpeed = 0.5;
//static const uint32_t constrainAndAdd = 3000;
//static const double addCof = 2;
//static const uint32_t controlRotateTimer = 7000;
static const uint32_t distCount = 20;
static const uint32_t stopRast = 70;
static const uint32_t FrontSideWallRast = 270;
static const uint16_t addReturn = 100;
static const uint16_t backAcel = 10000;
static const int32_t backSpeed = -700;
static const uint16_t levelingCinstrain = 700;
static const uint16_t frontScetConst = 5;
//static const uint16_t addConstLeft = 5;
//static const uint16_t addConstRight = 5;
//static const uint16_t addConstBack = 5;
static const uint16_t PCofRotate = 38;
static const int16_t DCofRotate = -100;
static const double ICofRotate = 0.005;
static const uint32_t IConstrain = 100000000;
static const uint16_t ContrainValueRotate = 8;
static const uint16_t MaxExitRotateValue = 20;
//static const uint16_t MinInt = 2000;
static const double addICof = 1;

Base_movement::Base_movement(Motor& motorFrRi,
                      Motor& motorFrLe,
                      Motor& motorBoRi,
                      Motor& motorBoLe,
                      mlx906014& heatRight,
                      mlx906014& heatLeft,
                      vl53l0x& FrontDist,
                      vl53l0x& RightDist,
                      vl53l0x& LeftDist,
                      vl53l0x& BotDist,
                      Camera& Cam,
                      Servo& vidServ,
                      Gyro& Gyros):
                        m_motorFrRi(motorFrRi),
                        m_motorFrLe(motorFrLe),
                        m_motorBoRi(motorBoRi),
                        m_motorBoLe(motorBoLe),
                        m_heatRight(heatRight),
                        m_heatLeft(heatLeft),
                        m_FrontDist(FrontDist),
                        m_RightDist(RightDist),
                        m_LeftDist(LeftDist),
                        m_BotDist(BotDist),
                        m_Cam(Cam),
                        m_vidServ(vidServ),
                        m_Gyros(Gyros)
{
    fullStop();
}
/*
Ошибка левой с провой стороны для синхронизации моторов
Выходные данные: ошибка энкодеров между правой и левой стороной
*/
int16_t Base_movement::m_calcEncErrLR()
{
    return (((m_motorBoLe.getEnc() + m_motorFrLe.getEnc())/2)-
            ((m_motorBoRi.getEnc() + m_motorFrRi.getEnc())/2))*encP;
}
/*
Ошибка передней с задней стороной для синхронизации моторов
Выходные данные: ошибка энкодеров между задней и передней стороной
*/
int16_t Base_movement::m_calcEncErrFB()
{
    return -(((m_motorBoLe.getEnc() + m_motorBoRi.getEnc())/2)-
             ((m_motorFrLe.getEnc() + m_motorFrRi.getEnc())/2))*encP;
}
/*
Ошибка дальномеров для синхронизации моторов
Выходные данные: ошибка дальномеров
*/
int16_t Base_movement::m_calcDistErr()
{
    uint16_t right = m_RightDist.takeSens();
    uint16_t left = m_LeftDist.takeSens();
    if(right < FrontSideWallRast)m_rightCount++;
    else m_rightCount = 0;
    if(left < FrontSideWallRast)m_leftCount++;
    else m_leftCount = 0;
    if(m_rightCount > distCount && m_leftCount > distCount)
    {
        int16_t u;
        int16_t err = (right - left);
        u = err * distP / 2;
        u += (double)(err - errPrev) / (time_service::getCurTime() - timePrev) * distD / 2;
        u += errSumm * distI / 2;
        errSumm += err;
        errSumm = constrain(errSumm, -Iconstrain, Iconstrain);
        errPrev = err;
        timePrev = time_service::getCurTime();
        return u;
    }
    if(m_rightCount < distCount && m_leftCount > distCount)
    {
        int16_t u;
        int16_t err = (WallRast0 - left);
        u = err * distP;
        u += (double)(err - errPrev) / (time_service::getCurTime() - timePrev) * distD;
        u += errSumm * distI;
        errSumm += err;
        errSumm = constrain(errSumm, -Iconstrain, Iconstrain);
        errPrev = err;
        timePrev = time_service::getCurTime();
        return u;
    }
    if(m_rightCount > distCount && m_leftCount < distCount)
    {    
        int16_t u;
        int16_t err = (right - WallRast0);
        u = err * distP;
        u += (double)(err - errPrev) / (time_service::getCurTime() - timePrev) * distD;
        u += errSumm * distI;
        errSumm += err;
        errSumm = constrain(errSumm, -Iconstrain, Iconstrain);
        errPrev = err;
        timePrev = time_service::getCurTime();
        return u;
    }
    errSumm = 0;
    
    return 0;
}
/*
Среднее значение 4 энкодеров
Выходные данные: uint32_t - среднее значение 4 энкодеров 
*/
uint32_t Base_movement::m_getEnc()
{
    return min(min(m_motorBoLe.getEnc(), m_motorBoRi.getEnc()),
        min(m_motorFrLe.getEnc(), m_motorFrRi.getEnc()));
    /*return (m_motorBoLe.getEnc() + m_motorBoRi.getEnc() +
            m_motorFrLe.getEnc() + m_motorFrRi.getEnc())/4;*/
}
/*
Функция езды вперед:
Входные данные: bool loop: 1 - первый цикл 0 - не первый цикл
Выходные данные: bool: 0 - конец проезда 1 - продолжай ехать
*/
bool Base_movement::movement(bool loop)
{
    if(loop)
    {
        m_motorBoLe.getEnc(1);
        m_motorBoRi.getEnc(1);
        m_motorFrLe.getEnc(1);
        m_motorFrRi.getEnc(1);
        //m_rightCount = 0;
        //m_leftCount = 0;
        m_stateMov = movement_state::MOVING;
    }
    int16_t errFB = m_calcEncErrFB();
    int16_t errLR = m_calcEncErrLR();
    int16_t errDist = - m_calcDistErr();
    if(m_stateMov == movement_state::MOVING)
    {
        m_motorFrRi.setPower(speed - errFB + errLR + errDist);
        m_motorFrLe.setPower(speed - errFB - errLR - errDist);
        m_motorBoRi.setPower(speed + errFB + errLR + errDist);
        m_motorBoLe.setPower(speed + errFB - errLR - errDist);
        if(m_FrontDist.takeSens() < stopRast) m_frontScet++;
        else m_frontScet = 0;
        if(m_getEnc() > rast /
            cos(float(m_Gyros.getRoll() / (180 / 3.1415))) || m_frontScet > frontScetConst)
        {
            //fullStop();
            if(m_FrontDist.takeSens() > WallRast) return 0;
            if(m_FrontDist.takeSens() > levelingRastFr)
            {
                m_stateMov = movement_state::LEVELING_FR;
            }
            else
            {
                m_stateMov = movement_state::LEVELING_BA;
            }
        }
    }
    if(m_stateMov == movement_state::LEVELING_FR)
    {
        uint16_t dist = m_FrontDist.takeSens() - 5;
        int16_t spd = (dist - levelingRastFr) * levelingK;
        if(dist < levelingRastFr || dist > WallRast)return 0;
        if(spd > levelingCinstrain)spd = levelingCinstrain;
        if(spd < -levelingCinstrain)spd = -levelingCinstrain;
        m_motorFrRi.setPower(spd);
        m_motorFrLe.setPower(spd);
        m_motorBoRi.setPower(spd);
        m_motorBoLe.setPower(spd);
    }
    if(m_stateMov == movement_state::LEVELING_BA)
    {
        uint16_t dist = m_FrontDist.takeSens() + 5;
        int16_t spd = (dist - levelingRastFr) * levelingK;
        if(dist > levelingRastFr || dist > WallRast) return 0;
        if(spd > levelingCinstrain)spd = levelingCinstrain;
        if(spd < -levelingCinstrain)spd = -levelingCinstrain;
        m_motorFrRi.setPower(spd);
        m_motorFrLe.setPower(spd);
        m_motorBoRi.setPower(spd);
        m_motorBoLe.setPower(spd);
    }
    return 1;
}
/*
Функция отъезда назад после проезда (черная клетка):
Входные данные: bool loop: 1 - первый цикл 0 - не первый цикл
Выходные данные: bool: 0 - конец проезда 1 - продолжай ехать
*/
bool Base_movement::goBack(bool loop)
{
    if(loop)
    {
        m_backEnc = m_getEnc();
        m_motorBoLe.getEnc(1);
        m_motorBoRi.getEnc(1);
        m_motorFrLe.getEnc(1);
        m_motorFrRi.getEnc(1); 
        m_motorBoLe.changeAccel(backAcel);
        m_motorBoRi.changeAccel(backAcel);
        m_motorFrLe.changeAccel(backAcel);
        m_motorFrRi.changeAccel(backAcel);
        m_backEnc += addReturn;
    }
    m_motorFrRi.setPower(backSpeed);
    m_motorFrLe.setPower(backSpeed);
    m_motorBoRi.setPower(backSpeed);
    m_motorBoLe.setPower(backSpeed);
    if(m_getEnc() > m_backEnc)
    {
        fullStop();
        m_motorBoLe.returnOldAccel();
        m_motorBoRi.returnOldAccel();
        m_motorFrLe.returnOldAccel();
        m_motorFrRi.returnOldAccel();
        return 0;
    }
    else return 1;
}
/*
Функция поворота:
Входные данные:uint8_t side: направление поворота: 1 - направо, 2 - налево, 3 - разворот
bool loop: 1 - превый вызов
Выходные данные: bool: 0 - закнчивай, 1 - не заканчивай
*/
bool Base_movement::rotate(uint8_t side, bool loop)
{
    if(loop)
    {
        m_motorBoLe.getEnc(1);
        m_motorBoRi.getEnc(1);
        m_motorFrLe.getEnc(1);
        m_motorFrRi.getEnc(1);
        m_rightCount = 0;
        m_leftCount = 0;
        m_startYaw = m_Gyros.getYaw();
        m_stateRot = rotate_state::ROTATE;
        m_startRotateTime = time_service::getCurTime();
        if(side == 2)
        {
            m_yawAim = m_startYaw + angL;
            m_oldErrOr = angL;
        }
        if(side == 1)
        {
            m_yawAim = m_startYaw - angR;
            m_oldErrOr = - angR;
        }
        if(side == 3)
        {
            m_yawAim = m_startYaw - angB;
            m_oldErrOr = - angB;
        }
        m_errOrSumm = 0;
        m_timeStartRotate = time_service::getCurTime();
    }
    if (m_stateRot == rotate_state::ROTATE)
    {
        int16_t yaw = m_Gyros.getYaw();
        int16_t errOr;
        if(yaw - m_yawAim < 180 && yaw - m_yawAim > -180)
        {
            errOr =  - (yaw - m_yawAim);
        }
        else
        {
            if(yaw < m_yawAim)
            {
                errOr =  - (yaw - m_yawAim + 360);
            }
            else
            {
                
                errOr =  - (yaw - m_yawAim - 360);
            }
        }
        if(abs(errOr) < ContrainValueRotate)m_rotateCount++;
        else m_rotateCount = 0;
        m_errOrSumm += errOr * addICof * (time_service::getCurTime() - m_timeStartRotate);
        m_errOrSumm = constrain(m_errOrSumm, -IConstrain, IConstrain);
        int16_t DSost = errOr - m_oldErrOr;
        m_oldErrOr = errOr;
        errOr *= PCofRotate;
        errOr = constrain(errOr, -rotateSpeed, rotateSpeed);
        int16_t IntS;
        //if(m_errOrSumm < MinInt && m_errOrSumm > 0) IntS = MinInt;
        //else if(m_errOrSumm > - MinInt && m_errOrSumm < 0) IntS = -MinInt;
        IntS = m_errOrSumm;
        int16_t Contr = errOr + IntS * ICofRotate + DSost * DCofRotate;
        int16_t errEnc = 0/*-m_calcEncErrLR()*/;
        m_motorFrRi.setPower((Contr + errEnc));
        m_motorFrLe.setPower(-(Contr + errEnc));
        m_motorBoRi.setPower((Contr + errEnc));
        m_motorBoLe.setPower(-(Contr + errEnc));
        if(m_rotateCount > MaxExitRotateValue)
        {
            fullStop();
            if(m_BotDist.takeSens() > WallRast) return 0;
            if(m_BotDist.takeSens() < levelingRastBa)
            {
                m_stateRot = rotate_state::LEVELING_FR;
            }
            else
            {
                m_stateRot = rotate_state::LEVELING_BA;
            }
        }
    }
    if(m_stateRot == rotate_state::LEVELING_FR)
    {
        uint16_t dist = m_BotDist.takeSens() + 5;
        int16_t spd = (- dist + levelingRastBa) * levelingK;
        if(dist > levelingRastBa  /*|| abs(spd) > 2500*/
        || dist > WallRast)return 0;
        if(spd > levelingCinstrain)spd = levelingCinstrain;
        if(spd < -levelingCinstrain)spd = -levelingCinstrain;
        m_motorFrRi.setPower(spd);
        m_motorFrLe.setPower(spd);
        m_motorBoRi.setPower(spd);
        m_motorBoLe.setPower(spd);
    }
    if(m_stateRot == rotate_state::LEVELING_BA)
    {
        uint16_t dist = m_BotDist.takeSens() - 5;
        int16_t spd = (- dist + levelingRastBa) * levelingK;
        if(dist < levelingRastBa /*|| abs(spd) > 2500 */
        || dist > WallRast) return 0;
        if(spd > levelingCinstrain)spd = levelingCinstrain;
        if(spd < -levelingCinstrain)spd = -levelingCinstrain;
        m_motorFrRi.setPower(spd);
        m_motorFrLe.setPower(spd);
        m_motorBoRi.setPower(spd);
        m_motorBoLe.setPower(spd);
    }
    return 1;
}
/*
Полная остановка 4-х моторов
*/
void Base_movement::fullStop()
{
    m_motorFrRi.setPower(0);
    m_motorFrLe.setPower(0);
    m_motorBoRi.setPower(0);
    m_motorBoLe.setPower(0);
}
