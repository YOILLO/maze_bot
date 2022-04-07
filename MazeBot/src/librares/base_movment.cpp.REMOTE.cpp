#include "base_movment.h"
#define encP 4
#define distP 10
#define speed 1200
#define rotateSpeed 1200
#define rast 810
#define angR 80
#define angL 80
#define angB 170
#define WallRast0 100
#define WallRast 220
#define levelingRast 25
#define levelingK 20
static const uint32_t distCount = 30;
static const uint32_t stopRast = 70;
static const uint32_t FrontSideWallRast = 300;
static const uint16_t addReturn = 100;

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
        return (right - left) * distP;
    if(m_rightCount < distCount && m_leftCount > distCount)
        return (WallRast0 - left) * distP;
    if(m_rightCount > distCount && m_leftCount < distCount)
        return (right - WallRast0) * distP;
    return 0;
}
/*
Среднее значение 4 энкодеров
Выходные данные: uint32_t - среднее значение 4 энкодеров 
*/
uint32_t Base_movement::m_getEnc()
{
    return (m_motorBoLe.getEnc() + m_motorBoRi.getEnc() +
            m_motorFrLe.getEnc() + m_motorFrRi.getEnc())/4;
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
        if(m_getEnc() > rast || m_FrontDist.takeSens() < stopRast)
        {
            fullStop();
            if(m_FrontDist.takeSens() > WallRast) return 0;
            if(m_FrontDist.takeSens() > levelingRast)
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
        uint16_t dist = m_FrontDist.takeSens() - 10;
        int16_t spd = (dist - levelingRast) * levelingK;
        if(dist < levelingRast || dist > WallRast)return 0;
        if(spd > 1000)spd = 1000;
        if(spd < -1000)spd = -1000;
        m_motorFrRi.setPower(spd);
        m_motorFrLe.setPower(spd);
        m_motorBoRi.setPower(spd);
        m_motorBoLe.setPower(spd);
    }
    if(m_stateMov == movement_state::LEVELING_BA)
    {
        uint16_t dist = m_FrontDist.takeSens() + 10;
        int16_t spd = (dist - levelingRast) * levelingK;
        if(dist > levelingRast || dist > WallRast) return 0;
        if(spd > 1000)spd = 1000;
        if(spd < -1000)spd = -1000;
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
        m_backEnc += addReturn;
    }
    m_motorFrRi.setPower(-500);
    m_motorFrLe.setPower(-500);
    m_motorBoRi.setPower(-500);
    m_motorBoLe.setPower(-500);
    if(m_getEnc() > m_backEnc)
    {
        fullStop();
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
    }
    if (m_stateRot == rotate_state::ROTATE)
    {
        if(side == 2)
        {
            int16_t errLR = m_calcEncErrLR();
            m_motorFrRi.setPower(rotateSpeed + errLR);
            m_motorFrLe.setPower(-rotateSpeed + errLR);
            m_motorBoRi.setPower(rotateSpeed + errLR);
            m_motorBoLe.setPower(-rotateSpeed + errLR);
            if(m_startYaw + angL >= 360)
            {
                volatile uint16_t yaw = m_Gyros.getYaw();
                if((m_Gyros.getYaw() < m_startYaw + angL - 360) 
                || (m_Gyros.getYaw() > m_startYaw - 150))return 1;
                else
                {
                    fullStop();
                    if(m_BotDist.takeSens() > WallRast) return 0;
                    if(m_BotDist.takeSens() < levelingRast)
                    {
                        m_stateRot = rotate_state::LEVELING_FR;
                    }
                    else
                    {
                        m_stateRot = rotate_state::LEVELING_BA;
                    }
                }
            }
            else
            {
                if(m_Gyros.getYaw() < m_startYaw + angL 
                && m_Gyros.getYaw() > m_startYaw - 30)return 1;
                else
                {
                    fullStop();
                    if(m_BotDist.takeSens() > WallRast) return 0;
                    if(m_BotDist.takeSens() < levelingRast)
                    {
                        m_stateRot = rotate_state::LEVELING_FR;
                    }
                    else
                    {
                        m_stateRot = rotate_state::LEVELING_BA;
                    }
                }                    
            }    
        }
        if(side == 1)
        {
            int16_t errLR = m_calcEncErrLR();
            m_motorFrRi.setPower(-rotateSpeed - errLR);
            m_motorFrLe.setPower(rotateSpeed - errLR);
            m_motorBoRi.setPower(-rotateSpeed - errLR);
            m_motorBoLe.setPower(rotateSpeed - errLR);
            if(m_startYaw - angR <0)
            {
                if((m_Gyros.getYaw() > m_startYaw - angR + 360) 
                || (m_Gyros.getYaw() < m_startYaw + 150))return 1;
                else
                {
                    fullStop();
                    if(m_BotDist.takeSens() > WallRast) return 0;
                    if(m_BotDist.takeSens() < levelingRast)
                    {
                        m_stateRot = rotate_state::LEVELING_FR;
                    }
                    else
                    {
                        m_stateRot = rotate_state::LEVELING_BA;
                    }
                }
            }
            else
            {
                if (m_Gyros.getYaw() > m_startYaw - angR 
                 && m_Gyros.getYaw() < m_startYaw + 30) return 1;
                else
                {
                    fullStop();
                    if(m_BotDist.takeSens() > WallRast) return 0;
                    if(m_BotDist.takeSens() < levelingRast)
                    {
                        m_stateRot = rotate_state::LEVELING_FR;
                    }
                    else
                    {
                        m_stateRot = rotate_state::LEVELING_BA;
                    }
                }
            }
        }
        if(side == 3)
        {
            int16_t errLR = m_calcEncErrLR();
            m_motorFrRi.setPower(rotateSpeed + errLR);
            m_motorFrLe.setPower(-rotateSpeed + errLR);
            m_motorBoRi.setPower(rotateSpeed + errLR);
            m_motorBoLe.setPower(-rotateSpeed + errLR);
            if(m_startYaw + angB > 360)
            {
                if((m_Gyros.getYaw() < m_startYaw + angB - 360) 
                || (m_Gyros.getYaw() > m_startYaw - 150))return 1;
                else
                {
                    fullStop();
                    if(m_BotDist.takeSens() > WallRast) return 0;
                    if(m_BotDist.takeSens() < levelingRast)
                    {
                        m_stateRot = rotate_state::LEVELING_FR;
                    }
                    else
                    {
                        m_stateRot = rotate_state::LEVELING_BA;
                    }
                }
            }
            else
            {
                if(m_Gyros.getYaw() < m_startYaw + angB 
                && m_Gyros.getYaw() > m_startYaw - 30)return 1;
                else
                {
                    fullStop();
                    if(m_BotDist.takeSens() > WallRast) return 0;
                    if(m_BotDist.takeSens() < levelingRast)
                    {
                        m_stateRot = rotate_state::LEVELING_FR;
                    }
                    else
                    {
                        m_stateRot = rotate_state::LEVELING_BA;
                    }
                }                    
            }
        }
    }
    if(m_stateRot == rotate_state::LEVELING_FR)
    {
        uint16_t dist = m_BotDist.takeSens() + 10;
        int16_t spd = (- dist + levelingRast) * levelingK;
        if(dist > levelingRast || abs(spd) > 2500
        || dist > WallRast)return 0;
        if(spd > 1000)spd = 1000;
        if(spd < -1000)spd = -1000;
        m_motorFrRi.setPower(spd);
        m_motorFrLe.setPower(spd);
        m_motorBoRi.setPower(spd);
        m_motorBoLe.setPower(spd);
    }
    if(m_stateRot == rotate_state::LEVELING_BA)
    {
        uint16_t dist = m_BotDist.takeSens() - 10;
        int16_t spd = (- dist + levelingRast) * levelingK;
        if(dist < levelingRast || abs(spd) > 2500 
        || dist > WallRast) return 0;
        if(spd > 1000)spd = 1000;
        if(spd < -1000)spd = -1000;
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
