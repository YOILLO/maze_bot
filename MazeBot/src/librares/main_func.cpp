#include "main_func.h"
#define resetEncoders 1
#define absFront 0
#define absRight 1
#define absLeft 3
#define absBack 2
#define robFront 0
#define robRight 1
#define robLeft -1
#define robBack 2
#define endMovement -2
#define reachFinish -3
#define rotateRight 1
#define rotateLeft 2
#define turnBack 3
#warning временное решение

static const uint32_t pauseButtonTimer = 1900;
static const uint32_t greyCountConst = 10;
static const uint32_t contTimer = 12000;
static const uint32_t victimTime = 900;
static const uint32_t greyValue = 1500;
static const uint32_t checkPointValue = 170;
static const uint16_t heatContValue = 31;
static const uint16_t fullKitGiveTime = 5000;
static const uint16_t firstPhaseKitGiveTime = 2000;
static const uint16_t secondPhaseKitGiveTime = 3000;
static const uint16_t blinkRate = 800;
static const uint16_t rightServPosition = 0;
static const uint16_t midServPosition = 90;
static const uint16_t leftServPosition = 180;
static const uint16_t hillCountLimitLeave = 25;
static const uint16_t hillCountLimitFind = 15;
static const uint16_t wallRast = 200;
static const uint8_t rampLength = 3;
/*
Конструктор класса, проинимает все объекты классов, созданные в меине
*/
MainFunc::MainFunc(vl53l0x &distFr,
                   vl53l0x &distRi,
                   vl53l0x &distLe,
                   vl53l0x &distBo,
                   vl53l0x &distRiFr,
                   vl53l0x &distLeFr,
                   mlx906014 &heatRi,
                   mlx906014 &heatLe,
                   Gyro &gyro,
                   Servo &serv,
                   Camera &cam,
                   Motor &motorFrLe,
                   Motor &motorFrRi,
                   Motor &motorBoLe,
                   Motor &motorBoRi,
                   Base_movement &movement,
                   Map_operations &mapOp,
                   OLED_GFX &workDisp,
                   Analog_input &analogSenor):
                    m_distFr(distFr),
                    m_distRi(distRi),
                    m_distLe(distLe),
                    m_distBo(distBo),
                    m_distRiFr(distRiFr),
                    m_distLeFr(distLeFr),
                    m_heatRi(heatRi),
                    m_heatLe(heatLe),
                    m_gyro(gyro),
                    m_serv(serv),
                    m_cam(cam),
                    m_motorFrLe(motorFrLe),
                    m_motorFrRi(motorFrRi),
                    m_motorBoLe(motorBoLe),
                    m_motorBoRi(motorBoRi),
                    m_movement(movement),
                    m_mapOp(mapOp),
                    m_workDisp(workDisp),
                    m_analogSenor(analogSenor)
{
    //m_state = State::EMPTY;
    ServiceTimer = time_service::getCurTime() - victimTime;
    m_state = State::MAP;
    m_isFirstCall = 1;
    m_isMapMovement= 0;
    //m_isNewHill = 1;
}
/*
Постоянно выполняющееся функция:
Входные данные: uint16_t light - показания датчика освященности, так как брать его в функции почему - то не получаеться
*/
void MainFunc::loop(bool isMovement)
{
    m_motorFrRi.work(time_service::getCurTime());
    m_motorFrLe.work(time_service::getCurTime());
    m_motorBoRi.work(time_service::getCurTime());
    m_motorBoLe.work(time_service::getCurTime());
    m_distFr.work(time_service::getCurTime());
    m_distRi.work(time_service::getCurTime());
    m_distLe.work(time_service::getCurTime());
    m_distBo.work(time_service::getCurTime());
    m_distRiFr.work(time_service::getCurTime());
    m_distLeFr.work(time_service::getCurTime());
    m_heatRi.work(time_service::getCurTime());
    m_heatLe.work(time_service::getCurTime());
    m_gyro.work(time_service::getCurTime());
    
    if(isMovement)
    {
        //m_callbackArray[m_state];
        if(m_state == State::MAP)//Построение карты, происходит каждый раз перед проездом
        {
            m_mapOp.makeMap();
            m_state = State::CHOSE_SIDE;
        }
        if(m_state == State::CHOSE_SIDE)//Выбор направления по провилу правой руки, или обнаружение факта, что его не найти
        {
            m_ChDir = m_mapOp.dirRob();
            switch (m_ChDir)
            {
                case robFront:
                    m_state = State::MOVING_FR;
                    break;
                case robRight:
                    m_state = State::ROTATE_R;
                    break;
                case robLeft:
                    m_state = State::ROTATE_L;
                    break;
                default:
                    //m_state = State::ROTATE_B;
                    m_state = State::MAKE_ROAD;
                    break;
            }
            //m_isFirstCall = 1;
        }
        if(m_state == State::MOVING_FR)//Проезд на 1 клетку вперед
        {
            if(m_hillCount == 0)
            {
                if(m_movement.movement(m_isFirstCall))
                {
                    if(m_isFirstCall)
                    {
                        m_contTimer = time_service::getCurTime();
                    }
                    m_isFirstCall = 0;
                    if(time_service::getCurTime() - m_contTimer > contTimer)
                    {
                        m_isFirstMapRotate = 0;
                        m_isFirstCall = 1;
                        m_mapOp.chngeCor(m_ChDir);
                        if(m_isMapMovement)
                        {
                            m_state = State::ROAD_MOVEMENT;
                            m_mapOp.mapingVizit[WHfloor][WHx][WHy] = 0;
                            if(m_analogSenor.getAdc(m_analogSenor.lightSensor) < checkPointValue)
                            {
                                //GPIO_SetPin(otherPins::victimLed);
                                m_mapOp.addCheckPoint();
                            }
                            m_mapOp.mapingVizit[WHfloor][WHx][WHy] = 1;
                        }
                        else
                        {
                            if(m_analogSenor.getAdc(m_analogSenor.lightSensor) < checkPointValue)
                            {
                                //GPIO_SetPin(otherPins::victimLed);
                                m_mapOp.addCheckPoint();
                            }
                            m_state = State::MAP;
                        }
                    }
                }
                else
                {
                    m_isFirstMapRotate = 0;
                    m_isFirstCall = 1;
                    m_mapOp.chngeCor(m_ChDir);
                    if(m_isMapMovement)
                    {
                        m_state = State::ROAD_MOVEMENT;
                        m_mapOp.mapingVizit[WHfloor][WHx][WHy] = 0;
                        m_mapOp.mapVizit[WHfloor][WHx][WHy] = 0;
                        if(m_analogSenor.getAdc(m_analogSenor.lightSensor) < checkPointValue)
                        {
                            //GPIO_SetPin(otherPins::victimLed);
                            m_mapOp.addCheckPoint();
                        }
                        m_mapOp.mapingVizit[WHfloor][WHx][WHy] = 1;
                        m_mapOp.mapVizit[WHfloor][WHx][WHy] = 1;
                    }
                    else
                    {
                        if(m_analogSenor.getAdc(m_analogSenor.lightSensor) < checkPointValue)
                        {
                            //GPIO_SetPin(otherPins::victimLed);
                            m_mapOp.addCheckPoint();
                        }
                        m_state = State::MAP;
                    }
                }
                //m_state = State::EMPTY;
            }
        }
        if(m_state == State::ROTATE_R)//Поворот на право
        {
            if(m_movement.rotate(rotateRight, m_isFirstCall))
            {
                if(m_isFirstCall)
                {
                    m_contTimer = time_service::getCurTime();
                }
                m_isFirstCall = 0;
                if(time_service::getCurTime() - m_contTimer > contTimer)
                {
                    m_movement.fullStop();
                    //m_isFirstMapRotate = 0;
                    m_isFirstCall = 1;
                    m_state = State::MOVING_FR;
                }        
            }
            else 
            {
                m_movement.fullStop();
                m_isFirstCall = 1;
                //m_isFirstMapRotate = 0;
                m_state = State::MOVING_FR;
                //m_state = State::ROTATE_L;
            }
        }
        if(m_state == State::ROTATE_L)//Поворот на лево
        {
            if(m_movement.rotate(rotateLeft, m_isFirstCall))
            {
                if(m_isFirstCall)
                {
                    m_contTimer = time_service::getCurTime();
                }
                m_isFirstCall = 0;
                if(time_service::getCurTime() - m_contTimer > contTimer)
                {
                    m_movement.fullStop();
                    //m_isFirstMapRotate = 0;
                    m_isFirstCall = 1;
                    m_state = State::MOVING_FR;
                }
            }
            else 
            {
                m_movement.fullStop();
                m_isFirstCall = 1;
                //m_isFirstMapRotate = 0;
                m_state = State::MOVING_FR;
                //m_state = State::ROTATE_R;
            }
        }
        if(m_state == State::ROTATE_B)//Разворот
        {
            if(m_movement.rotate(turnBack, m_isFirstCall))
            {
                if(m_isFirstCall)
                {
                    m_contTimer = time_service::getCurTime();
                }
                m_isFirstCall = 0;
                if(time_service::getCurTime() - m_contTimer > contTimer)
                {
                    m_movement.fullStop();
                    //m_isFirstMapRotate = 0;
                    m_isFirstCall = 1;
                    m_state = State::MOVING_FR;
                }
            }
            else 
            {
                m_movement.fullStop();
                //m_isFirstMapRotate = 0;
                m_isFirstCall = 1;
                m_state = State::MOVING_FR;
            }
        }
        if(m_state == State::MAKE_ROAD)//Построение маршрута (по факту только BFS)
        {
            m_movement.fullStop();
            if(m_isFirstCall == 1)
            {
                m_mapOp.TaleWhitePoints++;
                do
                {
                    m_mapOp.TaleWhitePoints--;
                    WHx = m_mapOp.whitePoints[m_mapOp.TaleWhitePoints - 1].x;
                    WHy = m_mapOp.whitePoints[m_mapOp.TaleWhitePoints - 1].y;
                    WHfloor = m_mapOp.whitePoints[m_mapOp.TaleWhitePoints - 1].floor;
                }
                while(m_mapOp.mapVizit[WHfloor][WHx][WHy] == 1 &&
                    (WHx != sizeX / 2 || WHy != sizeY / 2 || WHfloor != 0) &&
                     m_mapOp.TaleWhitePoints > 0/* &&
                     m_mapOp.mapWall[m_mapOp.floor][WHx * 2][WHy * 2] == 1*/);
            }
            #warning mappingState to STRONG_ENUM
            int8_t mappingState = m_mapOp.makeRoad(m_isFirstCall,
            m_mapOp.robCorX[m_mapOp.floor],m_mapOp.robCorY[m_mapOp.floor], m_mapOp.floor,
            WHx, WHy, WHfloor,1);
            if(mappingState == -2)
            {
                if(time_service::getCurTime() - ServiceTimer > victimTime * 3) m_isFirstMapRotate = 1;
                m_isFirstCall = 0;
            }
            else if(mappingState == 0)
            { 
                m_isMapMovement = 1;
                m_state = State::ROAD_MOVEMENT;
                if(time_service::getCurTime() - ServiceTimer > victimTime * 3) m_isFirstMapRotate = 1;
                m_isFirstCall = 1;
            }
            else 
            {
                if(time_service::getCurTime() - ServiceTimer > victimTime * 3) m_isFirstMapRotate = 1;
                //UMBA_ASSERT_FAIL();
                
                m_state = State::END_STATE;
                m_isFirstCall = 1;
            }
        }
        if(m_state == State::ROAD_MOVEMENT)//Езда после BFS
        {
            m_ChDir = m_mapOp.makeRoad(0 ,m_mapOp.robCorX[m_mapOp.floor],m_mapOp.robCorY[m_mapOp.floor], m_mapOp.floor,
            WHx, WHy, WHfloor, 1);
            switch(m_ChDir)
            {
                case robFront:
                    m_state = State::MOVING_FR;
                    break;
                case robRight:
                    m_state = State::ROTATE_R;
                    break;
                case robLeft:
                    m_state = State::ROTATE_L;
                    break;
                case robBack:
                    m_state = State::ROTATE_B;
                    break;
                case reachFinish:
                    m_movement.fullStop();
                    m_state = State::END_STATE;
                    break;
                default:
                    m_mapOp.TaleWhitePoints--;
                    //UMBA_ASSERT_FAIL();
                    m_isMapMovement = 0;
                    m_state = State::MAP;
                    break;
            }
        }
        if(m_state == State::BACK_MOVEMENT)//Отезд назад после обнаружения черной клетки
        {
            if(m_movement.goBack(m_isFirstCall))
            {
                m_isFirstCall = 0;
            }
            else
            {
                m_isFirstCall = 1;
                m_state = State::MAKE_ROAD;
                m_mapOp.dir = m_mapOp.convRobToAbs(m_ChDir);
                m_mapOp.addBlackPiont();
            }
        }
        if(m_state == State::HILL_MOVEMENT)//Езда по горке
        {
            m_movement.movement(resetEncoders);//нужно обнулять енкодеры каждый раз, чтоб они не ушли далеко вперед
            if(!m_gyro.isHill())
            {
                m_hillCount++;
            }
            else
            {
                m_hillCount = 0;
            }
            GPIO_SetPin(otherPins::victimLed);
            if(m_hillCount > hillCountLimitLeave)
            {
                GPIO_ResetPin(otherPins::victimLed);
                if(m_isMapMovement)
                    m_state = State::ROAD_MOVEMENT;
                else
                    m_state = State::MAP;
                m_hillCount = 0;
                m_isFirstCall = 1;
                int8_t hill = m_mapOp.isHill(m_mapOp.robCorX[m_mapOp.floor],m_mapOp.robCorY[m_mapOp.floor], m_mapOp.floor);
                m_mapOp.floor = !m_mapOp.floor;//if (floor == 0)floor = 1;else floor = 0;
                if (!m_isMapMovement)
                {
                    switch(m_mapOp.hills[m_mapOp.floor][hill].dir)
                    {
                        case (absFront):
                            //m_mapOp.dir = 2;
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor] - rampLength;
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills++].y = m_mapOp.robCorY[m_mapOp.floor];
                            m_mapOp.robCorX[m_mapOp.floor] -= rampLength;
                            break;
                        case (absRight):
                            //m_mapOp.dir = 3;
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills++].y = m_mapOp.robCorY[m_mapOp.floor] - rampLength;
                            m_mapOp.robCorY[m_mapOp.floor] -= rampLength;
                            break;
                        case (absBack):
                            //m_mapOp.dir = 0;
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor] + rampLength;
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills++].y = m_mapOp.robCorY[m_mapOp.floor];
                            m_mapOp.robCorX[m_mapOp.floor] += rampLength;
                            break;
                        case (absLeft):
                            //m_mapOp.dir = 1;
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills++].y = m_mapOp.robCorY[m_mapOp.floor] + rampLength;
                            m_mapOp.robCorY[m_mapOp.floor] += rampLength;
                            break;
                    }
                }
                else
                {
                    //m_mapOp.dir = m_mapOp.convRobToAbs(m_ChDir);
                    //m_isMapMovement = 0;
                    //m_isNewHill = 1;
                    m_mapOp.robCorX[m_mapOp.floor] = m_mapOp.hills[m_mapOp.floor][hill].x;
                    m_mapOp.robCorY[m_mapOp.floor] = m_mapOp.hills[m_mapOp.floor][hill].y;
                    switch(m_mapOp.hills[m_mapOp.floor][hill].dir)
                    {
                        case (absFront):
                            m_mapOp.dir = 2;
                            break;
                        case (absRight):
                            m_mapOp.dir = 3;
                            break;
                        case (absBack):
                            m_mapOp.dir = 0;
                            break;
                        case (absLeft):
                            m_mapOp.dir = 1;
                            break;
                    }
                    //m_mapOp.chngeCor(m_mapOp.dir);
                }
            }
        }

        if(m_state == State::VICTIM_RIGHT)//Выдача набора жертве с права
        {
            m_isFirstMapRotate = 0;
            m_movement.fullStop();
            m_serv.setAngle(rightServPosition);
            if(time_service::getCurTime() - ServiceTimer2 > blinkRate)
            {
                if(m_pinState == PinState::DISABLE)
                {
                    GPIO_SetPin(otherPins::victimLed);
                    m_pinState = PinState::ENABLE;
                }
                else
                {
                    GPIO_ResetPin(otherPins::victimLed);
                    m_pinState = PinState::DISABLE;                
                }
                ServiceTimer2 = time_service::getCurTime();
            }
            if(time_service::getCurTime() - ServiceTimer > fullKitGiveTime)
            {
                GPIO_ResetPin(otherPins::victimLed);
                m_serv.setAngle(midServPosition);
                m_state = m_ResetState;
                ServiceTimer = time_service::getCurTime();
                m_contTimer = time_service::getCurTime();
            }
            char str[15];
            sprintf(str, "time = %i    ", time_service::getCurTime() - ServiceTimer);
            m_workDisp.print_String(0, 0, str, FONT_8X16);
        }
        if(m_state == State::VICTIM_LEFT)//Выдача набора жертве слева
        {
            m_isFirstMapRotate = 0;
            m_movement.fullStop();
            m_serv.setAngle(leftServPosition);
            if(time_service::getCurTime() - ServiceTimer2 > blinkRate)
            {
                if(m_pinState == PinState::DISABLE)
                {
                    GPIO_SetPin(otherPins::victimLed);
                    m_pinState = PinState::ENABLE;
                }
                else
                {
                    GPIO_ResetPin(otherPins::victimLed);
                    m_pinState = PinState::DISABLE;                
                }
                ServiceTimer2 = time_service::getCurTime();
            }
            if(time_service::getCurTime() - ServiceTimer > fullKitGiveTime)
            {
                GPIO_ResetPin(otherPins::victimLed);
                m_serv.setAngle(midServPosition);
                m_state = m_ResetState;
                ServiceTimer = time_service::getCurTime();
                m_contTimer = time_service::getCurTime();
            }
            char str[15];
            sprintf(str, "time = %i    ", time_service::getCurTime() - ServiceTimer);
            m_workDisp.print_String(0, 0, str, FONT_8X16);
        }
        if(m_state == State::SIGNAL_VICTIM)//Просто просигналить о жертве, без выдачи набора
        {
            m_isFirstMapRotate = 0;
            m_movement.fullStop();
            if(time_service::getCurTime() - ServiceTimer2 > blinkRate)
            {
                if(m_pinState == PinState::DISABLE)
                {
                    GPIO_SetPin(otherPins::victimLed);
                    m_pinState = PinState::ENABLE;
                }
                else
                {
                    GPIO_ResetPin(otherPins::victimLed);
                    m_pinState = PinState::DISABLE;                
                }
                ServiceTimer2 = time_service::getCurTime();
            }
            if(time_service::getCurTime() - ServiceTimer > fullKitGiveTime)
            {
                GPIO_ResetPin(otherPins::victimLed);
                m_state = m_ResetState;
                ServiceTimer = time_service::getCurTime();
                m_contTimer = time_service::getCurTime();
            }
            char str[15];
            sprintf(str, "time = %i    ", time_service::getCurTime() - ServiceTimer);
            m_workDisp.print_String(0, 0, str, FONT_8X16);
        }
        if(m_state == State::DOUBLE_VICTIM_RIGHT)//Выдача 2-x наборoB жертве с права
        {
            m_isFirstMapRotate = 0;
            m_movement.fullStop();
            if(time_service::getCurTime() - ServiceTimer2 > blinkRate)
            {
                if(m_pinState == PinState::DISABLE)
                {
                    GPIO_SetPin(otherPins::victimLed);
                    m_pinState = PinState::ENABLE;
                }
                else
                {
                    GPIO_ResetPin(otherPins::victimLed);
                    m_pinState = PinState::DISABLE;                
                }
                ServiceTimer2 = time_service::getCurTime();
            }
            if(time_service::getCurTime() - ServiceTimer < firstPhaseKitGiveTime)m_serv.setAngle(rightServPosition);
            
            if(time_service::getCurTime() - ServiceTimer > firstPhaseKitGiveTime
            && time_service::getCurTime() - ServiceTimer < secondPhaseKitGiveTime)m_serv.setAngle(midServPosition);
            
            if(time_service::getCurTime() - ServiceTimer > secondPhaseKitGiveTime 
            && time_service::getCurTime() - ServiceTimer < fullKitGiveTime)m_serv.setAngle(rightServPosition);
            
            if(time_service::getCurTime() - ServiceTimer > fullKitGiveTime)
            {
                GPIO_ResetPin(otherPins::victimLed);
                m_serv.setAngle(midServPosition);
                m_state = m_ResetState;
                ServiceTimer = time_service::getCurTime();
                m_contTimer = time_service::getCurTime();
            }
            char str[15];
            sprintf(str, "time = %i    ", time_service::getCurTime() - ServiceTimer);
            m_workDisp.print_String(0, 0, str, FONT_8X16);
        }
        #warning я не понял почему таааааааак много похожего кода
        if(m_state == State::DOUBLE_VICTIM_LEFT)//Выдача 2-x наборoB жертве слева
        {
            m_isFirstMapRotate = 0;
            m_movement.fullStop();
            if(time_service::getCurTime() - ServiceTimer2 > blinkRate)
            {
                if(m_pinState == PinState::DISABLE)
                {
                    GPIO_SetPin(otherPins::victimLed);
                    m_pinState = PinState::ENABLE;
                }
                else
                {
                    GPIO_ResetPin(otherPins::victimLed);
                    m_pinState = PinState::DISABLE;                
                }
                ServiceTimer2 = time_service::getCurTime();
            }
            if(time_service::getCurTime() - ServiceTimer < firstPhaseKitGiveTime)m_serv.setAngle(leftServPosition);
            
            if(time_service::getCurTime() - ServiceTimer > firstPhaseKitGiveTime &&
               time_service::getCurTime() - ServiceTimer < secondPhaseKitGiveTime)m_serv.setAngle(midServPosition);
            
            if(time_service::getCurTime() - ServiceTimer > secondPhaseKitGiveTime && 
               time_service::getCurTime() - ServiceTimer < fullKitGiveTime)m_serv.setAngle(leftServPosition);
            
            if(time_service::getCurTime() - ServiceTimer > fullKitGiveTime)
            {
                GPIO_ResetPin(otherPins::victimLed);
                m_serv.setAngle(midServPosition);
                m_state = m_ResetState;
                ServiceTimer = time_service::getCurTime();
                m_contTimer = time_service::getCurTime();
            }
            char str[15];
            sprintf(str, "time = %i    ", time_service::getCurTime() - ServiceTimer);
            m_workDisp.print_String(0, 0, str, FONT_8X16);
        }
        if(m_heatRi.takeTemp() >= heatContValue
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE    
            && m_distRi.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "heat right", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();
            m_ResetState = m_state;
            m_state = State::VICTIM_RIGHT;
            ServiceTimer = time_service::getCurTime();
        }
        if(m_heatLe.takeTemp() >= heatContValue
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distLe.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "heat left ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::VICTIM_LEFT;
            ServiceTimer = time_service::getCurTime();
        }
        if(m_cam.takeSens() == Camera::CameraResult::RESULT_L0
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distLe.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "H - left  ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::DOUBLE_VICTIM_LEFT;
            ServiceTimer = time_service::getCurTime();        
        }
        if(m_cam.takeSens() == Camera::CameraResult::RESULT_R0
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distRi.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "H - right ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::DOUBLE_VICTIM_RIGHT;
            ServiceTimer = time_service::getCurTime();        
        }
        if(m_cam.takeSens() == Camera::CameraResult::RESULT_L1
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distLe.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "S - left  ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::VICTIM_LEFT;
            ServiceTimer = time_service::getCurTime();        
        }
        if(m_cam.takeSens() == Camera::CameraResult::RESULT_R1
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distRi.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "S - right ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::VICTIM_RIGHT;
            ServiceTimer = time_service::getCurTime();        
        }
        if(m_cam.takeSens() == Camera::CameraResult::RESULT_L2
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distLe.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "U - left  ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::SIGNAL_VICTIM;
            ServiceTimer = time_service::getCurTime();        
        }
        if(m_cam.takeSens() == Camera::CameraResult::RESULT_R2
            && time_service::getCurTime() - ServiceTimer > victimTime//защита от сногократного срабатывания
            && (!m_isMapMovement || m_isFirstMapRotate)//при езде по карте жертвы искать не нужно
            && m_state != State::VICTIM_LEFT//если уже выдаем, то тоже не нужно
            && m_state != State::VICTIM_RIGHT
            && m_state != State::DOUBLE_VICTIM_LEFT
            && m_state != State::DOUBLE_VICTIM_RIGHT
            && m_state != State::SIGNAL_VICTIM
            && m_state != State::PAUSE
            && m_distRi.takeSens() < wallRast)
        {
            m_workDisp.Set_Color(BLUE);
            m_workDisp.print_String(0, 45, "U - right ", FONT_5X8);
            m_workDisp.Set_Color(RED);
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::SIGNAL_VICTIM;
            ServiceTimer = time_service::getCurTime();        
        }
        if(m_state == State::PAUSE)
        {
            m_movement.fullStop();
         
            if(GPIO_Read(otherPins::button2) && time_service::getCurTime() - m_pauseButton > pauseButtonTimer)
            {
                m_isAimHill = 0;
                m_isFirstCall = 1;
                m_state = State::MAP;
                m_pauseButton = time_service::getCurTime();
            }
        }
        else
        {
            if(GPIO_Read(otherPins::button2) && time_service::getCurTime() - m_pauseButton > pauseButtonTimer)
            {
                GPIO_ResetPin(otherPins::victimLed);
                m_mapOp.useCheckPoint();
                m_state = State::PAUSE;
                m_pauseButton = time_service::getCurTime();
            }
        }
        //volatile uint16_t light = pins::adcData[4];
        
        if(m_state != State::HILL_MOVEMENT
        && m_state != State::BACK_MOVEMENT
        && m_state != State::ROTATE_B
        && m_state != State::ROTATE_R
        && m_state != State::ROTATE_L
        && m_state != State::PAUSE
        && m_state != State::DOUBLE_VICTIM_LEFT
        && m_state != State::DOUBLE_VICTIM_RIGHT
        && m_state != State::VICTIM_LEFT
        && m_state != State::VICTIM_RIGHT
        && m_state != State::SIGNAL_VICTIM)
        {
            if(m_analogSenor.getAdc(m_analogSenor.lightSensor) > greyValue 
            && m_state != State::BACK_MOVEMENT
            && m_state != State::ROTATE_B
            && m_state != State::ROTATE_R
            && m_state != State::ROTATE_L)
            {
                m_greyCount++;
            }
            else 
            {
                m_greyCount = 0;
            }
            if(m_greyCount >= greyCountConst)
            {
                m_isFirstCall = 1;
                m_state = State::BACK_MOVEMENT;
            }
            if(m_hillCount > hillCountLimitFind)
            {
                m_hillCount = 0;
                if(!m_isMapMovement)
                {
                    m_mapOp.dir = m_mapOp.convRobToAbs(m_ChDir);
                    switch(m_mapOp.dir)
                    {
                        case absFront:
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].y = m_mapOp.robCorY[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].dir = m_mapOp.dir;
                            m_mapOp.hills[!m_mapOp.floor][m_mapOp.numOfHills].dir = absBack;
                            break;
                        case absRight:
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].y = m_mapOp.robCorY[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].dir = m_mapOp.dir;
                            m_mapOp.hills[!m_mapOp.floor][m_mapOp.numOfHills].dir = absLeft;
                            break;
                        case absBack:
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].y = m_mapOp.robCorY[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].dir = m_mapOp.dir;
                            m_mapOp.hills[!m_mapOp.floor][m_mapOp.numOfHills].dir = absFront;
                            break;
                        case absLeft:
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].x = m_mapOp.robCorX[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].y = m_mapOp.robCorY[m_mapOp.floor];
                            m_mapOp.hills[m_mapOp.floor][m_mapOp.numOfHills].dir = m_mapOp.dir;
                            m_mapOp.hills[!m_mapOp.floor][m_mapOp.numOfHills].dir = absRight;
                            break;
                    }
                }
                else
                {
                    m_mapOp.dir = m_mapOp.convRobToAbs(m_ChDir);
                    //m_mapOp.dir = m_mapOp.convRobToAbs(m_ChDir);
                }
                m_state = State::HILL_MOVEMENT;
            }
            if(m_gyro.isHill())
            {
                m_hillCount++;
            }
            else m_hillCount = 0;
        }
        if(m_state == State::END_STATE)
        {
            m_pinState = PinState::DISABLE;
            ServiceTimer2 = time_service::getCurTime();        
            m_ResetState = m_state;
            m_state = State::SIGNAL_VICTIM;
            ServiceTimer = time_service::getCurTime();        
        }
    }
    else
    {
        m_movement.fullStop();
    }
}
