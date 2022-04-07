#include "map_operations.h"
#include "base_movment.h"
#define wallRast 270


/*
Конструктор класса
Входные данные:
vl53l0x& FrontDist - передний дальномер
vl53l0x& RightDist - проавый дальномер
vl53l0x& LeftDist - левый дальномер
vl53l0x& BotDist - задний дальномер
*/
Map_operations::Map_operations(vl53l0x& FrontDist,
                               vl53l0x& RightDist,
                               vl53l0x& LeftDist,
                               vl53l0x& BotDist,
                               OLED_GFX& workDisp):
                                m_FrontDist(FrontDist),
                                m_RightDist(RightDist),
                                m_LeftDist(LeftDist),
                                m_BotDist(BotDist),
                                m_workDisp(workDisp)
{
    //m_FrontDist = &FrontDist;
    //m_RightDist = &RightDist;
    //m_LeftDist = &LeftDist;
    //m_BotDist = &BotDist;
    robCorX[0] = sizeX / 2;
    robCorY[0] = sizeY / 2;
    robCorX[1] = sizeX / 2;
    robCorY[1] = sizeY / 2;
    TaleWhitePoints = 0;
    dir = 0;
    floor = startFloor;
    numOfHills = 0;
    
    for (int fl1 = 0; fl1 < floors; fl1++)
    {
        for(int x1 = 0; x1 < sizeX; x1++)
        {
            for(int y1 = 0; y1 < sizeY; y1++)
            {
                mapVizit[fl1][x1][y1] = 0;
            }
        }
    }
    for (int fl1 = 0; fl1 < floors; fl1++)
    {
        for(int x1 = 0; x1 < sizeX * 2; x1++)
        {
            for(int y1 = 0; y1 < sizeY * 2; y1++)
            {
                mapWall[fl1][x1][y1] = 0;
            }
        }
    }
    //whitePoints[floor][TaleWhitePoints[floor]].x = robCorX[floor];
    //whitePoints[floor][TaleWhitePoints[floor]++].y = robCorY[floor];
    addCheckPoint();
    whitePoints[TaleWhitePoints].floor = floor;
    whitePoints[TaleWhitePoints].x = robCorX[floor];
    whitePoints[TaleWhitePoints++].y = robCorY[floor];
}
/*
Функция возвращает абсолютное растояние
Входные данные: uint8_t dirT - абсолютное напровление
Выходные данные: uint16_t - растояние
*/
uint16_t Map_operations::takeAbsSens(uint8_t dirT)
{
    if (convAbsToRob(dirT) == 1)
    {
        return m_RightDist.takeSens();
    }
    if (convAbsToRob(dirT) == -1) 
    {
        return m_LeftDist.takeSens();
    }
    if (convAbsToRob(dirT) == 2) 
    {
        return m_BotDist.takeSens();
    }
    if (convAbsToRob(dirT) == 0) 
    {
        return m_FrontDist.takeSens();
    }
    return 0;
}
/*
Функция строит карту в данной ячейке
*/
void Map_operations::makeMap()
{
    char str[15];
    sprintf(str, "WHPs = %i    ", TaleWhitePoints);
    m_workDisp.print_String(0, 0, str, FONT_8X16);
    m_workDisp.Set_Color(GREEN);
    if(takeAbsSens(0) < wallRast)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 6 + 3 - 50 - 50 * floor, robCorY[floor] * 6 - 50, 3, 3);
        mapWall[floor][robCorX[floor] * 2 + 1][robCorY[floor] * 2] = 1;
    }
    if(takeAbsSens(1) < wallRast)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 6 - 50 - 50 * floor, robCorY[floor] * 6 + 3 - 50, 3, 3);
        mapWall[floor][robCorX[floor] * 2][robCorY[floor] * 2 + 1] = 1;
    }
    if(takeAbsSens(2) < wallRast)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 6 - 3 - 50 - 50 * floor, robCorY[floor] * 6 - 50, 3, 3);
        mapWall[floor][robCorX[floor] * 2 - 1][robCorY[floor] * 2] = 1;
    }
    if(takeAbsSens(3) < wallRast)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 6 - 50 - 50 * floor, robCorY[floor] * 6 - 3 - 50, 3, 3);
        mapWall[floor][robCorX[floor] * 2][robCorY[floor] * 2 - 1] = 1;
    }
    mapVizit[floor][robCorX[floor]][robCorY[floor]] = 1;
    m_workDisp.Set_Color(RED);
    m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 - 50 - 50 * floor, robCorY[floor] * 2 * 3 - 50, 3, 3);
}
/*
Функция пререводит абсолютное направление, в относительно робота
Входные даные: uint8_t dat - абсолютное направление
Выходные данные: int8_t - относительное направление
*/
int8_t Map_operations::convAbsToRob(uint8_t dat){
    if (dat - dir == 1 || dat - dir == -3)
        return 1;
    if (dat - dir == -1 || dat - dir == 3) 
        return -1;
    if (abs(dat - dir) == 2)
        return 2;
    if (dat - dir == 0) 
        return 0;
    return 0;
}
/*
Функция пререводит относительное направление, в абсолютное
Входные даные: uint8_t dat - абсолютное направление
Выходные данные: int8_t - относительное направление
*/
uint8_t Map_operations::convRobToAbs(int8_t dat){        
    if(dir + dat == 0 || dir + dat == 4)return 0;
    if(dir + dat == 1 || dir + dat == 5)return 1;
    if(dir + dat == 2)return 2;
    if(dir + dat == -1 || dir + dat == 3)return 3;
    return 1;
}
/*
Функция возвращает значение карты относительно робота:
Входные данные: int8_t dirT - направление 
Выходные данные: bool - значение карты
*/
bool Map_operations::robMap(int8_t dirT)
{
    if(convRobToAbs(dirT)==0)return mapVizit[floor][robCorX[floor]+1][robCorY[floor]];
    if(convRobToAbs(dirT)==1)return mapVizit[floor][robCorX[floor]][robCorY[floor]+1];
    if(convRobToAbs(dirT)==2)return mapVizit[floor][robCorX[floor]-1][robCorY[floor]];
    if(convRobToAbs(dirT)==3)return mapVizit[floor][robCorX[floor]][robCorY[floor]-1];
    return 1;
}
/*
Функция возвращает значение карты относительно робота:
Входные данные: int8_t dirT - направление 
Выходные данные: bool - значение карты
*/
bool Map_operations::robMapBl(int8_t dirT)
{
    if(convRobToAbs(dirT)==0)return mapWall[floor][(robCorX[floor] + 1)*2][robCorY[floor]*2];
    if(convRobToAbs(dirT)==1)return mapWall[floor][robCorX[floor] * 2][(robCorY[floor] + 1)*2];
    if(convRobToAbs(dirT)==2)return mapWall[floor][(robCorX[floor] - 1) * 2][robCorY[floor] * 2];
    if(convRobToAbs(dirT)==3)return mapWall[floor][robCorX[floor] * 2][(robCorY[floor]-1)*2];
    return 1;
}
/*
Функция возвращает направление по правилу правой руки:
Входные данные:
Выходные данные: int8_t - направление
*/
int8_t Map_operations::dirRob()
{
    uint16_t right = m_RightDist.takeSens();
    uint16_t left = m_LeftDist.takeSens();
    uint16_t bot =  m_BotDist.takeSens();
    uint16_t front = m_FrontDist.takeSens();
    bool flag = 1;
    bool addLeft = 0, addFront = 0;
    int8_t dirC = 2;
    if(right > wallRast && robMap(1) == 0 && robMapBl(1) == 0)
    {
        if(flag)
        {
            dirC=1;
            flag=0;
        }
    }
    if(front > wallRast && robMap(0) == 0 && robMapBl(0) == 0)
    {
        if(flag)
        {
            dirC=0;
            flag=0;
        }
        else
        {
            addFront = 1;
            /*if(convRobToAbs(0) == 0)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor] + 1;
                whitePoints[TaleWhitePoints++].y = robCorY[floor];
            }
            if(convRobToAbs(0) == 1)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor];
                whitePoints[TaleWhitePoints++].y = robCorY[floor] + 1;
            }
            if(convRobToAbs(0) == 2)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor] - 1;
                whitePoints[TaleWhitePoints++].y = robCorY[floor];
            }
            if(convRobToAbs(0) == 3)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor];
                whitePoints[TaleWhitePoints++].y = robCorY[floor] - 1;
            }*/
        }
    }
    if((left > wallRast) && (robMap(-1)==0) && robMapBl(-1) == 0)
    {
        if(flag)
        {
            dirC=-1;
            flag=0;
        }
        else
        {
            addLeft = 1;
            /*if(convRobToAbs(-1) == 0)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor] + 1;
                whitePoints[TaleWhitePoints++].y = robCorY[floor];
            }
            if(convRobToAbs(-1) == 1)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor];
                whitePoints[TaleWhitePoints++].y = robCorY[floor] + 1;
            }
            if(convRobToAbs(-1)==2)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor] - 1;
                whitePoints[TaleWhitePoints++].y = robCorY[floor];
            }
            if(convRobToAbs(-1)==3)
            {
                whitePoints[TaleWhitePoints].floor = floor;
                whitePoints[TaleWhitePoints].x = robCorX[floor];
                whitePoints[TaleWhitePoints++].y = robCorY[floor] - 1;
            }*/
        }
    }
    if(addLeft)
    {
        if(convRobToAbs(-1) == 0)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor] + 1;
            whitePoints[TaleWhitePoints++].y = robCorY[floor];
        }
        if(convRobToAbs(-1) == 1)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor];
            whitePoints[TaleWhitePoints++].y = robCorY[floor] + 1;
        }
        if(convRobToAbs(-1)==2)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor] - 1;
            whitePoints[TaleWhitePoints++].y = robCorY[floor];
        }
        if(convRobToAbs(-1)==3)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor];
            whitePoints[TaleWhitePoints++].y = robCorY[floor] - 1;
        }
    }
    if(addFront)
    {
        if(convRobToAbs(0) == 0)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor] + 1;
            whitePoints[TaleWhitePoints++].y = robCorY[floor];
        }
        if(convRobToAbs(0) == 1)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor];
            whitePoints[TaleWhitePoints++].y = robCorY[floor] + 1;
        }
        if(convRobToAbs(0) == 2)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor] - 1;
            whitePoints[TaleWhitePoints++].y = robCorY[floor];
        }
        if(convRobToAbs(0) == 3)
        {
            whitePoints[TaleWhitePoints].floor = floor;
            whitePoints[TaleWhitePoints].x = robCorX[floor];
            whitePoints[TaleWhitePoints++].y = robCorY[floor] - 1;
        }
    }
    return dirC;
}
/*
Функция возвращает направление по правилу правой руки:
Входные данные: bool loop - 1 - первый вызов
uint8_t startX - кордината начала x
uint8_t startY - кордината начала y
uint8_t startFloor - кордината начала этаж
uint8_t finishX - кордината конца x
uint8_t finishY - кордината конца y
uint8_t finishFloor - кордината конца этаж
bool moving - нужно ехать или нет пока не работает
Выходные данные: 
на 1 этапе - -2 продолжать вызывать, 0 - следующий этап,
на 2 этапе - направления, -2 - заканчивай
*/
int8_t Map_operations::makeRoad(bool loop, uint8_t startX,uint8_t startY, 
        uint8_t startFloor, uint8_t finishX, uint8_t finishY, 
        uint8_t finishFloor, bool moving)
{
    if(loop)
    {
        makeMap();
        m_mapX = finishX;
        m_mapY = finishY;
        m_mapFloor = finishFloor;
        mapingVizit[m_mapFloor][m_mapX][m_mapY] = 1;
        m_head=0;
        m_tale=0;
        m_state = mapState::BFS;
        mapVizit[floor][startX][startY] = 1;
        mapVizit[floor][m_mapX][m_mapY] = 1;
        for (uint8_t flr = 0; flr < floors; flr++)
        {
            for (uint16_t x = 0; x<sizeX; x++)
            {
                for (uint16_t y = 0; y < sizeX; y++)
                {
                    mapingVizit[flr][x][y] = 0;
                }                
            }
        }
    }
    if(m_state == mapState::BFS)
    {
        if (m_mapX == startX && m_mapY == startY && m_mapFloor == startFloor)
        {
            m_mapX = startX;
            m_mapY = startY;
            m_mapFloor = startFloor;
            m_state = mapState::ROAD_MAKE;
            return 0;
        }
        if((loop) || (m_head >= m_tale))
        {
            for(int8_t dx = -1; dx <= 1; dx += 1)
            {
                for(int8_t dy = -1; dy <= 1; dy += 1)
                {
                    if ((abs(dx) == 1) && (abs(dy) == 1))continue;
                    if ((dx == 0) && (dy == 0))continue;
                    if ((m_mapY + dy < 0) || (m_mapX + dx < 0))continue;
                    if ((m_mapX + dx >= sizeX)||(m_mapY + dy >= sizeY))continue;
                    if (mapWall[m_mapFloor][m_mapX * 2 + dx][m_mapY * 2 + dy] == 1)continue;
                    if (mapWall[m_mapFloor][m_mapX * 2 + dx * 2][m_mapY * 2 + dy * 2] == 1)
                        continue;
                    if (mapVizit[m_mapFloor][m_mapX + dx][m_mapY + dy] == 0)continue;
                    if (mapingVizit[m_mapFloor][m_mapX + dx][m_mapY + dy])continue;
                    mapingVizit[m_mapFloor][m_mapX+dx][m_mapY+dy] = 1;
                    m_roadMap[m_mapFloor][m_mapX + dx][m_mapY + dy].x = m_mapX;
                    m_roadMap[m_mapFloor][m_mapX + dx][m_mapY + dy].y = m_mapY;
                    m_stack[m_head].x = m_mapX + dx;
                    m_stack[m_head].y = m_mapY + dy;
                    m_stack[m_head].floor = m_mapFloor;
                    m_head++;
                    int8_t hill = isHill(m_mapX + dx, m_mapY + dy, m_mapFloor);
                    if (hill != -1)
                    {
                        m_stack[m_head].x = hills[!m_mapFloor][hill].x;
                        m_stack[m_head].y = hills[!m_mapFloor][hill].y;
                        m_stack[m_head].floor = !m_mapFloor;
                        mapingVizit[m_stack[m_head].floor][m_stack[m_head].x][m_stack[m_head].y] = 1;
                        m_head++;
                        if (hills[!m_mapFloor][hill].dir == 0)
                        {
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].x 
                                    = hills[!m_mapFloor][hill].x + 1;
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].y 
                                    = hills[!m_mapFloor][hill].y;
                        }
                        if (hills[!m_mapFloor][hill].dir == 1)
                        {
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].x 
                                    = hills[!m_mapFloor][hill].x;
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].y 
                                    = hills[!m_mapFloor][hill].y + 1;
                        }
                        if (hills[!m_mapFloor][hill].dir == 2)
                        {
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].x
                                    = hills[!m_mapFloor][hill].x - 1;
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].y
                                    = hills[!m_mapFloor][hill].y;
                        }
                        if (hills[!m_mapFloor][hill].dir == 3)
                        {
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].x
                                    = hills[!m_mapFloor][hill].x;
                            m_roadMap[!m_mapFloor][hills[!m_mapFloor][hill].x][hills[!m_mapFloor][hill].y].y
                                    = hills[!m_mapFloor][hill].y - 1;
                        }
                    }
                }
            }
            m_mapX = m_stack[m_tale].x;
            m_mapY = m_stack[m_tale].y;
            m_mapFloor = m_stack[m_tale].floor;
            m_tale++;
        }
        else
        {
          return -5;
        }
    }
    if(m_state == mapState::ROAD_MAKE)
    {
        m_mapX = robCorX[floor];
        m_mapY = robCorY[floor];
        if(robCorX[floor] == finishX && robCorY[floor] == finishY 
            && floor == finishFloor)
        {
            if(robCorX[floor] == sizeX / 2 
            && robCorY[floor] == sizeY / 2 && floor == 0)
            {
                //UMBA_ASSERT_FAIL();
                return -3;
            }
            return -2;
        }
        if((m_mapX - m_roadMap[floor][m_mapX][m_mapY].x == 1) 
            && (m_mapY - m_roadMap[floor][m_mapX][m_mapY].y == 0))
        {
            m_mapX = m_roadMap[floor][m_mapX][m_mapY].x;
            m_mapY = m_roadMap[floor][m_mapX][m_mapY].y;
            return convAbsToRob(2);
        }
        if((m_mapX - m_roadMap[floor][m_mapX][m_mapY].x == -1)
            && (m_mapY - m_roadMap[floor][m_mapX][m_mapY].y == 0))
        {
            m_mapX = m_roadMap[floor][m_mapX][m_mapY].x;
            m_mapY = m_roadMap[floor][m_mapX][m_mapY].y;
            return convAbsToRob(0);
        }
        if((m_mapX - m_roadMap[floor][m_mapX][m_mapY].x == 0)
            &&(m_mapY - m_roadMap[floor][m_mapX][m_mapY].y == 1))
        {
            m_mapX = m_roadMap[floor][m_mapX][m_mapY].x;
            m_mapY = m_roadMap[floor][m_mapX][m_mapY].y;
            return convAbsToRob(3);
        }
        if((m_mapX - m_roadMap[floor][m_mapX][m_mapY].x == 0 ) 
            && (m_mapY - m_roadMap[floor][m_mapX][m_mapY].y == -1))
        {
            m_mapX = m_roadMap[floor][m_mapX][m_mapY].x;
            m_mapY = m_roadMap[floor][m_mapX][m_mapY].y;
            return convAbsToRob(1);
        }
        //UMBA_ASSERT_FAIL();
    }
    return -2;
}
/*
просто вызывай после любого проезда
Входные данные: направление в котором был совершен проезд
*/
void Map_operations::chngeCor(uint8_t dirI)
{
    uint8_t dirA = convRobToAbs(dirI);
    if(dirA == 0)
    {
        dir = 0;
        robCorX[floor]++;
        robCorX[!floor]++;
    }
    if(dirA == 1)
    {
        dir = 1;
        robCorY[floor]++;
        robCorY[!floor]++;
    }
    if(dirA == 2)
    {
        dir = 2;
        robCorX[floor]--;
        robCorX[!floor]--;
    }
    if(dirA == 3)
    {
        dir = 3;
        robCorY[floor]--;
        robCorY[!floor]--;
    }
    char str[15];
    sprintf(str, "X = %i    Y = %i  ", robCorX[floor], robCorY[floor]);
    m_workDisp.print_String(0, 25, str, FONT_5X8);
    sprintf(str, "dir = %i   flor = %i ", dir, floor);
    m_workDisp.print_String(0, 35, str, FONT_5X8);
}
/*
Считает клетку перед роботом черной, и в такoм виде заносит ее в карту
*/
void Map_operations::addBlackPiont()
{
    if(convRobToAbs(0)==0)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 + 3 - 50 - 50 * floor, robCorY[floor] * 2 * 3 - 50, 3, 3);
        mapVizit[floor][robCorX[floor] + 1][robCorY[floor]] = 1;
        mapWall[floor][(robCorX[floor] + 1) * 2][robCorY[floor] * 2] = 1;
    }
    if(convRobToAbs(0)==1)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 - 50 - 50 * floor, robCorY[floor] * 2 * 3 + 3 - 50, 3, 3);
        mapVizit[floor][robCorX[floor]][robCorY[floor] + 1] = 1;
        mapWall[floor][robCorX[floor] * 2][(robCorY[floor] + 1) * 2] = 1;
    }
    if(convRobToAbs(0)==2)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 - 3 - 50 - 50 * floor, robCorY[floor] * 2 * 3 - 50, 3, 3);
        mapVizit[floor][robCorX[floor] - 1][robCorY[floor]] = 1;
        mapWall[floor][(robCorX[floor] - 1) * 2][robCorY[floor] * 2] = 1;
    }
    if(convRobToAbs(0)==3)
    {
        m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 - 50 - 40 * floor, robCorY[floor] * 2 * 3 - 3 - 50, 3, 3);
        mapVizit[floor][robCorX[floor]][robCorY[floor] - 1] = 1;
        mapWall[floor][robCorX[floor] * 2][(robCorY[floor] - 1) * 2] = 1;
    }
    //че это за хуйня -said cat
}
/*
CoxpaHяет все параметры роота, для последующего перезапуска
*/
void Map_operations::addCheckPoint()
{
    m_workDisp.Set_Color(YELLOW);
    m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 - 50 - 50 * floor, robCorY[floor] * 2 * 3 - 50, 3, 3);
    m_workDisp.print_String(0, 0, "checkPiont     ", FONT_8X16);
    m_workDisp.Set_Color(RED);

    m_checkPointfloor = floor;
    m_checkPointNumOfHills = numOfHills;
    m_checkPointDir = dir;

    for(int it = 0; it < floors; it++)
    {
        for(int x = 0; x < sizeX * 2; x++)
        {
            for(int y = 0;y < sizeY * 2; y++)
            {
                m_checkPointMapWall[it][x][y] = mapWall[it][x][y];
            }
        }
        m_checkPointRobCorX[it] = robCorX[it];
        m_checkPointRobCorY[it] = robCorY[it];
    }
    m_checkPointTaleWhitePoints = TaleWhitePoints;
    for(int it = 0; it < floors; it++)
    {
        for(int x = 0; x < sizeX; x++)
        {
            for(int y = 0;y < sizeY; y++)
            {
                m_checkPointMapVizit[it][x][y] = mapVizit[it][x][y];
            }
        }
    }

    for(int cp = 0;cp < TaleWhitePoints;cp++)
    {
        m_checkPointWhitePoints[cp].x = whitePoints[cp].x;  
        m_checkPointWhitePoints[cp].y = whitePoints[cp].y;
        m_checkPointWhitePoints[cp].dir = whitePoints[cp].dir;
        m_checkPointWhitePoints[cp].floor = whitePoints[cp].floor;
    }

    for(int cp = 0;cp < numOfHills;cp++)
    {
        for(int it = 0; it < floors; it++)
        {
            m_checkPointHills[it][cp].x = hills[it][cp].x;  
            m_checkPointHills[it][cp].y = hills[it][cp].y;
            m_checkPointHills[it][cp].dir = hills[it][cp].dir;
        }
    }
}
/*
Использует сохраненные параметры
*/
void Map_operations::useCheckPoint()
{
    floor = m_checkPointfloor;
    dir = m_checkPointDir;
    numOfHills = m_checkPointNumOfHills;

    for(int it = 0; it < floors; it++)
    {
        for(int x = 0; x < sizeX * 2; x++)
        {
            for(int y = 0;y < sizeY * 2; y++)
            {
                mapWall[it][x][y] = m_checkPointMapWall[it][x][y];
            }
        }
        robCorX[it] = m_checkPointRobCorX[it];
        robCorY[it] = m_checkPointRobCorY[it];
    }
    TaleWhitePoints = m_checkPointTaleWhitePoints;
    for(int it = 0; it < floors; it++)
    {
        for(int x = 0; x < sizeX; x++)
        {
            for(int y = 0;y < sizeY; y++)
            {
                mapVizit[it][x][y] = m_checkPointMapVizit[it][x][y];
            }
        }
    }
    for(int cp = 0;cp < m_checkPointTaleWhitePoints;cp++)
    {
        whitePoints[cp].x = m_checkPointWhitePoints[cp].x;  
        whitePoints[cp].y = m_checkPointWhitePoints[cp].y;
        whitePoints[cp].dir = m_checkPointWhitePoints[cp].dir;
        whitePoints[cp].floor = m_checkPointWhitePoints[cp].floor;
    }
    for(int cp = 0;cp < numOfHills; cp++)
    {
        for(int it = 0; it < floors; it++)
        {
            hills[it][cp].x = m_checkPointHills[it][cp].x;
            hills[it][cp].y = m_checkPointHills[it][cp].y;
            hills[it][cp].dir = m_checkPointHills[it][cp].dir;
        }
    }
    m_workDisp.Set_Color(BLUE);
    m_workDisp.Draw_Rect(robCorX[floor] * 2 * 3 - 50 - 50 * floor, robCorY[floor] * 2 * 3 - 50, 3, 3);
    switch(dir)
    {
        case 0:
            m_workDisp.print_String(0, 0, "DIR = 0     ", FONT_8X16);
            break;
        case 1:
            m_workDisp.print_String(0, 0, "DIR = 1     ", FONT_8X16);
            break;
        case 2:
            m_workDisp.print_String(0, 0, "DIR = 2     ", FONT_8X16);
            break;
        case 3:
            m_workDisp.print_String(0, 0, "DIR = 3     ", FONT_8X16);
            break;
    }
    m_workDisp.Set_Color(RED);
}
/*
Функция проверяет являеться ли точка - горкой
Входные данные: uint8_t x - кордината х проверяемой точки
                uint8_t y - кордината у проверяемой точки
                uint8_t floor - этаж проверяемой точки
Выходные данные:bool - 1 - горка, 0 - нет
*/
int8_t Map_operations::isHill(uint8_t x, uint8_t y, uint8_t floor)
{
    int8_t fl = -1;
    for(uint16_t hill = 0; hill < numOfHills; hill++)
    {
        if(hills[floor][hill].x == x && hills[floor][hill].y == y)fl = hill;
    }
    return fl;
}
