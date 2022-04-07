#pragma once 
#include "project_config.h"
#include "vl53l0x.h"
#include "display.h"
#define sizeX 42
#define sizeY 42
#define floors 2

/*
системы кординат:
Абсолютная:
**0**
*   *
3   1  
*   *
**2**
Относительная:
**0**
*   *
-1  1  
*   *
**2**
*/

class Map_operations{
    public:
        Map_operations(vl53l0x& FrontDist,
                       vl53l0x& RightDist,
                       vl53l0x& LeftDist,
                       vl53l0x& BotDist,
                       OLED_GFX& workDisp);
        uint16_t takeAbsSens(uint8_t dirT);
        void makeMap();
        int8_t dirRob();
        bool robMap(int8_t derT);
        bool robMapBl(int8_t derT);    
        
        void chngeCor(uint8_t dir);
        
        int8_t makeRoad(bool loop, uint8_t startX,uint8_t startY, uint8_t startFloor, 
            uint8_t finishX, uint8_t finishY, uint8_t finishFloor,
            bool moving);
    
        int8_t convAbsToRob(uint8_t dat);
        uint8_t convRobToAbs(int8_t dat);
    
        void addBlackPiont();
    
        STRONG_ENUM( floor, FIRST_floor = 0, SECOND_floor = 1);
        uint8_t floor;
        static const uint8_t startFloor = 0;
        bool mapWall[floors][sizeX*2][sizeY*2];
        bool mapVizit[floors][sizeX][sizeY];
        bool mapingVizit[floors][sizeX][sizeY];
        uint8_t robCorX[floors], robCorY[floors]; 
        uint8_t dir;
        
        void addCheckPoint();
        void useCheckPoint();
        struct point{
            uint8_t x;
            uint8_t y;
            uint8_t floor;
            uint8_t dir;
        };
        
        uint8_t TaleWhitePoints;
        
        point whitePoints[sizeX*sizeY];
    
        point hills[floors][sizeX * sizeY];
        uint32_t numOfHills;
        
        STRONG_ENUM( mapState, BFS, ROAD_MAKE );
        
        STRONG_ENUM(robRotate, 
            RIGHT = 1, 
            LEFT = -1, 
            FRONT = 0, 
            BACK = 2, 
            END_OF_MOVEMENT = -2);
        STRONG_ENUM(absRotate, 
            RIGHT = 1, 
            LEFT = 3, 
            FRONT = 0, 
            BACK = 2, 
            END_OF_MOVEMENT = -2);
            int8_t isHill(uint8_t x, uint8_t y, uint8_t floor);
    private:
        mapState::Type m_state;
        
        int8_t m_road[sizeX*sizeY];
        
        uint16_t m_head;
        uint16_t m_tale;
        uint16_t m_mapX;
        uint16_t m_mapY;
        uint16_t m_mapFloor;
        
        point m_stack[sizeX*sizeY];
        
        point m_roadMap[floors][sizeX][sizeY];
    
        vl53l0x & m_FrontDist;
        vl53l0x & m_RightDist;
        vl53l0x & m_LeftDist;
        vl53l0x & m_BotDist;
    
        OLED_GFX & m_workDisp;
    
        uint8_t m_checkPointfloor;
        bool m_checkPointMapWall[floors][sizeX * 2][sizeY * 2];
        bool m_checkPointMapVizit[floors][sizeX][sizeY];
        uint8_t m_checkPointRobCorX[floors], m_checkPointRobCorY[floors]; 
        uint8_t m_checkPointDir;
        
        uint8_t m_checkPointTaleWhitePoints;
        
        point m_checkPointWhitePoints[sizeX*sizeY];
    
        point m_checkPointHills[floors][sizeX * sizeY];
        uint32_t m_checkPointNumOfHills;
};
