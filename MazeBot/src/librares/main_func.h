#pragma once
#include "project_config.h"
#include "vl53l0x.h"
#include "mlx906014.h"
#include "servo.h"
#include "gyro.h"
#include "camera.h"
#include "motor.h"
#include "base_movment.h"
#include "map_operations.h"
#include "pinsStr.h"
#include "analog_input.h"

class MainFunc{
    public:
        MainFunc(vl53l0x &distFr,
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
                   Analog_input &analogSenor);
        void loop(bool isMovement);
        int8_t m_ChDir;
        uint16_t WHy;
        uint16_t WHx;
        uint8_t WHfloor;
    private:
        STRONG_ENUM( State,
            CHOSE_SIDE,
            MOVING_FR, ROTATE_R, ROTATE_L, ROTATE_B,
            MAP, 
            MAKE_ROAD, ROAD_MOVEMENT, 
            BACK_MOVEMENT,
            HILL_MOVEMENT,
            VICTIM_RIGHT,
            VICTIM_LEFT,
            DOUBLE_VICTIM_RIGHT,
            DOUBLE_VICTIM_LEFT,
            SIGNAL_VICTIM,
            EMPTY,
            END_STATE,
            PAUSE,
            TOTAL_StateS = 20);
        STRONG_ENUM( PinState, 
            ENABLE = 1,
            DISABLE = 0);
        uint32_t m_greyCount;
        uint32_t m_pauseButton;
        State::Type m_ResetState;
        State::Type m_state;
        uint32_t ServiceTimer2;
        uint32_t ServiceTimer;
        uint32_t m_hillCount;
        uint32_t m_contTimer;
        //bool m_isNewHill;
        PinState::Type m_pinState;
        vl53l0x & m_distFr;
        vl53l0x & m_distRi;
        vl53l0x & m_distLe;
        vl53l0x & m_distBo;
        vl53l0x & m_distRiFr;
        vl53l0x & m_distLeFr;
        mlx906014 & m_heatRi;
        mlx906014 & m_heatLe;
        Gyro & m_gyro;
        Servo & m_serv;
        Camera & m_cam;
        Motor & m_motorFrLe;
        Motor & m_motorFrRi;
        Motor & m_motorBoLe;
        Motor & m_motorBoRi;
        Base_movement & m_movement;
        Map_operations & m_mapOp;
        OLED_GFX & m_workDisp;
        Analog_input & m_analogSenor;
        
        bool m_isAimHill;
        bool m_isFirstCall;
        bool m_isMapMovement;
        bool m_isFirstMapRotate;
        //uint16_t min, nOfMin;
        //callback::Callback m_callbackArray[TOTAL_StateS = 20];
};
