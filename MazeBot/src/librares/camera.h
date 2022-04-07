#pragma once 
#include "project_config.h"
#include "pins.h"
using namespace pins;

class Camera
{
/* | L1 | L2 | R1 | R2 | */
/* |  0 |  1 |  2 |  3 | */

/* 
    NO_DATA :
    |  0 |  0 |  0 |  0 | - bin
    0 - dec
    
    RESULT_L0 :
    |  1 |  1 |  0 |  0 | - bin
    12 - dec
    
    RESULT_R0 :
    |  0 |  0 |  1 |  1 | - bin
    12 - dec
    
    RESULT_L1 :
    |  0 |  1 |  0 |  0 | - bin
    4 - dec
    
    RESULT_L2 :
    |  1 |  0 |  0 |  0 | - bin
    8 - dec
    
    RESULT_R1 :
    |  0 |  0 |  0 |  1 | - bin
    1 - dec
    
    RESULT_R2 :
    |  0 |  0 |  1 |  0 | - bin
    2 - dec
 */


    public:
    STRONG_ENUM(CameraResult,
            NO_DATA = 0,
            RESULT_L0 = 12,
            RESULT_R0 = 3,
            RESULT_L1 = 4,
            RESULT_L2 = 8,
            RESULT_R1 = 1,
            RESULT_R2 = 2);
        Camera(pinStructure pinL1,pinStructure pinL2,pinStructure pinR1,pinStructure pinR2);
        CameraResult takeSens();
    private:
        pinStructure m_pinL1;
        pinStructure m_pinL2;
        pinStructure m_pinR1;
        pinStructure m_pinR2;
};

