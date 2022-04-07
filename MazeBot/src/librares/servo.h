#pragma once 
#include "project_config.h"
#include "pins.h"
using namespace pins;

class Servo{
	public:
		Servo(pinStructure pin);
		void setAngle(uint16_t angle);
	private:
		pinStructure m_ServPin;
};
