#pragma once
#include "project_config.h"
#include "pins.h"
#include "softI2C.h"

using namespace pins;

class i2c{
	private:
		uint8_t m_adr;
	public:
        SoftI2C * m_softI2C;
        void adrChange(uint8_t adr);
		void init_I2C(SoftI2C& softI2C, uint8_t adr);
    
        void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
        void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);
		void writeReg(uint8_t reg, uint8_t value);
		void writeReg16Bit(uint8_t reg, uint16_t value);
		void writeReg32Bit(uint8_t reg, uint32_t value);
		uint8_t readReg(uint8_t reg);
		uint16_t readReg16Bit(uint8_t reg);
		uint32_t readReg32Bit(uint8_t reg);
        uint32_t readReg24Bit(uint8_t reg);
};
