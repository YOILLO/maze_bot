#include "vl53l0x.h"
#include "pins.h"
#include "time_service.h"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
/*
Конструктор класса, инициализирует шину i2c и задает адресс устройству.
Входные данные: 
        SoftI2C* softI2C - инициализированная шина softI2C;
        pinStructure xshutPin - xshut пин
        uint8_t adr - адресс, который будет дан устройству
*/
vl53l0x::vl53l0x(pinStructure xshutPin, pinStructure enPin, uint8_t address, SoftI2C* softI2C, uint8_t id)
{
    m_enPin = enPin;
    m_xshutPin = xshutPin;
    i2c::init_I2C(*softI2C, stdAdr);
    //pins::initFunc::pinInit(xshutPin);
    //GPIO_SetPin(xshutPin);
    m_dev.I2cDevAddr = sensor_default_address;
    m_dev.Present = 0;
    m_dev.Id = id;
    m_dev.softI2c = (void*) m_softI2C; 
 
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    #define CHECK_STATUS()     if (status != VL53L0X_ERROR_NONE) {     GPIO_ResetBits(m_xshutPin.GPIOx, m_xshutPin.pinNum);}

    uint32_t refSpadCount     = 0;
    uint8_t  isApertureSpads  = 0;
    uint8_t  VhvSettings      = 0;
    uint8_t  PhaseCal         = 0;
    
    GPIO_ResetBits(xshutPin.GPIOx, xshutPin.pinNum);
    time_service::delay_ms(100);
    GPIO_SetBits(xshutPin.GPIOx, xshutPin.pinNum);
    time_service::delay_ms(100);
    
    m_dev.I2cDevAddr = sensor_default_address;
    
    CHECK_STATUS();
   
    
    // дальше инициализация и калибровка, скопированная из примера
    CHECK_STATUS();

    status = VL53L0X_DataInit( &m_dev );
    CHECK_STATUS();

    status = VL53L0X_StaticInit( &m_dev );
    CHECK_STATUS();

    status = VL53L0X_PerformRefSpadManagement( &m_dev, &refSpadCount, &isApertureSpads);
    CHECK_STATUS();

    status = VL53L0X_PerformRefCalibration( &m_dev, &VhvSettings, &PhaseCal);
    CHECK_STATUS();

    status = VL53L0X_SetReferenceSpads( &m_dev, refSpadCount, isApertureSpads);
    CHECK_STATUS();

    status = VL53L0X_SetRefCalibration( &m_dev, VhvSettings, PhaseCal);
    CHECK_STATUS();

    status = VL53L0X_SetDeviceMode( &m_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    CHECK_STATUS();

    status = VL53L0X_SetLimitCheckValue( &m_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536) );
    CHECK_STATUS();

    status = VL53L0X_SetLimitCheckValue( &m_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536) );
    CHECK_STATUS();

    status =VL53L0X_SetMeasurementTimingBudgetMicroSeconds( &m_dev, 20000 );
    CHECK_STATUS();
    
    status = VL53L0X_SetDeviceAddress( &m_dev, address);
    m_dev.I2cDevAddr = address;
    m_address = address; 
    CHECK_STATUS();

    status = VL53L0X_StartMeasurement( &m_dev );
    m_numOfErrors = 0;
}

void vl53l0x::reInit()
{
    m_dev.I2cDevAddr = sensor_default_address;
 
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    #define CHECK_STATUS_FAIL()     if (status != VL53L0X_ERROR_NONE) UMBA_ASSERT_FAIL()

    uint32_t refSpadCount     = 0;
    uint8_t  isApertureSpads  = 0;
    uint8_t  VhvSettings      = 0;
    uint8_t  PhaseCal         = 0;
    
    m_dev.I2cDevAddr = sensor_default_address;
    
    CHECK_STATUS_FAIL();
    
    // дальше инициализация и калибровка, скопированная из примера
    CHECK_STATUS_FAIL();

    status = VL53L0X_DataInit( &m_dev );
    CHECK_STATUS_FAIL();

    status = VL53L0X_StaticInit( &m_dev );
    CHECK_STATUS_FAIL();

    status = VL53L0X_PerformRefSpadManagement( &m_dev, &refSpadCount, &isApertureSpads);
    CHECK_STATUS_FAIL();

    status = VL53L0X_PerformRefCalibration( &m_dev, &VhvSettings, &PhaseCal);
    CHECK_STATUS_FAIL();

    status = VL53L0X_SetReferenceSpads( &m_dev, refSpadCount, isApertureSpads);
    CHECK_STATUS_FAIL();

    status = VL53L0X_SetRefCalibration( &m_dev, VhvSettings, PhaseCal);
    CHECK_STATUS_FAIL();

    status = VL53L0X_SetDeviceMode( &m_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    CHECK_STATUS_FAIL();

    status = VL53L0X_SetLimitCheckValue( &m_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536) );
    CHECK_STATUS_FAIL();

    status = VL53L0X_SetLimitCheckValue( &m_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536) );
    CHECK_STATUS_FAIL();

    status =VL53L0X_SetMeasurementTimingBudgetMicroSeconds( &m_dev, 20000 );
    CHECK_STATUS_FAIL();
    
    status = VL53L0X_SetDeviceAddress( &m_dev, m_address);
    m_dev.I2cDevAddr = m_address;
    CHECK_STATUS_FAIL();

    status = VL53L0X_StartMeasurement( &m_dev );
}
/*
метод worker для ITask
*/
void vl53l0x::worker()
{
    uint8_t data_ready;
    VL53L0X_RangingMeasurementData_t measure;
    uint8_t status = VL53L0X_GetMeasurementDataReady(&m_dev, &data_ready);
    /*if (m_numOfErrors >= 10)
    {
        GPIO_ResetPin(m_enPin);
        m_numOfErrors++;
        if(m_numOfErrors >= 20)
        {
            GPIO_SetPin(m_enPin);      
            //reInit();
            m_numOfErrors = 0;
        }
        return;
    }*/
    if (status != VL53L0X_ERROR_NONE)
    {
        m_numOfErrors++;
        return;
        //m_data = 0;
    }

    status = VL53L0X_GetRangingMeasurementData(&m_dev, &measure);

    if (status != VL53L0X_ERROR_NONE)
    {
        m_numOfErrors++;
        return;
        //m_data = 0;
    }

    status = VL53L0X_ClearInterruptMask(&m_dev, 0);

    if (status != VL53L0X_ERROR_NONE)
    {
        m_numOfErrors++;
        return;
        //m_data = 0;
    }
    m_numOfErrors = 0;
    m_data = measure.RangeMilliMeter;
}
/*
функция возвращающая последнее значение дальномера:
Выходные данные: uint16_t значения в мм
*/
uint16_t vl53l0x::takeSens()
{
    return m_data;
    //return 0;
}
/*
диструктор чтоб ворнингов не было
*/
vl53l0x::~vl53l0x()
{    
}
