// #include "IncludeManager.h"
#pragma once
#ifdef SENSORMODULE
    #include "Sensor.h"
    #include <SensirionI2cScd4x.h>
    #include <SensorSCD40.h>

    #define MEASURE_DELAY 5000

class SensorSCD41 : public SensorSCD40
{
  private:
    uint32_t mMeasureInterval = 30;
    uint32_t mMeasureDelay = 0;
    bool mIsMeasuring = false;

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    int16_t measureSingleShot(bool blocking);

  public:
    SensorSCD41(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorSCD41(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorSCD41() {}

    bool begin() override;
    void setMeasureInterval(uint32_t iMeasureInterval);
    std::string logPrefix() override;
};
#endif