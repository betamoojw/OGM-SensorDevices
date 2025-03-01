// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSCD41.h"
    #include <Wire.h>

SensorSCD41::SensorSCD41(uint16_t iMeasureTypes, TwoWire* iWire)
    : SensorSCD40(iMeasureTypes, iWire, SCD40_I2C_ADDR){};

SensorSCD41::SensorSCD41(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : SensorSCD40(iMeasureTypes, iWire, iAddress){};

uint8_t SensorSCD41::getSensorClass()
{
    return SENS_SCD41;
}

std::string SensorSCD41::logPrefix()
{
    return "Sensor<SCD41>";
}

void SensorSCD41::setMeasureInterval(uint32_t iMeasureInterval)
{
    mMeasureInterval = MAX(iMeasureInterval, 5);
}

static uint8_t communication_buffer[9] = {0};

int16_t SensorSCD41::measureSingleShot(bool blocking) {
    int16_t localError = 0; //NO_ERROR
    uint8_t* buffer_ptr = communication_buffer;
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x219d, buffer_ptr, 2);
    localError =
        SensirionI2CCommunication::sendFrame(SCD40_I2C_ADDR, txFrame, *Sensor::pWire);
    if (localError != 0) {
        return localError;
    }
    if (blocking) delay(5000);
    return localError;
}



void SensorSCD41::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            // check for standard intervals
            if (mMeasureInterval == 5)
                startPeriodicMeasurement();
            else if (mMeasureInterval == 30)
                startLowPowerPeriodicMeasurement();
            else
            {
                // set ASC initial period according to data sheet:
                // 44 h for 5 min. intervals, scaled inversely proportional for other intervals
                uint16_t lAscInitial = 44 * (300 / mMeasureInterval);

                // it has to be dividable by 4
                lAscInitial = (lAscInitial + 3) & ~(decltype(lAscInitial))3;

                setAutomaticSelfCalibrationInitialPeriod(lAscInitial);

                // set ASC standard period according to data sheet:
                // 156 h for 5 min. intervals, scaled inversely proportional for other intervals
                uint16_t lAscStandard = 156 * (300 / mMeasureInterval);

                // it has to be dividable by 4
                lAscStandard = (lAscStandard + 3) & ~(decltype(lAscStandard))3;

                setAutomaticSelfCalibrationStandardPeriod(lAscStandard);
            }

            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            // if standard intervals used, we wait for first data
            if (mMeasureInterval == 5 || mMeasureInterval == 30)
            {
                // we ask for value until we get a valid value
                if (delayCheck(pSensorStateDelay, 2000))
                {
                    if (getSensorData())
                        pSensorState = Running;
                    pSensorStateDelay = delayTimerInit();
                }
            }
            else
                pSensorState = Running;

            break;
        case Running:
            if (mMeasureInterval == 5 || mMeasureInterval == 30)
            {
                // if standard intervals used, we just need to get data at defined intervals
                if (delayCheck(pSensorStateDelay, mMeasureInterval * 1000))
                {
                    getSensorData();
                    pSensorStateDelay = delayTimerInit();
                }
            }
            else
            {
                // for other intervals, measurement is triggered manually
                if (mIsMeasuring)
                {
                    if (delayCheck(mMeasureDelay, MEASURE_DELAY))
                    {
                        getSensorData();
                        pSensorStateDelay = delayTimerInit();
                        mIsMeasuring = false;
                    }
                }
                if (delayCheck(pSensorStateDelay, mMeasureInterval * 1000))
                {
                    measureSingleShot(false);
                    pSensorStateDelay = delayTimerInit();
                    mMeasureDelay = delayTimerInit();
                    mIsMeasuring = true;
                }
            }

            break;
        default:
            pSensorStateDelay = delayTimerInit();
            break;
    }
}

bool SensorSCD41::begin()
{
    logDebugP("Starting sensor SCD41... ");
    return beginInternal();
}
#endif