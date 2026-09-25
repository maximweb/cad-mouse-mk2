#pragma once

#include <Arduino.h>
#include <TLx493D_inc.hpp>
#include <Wire.h>

using namespace ifx::tlx493d;

// Bitmask returned by readRaw()/read(): 0 means all sensors okay, each bit flags one failed sensor.
enum HallSensorStatus : uint8_t {
    HALL_STATUS_OK = 0x00,
    HALL_STATUS_S1_FAIL = 0x01,
    HALL_STATUS_S2_FAIL = 0x02,
    HALL_STATUS_S3_FAIL = 0x04,
};

class HallSensorController {
public:
    HallSensorController(TwoWire& wire, uint8_t sensor1PowerPin, uint8_t sensor2PowerPin, uint8_t sensor3PowerPin, bool fullRange = true);

    void begin();

    /// @brief Reads raw 12-bit magnetic field values from all three sensors.
    /// @param out 9-element array filled as [s1x,s1y,s1z, s2x,s2y,s2z, s3x,s3y,s3z]; entries for a failed sensor are left untouched.
    /// @return HallSensorStatus bitmask, HALL_STATUS_OK if all sensors were read successfully.
    uint8_t readRaw(int16_t out[9]);

    /// @brief Reads magnetic field values from all three sensors, scaled to mT.
    /// @param out 9-element array filled as [s1x,s1y,s1z, s2x,s2y,s2z, s3x,s3y,s3z]; entries for a failed sensor are set to NAN.
    /// @return HallSensorStatus bitmask, HALL_STATUS_OK if all sensors were read successfully.
    uint8_t read(float out[9]);

    void printControlRegisters();
    void printControlRegisters(uint8_t sensorID);

    bool isInLowPowerMode();
    bool enterLowPowerMode();
    bool enterFastMode();

private:
    static void powerOff(uint8_t pin);
    static void powerOn(uint8_t pin);
    bool setPowerMode(TLx493D_PowerModeType_t mode);

    bool readSingleSensorRawFast(uint8_t sensorAddress, int16_t* outX, int16_t* outY, int16_t* outZ);

    bool m_in_low_power_mode;

    TLx493D_SensitivityType_t m_sensitivity;
    float m_scaleFactor;

    TwoWire& m_wire;

    uint8_t m_sensor1PowerPin;
    uint8_t m_sensor2PowerPin;
    uint8_t m_sensor3PowerPin;

    ifx::tlx493d::TLx493D_A2B6 m_sensor1;
    ifx::tlx493d::TLx493D_A2B6 m_sensor2;
    ifx::tlx493d::TLx493D_A2B6 m_sensor3;
};