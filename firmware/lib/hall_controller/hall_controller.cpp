#include "config.h"

#include "hall_controller.h"

HallSensorController::HallSensorController(TwoWire& wire, uint8_t sensor1PowerPin, uint8_t sensor2PowerPin, uint8_t sensor3PowerPin, bool fullRange)
: m_wire(wire)
, m_sensor1PowerPin(sensor1PowerPin)
, m_sensor2PowerPin(sensor2PowerPin)
, m_sensor3PowerPin(sensor3PowerPin)
, m_sensitivity(fullRange ? TLx493D_FULL_RANGE_e : TLx493D_SHORT_RANGE_e)
, m_scaleFactor(fullRange ? 7.7f : 15.4f)
, m_sensor1(m_wire, TLx493D_IIC_ADDR_A0_e)
, m_sensor2(m_wire, TLx493D_IIC_ADDR_A0_e)
, m_sensor3(m_wire, TLx493D_IIC_ADDR_A0_e)
{
}

void HallSensorController::begin()
{
    // Power pin configuration
    pinMode(m_sensor1PowerPin, OUTPUT);
    pinMode(m_sensor2PowerPin, OUTPUT);
    pinMode(m_sensor3PowerPin, OUTPUT);

    // All pins of initially set to LOW to power off the sensors
    powerOff(m_sensor1PowerPin);
    powerOff(m_sensor2PowerPin);
    powerOff(m_sensor3PowerPin);
    delay(5); // Wait for the sensors to power down

    // Initialize I2C communication
    m_wire.begin();
    // m_wire.setClock(800000); // Datasheet recommends >= 800kHz for fast mode.
    m_wire.setClock(1000000); // Set I2C clock to 1 MHz (max) for faster communication

    // Power up and initialize sensors one by one to get unique I2C addresses
    powerOn(m_sensor1PowerPin);
    m_sensor1.init(true, false, false, true);
    m_sensor1.setIICAddress(TLx493D_IIC_ADDR_A2_e);
    m_sensor1.setPowerMode(TLx493D_FAST_MODE_e);
    m_sensor1.setSensitivity(m_sensitivity);
    m_sensor1.setMeasurement(TLx493D_BxByBz_e); // Temperature is unused; measure magnetic axes only.

    powerOn(m_sensor2PowerPin);
    m_sensor2.init(true, false, false, true);
    m_sensor2.setIICAddress(TLx493D_IIC_ADDR_A1_e);
    m_sensor2.setPowerMode(TLx493D_FAST_MODE_e);
    m_sensor2.setSensitivity(m_sensitivity);
    m_sensor2.setMeasurement(TLx493D_BxByBz_e);

    powerOn(m_sensor3PowerPin);
    m_sensor3.init(true, false, false, true);
    m_sensor3.setIICAddress(TLx493D_IIC_ADDR_A0_e);
    m_sensor3.setPowerMode(TLx493D_FAST_MODE_e);
    m_sensor3.setSensitivity(m_sensitivity);
    m_sensor3.setMeasurement(TLx493D_BxByBz_e);

    delay(10); // Wait for all sensors to stabilize

    m_in_low_power_mode = false;
}

bool HallSensorController::isInLowPowerMode()
{
    return m_in_low_power_mode;
}

bool HallSensorController::enterLowPowerMode()
{
    if (!m_in_low_power_mode) {
        m_in_low_power_mode = setPowerMode(TLx493D_LOW_POWER_MODE_e);

    }
    return m_in_low_power_mode;
}

bool HallSensorController::enterFastMode()
{
    if (m_in_low_power_mode) {
        m_in_low_power_mode = !setPowerMode(TLx493D_FAST_MODE_e);
    }
    return m_in_low_power_mode;
}

bool HallSensorController::setPowerMode(TLx493D_PowerModeType_t mode)
{
    // Attempt all three writes even if an earlier sensor fails.
    const bool s1 = m_sensor1.setPowerMode(mode);
    const bool s2 = m_sensor2.setPowerMode(mode);
    const bool s3 = m_sensor3.setPowerMode(mode);
    return s1 && s2 && s3;
}

void HallSensorController::powerOff(uint8_t pin)
{
    digitalWrite(pin, LOW);
}

void HallSensorController::powerOn(uint8_t pin)
{
    digitalWrite(pin, HIGH);
    delay(5);
}

uint8_t HallSensorController::readRaw(int16_t out[9])
{
    // TODO: Original code uses getRawMagneticFieldAndTemperature,
    // but don't know why temperature is needed.
    // For now, just read magnetic field values.

    // Original read functions are slow as they read all registers.
    // Hence, implement a faster read function that reads only the necessary registers
    // still checking parity.

    uint8_t status = HALL_STATUS_OK;
    int16_t x = INT16_MIN, y = INT16_MIN, z = INT16_MIN;

    // Attempt to read sensor 1, only write to out if successful
    if (readSingleSensorRawFast(m_sensor1.getI2CAddress() >> 1, &x, &y, &z)) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }
    else {
        status |= HALL_STATUS_S1_FAIL;
    }

    // Reset
    x = INT16_MIN;
    y = INT16_MIN;
    z = INT16_MIN;

    // Attempt to read sensor 2, only write to out if successful
    if (readSingleSensorRawFast(m_sensor2.getI2CAddress() >> 1, &x, &y, &z)) {
        out[3] = x;
        out[4] = y;
        out[5] = z;
    }
    else {
        status |= HALL_STATUS_S2_FAIL;
    }

    // Reset
    x = INT16_MIN;
    y = INT16_MIN;
    z = INT16_MIN;

    // Attempt to read sensor 3, only write to out if successful
    if (readSingleSensorRawFast(m_sensor3.getI2CAddress() >> 1, &x, &y, &z)) {
        out[6] = x;
        out[7] = y;
        out[8] = z;
    }
    else {
        status |= HALL_STATUS_S3_FAIL;
    }

    return status;
}

uint8_t HallSensorController::read(float out[9])
{
    int16_t raw[9];
    const uint8_t status = readRaw(raw);

    // Per sensor: convert to float on success, NaN on failure, guided by the readRaw status mask.
    for (int sensorIdx = 0; sensorIdx < 3; ++sensorIdx) {
        const uint8_t failMask = 1u << sensorIdx;
        const int base = sensorIdx * 3;
        if (status & failMask) {
            out[base] = NAN;
            out[base + 1] = NAN;
            out[base + 2] = NAN;
        }
        else {
            out[base] = static_cast<float>(raw[base]) / m_scaleFactor;
            out[base + 1] = static_cast<float>(raw[base + 1]) / m_scaleFactor;
            out[base + 2] = static_cast<float>(raw[base + 2]) / m_scaleFactor;
        }
    }

    return status;
}

bool HallSensorController::readSingleSensorRawFast(uint8_t sensorAddress, int16_t* outX, int16_t* outY, int16_t* outZ)
{
    // 1-Byte read mode, request data directly from address without sending a register address first

    // Request first 7 bytes (data and diagnostics)
    if (m_wire.requestFrom(sensorAddress, static_cast<size_t>(7)) != 7)
        return false;

    // Safeguarded by setting to zero, despite not ever writing back
    // to outX, outY, outZ if read or parity fails
    uint8_t b[7] = {0};

    // Read bytes
    for (int i = 0; i < 7; ++i) {
        b[i] = m_wire.read();
    }

    // 1. XOR sum only the data registers (0x00 through 0x05)
    uint8_t parityCheck = 0;
    for (int i = 0; i < 6; ++i) {
        parityCheck ^= b[i];
    }

    // 2. Clear out the fuse/config flags from register 0x06,
    // leaving ONLY Bit 7 (Bus Parity) to add to the sum
    parityCheck ^= (b[6] & 0x80);

    // 3. Rapid bit-shift reduction to get the final 1-bit parity status
    parityCheck ^= parityCheck >> 4;
    parityCheck ^= parityCheck >> 2;
    parityCheck ^= parityCheck >> 1;

    // 4. Per your datasheet, the total parity must be ODD (lowest bit must be 1)
    if ((parityCheck & 0x01) == 0) {
        return false; // I2C transmission error detected! Drop the packet.
    }

    // 5. Parse the valid 12-bit signed data
    *outX = static_cast<int16_t>((b[0] << 8) | (b[4] & 0xF0)) >> 4;
    *outY = static_cast<int16_t>((b[1] << 8) | ((b[4] & 0x0F) << 4)) >> 4;
    *outZ = static_cast<int16_t>((b[2] << 8) | ((b[5] & 0x0F) << 4)) >> 4;

    return true;
}

void HallSensorController::printControlRegisters(uint8_t sensorID)
{
    ifx::tlx493d::TLx493D_A2B6* sensor = nullptr;
    if (sensorID == 1) {
        sensor = &m_sensor1;
    }
    else if (sensorID == 2) {
        sensor = &m_sensor2;
    }
    else if (sensorID == 3) {
        sensor = &m_sensor3;
    }
    else {
        Serial.println("Invalid sensor ID. Please provide a value between 1 and 3.");
    }
    if (sensor) {
        sensor->readRegisters();
        const uint8_t* map = sensor->getRegisterMap();
        const auto size = sensor->getRegisterMapSize();

        // Print 10h, 11h and 13h registers in binary format
        for (size_t i = 0; i < size; ++i) {
            if (i == 0x10 || i == 0x11 || i == 0x13) {
                Serial.print("Register 0x");
                Serial.print(i, HEX);
                Serial.print(": ");
                for (int bit = 7; bit >= 0; --bit) {
                    Serial.print((map[i] >> bit) & 1);
                }
                Serial.println();
            }
        }
    }
}

void HallSensorController::printControlRegisters()
{
    Serial.println("Sensor 1 Control Registers:");
    printControlRegisters(1);
    Serial.println("Sensor 2 Control Registers:");
    printControlRegisters(2);
    Serial.println("Sensor 3 Control Registers:");
    printControlRegisters(3);
}