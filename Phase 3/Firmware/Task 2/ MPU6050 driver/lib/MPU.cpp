#include "MPU.h"
#include <Arduino.h>
#include <math.h>

#define MPU_I2C_FREQUENCY 400000
#define RAD_TO_DEG 57.2957795131

void MPU::setup(const uint8_t address)
{
    this->wire = wire;
    this->address = address;
    this->gyroScale = 131.0;  // default for ±250°/s which is the default range
    this->accScale = 16384.0; // default for ±2g which is the default range
    wire->begin();
    wire->setClock(MPU_I2C_FREQUENCY);
}

acc MPU::get_acc()
{
    wire->beginTransmission(address);
    wire->write(0x3B);
    wire->endTransmission(false);

    acc values;
    wire->requestFrom(address, 6);
    int16_t X = (wire->read() << 8) | wire->read();
    int16_t Y = (wire->read() << 8) | wire->read();
    int16_t Z = (wire->read() << 8) | wire->read();

    values.x = X / accScale;
    values.y = Y / accScale;
    values.z = Z / accScale;

    return values;
}

gyro MPU::get_gyro()
{
    wire->beginTransmission(address);
    wire->write(0x43);
    wire->endTransmission(false);

    gyro values;
    wire->requestFrom(address, 6);
    int16_t X = (wire->read() << 8) | wire->read();
    int16_t Y = (wire->read() << 8) | wire->read();
    int16_t Z = (wire->read() << 8) | wire->read();

    values.x = X / gyroScale;
    values.y = Y / gyroScale;
    values.z = Z / gyroScale;

    return values;
}

void MPU::configure_gyro(const uint8_t range)
{
    if (range != 0 && range != 1 && range != 2 && range != 3)
    {
        return;
    }

    wire->beginTransmission(address);
    wire->write(0x1B);
    if (range == 0)
        wire->write(0);
    else if (range == 1)
        wire->write(0 | (1 << 3));
    else if (range == 2)
        wire->write(0 | (1 << 4));
    else if (range == 3)
        wire->write(0 | (1 << 4) | (1 << 3));
    wire->endTransmission(true);

    // for each range the scale changes accordingly and same for acc in next function
    switch (range)
    {
    case 0:
        gyroScale = 131.0;
        break;
    case 1:
        gyroScale = 65.5;
        break;
    case 2:
        gyroScale = 32.8;
        break;
    case 3:
        gyroScale = 16.4;
        break;
    }
}

void MPU::configure_acc(const uint8_t range)
{
    if (range != 0 && range != 1 && range != 2 && range != 3)
    {
        return;
    }

    wire->beginTransmission(address);
    wire->write(0x1C);
    if (range == 0)
        wire->write(0);
    else if (range == 1)
        wire->write(0 | (1 << 3));
    else if (range == 2)
        wire->write(0 | (1 << 4));
    else if (range == 3)
        wire->write(0 | (1 << 4) | (1 << 3));
    wire->endTransmission(true);

    switch (range)
    {
    case 0:
        accScale = 16384.0;
        break;
    case 1:
        accScale = 8192.0;
        break;
    case 2:
        accScale = 4096.0;
        break;
    case 3:
        accScale = 2048.0;
        break;
    }
}

float MPU::get_pitch()
{
    acc acceleration = get_acc();

    float pitch = atan2(-acceleration.x, sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z));

    return pitch * RAD_TO_DEG;
}

float MPU::get_roll()
{
    acc acceleration = get_acc();

    float roll = atan2(acceleration.y, acceleration.z);

    return roll * RAD_TO_DEG;
}