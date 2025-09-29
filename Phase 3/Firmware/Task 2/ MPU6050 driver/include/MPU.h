#ifndef MPU_H
#define MPU_H

#include <Wire.h>

typedef struct gyro
{
    float x;
    float y;
    float z;
} gyro;

typedef struct acc
{
    float x;
    float y;
    float z;
} acc;

class MPU
{
private:
    TwoWire *wire = &Wire;
    uint8_t address;
    float gyroScale;
    float accScale;

public:
    void setup(const uint8_t address);

    void configure_gyro(const uint8_t range);
    void configure_acc(const uint8_t range);
    acc get_acc();
    gyro get_gyro();

    float get_pitch();
    float get_roll();
};

#endif