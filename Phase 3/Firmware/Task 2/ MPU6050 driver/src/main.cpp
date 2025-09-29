#include <Arduino.h>
#include "MPU.h"

MPU mpu;

void setup()
{
  Serial.begin(115200);

  mpu.setup(0x68);

  // choosing the range
  mpu.configure_acc(0);
  mpu.configure_gyro(0);

  delay(100); // better safe than sorry

  Serial.println("Pitch\tRoll");
}

void loop()
{

  float pitch = mpu.get_pitch();
  float roll = mpu.get_roll();

  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);

  delay(50);
}