/*
 * Base test for Seeed XIAO nRF52840 Sense
 */

#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <LSM6DS3.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("XIAO nRF52840 Sense base test");
  if (myIMU.begin() != 0) {
    Serial.println("Accelerometer not detected");
    while (1);
  }
  Serial.println("Accelerometer ready");
}

void loop() {
  float x = myIMU.readFloatAccelX();
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();
  Serial.print(x, 2); Serial.print(" | ");
  Serial.print(y, 2); Serial.print(" | ");
  Serial.println(z, 2);
  delay(500);
}
