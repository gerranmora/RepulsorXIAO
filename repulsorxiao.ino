/*
 * XIAO nRF52840 Sense - LSM6DS3 Accelerometer
 * Board: Seeed nRF52 Boards > Seeed XIAO nRF52840 Sense (NON-MBED)
 * 
 * Install library: Seeed Arduino LSM6DS3 (from Library Manager)
 */

#include <LSM6DS3.h>
#include <Wire.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("XIAO nRF52840 Sense - Accelerometer Test");
  Serial.println("=========================================");
  
  Serial.println("Initializing LSM6DS3...");
  
  if (myIMU.begin() != 0) {
    Serial.println("ERROR: Failed to initialize LSM6DS3!");
    Serial.println("Check:");
    Serial.println("- Board: 'Seeed nRF52 Boards' (NON-mbed)");
    Serial.println("- Library: 'Seeed Arduino LSM6DS3' installed");
    while(1) delay(1000);
  }
  
  Serial.println("LSM6DS3 initialized successfully!");
  Serial.println("\nX | Y | Z | Magnitude");
  Serial.println("========================");
  delay(1000);
}

void loop() {
  float x = myIMU.readFloatAccelX();
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();
  float mag = sqrt(x*x + y*y + z*z);
  
  Serial.print(x, 3);
  Serial.print(" | ");
  Serial.print(y, 3);
  Serial.print(" | ");
  Serial.print(z, 3);
  Serial.print(" | ");
  Serial.print(mag, 3);
  
  if (mag > 1.2) {
    Serial.print(" <<< MOTION");
  }
  Serial.println();
  
  delay(100);
}