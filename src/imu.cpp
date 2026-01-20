/*#include <Wire.h>
#include <Arduino.h>
const int MPU_ADDR = 0x68;  // I2C address of MPU6050

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);       // Your forced GPIO pins
  Wire.setClock(100000);

  // Wake up the MPU6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);       // PWR_MGMT_1 register
  Wire.write(0);          // Set to 0 to wake up
  Wire.endTransmission();
}

void loop() {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Read 14 bytes: AccX/Y/Z, Temp, GyX/Y/Z

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  Serial.print("Acc: ");
  Serial.print(AcX); Serial.print(", ");
  Serial.print(AcY); Serial.print(", ");
  Serial.println(AcZ);

  Serial.print("Gyro: ");
  Serial.print(GyX); Serial.print(", ");
  Serial.print(GyY); Serial.print(", ");
  Serial.println(GyZ);

  delay(500);
}
*/