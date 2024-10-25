/*
  Arduino LSM6DSOX - Simple Accelerometer

  This example reads the acceleration values from the LSM6DSOX
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 10 May 2021
  by Arturo Guadalupi

  This example code is in the public domain.
*/

#include <Adafruit_LSM6DSOX.h>
#include <string>
#include <TimeLib.h>
Adafruit_LSM6DSOX sensor1;
uint8_t sensor1_addr = 0x6A;
Adafruit_LSM6DSOX sensor2;
uint8_t sensor2_addr = 0x6B;
String comma = ",";
float ax1, ay1, az1, ax2, gx1, gy1, gz1, ay2, az2, gx2, gy2, gz2;
uint32_t time_start;

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  Serial.begin(115200);
  while (!Serial);

  while (!sensor1.begin_I2C(sensor1_addr)) {
    Serial.println("Failed to initialize 0x6A!");
    delay(10);
  }

  sensor1.setAccelDataRate(LSM6DS_RATE_416_HZ);
  Serial.print("Accelerometer sample rate = ");
  Serial.print(sensor1.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  while (!sensor2.begin_I2C(sensor2_addr)) {
    Serial.println("Failed to initialize 0x6B!");
    delay(10);
  }
  sensor2.setAccelDataRate(LSM6DS_RATE_416_HZ);

  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);

  time_start = millis();
}

void loop() {
  if (sensor1.accelerationAvailable())
      sensor1.readAcceleration(ax1, ay1, az1);

  if (sensor1.gyroscopeAvailable())
      sensor1.readGyroscope(gx1, gy1, gz1);

  if (sensor2.accelerationAvailable())
      sensor2.readAcceleration(ax2, ay2, az2);

  if (sensor2.gyroscopeAvailable())
      sensor2.readGyroscope(gx2, gy2, gz2);
    
  Serial.print((millis() - time_start) + comma + ax1 + comma + ay1 + comma + az1 + comma + ax2 + comma + ay2 + comma + az2 + comma);
  Serial.println(gx1 + comma + gy1 + comma + gz1 + comma + gx2 + comma + gy2 + comma + gz2);
  
}
