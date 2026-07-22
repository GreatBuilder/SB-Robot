#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("MPU6050 ready");
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(&accel, &gyro, &temperature);

  float angleAroundX =
    atan2(
      accel.acceleration.y,
      accel.acceleration.z
    ) * 180.0 / PI;

  float angleAroundY =
    atan2(
      -accel.acceleration.x,
      accel.acceleration.z
    ) * 180.0 / PI;

  Serial.print("AX:");
  Serial.print(accel.acceleration.x, 2);

  Serial.print("\tAY:");
  Serial.print(accel.acceleration.y, 2);

  Serial.print("\tAZ:");
  Serial.print(accel.acceleration.z, 2);

  Serial.print("\tAngleX:");
  Serial.print(angleAroundX, 2);

  Serial.print("\tAngleY:");
  Serial.print(angleAroundY, 2);

  Serial.print("\tGX:");
  Serial.print(gyro.gyro.x * 180.0 / PI, 2);

  Serial.print("\tGY:");
  Serial.print(gyro.gyro.y * 180.0 / PI, 2);

  Serial.print("\tGZ:");
  Serial.println(gyro.gyro.z * 180.0 / PI, 2);

  delay(100);
}