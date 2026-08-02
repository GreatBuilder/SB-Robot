#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =====================================================
// Sensors
// =====================================================

// Left AS5600 on Wire
MagneticSensorI2C leftEncoder =
  MagneticSensorI2C(AS5600_I2C);

// Right AS5600 on Wire1
MagneticSensorI2C rightEncoder =
  MagneticSensorI2C(AS5600_I2C);

// MPU6050 on Wire
Adafruit_MPU6050 mpu;

// =====================================================
// I2C scanner
// =====================================================

void scanBus(TwoWire& bus, const char* busName) {
  Serial.print("Scanning ");
  Serial.println(busName);

  int deviceCount = 0;

  for (uint8_t address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();

    if (error == 0) {
      Serial.print("  Found device at 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("  No devices found");
  }

  Serial.println();
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("Starting three-sensor test");

  // Start each I2C bus only once
  Wire.begin();
  Wire.setClock(100000);

  Wire1.begin();
  Wire1.setClock(100000);

  // Expected:
  // Wire  = 0x36 and 0x68
  // Wire1 = 0x36
  scanBus(Wire, "Wire");
  scanBus(Wire1, "Wire1");

  // Initialize left AS5600
  leftEncoder.init(&Wire);
  Serial.println("Left AS5600 initialized on Wire");

  // Initialize right AS5600
  rightEncoder.init(&Wire1);
  Serial.println("Right AS5600 initialized on Wire1");

  // Initialize MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found");

    while (true) {
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("MPU6050 initialized on Wire");
  Serial.println("All three sensors initialized");
  Serial.println();
}

// =====================================================
// Loop
// =====================================================

void loop() {
  // Update both motor encoders
  leftEncoder.update();
  rightEncoder.update();

  float leftAngleDegrees =
    leftEncoder.getAngle() * 180.0 / PI;

  float rightAngleDegrees =
    rightEncoder.getAngle() * 180.0 / PI;

  // Read MPU6050
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;

  bool mpuReadSuccess =
    mpu.getEvent(&accel, &gyro, &temperature);

  if (!mpuReadSuccess) {
    Serial.println("MPU6050 read failed");
    delay(100);
    return;
  }

  float angleX =
    atan2(
      accel.acceleration.y,
      accel.acceleration.z
    ) * 180.0 / PI;

  // Print readings
  Serial.print("LeftAngle:");
  Serial.print(leftAngleDegrees, 2);

  Serial.print("\tRightAngle:");
  Serial.print(rightAngleDegrees, 2);

  Serial.print("\tAY:");
  Serial.print(accel.acceleration.y, 2);

  Serial.print("\tAngleX:");
  Serial.println(angleX, 2);

  delay(100);
}