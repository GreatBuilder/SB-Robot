#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =====================================================
// Encoders
// =====================================================

MagneticSensorI2C leftEncoder =
  MagneticSensorI2C(AS5600_I2C);

MagneticSensorI2C rightEncoder =
  MagneticSensorI2C(AS5600_I2C);

// =====================================================
// Motors and drivers
// =====================================================

BLDCMotor leftMotor = BLDCMotor(7);
BLDCDriver3PWM leftDriver =
  BLDCDriver3PWM(9, 10, 11, 12);

BLDCMotor rightMotor = BLDCMotor(7);
BLDCDriver3PWM rightDriver =
  BLDCDriver3PWM(3, 5, 6, 7);

// =====================================================
// MPU6050
// =====================================================

Adafruit_MPU6050 mpu;

// =====================================================
// Balance settings
// =====================================================

// Start gently
float balanceKp = 0.20;
float balanceKd = 0.015;

// Maximum requested wheel velocity
float maxMotorSpeed = 4.0;

// Stop balancing past this angle
float fallAngle = 30.0;

// Upright calibration values
float uprightAngle = 0.0;
float gyroOffsetX = 0.0;

// Filtered robot angle
float filteredAngle = 0.0;

// Timing
unsigned long lastBalanceMicros = 0;
unsigned long lastPrintMillis = 0;

void stopMotors() {
  leftMotor.target = 0.0;
  rightMotor.target = 0.0;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  SimpleFOCDebug::enable(&Serial);

  // =====================================================
  // I2C buses
  // =====================================================

  // Left encoder and MPU6050
  Wire.begin();
  Wire.setClock(100000);

  // Right encoder
  Wire1.begin();
  Wire1.setClock(100000);

  leftEncoder.init(&Wire);
  rightEncoder.init(&Wire1);

  leftMotor.linkSensor(&leftEncoder);
  rightMotor.linkSensor(&rightEncoder);

  // =====================================================
  // MPU6050
  // =====================================================

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // =====================================================
  // Drivers
  // =====================================================

  leftDriver.voltage_power_supply = 11.0;

  if (!leftDriver.init()) {
    Serial.println("Left driver failed");
    while (1);
  }

  leftMotor.linkDriver(&leftDriver);

  rightDriver.voltage_power_supply = 11.0;

  if (!rightDriver.init()) {
    Serial.println("Right driver failed");
    while (1);
  }

  rightMotor.linkDriver(&rightDriver);

  // =====================================================
  // Same working closed-loop motor settings
  // =====================================================

  leftMotor.controller = MotionControlType::velocity;
  leftMotor.torque_controller = TorqueControlType::voltage;

  leftMotor.voltage_limit = 1.0;
  leftMotor.voltage_sensor_align = 1.0;
  leftMotor.velocity_limit = 15.0;

  leftMotor.PID_velocity.P = 0.15;
  leftMotor.PID_velocity.I = 0.5;
  leftMotor.PID_velocity.D = 0.0;
  leftMotor.LPF_velocity.Tf = 0.02;

  rightMotor.controller = MotionControlType::velocity;
  rightMotor.torque_controller = TorqueControlType::voltage;

  rightMotor.voltage_limit = 1.0;
  rightMotor.voltage_sensor_align = 1.0;
  rightMotor.velocity_limit = 15.0;

  rightMotor.PID_velocity.P = 0.15;
  rightMotor.PID_velocity.I = 0.5;
  rightMotor.PID_velocity.D = 0.0;
  rightMotor.LPF_velocity.Tf = 0.02;

  // =====================================================
  // Motor initialization
  // =====================================================

  if (!leftMotor.init()) {
    Serial.println("Left motor init failed");
    while (1);
  }

  if (!rightMotor.init()) {
    Serial.println("Right motor init failed");
    while (1);
  }

  Serial.println("Aligning left motor");

  if (!leftMotor.initFOC()) {
    Serial.println("Left FOC failed");
    leftDriver.disable();
    rightDriver.disable();
    while (1);
  }

  Serial.println("Aligning right motor");

  if (!rightMotor.initFOC()) {
    Serial.println("Right FOC failed");
    leftDriver.disable();
    rightDriver.disable();
    while (1);
  }

  // =====================================================
  // Upright calibration
  // Hold the robot completely upright and motionless
  // =====================================================

  Serial.println("Hold robot upright and still");
  delay(2000);

  float angleSum = 0.0;
  float gyroSum = 0.0;

  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&accel, &gyro, &temperature);

    float angleX =
      atan2(
        accel.acceleration.y,
        accel.acceleration.z
      ) * 180.0 / PI;

    angleSum += angleX;
    gyroSum += gyro.gyro.x;

    delay(4);
  }

  uprightAngle = angleSum / samples;
  gyroOffsetX = gyroSum / samples;
  filteredAngle = uprightAngle;

  lastBalanceMicros = micros();

  Serial.print("Upright angle: ");
  Serial.println(uprightAngle, 2);

  Serial.println("Balance control started");
}

void loop() {
  // Must run continuously
  leftMotor.loopFOC();
  rightMotor.loopFOC();

  unsigned long now = micros();

  // Balance update at 100 Hz
  if (now - lastBalanceMicros >= 10000) {
    float dt =
      (now - lastBalanceMicros) / 1000000.0;

    lastBalanceMicros = now;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&accel, &gyro, &temperature);

    float AY = accel.acceleration.y;
    float AZ = accel.acceleration.z;

    // Positive forward, approximately zero upright,
    // negative backward for your sensor orientation
    float angleX =
      atan2(AY, AZ) * 180.0 / PI;

    // Gyroscope rotation rate around X axis
    float gyroRateX =
      (gyro.gyro.x - gyroOffsetX) *
      180.0 / PI;

    bool validReading =
      isfinite(angleX) &&
      isfinite(gyroRateX) &&
      fabs(angleX) <= 180.0 &&
      fabs(gyroRateX) <= 550.0 &&
      dt > 0.001 &&
      dt < 0.05;

    if (!validReading) {
      stopMotors();
      filteredAngle = angleX;
    } else {
      // Combine fast gyro response with stable accelerometer angle
      filteredAngle =
        0.98 * (filteredAngle + gyroRateX * dt)
        + 0.02 * angleX;

      float angleError =
        filteredAngle - uprightAngle;

      float motorCommand = 0.0;

      if (fabs(angleError) < fallAngle) {
        motorCommand =
          balanceKp * angleError +
          balanceKd * gyroRateX;

        motorCommand =
          constrain(
            motorCommand,
            -maxMotorSpeed,
            maxMotorSpeed
          );
      }

      // Motors are physically mirrored
      leftMotor.target = -motorCommand;
      rightMotor.target = motorCommand;

      if (millis() - lastPrintMillis >= 100) {
        lastPrintMillis = millis();

        Serial.print("AY:");
        Serial.print(AY, 2);

        Serial.print("\tAngleX:");
        Serial.print(angleX, 2);

        Serial.print("\tFiltered:");
        Serial.print(filteredAngle, 2);

        Serial.print("\tError:");
        Serial.print(angleError, 2);

        Serial.print("\tGX:");
        Serial.print(gyroRateX, 2);

        Serial.print("\tMotor:");
        Serial.print(motorCommand, 2);

        Serial.print("\tStatus:");
        Serial.println(
          fabs(angleError) < fallAngle
            ? "ACTIVE"
            : "FALLEN"
        );
      }
    }
  }

  leftMotor.move();
  rightMotor.move();
}