#define DEBUG_SERIAL Serial
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C_CLOCK 100000 // 100KHz

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

float balanceKp = 0.80;
float balanceKd = 0.04;

float maxMotorSpeed = 8.0;
float fallAngle = 20.0;

float uprightAngle = 0.0;
float gyroOffsetX = 0.0;
float filteredAngle = 0.0;

unsigned long lastBalanceMicros = 0;
unsigned long lastPrintMillis = 0;

// =====================================================
// Helper functions
// =====================================================

void stopMotors() 
{
  leftMotor.target = 0.0;
  rightMotor.target = 0.0;
}

bool devicePresent(TwoWire& bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void recoverMainI2CBus() {
  Wire.end();

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, HIGH);

  // Clock pulses can release a device stuck during a transaction
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);

    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }

  // Generate an I2C STOP condition
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);

  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);

  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(I2C_CLOCK);
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  SimpleFOCDebug::enable(&Serial);

  // -----------------------------------------------------
  // Start both I2C buses once
  // -----------------------------------------------------

  recoverMainI2CBus();

  Wire1.begin();
  Wire1.setClock(I2C_CLOCK);

  delay(100);

  Serial.print("SDA idle state: ");
  Serial.println(digitalRead(SDA));

  Serial.print("SCL idle state: ");
  Serial.println(digitalRead(SCL));

  // -----------------------------------------------------
  // Verify all sensors before continuing
  // -----------------------------------------------------

  Serial.println("Checking sensors...");

  if (!devicePresent(Wire, 0x36)) {
    Serial.println("Left AS5600 missing");
    // while (1);
  }

  if (!devicePresent(Wire, 0x68)) {
    Serial.println("MPU6050 missing");
    // while (1);
  }

  if (!devicePresent(Wire1, 0x36)) {
    Serial.println("Right AS5600 missing");
    // while (1);
  }

  Serial.println("All sensors detected");

  // -----------------------------------------------------
  // Initialize MPU6050 first
  // -----------------------------------------------------

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 initialization failed");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 initialized");

  // -----------------------------------------------------
  // Initialize encoders
  // -----------------------------------------------------

  leftEncoder.init(&Wire);
  rightEncoder.init(&Wire1);

  leftMotor.linkSensor(&leftEncoder);
  rightMotor.linkSensor(&rightEncoder);

  Serial.println("Encoders initialized");

  // -----------------------------------------------------
  // Left driver
  // -----------------------------------------------------

  leftDriver.voltage_power_supply = 11.0;

  if (!leftDriver.init()) {
    Serial.println("Left driver failed");
    while (1);
  }

  // Keep driver outputs disabled until we are ready to run FOC
  leftDriver.disable();

  leftMotor.linkDriver(&leftDriver);

  // -----------------------------------------------------
  // Right driver
  // -----------------------------------------------------

  rightDriver.voltage_power_supply = 11.0;

  if (!rightDriver.init()) {
    Serial.println("Right driver failed");
    while (1);
  }

  // Keep driver outputs disabled until we are ready to run FOC
  rightDriver.disable();

  rightMotor.linkDriver(&rightDriver);

  // -----------------------------------------------------
  // Left motor settings
  // -----------------------------------------------------

  leftMotor.controller = MotionControlType::velocity;
  leftMotor.torque_controller = TorqueControlType::voltage;

  leftMotor.voltage_limit = 3.0;
  leftMotor.voltage_sensor_align = 3.0;
  leftMotor.velocity_limit = 15.0;

  leftMotor.PID_velocity.P = 0.15;
  leftMotor.PID_velocity.I = 0.5;
  leftMotor.PID_velocity.D = 0.0;
  leftMotor.LPF_velocity.Tf = 0.02;

  // -----------------------------------------------------
  // Right motor settings
  // -----------------------------------------------------

  rightMotor.controller = MotionControlType::velocity;
  rightMotor.torque_controller = TorqueControlType::voltage;

  rightMotor.voltage_limit = 3.0;
  rightMotor.voltage_sensor_align = 3.0;
  rightMotor.velocity_limit = 15.0;

  rightMotor.PID_velocity.P = 0.15;
  rightMotor.PID_velocity.I = 0.5;
  rightMotor.PID_velocity.D = 0.0;
  rightMotor.LPF_velocity.Tf = 0.02;

  // -----------------------------------------------------
  // Initialize motors
  // -----------------------------------------------------

  if (!leftMotor.init())
  {
    Serial.println("Left motor init failed");
    // while (1);
  }

  if (!rightMotor.init()) {
    Serial.println("Right motor init failed");
    // while (1);
  }

  // -----------------------------------------------------
  // FOC alignment
  // -----------------------------------------------------

  Serial.println("Aligning left motor");

  // Enable driver shortly before running alignment so PWM doesn't start earlier
  leftDriver.enable();

  if (!leftMotor.initFOC()) 
  {
    Serial.println("Left FOC failed");

    leftDriver.disable();
    rightDriver.disable();

    // while (1);
  }

  Serial.println("Aligning right motor");

  if (!rightMotor.initFOC()) {
    Serial.println("Right FOC failed");

    leftDriver.disable();
    rightDriver.disable();

    // while (1);
  }

  // -----------------------------------------------------
  // Upright MPU6050 calibration
  // -----------------------------------------------------

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

// =====================================================
// Main loop
// =====================================================

void loop() {
  // Run FOC as frequently as possible
  leftMotor.loopFOC();
  delayMicroseconds(100);
  rightMotor.loopFOC();

  unsigned long now = micros();

  // Balance update at 100 Hz
  if (now - lastBalanceMicros >= 10000) 
  {
    float dt = (now - lastBalanceMicros) / 10000.0;

    lastBalanceMicros = now;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&accel, &gyro, &temperature);
    delayMicroseconds(100);

    float AY = accel.acceleration.y;
    float AZ = accel.acceleration.z;

    // Positive forward, zero near upright, negative backward
    float angleX =
      atan2(AY, AZ) * 180.0 / PI;

    float gyroRateX =
      (gyro.gyro.x - gyroOffsetX) *
      180.0 / PI;

    bool validReading = true;
//      isfinite(angleX) &&
//      isfinite(gyroRateX) &&
//      fabs(angleX) <= 180.0 &&
//      fabs(gyroRateX) <= 550.0 &&
//      dt > 0.001 &&
//      dt < 0.05;

    if (!validReading) {
      stopMotors();

      // Do not integrate a corrupted gyro reading
      if (isfinite(angleX)) {
        filteredAngle = angleX;
      }

      if (false) // if (millis() - lastPrintMillis >= 100) 
      {
        lastPrintMillis = millis();

        Serial.print("Invalid MPU reading");
        Serial.print("\tAngleX:");
        Serial.print(angleX, 2);
        Serial.print("\tGX:");
        Serial.println(gyroRateX, 2);
      }
    } else {
      // Complementary filter
      filteredAngle =
        0.98 * (filteredAngle + gyroRateX * dt) +
        0.02 * angleX;

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
      } else {
        motorCommand = 0.0;

        // Prevent the filter from staying at a huge angle
        filteredAngle = angleX;
      }

      // Motors face opposite directions
      leftMotor.target = motorCommand;
      rightMotor.target = -motorCommand;

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

  // Run velocity controllers continuously
  delayMicroseconds(100);
  leftMotor.move();
  delayMicroseconds(100);
  rightMotor.move();
}