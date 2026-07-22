#include <SimpleFOC.h>
#include <Wire.h>

// ---------------- Encoders ----------------

MagneticSensorI2C leftEncoder =
  MagneticSensorI2C(AS5600_I2C);

MagneticSensorI2C rightEncoder =
  MagneticSensorI2C(AS5600_I2C);

// ---------------- Left motor ----------------

BLDCMotor leftMotor = BLDCMotor(7);
BLDCDriver3PWM leftDriver =
  BLDCDriver3PWM(9, 10, 11, 12);

// ---------------- Right motor ----------------

BLDCMotor rightMotor = BLDCMotor(7);
BLDCDriver3PWM rightDriver =
  BLDCDriver3PWM(3, 5, 6, 7);

void setup() {
  Serial.begin(115200);
  delay(2000);

  SimpleFOCDebug::enable(&Serial);

  // Main I2C bus: left encoder
  Wire.begin();
  Wire.setClock(400000);

  // Qwiic I2C bus: right encoder
  Wire1.begin();
  Wire1.setClock(400000);

  leftEncoder.init(&Wire);
  rightEncoder.init(&Wire1);

  leftMotor.linkSensor(&leftEncoder);
  rightMotor.linkSensor(&rightEncoder);

  // ---------------- Left driver ----------------

  leftDriver.voltage_power_supply = 11.0;
  leftDriver.voltage_limit = 1.5;

  if (!leftDriver.init()) {
    Serial.println("Left driver failed");
    while (1);
  }

  leftMotor.linkDriver(&leftDriver);

  // ---------------- Right driver ----------------

  rightDriver.voltage_power_supply = 11.0;
  rightDriver.voltage_limit = 1.5;

  if (!rightDriver.init()) {
    Serial.println("Right driver failed");
    while (1);
  }

  rightMotor.linkDriver(&rightDriver);

  // ---------------- Closed-loop settings ----------------

  leftMotor.controller = MotionControlType::velocity;
  rightMotor.controller = MotionControlType::velocity;

  leftMotor.torque_controller = TorqueControlType::voltage;
  rightMotor.torque_controller = TorqueControlType::voltage;

  leftMotor.voltage_limit = 1.5;
  rightMotor.voltage_limit = 1.5;

  leftMotor.voltage_sensor_align = 1.0;
  rightMotor.voltage_sensor_align = 1.0;

  leftMotor.velocity_limit = 20.0;
  rightMotor.velocity_limit = 20.0;

  // Conservative starting PID values
  leftMotor.PID_velocity.P = 0.2;
  leftMotor.PID_velocity.I = 1.0;
  leftMotor.PID_velocity.D = 0.0;
  leftMotor.LPF_velocity.Tf = 0.02;

  rightMotor.PID_velocity.P = 0.2;
  rightMotor.PID_velocity.I = 1.0;
  rightMotor.PID_velocity.D = 0.0;
  rightMotor.LPF_velocity.Tf = 0.02;

  if (!leftMotor.init()) {
    Serial.println("Left motor init failed");
    while (1);
  }

  if (!rightMotor.init()) {
    Serial.println("Right motor init failed");
    while (1);
  }

  Serial.println("Aligning left motor...");

  if (!leftMotor.initFOC()) {
    Serial.println("Left motor FOC failed");
    while (1);
  }

  Serial.println("Aligning right motor...");

  if (!rightMotor.initFOC()) {
    Serial.println("Right motor FOC failed");
    while (1);
  }

  Serial.println("Both motors ready");
}

void loop() {
  // These must run as frequently as possible
  leftMotor.loopFOC();
  rightMotor.loopFOC();

  // Opposite signs because the motors face opposite directions
  leftMotor.move(-5.0);
  rightMotor.move(5.0);
}