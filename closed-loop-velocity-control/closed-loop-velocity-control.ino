#include <SimpleFOC.h>
#include <Wire.h>

// Encoders
MagneticSensorI2C leftEncoder = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rightEncoder = MagneticSensorI2C(AS5600_I2C);

// Left motor
BLDCMotor leftMotor = BLDCMotor(7);
BLDCDriver3PWM leftDriver = BLDCDriver3PWM(9, 10, 11, 12);

// Right motor
BLDCMotor rightMotor = BLDCMotor(7);
BLDCDriver3PWM rightDriver = BLDCDriver3PWM(3, 5, 6, 7);

// Target velocities
float leftTarget = 0.0;
float rightTarget = 0.0;

// Serial commander
Commander command = Commander(Serial);

void setLeftVelocity(char* cmd) {
  command.scalar(&leftTarget, cmd);

  Serial.print("Left target: ");
  Serial.println(leftTarget);
}

void setRightVelocity(char* cmd) {
  command.scalar(&rightTarget, cmd);

  Serial.print("Right target: ");
  Serial.println(rightTarget);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  SimpleFOCDebug::enable(&Serial);

  // Left encoder on Wire
  Wire.begin();
  Wire.setClock(100000);

  // Right encoder on Wire1
  Wire1.begin();
  Wire1.setClock(100000);

  leftEncoder.init(&Wire);
  rightEncoder.init(&Wire1);

  leftMotor.linkSensor(&leftEncoder);
  rightMotor.linkSensor(&rightEncoder);

  // Left driver
  leftDriver.voltage_power_supply = 11.0;

  if (!leftDriver.init()) {
    Serial.println("Left driver failed");
    while (1);
  }

  leftMotor.linkDriver(&leftDriver);

  // Right driver
  rightDriver.voltage_power_supply = 11.0;

  if (!rightDriver.init()) {
    Serial.println("Right driver failed");
    while (1);
  }

  rightMotor.linkDriver(&rightDriver);

  // Left motor settings
  leftMotor.controller = MotionControlType::velocity;
  leftMotor.torque_controller = TorqueControlType::voltage;

  leftMotor.voltage_limit = 1.0;
  leftMotor.voltage_sensor_align = 1.0;
  leftMotor.velocity_limit = 15.0;

  leftMotor.PID_velocity.P = 0.15;
  leftMotor.PID_velocity.I = 0.5;
  leftMotor.PID_velocity.D = 0.0;
  leftMotor.LPF_velocity.Tf = 0.02;

  // Right motor settings
  rightMotor.controller = MotionControlType::velocity;
  rightMotor.torque_controller = TorqueControlType::voltage;

  rightMotor.voltage_limit = 1.0;
  rightMotor.voltage_sensor_align = 1.0;
  rightMotor.velocity_limit = 15.0;

  rightMotor.PID_velocity.P = 0.15;
  rightMotor.PID_velocity.I = 0.5;
  rightMotor.PID_velocity.D = 0.0;
  rightMotor.LPF_velocity.Tf = 0.02;

  // Initialize motors
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

  command.add('L', setLeftVelocity, "left velocity");
  command.add('R', setRightVelocity, "right velocity");

  Serial.println("Both motors ready");
  Serial.println("Examples:");
  Serial.println("L5");
  Serial.println("R5");
  Serial.println("L0");
  Serial.println("R0");
}

void loop() {
  leftMotor.loopFOC();
  rightMotor.loopFOC();

  leftMotor.move(leftTarget);
  rightMotor.move(rightTarget);

  command.run();
}