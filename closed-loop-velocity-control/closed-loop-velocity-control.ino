#include <SimpleFOC.h>
#include <Wire.h>

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

float targetVelocity = 0.0;

Commander command = Commander(Serial);

void setVelocity(char* cmd) {
  command.scalar(&targetVelocity, cmd);

  Serial.print("Target velocity: ");
  Serial.println(targetVelocity);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  SimpleFOCDebug::enable(&Serial);

  Wire1.begin();
  Wire1.setClock(100000);

  encoder.init(&Wire1);
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = 11.0;

  if (!driver.init()) {
    Serial.println("Driver initialization failed");
    while (1);
  }

  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = 1.0;
  motor.voltage_sensor_align = 1.0;
  motor.velocity_limit = 15.0;

  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.02;

  if (!motor.init()) {
    Serial.println("Motor initialization failed");
    while (1);
  }

  if (!motor.initFOC()) {
    Serial.println("FOC initialization failed");
    driver.disable();
    while (1);
  }

  command.add('T', setVelocity, "target velocity");

  Serial.println("Motor ready");
  Serial.println("Enter T followed by velocity in rad/s");
  Serial.println("Examples: T5, T-5, T0");
}

void loop() {
  motor.loopFOC();
  motor.move(targetVelocity);

  command.run();
}