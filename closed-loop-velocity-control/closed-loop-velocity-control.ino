#include <SimpleFOC.h>
#include <Wire.h>

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

void setup() {
  Serial.begin(115200);
  delay(2000);

  SimpleFOCDebug::enable(&Serial);

  // Encoder is connected to Wire1
  Wire1.begin();
  Wire1.setClock(100000);

  encoder.init(&Wire1);
  motor.linkSensor(&encoder);

  // Driver setup
  driver.voltage_power_supply = 11.0;

  if (!driver.init()) {
    Serial.println("Driver init failed");
    while (1);
  }

  motor.linkDriver(&driver);

  // Closed-loop velocity control
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;

  // Motor safety limits
  motor.voltage_limit = 1.0;
  motor.voltage_sensor_align = 1.0;
  motor.velocity_limit = 15.0;

  // Gentle velocity controller settings
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0.0;

  motor.LPF_velocity.Tf = 0.02;

  if (!motor.init()) {
    Serial.println("Motor init failed");
    while (1);
  }

  Serial.println("Starting FOC alignment");

  if (!motor.initFOC()) {
    Serial.println("FOC alignment failed");
    driver.disable();
    while (1);
  }

  motor.target = 5.0;

  Serial.println("Closed-loop motor ready");
}

void loop() {
  motor.loopFOC();
  motor.move();
}