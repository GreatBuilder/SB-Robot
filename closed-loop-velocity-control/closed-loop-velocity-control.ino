#include <SimpleFOC.h>
#include <Wire.h>

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();
  Wire.setClock(100000);

  encoder.init(&Wire1);

  driver.voltage_power_supply = 11.0;
  driver.init();

  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 1.0;
  motor.velocity_limit = 15.0;
  motor.target = 5;

  motor.init();

  Serial.println("Open-loop encoder test");
}

void loop() {
  motor.loopFOC();
  motor.move();

  encoder.update();

  Serial.print("Angle: ");
  Serial.println(encoder.getAngle(), 4);
}