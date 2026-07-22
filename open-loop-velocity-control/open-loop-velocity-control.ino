#include <SimpleFOC.h>

// Left Motor
BLDCMotor leftMotor = BLDCMotor(7);
BLDCDriver3PWM leftDriver = BLDCDriver3PWM(9, 10, 11, 12);

// Right Motor
BLDCMotor rightMotor = BLDCMotor(7);
BLDCDriver3PWM rightDriver = BLDCDriver3PWM(3, 5, 6, 7);

Commander command = Commander(Serial);
void doLeftMotor(char* cmd) { command.motor(&leftMotor, cmd); }
void doRightMotor(char* cmd) { command.motor(&rightMotor, cmd); }

void setup() {
  Serial.begin(115200);
  while (!Serial);
  SimpleFOCDebug::enable(&Serial);

  // Pin 12 used as GND for specific shield stacking
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  // Driver 1 & Motor 1 Setup
  leftDriver.voltage_power_supply = 11;
  leftDriver.init();
  leftMotor.linkDriver(&leftDriver);
  leftMotor.voltage_limit = 1.5;
  rightMotor.velocity_limit = 30;
  leftMotor.controller = MotionControlType::velocity_openloop;
  leftMotor.init();

  // Driver 2 & Motor 2 Setup
  rightDriver.voltage_power_supply = 11;
  rightDriver.init();
  rightMotor.linkDriver(&rightDriver);
  rightMotor.voltage_limit = 1.5;
  rightMotor.velocity_limit = 30;
  rightMotor.controller = MotionControlType::velocity_openloop;
  rightMotor.init();

  // Commands: Use L to control left motor, R for right motor
  command.add('L', doLeftMotor, "Left Motor");
  command.add('R', doRightMotor, "Right Motor");

  leftMotor.target = -10; 
  rightMotor.target = 10; 

  Serial.println(F("Motors ready. Set target velocity (e.g., 'L10' or 'R5')"));
  _delay(1000);
}

void loop() {
  // In Open Loop, loopFOC is not strictly required but kept for structure
  leftMotor.loopFOC();
  rightMotor.loopFOC();

  leftMotor.move();
  rightMotor.move();

  command.run();
}