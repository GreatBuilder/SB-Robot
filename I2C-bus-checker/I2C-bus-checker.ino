#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);

  Serial.println("Scanning SDA/SCL bus...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device at 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
    }
  }
}

void loop() {
}