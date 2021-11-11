/*  Wiring:
   3v3 on ESP32 goes to VCC on CMJU-531
   GND on ESP32 goes to GND on CMJU-531
   D21 (GPIO21, I2C SDA) on ESP32 goes to SDA on CMJU-531
   D22 (GPIO22, I2C SCL) on ESP32 goes to SCL on CMJU-531
*/

#include <Wire.h>
#include <SparkFun_VL53L1X.h> // https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/

SFEVL53L1X distanceSensor;
int budgetIndex = 4;
int dist = 0;
int budgetValue[7] = {15, 20, 33, 50, 100, 200, 500};
int LED = LED_BUILTIN;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  Wire.begin();
  delay(250);
  if (distanceSensor.begin() == 0)
    Serial.println("Sensor online!");
  distanceSensor.startRanging();
  distanceSensor.setIntermeasurementPeriod(budgetValue[3]);
  digitalWrite(LED, LOW);
}

void loop() {
  dist = distanceSensor.getDistance();
  if (dist<300){
    digitalWrite(LED, HIGH);
  }else {
    digitalWrite(LED, LOW);
    }
  Serial.println(distanceSensor.getDistance());
}
