#include <Wire.h>
#include <SparkFun_VL53L1X.h> // https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/

#define BLYNK_PRINT Serial
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <WiFiClient.h>

char auth[] = "4Mp_V5mFllQKyaSjzqwILkMZZTRSuPD-";
char ssid[] = "lemur";
char pass[] = "lemur9473";

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

SFEVL53L1X distanceSensor;
int budgetIndex = 3;
int dist = 0;
int budgetValue[7] = {15, 20, 33, 50, 100, 200, 500};
int LED = LED_BUILTIN;
int detect_thres = 300; // mm
int mt_speed = 50;
float Kp = 0;
int new_speed = 0 ;
int flag = 0;

void setup() {
//  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  AFMS.begin();

  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  Wire.begin();
  delay(250);
  if (distanceSensor.begin() == 0)
    Serial.println("Sensor online!");
  distanceSensor.startRanging();
  distanceSensor.setIntermeasurementPeriod(budgetValue[budgetIndex]);
  digitalWrite(LED, LOW);
}

void forward(){
     myMotor3->run(BACKWARD);  
     myMotor4->run(FORWARD);
}

void backward(){
     myMotor3->run(FORWARD);
     myMotor4->run(BACKWARD);
}

void turn_left(){
  myMotor4->run(FORWARD);
  myMotor3->run(RELEASE);
}

void turn_right(){
  myMotor3->run(BACKWARD);
  myMotor4->run(RELEASE);
}

void up(){
     myMotor2->run(FORWARD);
     myMotor1->run(FORWARD);
}

void down(){
     myMotor2->run(BACKWARD);
     myMotor1->run(BACKWARD);
}

void stopfb(){
     myMotor3->run(RELEASE);
     myMotor4->run(RELEASE);
}

void stopud(){
     myMotor2->run(RELEASE);
     myMotor1->run(RELEASE);
}


void SpeedControl(int Speed){
    myMotor1->setSpeed(Speed);
    myMotor2->setSpeed(Speed);
    myMotor3->setSpeed(Speed);
    myMotor4->setSpeed(Speed);
}

void loop() {
  dist = distanceSensor.getDistance();
  Serial.println(distanceSensor.getDistance());
//  if (flag == 0){
//    stopfb();
//    stopud();
//  }
//  else if (flag == 1){
    if (dist < detect_thres){
      SpeedControl(mt_speed);
      forward();
    }
    else if (dist > detect_thres){
      /*
      Kp = (dist - detect_thres)/ 4000;
      Serial.println("kp: ");
      Serial.println(Kp);
      float new_speed = Kp * (255-mt_speed) + mt_speed;
      Serial.println("newspeed: ");
      Serial.println(new_speed);*/
 
      SpeedControl(150);
      forward();
    }
//  }
  Blynk.run();
}

BLYNK_WRITE(V2){
  int Speed = param.asInt();
  if (Speed == 0){
    flag = 0;
  }else{
    flag = 1;
  }
}
