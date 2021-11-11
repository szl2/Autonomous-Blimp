#include <Wire.h>
#include <VL53L1X.h>

#define XSHUT_pin1 A7
#define XSHUT_pin2 A6


//ADDRESS_DEFAULT 0b0101001 or 41
#define Sensor1_newAddress 43 
#define Sensor2_newAddress 45


VL53L1X Sensor1;
VL53L1X Sensor2;


void setup()
{ 

  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);

  
  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(100000); 


  pinMode(XSHUT_pin1, INPUT);
     if (!Sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1);
  }
  delay(10);
  Sensor1.setAddress(Sensor1_newAddress);

 pinMode(XSHUT_pin2, INPUT);
     if (!Sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1);
  }
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  
  

  
  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);


  
  Sensor1.setDistanceMode(VL53L1X::Medium);
  Sensor1.setMeasurementTimingBudget(33000);
  Sensor2.setDistanceMode(VL53L1X::Medium);
  Sensor2.setMeasurementTimingBudget(33000);

}


void loop()
{
   Sensor1.startContinuous(50);
   Sensor1.read();
   Sensor1.stopContinuous();
   long time1 = millis();
   Sensor2.startContinuous(50);
   Sensor2.read();
   Sensor2.stopContinuous();
   long time2 = millis();
   
   
  Serial.print("Sensor 1: ");
  Serial.print(Sensor1.ranging_data.range_mm);
  Serial.print(" ");
  Serial.print("mm \t Sensor 2: ");
  Serial.print(Sensor2.ranging_data.range_mm);
  Serial.print(" ");
  Serial.print("mm \t Time: ");
  Serial.print(time2-time1);

  Serial.println();

}
