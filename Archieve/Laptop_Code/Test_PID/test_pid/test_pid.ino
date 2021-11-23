#include <PID_v1.h>
#include <esp_now.h>
#include <WiFi.h>

double Setpointx, Inputx, Outputx;
double Setpointy, Inputy, Outputy;

double Kpx=2, Kix=0.1, Kdx=0.25;
double Kpy=1, Kiy=0.1, Kdy=0.25;

const double RESOLUTION_W = 320.0;
const double RESOLUTION_H = 240.0;
int count  = 0;
String strdata = "";
// ========================== PID part ====================================
// Setup PID
PID PID_x(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, DIRECT);
PID PID_y(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    //the values that the PID will try to reach
    Setpointx = 160.0; 
    Setpointy = 120.0;
  PID_y.SetOutputLimits(-255, 255); //up positive
  PID_x.SetOutputLimits(-255, 255); //left positive
  PID_x.SetMode(AUTOMATIC);
  PID_y.SetMode(AUTOMATIC);
  count  = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  int tx,ty;
  /*
    while (Serial.available()>0){
      int inChar = Serial.read();
      strdata += char(inChar);
      delay(10);
      count +=1;
      if (count == 3){
        Setpointx = strdata.toDouble();
        strdata = ""; 
      }else if(count==6){
        Setpointy = strdata.toDouble();
        strdata = ""; 
      }

      if (inChar == '\n'){
        count = 0; 
      }
    }*/

    if (count < 30){
    Setpointx = 160.0; 
    Setpointy = 120.0;
    tx = 200;
    ty = 150;
    Inputx = tx/1.00;
    Inputy = ty/1.00;
    PID_x.Compute();
    PID_y.Compute();
    Serial.print("Outputy:");
    Serial.println(Outputy);
    Serial.print("Outputx:");
    Serial.println(Outputx); 
    }

    if (count > 30 && count < 60){
    Setpointx = 400.0; 
    Setpointy = 300.0;
    tx = 200;
    ty = 150;
    Inputx = tx/1.00;
    Inputy = ty/1.00;
    PID_x.Compute();
    PID_y.Compute();
    Serial.print("Outputy:");
    Serial.println(Outputy);
    Serial.print("Outputx:");
    Serial.println(Outputx); 
    }
    
    count += 1;
    delay(100);
}
