#include <esp_now.h> 
#include <WiFi.h> 
#include <Wire.h> 
#include <SparkFun_VL53L1X.h> 
#include <Arduino.h> 
 
#include "Camera.h" 
#include "utilities.h" 
#include <Adafruit_MotorShield.h> 
 
// REPLACE WITH THE MAC Address of your receiver (MASTER) 
// Slave: 40:F5:20:44:B6:4C 
// uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x4A, 0xCF, 0x00}; 
 uint8_t broadcastAddress[] = {0x40, 0xF5, 0x20, 0x44, 0xB6, 0x4C}; // #4 
// uint8_t broadcastAddress[] = {0xc4, 0xDD, 0x57, 0x9E, 0x8A, 0x98}; // #3
String success; 
 
// Define variables to store incoming readings 
String sentDebugM = ""; 
int Rec_pwm1; 
int Rec_pwm2; 
int Rec_pwm3; 
int Rec_pwm4; 
String Rec_dir1; 
String Rec_dir2; 
String Rec_dir3; 
String Rec_dir4; 
 
int count = 0; 
int count_var = 0; 
int print_count = 0; 
int change_count =0; 
int Lidar_flag = 0; 
 
// Define variables to be sent; 
int Sent_tx = 0; 
int Sent_ty = 0; 
int Sent_tz = 0; 
int Sent_rx = 0; 
int Sent_ry = 0; 
int Sent_rz = 0; 
int Sent_dist = 0; 
 
int8_t g1 = 0,g2=1,g3=2; 
int8_t goal_id[3] = {g1, g2, g3}; 
 
// Define Lidar variables 
SFEVL53L1X distanceSensor; 
int budgetIndex = 4; 
int budgetValue[7] = {15, 20, 33, 50, 100, 200, 500}; 
 
// Define LED variable 
int LED = LED_BUILTIN; 
 
// ==================================== data structure ================================================= 
//Structure the sending data 
//Must match the receiver structure 
typedef struct struct_message { 
  int Rtx; 
  int Rty; 
  int Rtz; 
  int Rrx; 
  int Rry; 
  int Rrz; 
  int Rdist; 
  String DebugM; 
  int Spwm1; 
  int Spwm2; 
  int Spwm3; 
  int Spwm4; 
  String Sdir1; 
  String Sdir2; 
  String Sdir3; 
  String Sdir4; 
} struct_message; 
// Create a struct_message to hold incoming sensor readings 
// struct_message incomingReadings; 
struct_message receivedData; 
 
// =================================== send and received function ===================================== 
// Callback when data is sent 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { 
  Serial.print("\r\nLast Packet Send Status:\t"); 
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"); 
  if (status ==0){ 
    success = "Delivery Success 🙂"; 
  } 
  else{ 
    success = "Delivery Fail 🙁"; 
  } 
} 
 
// Callback when data is received 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { 
  memcpy(&receivedData, incomingData, sizeof(receivedData)); 
  // the data format: 225 225 225 225 + + - - 
  // + up / forward  
  // - down / backward 
  Rec_pwm1 = receivedData.Spwm1; 
  Rec_pwm2 = receivedData.Spwm2;  
  Rec_pwm3 = receivedData.Spwm3; 
  Rec_pwm4 = receivedData.Spwm4;     
  Rec_dir1 = receivedData.Sdir1;   
  Rec_dir2 = receivedData.Sdir2;  
  Rec_dir3 = receivedData.Sdir3;  
  Rec_dir4 = receivedData.Sdir4;    
  count_var = 0; 
  count = 0; 
  print_count=0; 
  change_count = 0;   
   change_count = 0;   
} 
 
// =============================== All the setup =========================================== 
//Create the interface that will be used by the camera 
openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer. 
openmv::rpc_i2c_master interface(0x12, 100000); //to make this more robust, consider making address and rate as constructor argument 
Camera cam(&interface); 
// ========================== Motor part ==================================== 
// Create the motor shield object with the default I2C address 
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// changed 
Adafruit_DCMotor *motorVertical_L = AFMS.getMotor(1);  // pwm1 
Adafruit_DCMotor *motorVertical_R = AFMS.getMotor(2);  // pwm2 
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);        // pwm3 
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);       // pwm4  
// ==================================== Set up ================================================= 
void setup() { 
  // Init Serial Monitor 
  Serial.begin(115200); 
  Wire.begin(); 
  interface.begin(); //communication between ESP and OpenMV 
  AFMS.begin();  // create with the default frequency 1.6KHz 
  // -------------- LED part -------------------- 
  pinMode(LED, OUTPUT); 
  digitalWrite(LED, HIGH); 
  digitalWrite(LED, LOW); 
   
  // -------------- lidar part -------------------- 
  if (distanceSensor.begin() == 0){ 
    Serial.println("Sensor online!"); 
    Lidar_flag = 1; 
  }else { 
    Lidar_flag = 0; 
  } 
      
  distanceSensor.startRanging(); 
  distanceSensor.setIntermeasurementPeriod(budgetValue[budgetIndex]); 
   
 // --------------------------- esp now --------------------------- 
  // Set device as a Wi-Fi Station 
  WiFi.mode(WIFI_STA); 
 
  // Init ESP-NOW 
  if (esp_now_init() != ESP_OK) { 
    Serial.println("Error initializing ESP-NOW"); 
    return; 
  } 
   
  // Once ESPNow is successfully Init, we will register for Send CB to 
  // get the status of Trasnmitted packet 
  esp_now_register_send_cb(OnDataSent); 
   
  // Register peer 
  esp_now_peer_info_t peerInfo; 
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); 
  peerInfo.channel = 0;   
  peerInfo.encrypt = false; 
   
  // Add peer         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ 
    Serial.println("Failed to add peer"); 
    return; 
  } 
  // Register for a callback function that will be called when data is received 
  esp_now_register_recv_cb(OnDataRecv); 
   
} 
 
// ================================== Main loop =================================================== 
void loop() 
{  
  //------------------------------------------------------------------------------------- 
  // ========== goal finder ========= 
  int id = -1; 
  int tx = 0; int ty = 0; int tz = 0; 
  int rx = 0; int ry = 0; int rz = 0; 
  int x = 0; 
  int y = 0;  
  bool goalfind_flag = 0; 
   
  goalfind_flag = cam.exe_goalfinder(goal_id[0],goal_id[1],goal_id[2], id, tx, ty, tz, rx, ry, rz); 
  if (goalfind_flag){ 
      Sent_tx = tx; 
      Sent_ty = ty; 
      Sent_tz = tz; // cm 
      Sent_rx = rx; 
      Sent_ry = ry; 
      Sent_rz = rz;  
  }else{ 
      Sent_tx = 100000; 
      Sent_ty = 100000; 
      Sent_tz = 0; // cm 
      Sent_rx = 0; 
      Sent_ry = 0; 
      Sent_rz = 0;  
  } 
  if (Lidar_flag == 1){ 
    Sent_dist = distanceSensor.getDistance(); // Lidar distance (use the real Lidar data) 
  }else { 
    Sent_dist = 0; 
  } 
  // ========== lidar state info ========= 
  if (Sent_dist < 300 && Sent_dist > 0){ 
    receivedData.DebugM = "found b"; 
  }else{ 
    receivedData.DebugM = "no b"; 
  } 
  // ========== send info ========= 
  receivedData.Rtx = Sent_tx; 
  receivedData.Rty = Sent_ty; 
  receivedData.Rtz = Sent_tz;   
  receivedData.Rrx = Sent_rx;   
  receivedData.Rry = Sent_ry;  
  receivedData.Rrz = Sent_rz; 
  receivedData.Rdist = Sent_dist; 
  send_var_once(); 
  print_received_Data(); 
   
  //------------------------------------------------------------------------------------- 
  control_motion(); 
  //------------------------------------------------------------------------------------- 
} 
// ================================== ^ Main loop ^ =================================================== 
void control_motion(){ 
  // vertical motor 
  motorVertical_L->setSpeed(abs(Rec_pwm1)); 
  motorVertical_R->setSpeed(abs(Rec_pwm2));   
   
  if (Rec_dir1 == "+"){ 
    motorVertical_L->run(BACKWARD); // up  
  }else if (Rec_dir1 == "-"){ 
    motorVertical_L->run(FORWARD); // down  
  } 
 
  if (Rec_dir2 == "+"){  
    motorVertical_R->run(FORWARD); 
  }else if (Rec_dir2 == "-"){ 
    motorVertical_R->run(BACKWARD);     
  } 
 
  // horizontal motor 
  motorLeft->setSpeed(abs(Rec_pwm3)); 
  motorRight->setSpeed(abs(Rec_pwm4)); 
 
  if (Rec_dir3 == "+"){  
    motorLeft->run(BACKWARD); // make it move forward  
  }else if (Rec_dir3 == "-"){ 
    motorLeft->run(FORWARD);   // make it move backward   
  } 
 
  if (Rec_dir4 == "+"){  
    motorRight-> run(FORWARD);  // make it move forward  
  }else if (Rec_dir4 == "-"){ 
    motorRight-> run(BACKWARD); // make it move backward  
  } 
   
} 
 
void print_received_Data(){ 
  if (print_count == 0){ 
  Serial.print("Rec_pwm1:"); 
  Serial.println(Rec_pwm1); 
  Serial.print("Rec_pwm2:"); 
  Serial.println(Rec_pwm2); 
  Serial.print("Rec_pwm3:"); 
  Serial.println(Rec_pwm3); 
  Serial.print("Rec_pwm4:"); 
  Serial.println(Rec_pwm4);  
  Serial.print("Rec_dir1:"); 
  Serial.println(Rec_dir1);  
  Serial.print("Rec_dir2:"); 
  Serial.println(Rec_dir2);  
  Serial.print("Rec_dir3:"); 
  Serial.println(Rec_dir3); 
  Serial.print("Rec_dir4:"); 
  Serial.println(Rec_dir4);     
Serial.println("_______________________"); 
  } 
  print_count +=1; 
} 
 
void send_var_once(){ 
  if(count_var==0){ 
    send_message();  
    count_var+=1; 
  } 
} 
 
void send_message_once(){ 
  if(count==0){ 
    send_message();  
    count+=1; 
    receivedData.DebugM = ""; 
  } 
} 
 
void send_message(){ 
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &receivedData, sizeof(receivedData)); 
  // esp_now_send(RxMACaddress, (uint8_t *) &sentData, sizeof(sentData)); 
  //------------------------------------------------------------------------------------- 
  if (result == ESP_OK) { 
    Serial.println("Sent with success"); 
  } 
  else { 
    Serial.println("Error sending the data"); 
  } 
  //------------------------------------------------------------------------------------- 
  delay(50); // delay 50 ms after send out the message  
}
