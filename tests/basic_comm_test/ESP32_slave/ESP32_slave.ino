#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// REPLACE WITH THE MAC Address of your receiver (MASTER)
// Slave: 40:F5:20:44:B6:4C
// uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x4A, 0xCF, 0x00};
uint8_t broadcastAddress[] = {0x40, 0xF5, 0x20, 0x44, 0xB6, 0x4C}; // #4
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

// Define variables to be sent;
int Sent_tx = 0;
int Sent_ty = 0;
int Sent_tz = 0;
int Sent_rx = 0;
int Sent_ry = 0;
int Sent_rz = 0;
int Sent_dist = 0;

String strData = "";
double valData = 0.0;
String queData = "";
double Kpx = 2, Kix = 0.1, Kdx = 0.25;
double Kpy = 1, Kiy = 0.1, Kdy = 0.25;
int g1 = 0, g2 = 1 , g3 = 2;
int goal_id[3] = {g1, g2, g3};
int8_t Lmin = 30,Lmax = 100, Amin = -49,Amax = -22,Bmin = 31,Bmax = 127;
int8_t threshold[6] = {Lmin, Lmax, Amin, Amax, Bmin, Bmax};
int base_speed = 70;
int seeking_speed = 70;
int lidar_thres = 300; // mm 
int gbx = 0, gby = 0,gbd = 0;
double gbc = 0;

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
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
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
}

// ==================================== Set up =================================================
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
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
  Sent_tx = 100;
  Sent_ty = 200;
  Sent_tz = 40; // cm
  Sent_rx = 111;
  Sent_ry = 222;
  Sent_rz = 333;
  Sent_dist = 300; // Lidar distance
  
  receivedData.Rtx = Sent_tx;
  receivedData.Rty = Sent_ty;
  receivedData.Rtz = Sent_tz;  
  receivedData.Rrx = Sent_rx;  
  receivedData.Rry = Sent_ry; 
  receivedData.Rrz = Sent_rz;
  receivedData.Rdist = Sent_dist;
  receivedData.DebugM = "Testing";
  send_var_once();

  print_received_Data();
  
  // FOR LIDAR
  /*
  if(lidar_thres < 300 && seeking_speed < 50){
    receivedData.DebugM = "ca b se g";
    send_message_once();
  }else if(lidar_thres < 300 && seeking_speed > 50){
    receivedData.DebugM = "catch ball";
    send_message_once();
  }else if (lidar_thres > 300 && seeking_speed < 50){
    receivedData.DebugM = "seeking";
    send_message_once(); 
  }
  */
  //-------------------------------------------------------------------------------------
}
// ================================== ^ Main loop ^ ===================================================
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
Serial.println("_________________________");
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
