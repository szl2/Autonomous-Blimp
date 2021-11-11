#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// ==================================== broadcast address ============================================
// MAC Address of the receiver (SLAVE)
//uint8_t broadcastAddress[] = {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x74}; // #1
//uint8_t broadcastAddress[] = {0x40, 0xF5, 0x20, 0x44, 0xBD, 0xF8}; // #2
//uint8_t broadcastAddress[] = {0xC4, 0xDD, 0x57, 0x9E, 0x8A, 0x98}; // #3
//uint8_t broadcastAddress[] = {0x40, 0xF5, 0x20, 0x44, 0xB6, 0x4C}; // #4
//uint8_t broadcastAddress[] = {0x40, 0xF5, 0x20, 0x44, 0xC0, 0xD8}; // #5
uint8_t broadcastAddress[] = {0x40, 0xF5, 0x20, 0x44, 0xDE, 0xE0}; // #6
//uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x4A, 0xD3, 0x3C}; // #7
//uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x9B, 0x04, 0x98}; // #8
// ==================================== global data =================================================
String success;
// Define variables to store BME280 readings to be sent
String strdata = "";
int count = 0;
int print_flag = 0;

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
  int Rdist1;
  int Rdist2; 
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
struct_message sentData;

// =================================== send and received function =====================================
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  /*
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }*/
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sentData, incomingData, sizeof(sentData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  Serial.println("SERIAL_IN_START");
  Serial.println(sentData.Rtx);
  Serial.println(sentData.Rty);
  Serial.println(sentData.Rtz);
  Serial.println(sentData.Rrx);
  Serial.println(sentData.Rry);
  Serial.println(sentData.Rrz);
  Serial.println(sentData.Rdist1);
  Serial.println(sentData.Rdist2);
  Serial.println(sentData.DebugM);
}
// =================================== setup ==================================================

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

// ======================================== main =============================================
void loop()
{ 
  while (Serial.available()>0){
   // the data format: 225 225 225 225 + + - - 
   int inChar = Serial.read();
   strdata += char(inChar);
   //delay(10);
   count +=1;
   
   if (count == 3){ 
    sentData.Spwm1 = strdata.toInt();
    if (print_flag == 1){
      Serial.println(sentData.Spwm1);
    }
    strdata = "";
   }

   if (count == 6){ 
    sentData.Spwm2 = strdata.toInt();
    if (print_flag == 1){
      Serial.println(sentData.Spwm2);
    }
    strdata = "";
   }

   if (count == 9){ 
    sentData.Spwm3 = strdata.toInt();
    if (print_flag == 1){
      Serial.println(sentData.Spwm3);
    }
    strdata = "";
   }

   if (count == 12){ 
    sentData.Spwm4 = strdata.toInt();
    if (print_flag == 1){
      Serial.println(sentData.Spwm4);
    }
    strdata = "";
   }

   if (count == 13){  
    sentData.Sdir1 = strdata;
    if (print_flag == 1){
      Serial.println(sentData.Sdir1);
    }
    strdata = "";
   }

   if (count == 14){ 
    sentData.Sdir2 = strdata;
    if (print_flag == 1){
      Serial.println(sentData.Sdir2);
    }
    strdata = "";
   }

   if (count == 15){ 
    sentData.Sdir3 = strdata;
    if (print_flag == 1){
      Serial.println(sentData.Sdir3);
    }
    strdata = "";
   }

   if (count == 16){ 
    sentData.Sdir4 = strdata;
    if (print_flag == 1){
      Serial.println(sentData.Sdir4);
    }
    strdata = "";
   }
   
  if (inChar == '\n'){ //after message is sent
    
    // esp_err_t result = 
    esp_now_send(broadcastAddress, (uint8_t *) &sentData, sizeof(sentData));
    // esp_now_send(RxMACaddress, (uint8_t *) &sentData, sizeof(sentData));
    //-------------------------------------------------------------------------------------
    /*
    if (result == ESP_OK) Serial.println("Sent with success");
    else Serial.println("Error sending the data");
    */
    //-------------------------------------------------------------------------------------
    //delay(50); // delay 50 ms
    strdata = "";
    count = 0;
  }
  
  }

  //-------------------------------------------------------------------------------------

  // delay(50);
}
