#include "Camera.h"
#include <Wire.h>
#include <Arduino.h>
#include <openmvrpc.h>

Camera::Camera(openmv::rpc_i2c_master *intface){
  interface = intface;
}

void Camera::exe_face_detection(){
    struct { uint16_t x, y, w, h; } face_detection_result;
    if (interface->call_no_args(F("face_detection"), &face_detection_result, sizeof(face_detection_result))) {
        Serial.print(F("Largest Face Detected [x="));
        Serial.print(face_detection_result.x);
        Serial.print(F(", y="));
        Serial.print(face_detection_result.y);
        Serial.print(F(", w="));
        Serial.print(face_detection_result.w);
        Serial.print(F(", h="));
        Serial.print(face_detection_result.h);
        Serial.println(F("]"));
    }
}

bool Camera::exe_apriltag_detection(int ID,int *x_temp,int *y_temp,int *angle_temp){
    struct { uint16_t cx, cy, rot; } result;
    if (interface->call(F("apriltags"), &ID, sizeof(ID), &result, sizeof(result))) {
    }
    if (result.cx == 0 && result.cy == 0 && result.rot == 0){
      return false;
    } else {
      *x_temp = result.cx;
      *y_temp = result.cy;
      *angle_temp = result.rot;
      return true;
    }
}
  
bool Camera::exe_color_detection(int8_t l_min, int8_t l_max, int8_t a_min, int8_t a_max, int8_t b_min, int8_t b_max){
    int8_t color_thresholds[6] = {l_min, l_max, a_min, a_max, b_min, b_max};
    short buff[128 + 1] = {};
    if (interface->call(F("color_detection"), color_thresholds, sizeof(color_thresholds), buff, sizeof(buff)-1)) {
          int i = 0;
          while (buff[i] != '\0' && i<100) {
            Serial.print(buff[i]);
            i++;  
          }
          Serial.println("");
    }
    if (buff[0] == 0){ //no blob detected
      return false;
    } else {
      return true;
    }
}

bool Camera::exe_color_detection_biggestblob(int8_t l_min, int8_t l_max, int8_t a_min, int8_t a_max, int8_t b_min, int8_t b_max, int& x, int&y){
  int8_t color_thresholds[6] = {l_min, l_max, a_min, a_max, b_min, b_max};
  struct { uint16_t cx, cy; } color_detection_result;
  if (interface->call(F("color_detection_single"), color_thresholds, sizeof(color_thresholds), &color_detection_result, sizeof(color_detection_result))) {
    
  }
  x = color_detection_result.cx;
  y = color_detection_result.cy;
  if (x == 0 && y == 0){
    return false;
  } else {
    return true;
  }
}


bool Camera::exe_goalfinder(int8_t goal1, int8_t goal2, int8_t goal3, int&id, int&tx, int&ty, int&tz, int&rx, int&ry, int&rz){
  int8_t goalid[3] = {goal1, goal2, goal3};
  struct { int16_t cid, ctx, cty, ctz, crx, cry, crz; } goalfinder_result;
  if(interface->call(F("goalfinder"), goalid, sizeof(goalid), &goalfinder_result, sizeof(goalfinder_result))){
    
  }
  if (goalfinder_result.crx == 0 && goalfinder_result.cry == 0){
    return false;
  } else {
    id = goalfinder_result.cid;
    tx = goalfinder_result.ctx;
    ty = goalfinder_result.cty;
    tz = goalfinder_result.ctz;
    rx = goalfinder_result.crx;
    ry = goalfinder_result.cry;
    rz = goalfinder_result.crz;
    return true;
  }
}
