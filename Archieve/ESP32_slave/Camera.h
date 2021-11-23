#ifndef CAMERA_H
#define CAMERA_H
#include <openmvrpc.h>

class Camera 
{
  private:
  openmv::rpc_i2c_master *interface;
  
  public:
  Camera(openmv::rpc_i2c_master *intface);
  void exe_face_detection(); // Face should be about 2ft away.
  bool exe_apriltag_detection(int ID,int *x_temp,int *y_temp,int *angle_temp);
  bool exe_color_detection(int8_t l_min, int8_t l_max, int8_t a_min, int8_t a_max, int8_t b_min, int8_t b_max);
  bool exe_color_detection_biggestblob(int8_t l_min, int8_t l_max, int8_t a_min, int8_t a_max, int8_t b_min, int8_t b_max, int& x, int&y);
  bool exe_goalfinder(int8_t goal1, int8_t goal2, int8_t goal3, int&id, int&tx, int&ty, int&tz, int&rx, int&ry, int&rz); //optional to add tag size as parameter?

};
#endif
