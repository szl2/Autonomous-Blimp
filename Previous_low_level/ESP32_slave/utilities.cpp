#include <math.h>

//Source: http://www.easyrgb.com/index.php?X=MATH&H=01#text1
void lab2rgb( float l_s, float a_s, float b_s, float& R, float& G, float& B )
{
    float var_Y = ( l_s + 16. ) / 116.;
    float var_X = a_s / 500. + var_Y;
    float var_Z = var_Y - b_s / 200.;

    if ( pow(var_Y,3) > 0.008856 ) var_Y = pow(var_Y,3);
    else                      var_Y = ( var_Y - 16. / 116. ) / 7.787;
    if ( pow(var_X,3) > 0.008856 ) var_X = pow(var_X,3);
    else                      var_X = ( var_X - 16. / 116. ) / 7.787;
    if ( pow(var_Z,3) > 0.008856 ) var_Z = pow(var_Z,3);
    else                      var_Z = ( var_Z - 16. / 116. ) / 7.787;

    float X = 95.047 * var_X ;    //ref_X =  95.047     Observer= 2°, Illuminant= D65
    float Y = 100.000 * var_Y  ;   //ref_Y = 100.000
    float Z = 108.883 * var_Z ;    //ref_Z = 108.883


    var_X = X / 100. ;       //X from 0 to  95.047      (Observer = 2°, Illuminant = D65)
    var_Y = Y / 100. ;       //Y from 0 to 100.000
    var_Z = Z / 100. ;      //Z from 0 to 108.883

    float var_R = var_X *  3.2406 + var_Y * -1.5372 + var_Z * -0.4986;
    float var_G = var_X * -0.9689 + var_Y *  1.8758 + var_Z *  0.0415;
    float var_B = var_X *  0.0557 + var_Y * -0.2040 + var_Z *  1.0570;

    if ( var_R > 0.0031308 ) var_R = 1.055 * pow(var_R , ( 1 / 2.4 ))  - 0.055;
    else                     var_R = 12.92 * var_R;
    if ( var_G > 0.0031308 ) var_G = 1.055 * pow(var_G , ( 1 / 2.4 ) )  - 0.055;
    else                     var_G = 12.92 * var_G;
    if ( var_B > 0.0031308 ) var_B = 1.055 * pow( var_B , ( 1 / 2.4 ) ) - 0.055;
    else                     var_B = 12.92 * var_B;

    R = var_R * 255.;
    G = var_G * 255.;
    B = var_B * 255.;

}





//old codes

//      motorVertical->setSpeed(0);
//      motorVertical->run(FORWARD);
//      // turn on motor
//      motorVertical->run(RELEASE);
//      Serial.println(SEEKING_SPEED);
//      motorLeft->setSpeed(SEEKING_SPEED);
//      motorLeft->run(FORWARD); 
//      motorRight->setSpeed(SEEKING_SPEED);

//    if (abs(Outputx) < 125){
//      motorRight->setSpeed(BASE_SPEED - Outputx); 
//      motorRight->run(BACKWARD);
//      motorLeft->setSpeed(BASE_SPEED + Outputx); //this need to be higher
//      motorLeft->run(BACKWARD);
//    } else if (Outputx >= 125) { //propeller push in opposite direction, moving to the right
//      motorRight->setSpeed(Outputx); 
//      motorRight->run(FORWARD);
//      motorLeft->setSpeed(255); //this need to be higher
//      motorLeft->run(BACKWARD);
//    } else {
//      motorRight->setSpeed(255); 
//      motorRight->run(BACKWARD);
//      motorLeft->setSpeed(Outputx); //this need to be higher
//      motorLeft->run(FORWARD);
//    }

//    motorVertical->setSpeed(0);
//    motorVertical->run(FORWARD);
//    // turn on motor
//    motorVertical->run(RELEASE);
//    motorLeft->setSpeed(0);
//    motorLeft->run(FORWARD);
//    motorRight->setSpeed(0);
//    motorRight->run(BACKWARD);
//      motorRight->run(BACKWARD);
