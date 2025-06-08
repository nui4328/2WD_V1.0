#include <Wire.h>
#include <my_TCS34725.h> 
#include "EncoderLibrary.h"
EncoderLibrary encoder(7, 2, 11, 10);

//  #include <my_TCS34725.h>     
       // my_tcs('r'),   อ่านค่า r
       // my_tcs('g'),   อ่านค่า g
       // my_tcs('b')    อ่านค่า b 
void setup() 
  {
    _setup();
    sw();

   
  }

void loop() {

  //servo(36, 30);  delay(300); // รอ 1 วินาที
  //servo(36, 150);  delay(300); // รอ 1 วินาที
  Serial.print(analogRead(46));Serial.print("  ");Serial.print(analogRead(47));Serial.println("  ");
  //Serial.print(encoder.Poss_L());Serial.print("  ");
  //Serial.print(encoder.Poss_R());Serial.println("  ");
  /*
  Motor(50, 50);delay(1000);
  Motor(-50, -50);delay(1000);
  
  Serial.println(digitalRead(33));
  delay(10); // รอ 1 วินาที
  */

   delay(10); // รอ 1 วินาที
}