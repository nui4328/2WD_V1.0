
#include <My2WD_V1.h>
#include <my_TCS.h>
#include <my2wd_servo.h>
my_TCS color;                // สร้างอ็อบเจ็กต์เซ็นเซอร์สี
bool color_is_available = false;  // เพิ่มบรรทัดนี้ด้านบน setup()
int mainMenu = 0;               // 0=RUN, 1=cal_ADC, 2=cal_COLOR
int colorSubMenu = 0;           // 0=red, 1=green, ..., 6=DONE
bool inColorCalMenu = false;    // สถานะอยู่ในหน้า calibrate สีหรือไม่
const int potPin = 29;       // ปุ่มหมุน
const int buttonPin = 9;     // ปุ่มกด (pull-down)
int red_box, green_box, blue_box, yello_box, ch_poit;
int servo27 = 80;
int servo28 = 90;
int servo20 = 120;


//----------------------------------------------------------------------------------------------------------------------->>>
int retreat_distance = 210;       //--------------->>::  กำหนดค่าระยะการถอยก่อนหมุนตัว
int shop_distance = 52;       //--------------->>::  กำหนดค่าระยะการเดินข้ามตะเกียบ
int miss_distance = 13;       //--------------->>::  กำหนดค่าระยะการเดินข้ามตะเกียบ
//----------------------------------------------------------------------------------------------------------------------->>>

void setup() 
  {
    _set();  
     
    set_pid_moveL(1.75, 0.0033, 0.025);   //------->> ตั้งค่า pid สำหรับหมุนซ้าย    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    set_pid_moveR(1.75, 0.0033, 0.032);   //------->> ตั้งค่า pid สำหรับหมุนขวา    ให้ปรับตัวแปรตัวสุดท้ายตัวเดียว  ถ้าหมุนเลยองศาให้เพิ่มขึ้น     **************************
    //test_color();
    Move_start(30, 30, 1.2, 30);         //--------->>Move_start(30, 30, 1.2, 30);  กำหนดระยะที่เดินออาจากจุดเริ่มต้น คือ เลข 30
  
    for(int i=0; i<3000; i++)
      {
        Move_forward (30, 30, 1.2, 30);        //--------->>Move_start(30, 30, 1.2, 30);  กำหนดระยะที่เดินแต่ละบล๊อก คือ เลข 30
      }

    
  }

void loop() 
    {

      Serial.println( digitalRead(0) ); 
      /*
      for(int i=0; i<8; i++)
        {
             Serial.print( md_sensor(i));
             Serial.print( " ");
        }
      Serial.println( " ");
      
      if(digitalRead(9)==0 )
        {
          my.resetAngles();
        }*/
     // Serial.print(my.gyro('x'));Serial.print( "   " ); Serial.print(abs(my.gyro('y'))); Serial.print( "   " ); Serial.print(my.gyro('z')); Serial.println( "   " ); 
      //Serial.print(encoder.Poss_L() );Serial.print("  "); Serial.println( encoder.Poss_R() ); 
    
     delay(10);

}
