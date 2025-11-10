#include <Wire.h>
#include <my_GYRO.h>
#include <Adafruit_ST7735.h>
Adafruit_ST7735 tft = Adafruit_ST7735(22, 21, -1);
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
float p = 3.1415926;
uint16_t red = ST77XX_YELLOW;
uint16_t yellow =  ST77XX_RED ;   
uint16_t green = ST77XX_GREEN;
uint16_t black = ST77XX_WHITE;
uint16_t white = ST77XX_BLACK;
uint16_t blue = ST77XX_BLUE;

#include "EncoderAdvanced.h"
EncoderAdvanced encoder4set(17, 20, 23, 12, 31, 30, 33, 32);

#define MCP3421_ADDR 0x68  // I2C Address when A0 = GND  
int rgb[] = {26, 25, 9};


// ค่าคงที่สำหรับการคำนวณ
const float wheelDiameter = 5.0;                      // เส้นผ่านศูนย์กลางล้อ (เซนติเมตร)
const float wheelCircumference = PI * wheelDiameter;  // เส้นรอบวงล้อ
const int pulsesPerRevolution = 450;                  // จำนวนพัลส์ต่อรอบ
const float pulsesPerCm = pulsesPerRevolution / wheelCircumference; // พัลส์ต่อเซนติเมตร

bool lines;
bool lines_fw;
bool lines_bw;
bool sett_f = false;
bool ch_lr = false;
bool ch_set_fb = false;
bool ch_bw = false;
bool line_l = true;
bool line_r = true;
char ch_lrs;

bool set_bb = false;
int motor_slow = 18;
int fw_to_rotate = 500;

// กำหนดค่า PID
float Kp = 0.05;  // Proportional gain
float Ki = 0.000015; // Integral gain
float Kd = 0.015;  // Derivative gain

// ตัวแปรสำหรับ PID Control
float previousError = 0;
float integral = 0;


unsigned long lastTime = millis();
bool error_servo  = false;
//void encoderISR(void) ;
//void set_f(int _time);

float lr_kp, lr_ki, lr_kd;
bool chopsticks  = false;
unsigned long lastTimes = millis();
float Kpp = 0.35, Kii = 0.00001, Kdd = 0.03;
float _integral = 0, _prevErr = 0;
unsigned long prevT;
float error_moveLR , output_moveLR;

#define EEPROM_ADDRESS 0x50 // ที่อยู่ I2C ของ CAT24C256
const int numSensors = 8;
const int numSamples = 1000;
int sensorValuesA[numSensors][numSamples]; // เก็บข้อมูลเซนเซอร์จาก read_sensorA
int sensorMaxA[numSensors];
int sensorMinA[numSensors];
int sensorValuesB[numSensors][numSamples]; // เก็บข้อมูลเซนเซอร์จาก read_sensorB
int sensorMaxB[numSensors];
int sensorMinB[numSensors];
int sensorValuesC[2][numSamples]; // เก็บข้อมูลจาก analogRead(46) และ analogRead(47)
int sensorMaxC[2]; // Max สำหรับพิน 46 และ 47
int sensorMinC[2]; // Min สำหรับพิน 46 และ 47
int sensorMax_A[numSensors];
int sensorMin_A[numSensors];
int sensorMax_B[numSensors];
int sensorMin_B[numSensors];
int sensorMax_C[2]; // Max สำหรับพิน 26 และ 27
int sensorMin_C[2]; // Min สำหรับพิน 26 และ 27
float P, I, D, previous_I, previous_error,errors, PID_output, present_position, previous_integral; 
int numSensor = 6; 
int state_on_Line = 0;
int setpoint;
int _lastPosition;
int sensor_pin_A[] = {1,2,3,4,5,6}; 
int sensor_pin_B[] = {1,2,3,4,5,6}; 
const float I_MAX = 1000.0; // ขีดจำกัดบนของ integral
const float I_MIN = -1000.0; // ขีดจำกัดล่างของ integral

void setup()
  {    
    _setup();
    setup_bno055();
    Acceptable_values_moveLR(1,  5);     //------->> ตั้งค่า error  output ที่ยอมรับได้ในการหมุน
    set_pid_moveLR(1.2, 0.0005, 0.13);   //------->> ตั้งค่า pid สำหรับหมุนตัว
    set_move_before_moveLR(110);   //------->> ตั้งค่า เดินหน้า ถอยหลัง ก่อนการหมุน

    mydisplay_background(black);
    mydisplay("MY-MAKERS", 20, 30, 2, white);
    start_led();
    servo_open();

    sw();
    /*
   blak_Distance(12, 200);
   moveLR(60, 90); 
   goDistance(12, 500);
   moveLR(60, 90);
   goDistance(12, 500);
   moveLR(60, 90);
   goDistance(12, 500);
   moveLR(60, 90);
   //righ_distance(15, 500); 
*/
   /*
    goTime(30, 500); // ไสลด์ขวา 50% นาน 2 วินาที
  
    delay(500);
   slideRightTime(30, 500); // ไสลด์ขวา 50% นาน 2 วินาที
    delay(500);
    goTime(30, 500); // ไสลด์ขวา 50% นาน 2 วินาที
  */

    
  }

void loop()
  {
    displayYaw();
    //display_gyro();
     //Serial.println(knopRead());   delay(10);

  
    //Serial.print( analogRead(40) );   Serial.print( "   " );   Serial.println( ); delay(10);
    //Serial.println(my.gyro('z'));   delay(10);
    
    //display_gyro();
    //Serial.println(digitalRead(24));delay(10);
   /*
    Serial.print("ENC1: "); Serial.print(encoder4set.Poss_1A());

    Serial.print(" || ENC2: "); Serial.print(encoder4set.Poss_2A());

    Serial.print(" || ENC3: "); Serial.print(encoder4set.Poss_3A());

    Serial.print(" || ENC4: "); Serial.println(encoder4set.Poss_4A());

    delay(10);  // แสดงผลทุก 200 มิลลิวินาที
   */
    
  }