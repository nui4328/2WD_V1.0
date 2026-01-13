#ifndef _MY_SENSOR_H_
#define _MY_SENSOR_H_

#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_ST7735.h"
#include  <my_GYRO1600.h>
#include "../My2WD_V1/my_MCP3008.h"
#include <EncoderLibrarys.h>


my_MCP3008 adc;
Adafruit_ST7735 tft = Adafruit_ST7735(15, 14, -1);
EncoderLibrarys encoder(11, 10, 24, 23);
my_GYRO1600 my; // สร้างอ็อบเจ็กต์ด้วยที่อยู่เริ่มต้น (0x69)

#define NUM_SENSORS    8
#define NUM_SAMPLES    502
#define EEPROM_START   0        // ที่อยู่เริ่มต้นใน EEPROM



// ค่า min/max ที่อ่านจาก EEPROM
uint16_t sensor_min[NUM_SENSORS];
uint16_t sensor_max[NUM_SENSORS];
uint16_t analog_min[2] = {0, 0};  // GP26, GP27
uint16_t analog_max[2] = {0, 0};

float errors, P, I = 0, D, previous_error = 0;
float present_position;
float setpoint = 50.0;
uint32_t _lastPosition = 3500;

// สีสำหรับจอ
const uint16_t BLACK   = 0xFFFF;
const uint16_t WHITE   = 0x0000;
const uint16_t RED     = 0xFFE0;  // แดงเต็ม
const uint16_t GREEN   = 0xF81F;  // เขียวเต็ม
const uint16_t BLUE    = 0x07FF;  // น้ำเงินเต็ม
const uint16_t YELLOW  = 0xF800;  // เหลือง = แดง + เขียวเต็ม
const uint16_t CYAN    = 0x001F;  // ฟ้า = เขียว + น้ำเงิน
const uint16_t MAGENTA = 0x07E0;  // ม่วง = แดง + น้ำเงิน
const uint16_t ORANGE  = 0xFD20;  // ส้ม
const uint16_t PINK    = 0xF81F;  // ชมพู (เหมือน MAGENTA)
const uint16_t GRAY    = 0x8410;  // เทา
const uint16_t LIGHT_GREEN = 0x07EF; // เขียวอ่อน

const uint16_t LIGHT_GRAY  = 0xC618;   // เพิ่มบรรทัดนี้ (แก้ error LIGHT_GRAY)



// ค่าคงที่สำหรับการคำนวณ
const float wheelDiameter = 4.1;                      // เส้นผ่านศูนย์กลางล้อ (เซนติเมตร)
const float wheelCircumference = PI * wheelDiameter;  // เส้นรอบวงล้อ
const int pulsesPerRevolution = 420;                  // จำนวนพัลส์ต่อรอบ
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
int fw_to_rotate = 180;

// กำหนดค่า PID
float Kp = 0.05;  // Proportional gain
float Ki = 0.000015; // Integral gain
float Kd = 0.015;  // Derivative gain

// ตัวแปรสำหรับ PID Control
float previousError = 0;
float integral = 0;


unsigned long lastTime = millis();
bool error_servo  = false;
void encoderISR(void) ;
void set_f(int _time);

float lr_kp, lr_ki, lr_kd;
float l_kp, l_ki, l_kd;
float r_kp, r_ki, r_kd;
bool chopsticks  = false;
unsigned long lastTimes = millis();
float Kpp = 0.35, Kii = 0.00001, Kdd = 0.03;
float _integral = 0, _prevErr = 0;
unsigned long prevT;
float error_moveLR , output_moveLR;

bool exitCompletely = false;  // ตัวแปรควบคุมการออกจากฟังก์ชันทันทีเมื่อหมุนเกินองศา


int set_dis = 10;

void dis_to_stand(int diss)
  {
    set_dis = diss;
  }
void set_move_before_moveLR(int _val)
  {
    fw_to_rotate = _val;
  }

void Acceptable_values_moveLR(float errors_moveLR,  float outputs_moveLR)
  {
    error_moveLR = errors_moveLR;
    output_moveLR = outputs_moveLR;
  }
void set_pid_moveLR(float _lr_kp, float _lr_ki, float _lr_kd)
  {
    lr_kp = _lr_kp;
    lr_ki = _lr_ki;
    lr_kd = _lr_kd;
  }

  void set_pid_moveL(float _lr_kp, float _lr_ki, float _lr_kd)
  {
    l_kp = _lr_kp;
    l_ki = _lr_ki;
    l_kd = _lr_kd;
  }

  void set_pid_moveR(float _lr_kp, float _lr_ki, float _lr_kd)
  {
    r_kp = _lr_kp;
    r_ki = _lr_ki;
    r_kd = _lr_kd;
  }

void set_pid_chopsticks(float _lr_kp, float _lr_ki, float _lr_kd)
  {
    Kpp= _lr_kp;
    Kii = _lr_ki;
    Kdd = _lr_kd;
  }
// ------------------------------------------------------------------
// ฟังก์ชันช่วยเหลือ
// ------------------------------------------------------------------
void bz(int dl) {
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH); delay(dl);
  digitalWrite(21, LOW); delay(dl / 2);
}

void mydisplay(String text, int x, int y, int size, uint16_t color) {
  tft.setCursor(x, y);
  tft.setTextSize(size);
  tft.setTextColor(color);
  tft.print(text);
}

void mydisplay_background(uint16_t color) {
  tft.fillScreen(color);
}

// อ่านค่า MCP3008 (เซ็นเซอร์หน้า)
uint16_t readSensor(uint8_t ch) {
  adc.begin(5, 4, 12, 13);  // CLK, DOUT, DIN, CS
  return adc.readADC(ch);
}

void add_sensor_F() {
  bz(100); bz(100);
  mydisplay_background(BLACK);
  mydisplay("Calibrating Front", 10, 20, 2, WHITE);
  mydisplay("Move on Black & WHITE", 5, 45, 1, WHITE);
  mydisplay("Time: 5.0s", 50, 65, 1, WHITE);

  // รีเซ็ตค่า min/max ให้เป็นค่าตัวอย่างแรก
  uint16_t current;
  for (int i = 0; i < NUM_SENSORS; i++) {
    current = readSensor(i);
    sensor_min[i] = current;
    sensor_max[i] = current;
  }
  analog_min[0] = analog_max[0] = analogRead(26);
  analog_min[1] = analog_max[1] = analogRead(27);

  unsigned long startTime = millis();
  unsigned long endTime = startTime + 5000;  // calibrate 5 วินาทีเต็ม
  unsigned long lastDisplay = startTime;
  unsigned long sampleCount = 0;

  while (millis() < endTime) {
    // อ่านเซ็นเซอร์ MCP3008 ทั้ง 8 ตัว (เร็วที่สุด)
    for (int i = 0; i < NUM_SENSORS; i++) {
      current = readSensor(i);
      if (current < sensor_min[i]) sensor_min[i] = current;
      if (current > sensor_max[i]) sensor_max[i] = current;
    }

    // อ่าน analog บนบอร์ด GP26 และ GP27
    current = analogRead(26);
    if (current < analog_min[0]) analog_min[0] = current;
    if (current > analog_max[0]) analog_max[0] = current;

    current = analogRead(27);
    if (current < analog_min[1]) analog_min[1] = current;
    if (current > analog_max[1]) analog_max[1] = current;

    sampleCount++;

    // อัพเดทจอแสดงเวลานับถอยหลังทุก 200ms
    if (millis() - lastDisplay >= 200) {
      float remaining = (endTime - millis()) / 1000.0;
      char buf[20];
      sprintf(buf, "Time: %.1fs", remaining);
      tft.fillRect(50, 65, 100, 12, BLACK);  // ลบข้อความเก่า
      mydisplay(buf, 50, 65, 1, WHITE);
      lastDisplay = millis();
    }
  }

  // บันทึกลง EEPROM (20 ค่า)
  uint16_t data[20] = {
    analog_max[0], analog_min[0], analog_max[1], analog_min[1],
    sensor_max[0], sensor_max[1], sensor_max[2], sensor_max[3],
    sensor_max[4], sensor_max[5], sensor_max[6], sensor_max[7],
    sensor_min[0], sensor_min[1], sensor_min[2], sensor_min[3],
    sensor_min[4], sensor_min[5], sensor_min[6], sensor_min[7]
  };

  for (int i = 0; i < 20; i++) {
    EEPROM.put(EEPROM_START + i * 2, data[i]);
  }
  EEPROM.commit();

  // เสร็จสิ้น
  bz(500); delay(200); bz(100); bz(100);
  mydisplay_background(BLACK);
  mydisplay("CAL COMPLETE!", 20, 20, 2, WHITE);

  char buf[40];
  sprintf(buf, "Samples: ~%lu", sampleCount);
  mydisplay(buf, 20, 50, 1, WHITE);

  delay(2000);  // ให้ดูผลได้สักพัก
}
// ------------------------------------------------------------------
// อ่านค่า min/max จาก EEPROM
// ------------------------------------------------------------------
void loadCalibration() {
  uint16_t data[20];
  for (int i = 0; i < 20; i++) {
    EEPROM.get(EEPROM_START + i * 2, data[i]);
  }
  analog_max[0] = data[0]; analog_min[0] = data[1];
  analog_max[1] = data[2]; analog_min[1] = data[3];
  for (int i = 0; i < 8; i++) {
    sensor_max[i] = data[4 + i];
    sensor_min[i] = data[12 + i];
  }
}
uint16_t md_analog(uint8_t pin) {
  if (pin == 26) {
    return (analog_max[0] + analog_min[0]) / 2;
  } else if (pin == 27) {
    return (analog_max[1] + analog_min[1]) / 2;
  }
  return 0;
}

// ------------------------------------------------------------------
// ค่ากลางของเซ็นเซอร์ MCP3008 หน้า (ช่อง 0-7)
// ------------------------------------------------------------------
uint16_t md_sensor(uint8_t channel) {
  if (channel >= NUM_SENSORS) return 0;
  return (sensor_max[channel] + sensor_min[channel]) / 2;
}


// ------------------------------------------------------------------
// ------------------------------------------------------------------
// คำนวณตำแหน่งเส้น (Position)
// ------------------------------------------------------------------
uint16_t Position() {
  bool onLine = false;
  long avg = 0, sum = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t raw = readSensor(i);
    long value = map(raw, sensor_min[i], sensor_max[i], 1000, 0);
    if (value > 200) onLine = true;
    if (value > 50) {
      avg += value * (i * 1000L);
      sum += value;
    }
  }

  if (!onLine) return 3500;  // หลุดเส้น

  _lastPosition = avg / sum;
  return _lastPosition;
}

// ------------------------------------------------------------------
// คำนวณ Error และ PID พื้นฐาน
// ------------------------------------------------------------------
float error() {
  present_position = Position() / 70.0;  // (8-1)*10 = 70
  errors = setpoint - present_position;

  P = errors;
  I += errors;
  D = errors - previous_error;
  previous_error = errors;

  return errors;
}

float kp() { return P; }
float ki() { return I; }
float kd() { return D; }

// ------------------------------------------------------------------
// Setup เบื้องต้น
// ----------------------------------------------------------------------------------------------------SENSOR_SET
void sensor_set() {
  EEPROM.begin(2048);
  analogReadResolution(12);
  my.begin();
  my.resetAngles();

  tft.initR(INITR_MINI160x80);
  tft.setRotation(3);
  tft.fillScreen(WHITE);

  encoder.setupEncoder();
  encoder.resetEncoders();
  mydisplay_background(BLACK);
  mydisplay("MYMAKERS", 10, 20, 2, WHITE);

  loadCalibration();  // โหลดค่าที่ calibrate ไว้

  
      analogWriteResolution(12);
      analogWriteFreq(20000);
      pinMode(7,OUTPUT);
      pinMode(6,OUTPUT);
      pinMode(8,OUTPUT);            
      pinMode(22,OUTPUT); 
      pinMode(18,OUTPUT);
      pinMode(19,OUTPUT);
}
// ----------------------------------------------------------------------------------------------------SENSOR_SET

void robot_start()
  {
    bz(100);
    bz(100); 
    digitalWrite(25, 1);
    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;   

    while(1)
      {       
        for(int i = 0; i<8; i++)
          {
            Serial.print(readSensor(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" "); 
        buttonState = digitalRead(9);
        if (buttonState == LOW) 
          {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
            digitalWrite(25, 0);
            if (!isPressed) \
              {
                pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
                isPressed = true;
              } 
            else 
              {
                unsigned long pressDuration = millis() - pressStartTime;    
                if (pressDuration >= 3000) 
                  {  // กดค้าง 3 วินาที
                    Serial.println("Entering Mode A");
                    add_sensor_F();
                    while (digitalRead(9) == LOW);  // รอให้ปล่อยปุ่ม
                    delay(200);  // ป้องกันการเด้งของปุ่ม
                  }
              }
          } 
        else 
          {
            if (isPressed) 
              {
                unsigned long pressDuration = millis() - pressStartTime;
                
                if (pressDuration >= 50 && pressDuration < 3000) 
                  {  
                    Serial.println("Entering Mode B");
                    break;
                  }
                isPressed = false;
              }
          }
      }
    bz(500);
    mydisplay_background(BLACK);
    mydisplay("End", 5, 10 ,2, WHITE);    
  }



int sl = 0;
int sr = 0;

void Motor(int spl,int spr)    
   {   
      Serial.print(spl);Serial.print("  "); Serial.println( spl );  
    delayMicroseconds(50); 
      sl = map(spl, -100, 100, -4095,4095);
      sr = map(spr, -100, 100, -4095,4095);      

    if(sr>4095)
      sr = 4095;
      else if(sr<-4095)
      sr = -4095;
               
                  if(sl>4095)
                  sl = 4095;
                  else if(sl<-4095)
                  sl = -4095;
               
                  if(sl>0)
                     {
                        digitalWrite(8,HIGH);
                        digitalWrite(6,LOW);     
                        analogWrite(7,sl);
                     }
                  else if(sl<0)
                     {    
                        digitalWrite(8,LOW);
                        digitalWrite(6,HIGH);
                        analogWrite(7,-sl);
                     }
                  else
                     {        
                        digitalWrite(6,LOW);
                        digitalWrite(8,LOW);
                        analogWrite(7,0);
                     }  
            
                  if(sr>0)
                     {
                        digitalWrite(19,HIGH);
                        digitalWrite(18,LOW);
                        analogWrite(22,sr);
                     }
                  else if(sr<0)
                     {    
                        digitalWrite(19,LOW);
                        digitalWrite(18,HIGH);
                        analogWrite(22,-sr);
                     }
                  else
                     {        
                        digitalWrite(18,LOW);
                        digitalWrite(19,LOW);
                        analogWrite(22,0);
                     }              
    }



    ///------------------------------------------------------------------------------------------------>>>> หมุนซ้ายขวา
// ===================================================================
// หมุนหุ่นยนต์ซ้าย-ขวา ตามมุมที่ต้องการ (degree)
// รวมการถอยออกจากเส้นก่อนหมุน เพื่อป้องกันติดเส้นหรือกำแพง
// speed  : ความเร็วสูงสุดในการหมุน (แนะนำ 40-70)
// degree : มุมที่ต้องการหมุน (+ = ขวา, - = ซ้าย)
// ===================================================================
void moveLR(int speed, int degree) {
  const int  BACKUP_SPEED    = 28;     // ความเร็วถอย (20-35 ดีที่สุด)
  const int  BACKUP_PULSES   = 140;    // จำนวน pulse ที่ถอย
  const float TOLERANCE      = 2.8;    // หยุดเมื่อ error < 2.8°
  const int   TIMEOUT_MS     = 300;   // สูงสุด 3.5 วินาที (แก้จาก 500 → 3500)

  // ==================== ถอยออกจากเส้น ====================
  bool needForwardBackup  = (lines && lines_fw) || sett_f;
  bool needBackwardBackup = (lines && lines_bw) || set_bb;

  if (needForwardBackup || needBackwardBackup) {
    encoder.resetEncoders();

    if (needForwardBackup) {
      // ถอยไปข้างหน้า (มอเตอร์หมุนถอยหลัง)
      do {
        Motor(-BACKUP_SPEED, -BACKUP_SPEED);
      } while (encoder.Poss_L() > -fw_to_rotate );

      Motor(20, 20);  delay(20);
      Motor(-1, -1);  delay(10);
    }
    else if (needBackwardBackup) {
      // ถอยไปข้างหลัง (มอเตอร์หมุนเดินหน้า)
      do {
        Motor(BACKUP_SPEED, BACKUP_SPEED);
      } while (encoder.Poss_L() < fw_to_rotate );

      Motor(-20, -20); delay(20);
      Motor(1, 1);     delay(10);
    }
  }
  else {
    // ไม่เจอเส้น → เบรกเบา ๆ
    Motor(-2, -2); delay(10);
  }

  // เบรกก่อนเริ่มหมุนจริง
  Motor(-1, -1);
  delay(40);

  // ==================== หมุนด้วย Gyro + PID ====================
  my.resetAngles();  // รีเซ็ตมุมทั้งหมด
  delay(10);

  // อ่าน initial angle แบบเฉลี่ย (แม่นยำ)
  float initialAngle = 0;
  for (int i = 0; i < 10; i++) {
    initialAngle += my.gyro('z');
    delay(2);
  }
  initialAngle /= 20.0;

  float targetAngle = initialAngle + degree;

  float error = 0, lastError = 0, integral = 0, output = 0;
  unsigned long lastTime = millis();
  unsigned long startTime = millis();

  Serial.print("Turning ");
  Serial.print(degree > 0 ? "RIGHT" : "LEFT");
  Serial.print(" ");
  Serial.print(abs(degree));
  Serial.println("°");

  while (true) {
    float currentAngle = my.gyro('z');
    error = targetAngle - currentAngle;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    if (dt > 0.001) {
      integral += error * dt;
      integral = constrain(integral, -80, 80);  // anti-windup

      float derivative = (error - lastError) / dt;
      lastError = error;
      if(degree > 0)
        {
          output = r_kp * error + r_ki * integral + r_kd * derivative;
        }
      else  
        {
          output = l_kp * error + l_ki * integral + l_kd * derivative;
        }
      
      output = constrain(output, -speed, speed);
    }

    Motor((int)output, (int)-output);  // หมุนตาม PID

    // เงื่อนไขหยุดแม่นยำ
    if (abs(error) < TOLERANCE && abs(output) < 12) {
      // เบรกแบบ opposite direction นิดนึงเพื่อหยุดสนิท
      if (degree > 0) {
        Motor(-1, 1);
      } else {
        Motor(1, -1);
      }
      delay(40);
      break;
    }

    // Timeout ป้องกันค้าง
    if (millis() - startTime > TIMEOUT_MS) {
      Serial.println("Turn Timeout!");
      if (degree > 0) {
        Motor(-1, 1);
      } else {
        Motor(1, -1);
      }
      delay(40);
      break;
    }

    delay(5);
  }

  // ==================== หยุดสนิท + Quick Re-calibrate ====================
  Motor(0, 0);          // หยุดมอเตอร์สนิท
  delay(50);           // รอ vibration หายเล็กน้อย

  Serial.println("Quick re-calibrating gyro after turn...");
  my.reCalibrateGyro();         // calibrate offset ใหม่ (เร็ว ~150 ms)
         // รีเซ็ต bias และ low-pass filter สนิท → ไม่ลอยแน่นอน!

 // Serial.println("Gyro fully stabilized! Ready for next move.");

  // ==================== แสดงผลลัพธ์ ====================
  float finalAngle = my.gyro('z');
  float actualTurn = finalAngle - initialAngle;
/*
  Serial.print("Turn Complete! Actual: ");
  Serial.print(actualTurn, 1);
  Serial.print("° (Target: ");
  Serial.print(degree);
  Serial.print("°) Error: ");
  Serial.print(abs(actualTurn - degree), 1);
  Serial.println("°");
*/
}





// ฟังก์ชัน PID Control
int computePID(float setpoint, float currentValue) 
{
    // คำนวณ error
    float error = setpoint - currentValue;
    
    // คำนวณ integral
    integral += error;
    
    // คำนวณ derivative
    float derivative = error - previousError;
    
    // คำนวณ output
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    if(chopsticks  == true)
      {
        output = (Kp*1.5 * error) + (Ki * integral) + (Kd * derivative);
      }
   
    
    
    // อัพเดต previousError
    previousError = error;
    
    return (int)output;
}

void fw(int spl, int spr, float kps, int targetDistanceCm, String _line) 
{  
    char lr;
    encoder.resetEncoders();
    lines_fw = true;  
    lines_bw = false;  

    if (set_bb == true) {
        targetDistanceCm = targetDistanceCm + 5;
        set_bb = false;
    }

    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซต Motor และ Gyro
    Motor(-1, -1); delay(10);
     my.resetAngles();

    // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
    float yaw_offset = my.gyro('z'); // << เก็บค่าตอนเริ่มต้น
    _integral = 0;
    _prevErr = 0;
    prevT = millis();

    // เตรียมตัวสำหรับการเร่งช้าๆ

    float rampUpDistance = targetPulses * 0.1;   // ช่วงเร่ง 20% แรก
    float rampDownDistance = targetPulses * 0.7; // ช่วงเริ่มผ่อน 20% ท้าย
    int minSpeed = 30; // กำหนดสปีดขั้นต่ำ
    if(targetDistanceCm  > 60)
      {
        rampUpDistance = targetPulses * 0.1;   // ช่วงเร่ง 20% แรก
        rampDownDistance = targetPulses * 0.8; // ช่วงเริ่มผ่อน 20% ท้าย
        minSpeed = 30; // กำหนดสปีดขั้นต่ำ
      }
    int maxLeftSpeed = spl;
    int maxRightSpeed = spr;

    while (true) {
        // อ่านค่าจาก Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว
        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - currentPulses;

        // อ่านเวลา
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; // ป้องกันหาร 0
        prevT = now;

        // อ่าน Gyro และคำนวณ Error
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        // PID
        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + Kii * _integral + Kdd * deriv;

        // คำนวณสปีดพื้นฐาน
        int baseLeftSpeed = maxLeftSpeed;
        int baseRightSpeed = maxRightSpeed;

        // ทำ Ramp-up และ Ramp-down
        if (currentPulses < rampUpDistance) {
            float rampFactor = currentPulses / rampUpDistance;
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }
        else if (currentPulses > rampDownDistance) {
            float rampFactor = (targetPulses - currentPulses) / (targetPulses - rampDownDistance);
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }

        // สั่งมอเตอร์ โดยชดเชย PID correction
        int leftSpeed = constrain(baseLeftSpeed - corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + corr, -100, 100);

        Motor(leftSpeed, rightSpeed);
        
       // Serial.println(yaw); // Debug ดูค่า yaw

        if (currentPulses >= targetPulses) {
            break;
          }
        if (readSensor(0) < md_sensor(0)-30 && readSensor(3) > md_sensor(3)) 
          {
            Motor(leftSpeed, rightSpeed/2);
             my.resetAngles();
          } 
        else if (readSensor(0) > md_sensor(0) && readSensor(3) < md_sensor(3)-30) 
          {
            Motor(leftSpeed/2, rightSpeed);
             my.resetAngles();
          }
        else if (readSensor(1) < md_sensor(1) - 50 || readSensor(2) < md_sensor(2) - 50) 
          {
            Motor(-30, -30); delay(30);  
            Motor(-1, -1); delay(10);  

            while (1) {
                if (readSensor(1) < md_sensor(1)- 50 && readSensor(2) > md_sensor(2)) {
                    lr = 'l';
                    Motor(-10, 30);        
                }
                else if (readSensor(1) > md_sensor(1) && readSensor(2) < md_sensor(2)- 50) {
                    lr = 'r';
                    Motor(30, -10);           
                }
                else if ((readSensor(0) < md_sensor(0) - 50&& readSensor(3) < md_sensor(3)- 50)
                         || (readSensor(1) < md_sensor(1)- 50 && readSensor(2) < md_sensor(2)- 50)) {
                    if (lr == 'l') {
                        Motor(15, -15); delay(20);
                        Motor(1, -1); delay(10);
                        Motor(0, 0); delay(10);
                        break; 
                    }
                    if (lr == 'r') {
                        Motor(-15, 15); delay(20);
                        Motor(-1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break; 
                    }
                    else {
                        Motor(-15, -15); delay(20);
                        Motor(1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break;
                    }
                }
                else {
                    Motor(motor_slow, motor_slow);
                }
            }   
            encoder.resetEncoders();
            do{Motor(-20, -20);}while(encoder.Poss_L() > -fw_to_rotate);
            Motor(20, 20); delay(20);
            Motor(-1, -1);
            delay(10);             
            break;                  
        }
    }

    // พอถึงระยะ ต้องตรวจเส้นหรือไม่
    if (_line == "line") {
        while (1) {
            Motor(motor_slow, motor_slow);

            if (readSensor(0) < md_sensor(0) - 50 && readSensor(3) > md_sensor(3)) {                    
                Motor(40, -10);
            }
            else if (readSensor(0) > md_sensor(0) && readSensor(3) < md_sensor(3) - 50 ){
                Motor(-10, 40);
            }
            else if (readSensor(1) < md_sensor(1) - 50 || readSensor(2) < md_sensor(2) - 50) {
                Motor(-30, -30); delay(30);  
                Motor(-1, -1); delay(10);  

                while (1) {
                    if (readSensor(1) < md_sensor(1)- 50 && readSensor(2) > md_sensor(2)) {
                        lr = 'l';
                        Motor(-10, 30);        
                    }
                    else if (readSensor(1) > md_sensor(1) && readSensor(2) < md_sensor(2)- 50) {
                        lr = 'r';
                        Motor(30, -10);           
                    }
                    else if ((readSensor(0) < md_sensor(0) - 50&& readSensor(3) < md_sensor(3)- 50)
                             || (readSensor(1) < md_sensor(1)- 50 && readSensor(2) < md_sensor(2)- 50)) {
                        if (lr == 'l') {
                            Motor(15, -15); delay(20);
                            Motor(1, -1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        if (lr == 'r') {
                            Motor(-15, 15); delay(20);
                            Motor(-1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        else {
                            Motor(-15, -15); delay(20);
                            Motor(1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break;
                        }
                    }
                    else {
                        Motor(motor_slow, motor_slow);
                    }
                }                  
                break;                  
            }                  
        }
        lines = true;
    } 
    else {
        Motor(-1, -1);
        delay(50);
        lines = false;
    }

    sett_f = false;
    set_bb = false;
}


void bw(int spl, int spr, float kps, int targetDistanceCm, String _line) 
{  
   char lr; // ตัวแปรเก็บทิศทางซ้าย/ขวา
    encoder.resetEncoders(); // รีเซตค่า Encoder
    lines_fw = false; // ไม่ใช่การเดินหน้า
    lines_bw = true; // เป็นการถอยหลัง

    // ถ้ามีการตั้งค่า set_bb ให้เพิ่มระยะทาง 10 ซม.
    if (set_bb == true || sett_f == true) {
        targetDistanceCm = targetDistanceCm + 10;
        set_bb = false;
    }

    // แปลงระยะทางเป็นจำนวนพัลส์
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซตมอเตอร์และไจโร
    Motor(1, 1); delay(10); // หยุดมอเตอร์ชั่วคราว
     my.resetAngles(); // รีเซทมุมไจโร

    // ตั้งค่าตัวแปร PID
    float yaw_offset = my.gyro('z'); // เก็บค่า yaw เริ่มต้น
    _integral = 0; // รีเซต integral
    _prevErr = 0; // รีเซต error ก่อนหน้า
    prevT = millis(); // เก็บเวลาเริ่มต้น

    // กำหนดช่วงเร่งและลดความเร็ว
    float rampUpDistance = targetPulses * 0.2;   // ช่วงเร่ง 20% แรก
    float rampDownDistance = targetPulses * 0.6; // ช่วงเริ่มผ่อน 20% ท้าย
    int minSpeed = 15; // กำหนดสปีดขั้นต่ำ
    if(targetDistanceCm  > 60)
      {
        rampUpDistance = targetPulses * 0.1;   // ช่วงเร่ง 20% แรก
        rampDownDistance = targetPulses * 0.8; // ช่วงเริ่มผ่อน 20% ท้าย
        minSpeed = 15; // กำหนดสปีดขั้นต่ำ
      }
    int maxLeftSpeed = spl; // ความเร็วสูงสุดซ้าย
    int maxRightSpeed = spr; // ความเร็วสูงสุดขวา

    while (true) {
        // อ่านค่า Encoder
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว (ใช้ค่า absolute)
        float currentPulses = abs((leftPulses + rightPulses) / 2);
        float remainingPulses = targetPulses - currentPulses;

        // อ่านเวลาและคำนวณ delta time
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; // ป้องกันการหารด้วย 0
        prevT = now;

        // อ่านค่าไจโรและคำนวณ error (กลับทิศ)
        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw; // กลับทิศของ yaw เพื่อแก้สลับทิศ

        // ดีบัก: พิมพ์ค่า yaw เพื่อตรวจสอบ
        Serial.print("Yaw: "); Serial.println(err);

        // คำนวณ PID
        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + Kii * _integral + Kdd * deriv;

        // กำหนดความเร็วพื้นฐาน
        int baseLeftSpeed = maxLeftSpeed;
        int baseRightSpeed = maxRightSpeed;

        // ปรับความเร็วแบบ Ramp-up และ Ramp-down
        if (currentPulses < rampUpDistance) {
            float rampFactor = currentPulses / rampUpDistance;
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }
        else if (currentPulses > rampDownDistance) {
            float rampFactor = (targetPulses - currentPulses) / (targetPulses - rampDownDistance);
            baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
            baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
        }

        // สั่งมอเตอร์ (ถอยหลัง, กลับเครื่องหมาย corr)
        int leftSpeed = constrain(baseLeftSpeed + corr, -100, 100); // เปลี่ยน - เป็น +
        int rightSpeed = constrain(baseRightSpeed - corr, -100, 100); // เปลี่ยน + เป็น -
        Motor(-leftSpeed, -rightSpeed); // ความเร็วเป็นลบเพื่อถอยหลัง

        // ตรวจสอบว่าถึงระยะทางเป้าหมายหรือยัง
        if (currentPulses >= targetPulses) {
            break;
        }

        // ตรวจ MCP (ปรับทิศ)
        if (readSensor(4) < md_sensor(4) - 30 && readSensor(7) > md_sensor(7)) {
            Motor(-(leftSpeed / 3), -rightSpeed);
             my.resetAngles();
        } 
        else if (readSensor(4) > md_sensor(4) && readSensor(7) < md_sensor(7) - 30) {
            Motor(-leftSpeed, -(rightSpeed / 3));
             my.resetAngles();
        }

        else if (readSensor(5) < md_sensor(5) - 50 || readSensor(6) < md_sensor(6) - 50) 
        {
            Motor(30, 30); delay(30); // เคลื่อนไปข้างหน้าเล็กน้อย
            Motor(1, 1); delay(10); // หยุดชั่วคราว

            while (1) {
                if (readSensor(5) < md_sensor(5)-50 && readSensor(6) > md_sensor(6)) {
                    lr = 'l';
                    Motor(10, -30); // หมุนซ้าย
                }
                else if (readSensor(5) > md_sensor(5) && readSensor(6) < md_sensor(6)-50) {
                    lr = 'r';
                    Motor(-30, 10); // หมุนขวา
                }
                else if ((readSensor(4) < md_sensor(4)-50 && readSensor(7) < md_sensor(7)-50)
                         || (readSensor(5) < md_sensor(5)-50 && readSensor(6) < md_sensor(6)-50)) {
                    if (lr == 'l') {
                        Motor(-15, 15); delay(20); // หมุนปรับ
                        Motor(-1, 1); delay(10);
                        Motor(0, 0); delay(10);
                        break;
                    }
                    if (lr == 'r') {
                        Motor(15, -15); delay(20); // หมุนปรับ
                        Motor(1, -1); delay(10);
                        Motor(0, 0); delay(10);
                        break;
                    }
                    else {
                        Motor(15, 15); delay(20); // ถอยเล็กน้อย
                        Motor(-1, -1); delay(10);
                        Motor(0, 0); delay(10);
                        break;
                    }
                }
                else {
                    Motor(-motor_slow, -motor_slow); // ถอยช้าๆ
                }
            }   
            encoder.resetEncoders();
            do {Motor(20, 20);} while(abs(encoder.Poss_L()) < fw_to_rotate); // เคลื่อนไปข้างหน้า
            Motor(-20, -20); delay(20); // ถอยเล็กน้อย
            Motor(1, 1); delay(10); // หยุด
            break;
        }
    }

    // ถ้าต้องจับเส้นต่อ
    if (_line == "line") 
      {
        while (1) 
          {
            Motor(-motor_slow, -motor_slow);

            if (readSensor(4) < md_sensor(4) - 50 && readSensor(7) > md_sensor(7)) {                    
                Motor(-2, -40);
            }
            else if (readSensor(4) > md_sensor(4) && readSensor(7) < md_sensor(7) - 50 ){
                Motor(-40, -2);
            }
            else if (readSensor(5) < md_sensor(5)  || readSensor(6) < md_sensor(6) ) {
                Motor(30, 30); delay(20);  
                Motor(1, 1); delay(10);  

                while (1) {
                    if (readSensor(5) < md_sensor(5) && readSensor(6) > md_sensor(6)) {
                        lr = 'l';                        
                        Motor(-20, 5);         
                    }
                    else if (readSensor(5) > md_sensor(5) && readSensor(6) < md_sensor(6)) {
                        lr = 'r';
                        Motor(5, -20);         
                    }
                    else if ((readSensor(4) < md_sensor(4) - 50 && readSensor(7) < md_sensor(7))
                             || (readSensor(5) < md_sensor(5)- 50 && readSensor(6) < md_sensor(6))) {
                        if (lr == 'l') {
                            Motor(-15, 15); delay(10);
                            Motor(1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        if (lr == 'r') {
                            Motor(15, -15); delay(10);
                            Motor(-1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break; 
                        }
                        else {
                            Motor(15, 15); delay(20);
                            Motor(1, 1); delay(10);
                            Motor(0, 0); delay(10);
                            break;
                        }
                    }
                    else {
                        Motor(-motor_slow, -motor_slow);
                    }
                }                  
                break;                  
            }                  
        }
        lines = true;
      } 
    else {
        Motor(1, 1); delay(20);
        lines = false;
    }

    sett_f = false;
    set_bb = false;
}

void fw_chopsticks(int spl, int spr, float kps, int targetDistanceCm, String _line) 
  {  
    chopsticks  = true;
    char lr ;
   
    encoder.resetEncoders();
    lines_fw = true;  
    lines_bw = false;  
    if(set_bb == true)
      {
        targetDistanceCm = targetDistanceCm + 10;
        set_bb = false;
      }   
      // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
      encoder.resetEncoders();
    
      // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
      float targetPulses = targetDistanceCm * pulsesPerCm;
  
      // รีเซต Motor และ Gyro
      Motor(-1, -1); delay(10);
       my.resetAngles();
  
      // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
      float yaw_offset = my.gyro('z'); // << เก็บค่าตอนเริ่มต้น
      _integral = 0;
      _prevErr = 0;
      prevT = millis();    
  
      // เตรียมตัวสำหรับการเร่งช้าๆ
      float rampUpDistance = targetPulses * 0.2;   // ช่วงเร่ง 20% แรก
      float rampDownDistance = targetPulses * 0.8; // ช่วงเริ่มผ่อน 20% ท้าย
      int minSpeed = 10; // กำหนดสปีดขั้นต่ำ
      int maxLeftSpeed = spl;
      int maxRightSpeed = spr;
  
      while (true) {
          // อ่านค่าจาก Encoder
          float leftPulses = encoder.Poss_L();
          float rightPulses = encoder.Poss_R();
  
          // คำนวณระยะทางที่เคลื่อนที่แล้ว
          float currentPulses = (leftPulses + rightPulses) / 2;
          float remainingPulses = targetPulses - currentPulses;
  
          // อ่านเวลา
          unsigned long now = millis();
          float dt = (now - prevT) / 1000.0;
          if (dt <= 0) dt = 0.001; // ป้องกันหาร 0
          prevT = now;
  
          // อ่าน Gyro และคำนวณ Error
          float yaw = my.gyro('z') - yaw_offset;
          float err = yaw;
  
          // PID
          _integral += err * dt;
          float deriv = (err - _prevErr) / dt;
          _prevErr = err;
          float corr = kps * err + 0.00001 * _integral + 0.035 * deriv;
  
          // คำนวณสปีดพื้นฐาน
          int baseLeftSpeed = maxLeftSpeed;
          int baseRightSpeed = maxRightSpeed;
  
          // ทำ Ramp-up และ Ramp-down
          if (currentPulses < rampUpDistance) {
              float rampFactor = currentPulses / rampUpDistance;
              baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
              baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
          }
          else if (currentPulses > rampDownDistance) {
              float rampFactor = (targetPulses - currentPulses) / (targetPulses - rampDownDistance);
              baseLeftSpeed = minSpeed + (maxLeftSpeed - minSpeed) * rampFactor;
              baseRightSpeed = minSpeed + (maxRightSpeed - minSpeed) * rampFactor;
          }
  
          // สั่งมอเตอร์ โดยชดเชย PID correction
          int leftSpeed = constrain(baseLeftSpeed - corr, -100, 100);
          int rightSpeed = constrain(baseRightSpeed + corr, -100, 100);
  
          Motor(leftSpeed, rightSpeed);
  
              
              if (currentPulses >= targetPulses) {
                          break;
                      }   
           
     }

   if(_line == "line")
      {  
        while(1)      
           {    
              Motor(motor_slow, motor_slow);        
              if(readSensor(0) < md_sensor(0)-30 && readSensor(3) > md_sensor(3))
                  {                    
                    Motor(40, -10);
                  }
              else if(readSensor(0) > md_sensor(0) && readSensor(3) < md_sensor(3)-30)
                  {
                    Motor(-10, 40);
                  }
              else if(readSensor(1) < md_sensor(1) || readSensor(2) < md_sensor(2))
                  {
                    Motor(-30, -30); delay(30);  
                    Motor(-1, -1);delay(10);  
                    while(1)
                      {
                        if(readSensor(1) < md_sensor(1) && readSensor(2) > md_sensor(2)) 
                          {
                            lr = 'l';
                             Motor(-10, 30);        
                          }
                        else if(readSensor(1) > md_sensor(1) && readSensor(2) < md_sensor(2))
                          {
                            lr = 'r';
                            Motor(30, -10);           
                          }
                        else if(readSensor(0) < md_sensor(0) && readSensor(3) < md_sensor(3)
                              || readSensor(1) < md_sensor(1) && readSensor(2) < md_sensor(2))
                          {   
                            if(lr == 'l')
                              {
                                Motor(15, -15);delay(20);
                                Motor(1, -1);delay(10);
                                Motor(0, 0); delay(10);
                                break; 
                              }
                            if(lr == 'r')
                              {
                                Motor(-15, 15);delay(20);
                                Motor(-1, 1);delay(10);
                                Motor(0, 0); delay(10);
                                break; 
                              }
                             else
                              {
                                Motor(-15, -15);delay(20);
                                Motor(1, 1);delay(10);
                                Motor(0, 0); delay(10);
                                break;
                              }            
                          }
                         else
                          {
                            Motor(motor_slow, motor_slow);
                          }
                        }                  
                      break;                  
                      
                  }                  
          }
  
        lines = true;         
                           
      } 
    else
      {
        Motor(-1, -1);
        delay(50);
        lines = false;
      }   
    sett_f = false; 
    set_bb = false;
    chopsticks  = false; 
}

void set_f(int _time) {
  sett_f = true;
  lines = false;
  

  for (int i = 0; i < _time && !exitCompletely; i++) {
    my.resetAngles();  // รีเซ็ต gyro ทุกครั้งที่เริ่มรอบใหม่

    unsigned long startWhile = millis();
    
    while (millis() - startWhile < 4000) {  // timeout 3 วินาที ป้องกันค้าง
      int s1 = readSensor(1);
      int s2 = readSensor(2);
      int md1 = md_sensor(1);
      int md2 = md_sensor(2);

      // Case 1: ทั้งสองเซ็นเซอร์เจอดำ → หน้าตรงแล้ว → จบฟังก์ชันปกติ
      if (s1 < md1-50 && s2 < md2-50) {
        Motor(-20, -20); delay(20);  // เบรกเบา ๆ
        Motor(0, 0); delay(50);
        //exitCompletely = true;       // เจอเส้นแล้ว → จบเลย
        break;
      }

      // Case 2: ซ้ายเจอดำ ขวาเจอขาว → หมุนขวา
      else if (s1 < md1 && s2 >= md2) {
        Motor(-2, 30);  // หมุนขวา

        float angleZ = my.gyro('z');

        // หมุนขวาเกิน 10 องศา → จบฟังก์ชันทันที
        if (angleZ < -15.0) {
          Motor(0, 0); delay(50);
          exitCompletely = true;  // สั่งให้ออกจาก for และจบฟังก์ชันเลย
          break;
        }
      }

      // Case 3: ซ้ายเจอขาว ขวาเจอดำ → หมุนซ้าย
      else if (s1 >= md1 && s2 < md2) {
        Motor(30, -2);  // หมุนซ้าย

        float angleZ = my.gyro('z');

        // หมุนซ้ายเกิน 10 องศา → จบฟังก์ชันทันที
        if (angleZ > 15.0) {
          Motor(0, 0); delay(50);
          my.resetAngles();  // รีเซ็ต gyro ทุกครั้งที่เริ่มรอบใหม่
          do{Motor(-25, 15);delay(5);}while(my.gyro('z') > -15);
          
          Motor(0, 0); delay(50);

          exitCompletely = true;  // สั่งให้ออกจาก for และจบฟังก์ชันเลย
          break;
        }
      }

      // Case 4: ไม่เจอเส้นเลย → เดินหน้าช้า ๆ หาเส้น
      else {
        Motor(15, 15);
      }

      // ถ้าสั่ง exitCompletely แล้ว ให้ออกจาก while ทันที
      if (exitCompletely) {
        break;
      }
    }

    // ถ้าออกเพราะหมุนเกินองศา หรือเจอเส้นแล้ว → ไม่ต้องวนรอบต่อไป
    if (exitCompletely) {
      break;
    }

    // เตรียมรอบถัดไป (กรณีปกติที่ยังไม่เจอเส้นและยังไม่เกินองศา)
    if (i < _time - 1) {
      Motor(-20, -20); delay(50);
      Motor(20, 20); delay(20);
      Motor(0, 0); delay(20);
    }
  }

  // สิ้นสุดฟังก์ชัน: หยุดมอเตอร์เสมอ
  Motor(0, 0);
  delay(50);
  lines = false;
}

void set_b(int _time)
  {    
    set_bb = true;
    for(int i = 0; i<_time; i++)
      {               
        while(1)
          {
            if(readSensor(4) < md_sensor(4) && readSensor(7) > md_sensor(7)) 
              {
                ch_lrs = 'l';
                 Motor(-20, 5);        
              }
            else if(readSensor(4) > md_sensor(4) && readSensor(7) < md_sensor(7))
              {
                ch_lrs = 'r';
                Motor(5, -20);           
              }
           else if(readSensor(7) < md_sensor(7) && readSensor(4) < md_sensor(4))
              {   
                if(ch_lrs == 'l')
                  {
                     Motor(15, 5);delay(20);
                  }
                else
                  {
                     Motor(5, 15);delay(20);
                  }
                Motor(-1, -1); delay(50);
                break;            
              }
            else
              {
                Motor(-20, -20);
              }
          }
        if(i < _time-1)
          {
            while(1)
              {
                Motor(15, 15);
                if(readSensor(7) > md_sensor(7) && readSensor(4) > md_sensor(4))
                  {
                    break;
                  }
              }
            while(1)
              {
                Motor(15, 15);
                if(readSensor(5) > md_sensor(5) && readSensor(6) > md_sensor(6))
                  {
                    Motor(-5, -5); delay(10);
                    break;
                  }
              }
          }
      }
     my.resetAngles();
    lines = false;
      
  }

#endif