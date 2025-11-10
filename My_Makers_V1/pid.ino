
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
    encoder4set.resetEncoders();
    lines_fw = true;  
    lines_bw = false;  

    if (set_bb == true) {
        targetDistanceCm = targetDistanceCm + 5;
        set_bb = false;
    }

    // คำนวณจำนวนพัลส์เป้าหมายจากระยะทาง
    float targetPulses = targetDistanceCm * pulsesPerCm;

    // รีเซต Motor และ Gyro
    Motor('A', -1) ;  Motor('C', -1) ;
    Motor('B', -1) ;  Motor('D', -1) ;
    delay(10);

     my_GYRO::resetAngles();

    // *** เริ่มต้นตั้งค่าตัวแปรสำหรับ PID ***
    
    _integral = 0;
    _prevErr = 0;
    prevT = millis();

    // เตรียมตัวสำหรับการเร่งช้าๆ

    float rampUpDistance = targetPulses * 0.2;   // ช่วงเร่ง 20% แรก
    float rampDownDistance = targetPulses * 0.6; // ช่วงเริ่มผ่อน 20% ท้าย
    int minSpeed = 12; // กำหนดสปีดขั้นต่ำ
    if(targetDistanceCm  >= 60)
      {
        rampUpDistance = targetPulses * 0.1;   // ช่วงเร่ง 20% แรก
        rampDownDistance = targetPulses * 0.7; // ช่วงเริ่มผ่อน 20% ท้าย
        minSpeed = 12; // กำหนดสปีดขั้นต่ำ
      }
    int maxLeftSpeed = spl;
    int maxRightSpeed = spr;

    while (true) {
      //Serial.print("ENC1: "); Serial.print(encoder4set.Poss_1A());
      //Serial.print(" || ENC2: "); Serial.println(encoder4set.Poss_3A());
        // อ่านค่าจาก Encoder
        float leftPulses = encoder4set.Poss_1A();
        float rightPulses = encoder4set.Poss_3A();

        // คำนวณระยะทางที่เคลื่อนที่แล้ว
        float currentPulses = (leftPulses + rightPulses) / 2;
        float remainingPulses = targetPulses - currentPulses;

        // อ่านเวลา
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; // ป้องกันหาร 0
        prevT = now;

        // อ่าน Gyro และคำนวณ Error
        //float yaw = my.gyro('z');
        float err = my.gyro('z');
         Serial.println(err);
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
        Motor('A', leftSpeed) ;  Motor('C', rightSpeed) ;
        Motor('B', leftSpeed) ;  Motor('D', rightSpeed) ;
        
       // Serial.println(yaw); // Debug ดูค่า yaw

        if (currentPulses >= targetPulses) {
            break;
          }
        
        if (analogRead(40) < md_sensor(40)-30 && analogRead(43) > md_sensor(43)) 
          {
            
            Motor('A', leftSpeed) ;  Motor('C', rightSpeed/2) ;
            Motor('B', leftSpeed) ;  Motor('D', rightSpeed/2) ;
             my_GYRO::resetAngles();
          } 
        else if (analogRead(40) > md_sensor(40) && analogRead(43) < md_sensor(43)-30) 
          {
            Motor('A', leftSpeed/2) ;  Motor('C', rightSpeed) ;
            Motor('B', leftSpeed/2) ;  Motor('D', rightSpeed) ;
             my_GYRO::resetAngles();
          }
        else if (analogRead(41) < md_sensor(41) - 50 ) 
          {
            Motor('A', -20) ;  Motor('C', -20) ;
            Motor('B', -20) ;  Motor('D', -20) ;
            delay(20);  

            Motor('A', -1) ;  Motor('C', -1) ;
            Motor('B', -1) ;  Motor('D', -1) ;
            delay(10);  

            while (1) {
                if (analogRead(40) < md_sensor(40)- 50 && analogRead(43) > md_sensor(43)) 
                  {
                    lr = 'l';
                    Motor('A', -10) ;  Motor('C', 30) ;
                    Motor('B', -10) ;  Motor('D', 30) ;    
                  }
                else if (analogRead(40) > md_sensor(40) && analogRead(43) < md_sensor(43)- 50) 
                  {
                    lr = 'r';
                    Motor('A', 30) ;  Motor('C', -10) ;
                    Motor('B', 30) ;  Motor('D', -10) ; 
                  }
                else if ((analogRead(40) < md_sensor(40) - 50&& analogRead(43) < md_sensor(43)- 50)) 
                  {
                    if (lr == 'l') 
                      {
                        Motor('A', 15) ;  Motor('C', -15) ;
                        Motor('B', 15) ;  Motor('D', -15) ;
                        delay(20);  

                        Motor('A', -1) ;  Motor('C', -1) ;
                        Motor('B', -1) ;  Motor('D', -1) ;
                        delay(10); 
                        break; 
                      }
                    else if (lr == 'r') 
                      {
                        Motor('A', -15) ;  Motor('C', 15) ;
                        Motor('B', -15) ;  Motor('D', 15) ;
                        delay(20);  

                        Motor('A', -1) ;  Motor('C', -1) ;
                        Motor('B', -1) ;  Motor('D', -1) ;
                        break; 
                      }
                    else 
                      {
                        Motor('A', -15) ;  Motor('C', -15) ;
                        Motor('B', -15) ;  Motor('D', -15) ;
                        delay(20);  

                        Motor('A', -1) ;  Motor('C', -1) ;
                        Motor('B', -1) ;  Motor('D', -1) ;
                        Motor(0, 0); delay(10);
                        break;
                      }
                  }
                else 
                  {
                    Motor('A', motor_slow) ;  Motor('C', motor_slow) ;
                    Motor('B', motor_slow) ;  Motor('D', motor_slow) ;
                  }
            }   
            encoder4set.resetEncoders();
            do{Motor('A', -motor_slow) ;  Motor('C', -motor_slow) ;
              Motor('B', -motor_slow) ;  Motor('D', -motor_slow) ;}while(encoder4set.Poss_1A() > -fw_to_rotate);
            
            Motor('A', motor_slow) ;  Motor('C', motor_slow) ;
            Motor('B', motor_slow) ;  Motor('D', motor_slow) ;
            Motor('A', -1) ;  Motor('C', -1) ;
            Motor('B', -1) ;  Motor('D', -1) ;
            Motor(0, 0); delay(10);             
            break;                  
        }
      
    }
/*
    // พอถึงระยะ ต้องตรวจเส้นหรือไม่
    if (_line == "line") {
        while (1) {
            Motor(motor_slow, motor_slow);

            if (analogRead(40) < md_sensor(40) - 50 && analogRead(43) > md_sensor(43)) {                    
                Motor(40, -10);
            }
            else if (analogRead(40) > md_sensor(40) && analogRead(43) < md_sensor(43) - 50 ){
                Motor(-10, 40);
            }
            else if (analogRead(41) < md_sensor(41) - 50 ) {
                Motor(-30, -30); delay(30);  
                Motor(-1, -1); delay(10);  

                while (1) {
                    if (analogRead(40) < md_sensor(40)- 50 && analogRead(43) > md_sensor(43)) {
                        lr = 'l';
                        Motor(-10, 30);        
                    }
                    else if (analogRead(40) > md_sensor(40) && analogRead(43) < md_sensor(43)- 50) {
                        lr = 'r';
                        Motor(30, -10);           
                    }
                    else if ((analogRead(40) < md_sensor(40) - 50&& analogRead(43) < md_sensor(43)- 50)) {
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
        delay(10);
        lines = false;
    }
*/
    sett_f = false;
    set_bb = false;
}
