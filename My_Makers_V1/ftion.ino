
/*
void sw()
  {
    
    delay(200); // รอ 1 วินาที
    while(digitalRead(24) == 1)
      {
          
          delay(10);  // อัปเดตทุก 100ms
          
      }
    tone(34, 3050, 400);
    delay(500);
  }
*/
void sw()
  {
    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;   

    while(1)
      {           
        buttonState = digitalRead(24);
        for(int i=40; i<48; i++)
          {
            Serial.print(analogRead(i)); 
            Serial.print("  "); 
          }
        Serial.println("  "); 
        
        if (buttonState == LOW) 
          {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
            digitalWrite(rgb[2],0);
            
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
                    digitalWrite(rgb[1],1);
                    tone(32, 950, 100);
                    delay(200); // รอ 1 วินาที
                    tone(32, 950, 200);
                    delay(200); // รอ 1 วินาที
                    Serial.println("Entering Mode A");
                    get_maxmin_A();
                    digitalWrite(rgb[1],0);
                    delay(100);
                    digitalWrite(rgb[1],1);
                    delay(100);
                    digitalWrite(rgb[1],0);
                    delay(300);;
                    digitalWrite(rgb[2],1);
                    while (digitalRead(24) == LOW);  // รอให้ปล่อยปุ่ม
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
    tone(34, 3200, 500);
    delay(500);
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

void moveLR(int speed, int degree) 
  {
  my_GYRO::resetAngles();
  delay(10);

  // อ่านมุมเริ่มต้นแบบเฉลี่ย
  float initialDegree = 0;
  for (int i = 0; i < 5; i++) {
    initialDegree += my.gyro('z');
    delay(5);
  }
  initialDegree /= 5.0;

  // ตั้งมุมเป้าหมาย
  float targetDegree = initialDegree + degree;
  float error = 0, previous_error = 0;
  float integral = 0, output = 0;

  unsigned long lastTime = millis();
  unsigned long timeout = 500;
  unsigned long startTime = millis();
  //my_GYRO::resetAngles(); 
  while (true) {
    float currentDegree = my.gyro('z');
    error = targetDegree - currentDegree;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    if (dt > 0) {
      integral += error * dt;
      float derivative = (error - previous_error) / dt;
      previous_error = error;
      output = lr_kp * error + lr_ki * integral + lr_kd * derivative;
    }

    // 🔸 จำกัด output เมื่อใกล้มุมเป้าหมาย เพื่อเบรกไม่ให้เลย
    if (abs(error) < 25) {
      output = constrain(output, -8, 8);
    } else {
      output = constrain(output, -speed, speed);
    }
    Motor('A', -output) ;  Motor('C', output) ;
    Motor('B', -output) ;  Motor('D', output) ;
    //Motor(output, -output); // ซ้ายบวก ขวาลบ    
    // 🔸 ปรับเงื่อนไขหยุดให้แม่นขึ้น
    if (abs(error) < 1 && abs(output) < 2) break;
    if (millis() - startTime > timeout) break;

  }

  // หยุดมอเตอร์
  if(degree > 0)
    {
      Motor('A', 20) ;  Motor('C', -20) ;
      Motor('B', 20) ;  Motor('D', -20) ;
      delay(20);
    }
  else  
    {
      Motor('A', -20) ;  Motor('C', 20) ;
      Motor('B', -20) ;  Motor('D', 20) ;
      delay(20);
    }
  Motor('A', -1) ;  Motor('C', -1) ;
  Motor('B', -1) ;  Motor('D', -1) ;
  delay(10);
  }

void servo_open()
  {
    servo(39,130);
  }
