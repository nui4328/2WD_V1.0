
void Move_forward (int spl, int spr, float kps, int targetDistanceCm) 
{ 
    encoder.resetEncoders();  
    
    float targetPulses = targetDistanceCm * pulsesPerCm;

    Motor(-1, -1); delay(10);
    my.resetAngles();

    float yaw_offset = my.gyro('z');
    _integral = 0;
    _prevErr = 0;
    prevT = millis();    

    // === พารามิเตอร์ Ramp-up ===
    const int startSpeed = 15;              // ความเร็วเริ่มต้น (ต่ำเพื่อออกตัวนุ่ม)
    const float rampUpPulses = targetPulses * 0.4;  // เร่งใน 20% แรกของระยะทาง
    int currentMaxSpeed = startSpeed;       // ความเร็วสูงสุดปัจจุบัน (จะเพิ่มขึ้นเรื่อย ๆ)

    while (true) 
    {
        float leftPulses = encoder.Poss_L();
        float rightPulses = encoder.Poss_R();
        float currentPulses = (leftPulses + rightPulses) / 2;

        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001;
        prevT = now;

        float yaw = my.gyro('z') - yaw_offset;
        float err = yaw;

        _integral += err * dt;
        float deriv = (err - _prevErr) / dt;
        _prevErr = err;
        float corr = kps * err + 0.00001 * _integral + 0.035 * deriv;

        // === Ramp-up: ค่อย ๆ เพิ่มความเร็วสูงสุด ===
        if (currentPulses < rampUpPulses) {
            float rampFactor = currentPulses / rampUpPulses;
            currentMaxSpeed = startSpeed + (int)((spl - startSpeed) * rampFactor);
            if (currentMaxSpeed > spl) currentMaxSpeed = spl;
        } else {
            currentMaxSpeed = spl;  // หลัง 20% แรก → เต็มสปีด
        }

        // ความเร็วซ้าย-ขวา แยกกัน (chopsticks style)
        int baseLeftSpeed = currentMaxSpeed;
        int baseRightSpeed = (spr * currentMaxSpeed) / spl;  // รักษาอัตราส่วน spr/spl

        int leftSpeed = constrain(baseLeftSpeed - (int)corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + (int)corr, -100, 100);

        Motor(leftSpeed, rightSpeed);
        
        if (digitalRead(0) == 0) {
            
                    // เจอตะเกียบแน่นอน!
                    Motor(-20, -20);
                    delay(20);
                    Motor(-1, -1);
                    delay(200);

                    tft.fillScreen(BLACK);
                    tft.setTextSize(2);
                    tft.setTextColor(GREEN);
                    tft.setCursor(20, 25);
                    tft.print("Chopsticks");

                    Move_shop(35, 35, 1.4, shop_distance );  // วิ่งผ่านแรง ๆ

                    sett_f = false;
                    set_bb = false;
                    break;
                
           
        }
        // === หยุดเมื่อถึงเป้า ===
        if (currentPulses >= targetPulses) 
            {
                Motor(-1, -1);
                delay(100);
                moveLR(70, 90);      // หมุนขวา 90°
                sett_f = false;
                set_bb = false;
                break;
            }
        
          
        // === ตรวจสี ===
        if ( color.readFast() ) 
           {
            String col = color.getColor();
            //Serial.println(col);
            
            if (col == "RED")
                {
                    Motor(-spl, -spr);
                    delay(50);
                    bz(50);
                    Motor(-1, -1);
                    delay(100);
                    mydisplay_background(RED);
                    do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                    Motor(10, 10);
                    delay(20);
                    set_f(1);
                    
                    servo_red();
                    set_f(2);
                    bw_chop(30, 30, 1.2,miss_distance, "none_line");
                    moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                    sett_f = false;
                    set_bb = false;
                    set_move_before_moveLR(retreat_distance);
                    break;

                }
            else if (col == "GREEN")
                {
                    Motor(-spl, -spr);
                    delay(50);
                    bz(50);bz(50);
                    Motor(-1, -1);
                    delay(100);
                    mydisplay_background(GREEN);
                    do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                    Motor(10, 10);
                    delay(20);
                    set_f(1);
                    
                    servo_green();
                    set_f(2);
                    bw_chop(30, 30, 1.2,miss_distance, "none_line");
                    moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                    sett_f = false;
                    set_bb = false;
                    
                    break;

                }
            else if (col == "BLUE")
                {
                    Motor(-spl, -spr);
                    delay(50);
                    bz(50);bz(50);
                    Motor(-1, -1);
                    delay(100);
                    mydisplay_background(BLUE);
                    do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                    Motor(10, 10);
                    delay(20);
                    set_f(1);
                    
                    servo_blue();
                    set_f(2);
                    bw_chop(30, 30, 1.2,miss_distance, "none_line");
                    moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                    sett_f = false;
                    set_bb = false;
                    
                    break;

                }
             else if (col == "YELLOW")
                {
                    Motor(-spl, -spr);
                    delay(20);
                    bz(50); bz(50);
                    Motor(-1, -1);
                    delay(100);
                    mydisplay_background(YELLOW);

                    // เดินหน้าเข้าไปให้เซ็นเซอร์อยู่กลางพื้นเหลืองชัด ๆ (ปรับตามระยะจริงของหุ่นคุณ)
                    fw_chopsticks(25, 25, 1.4, 10, "none_line"); 
                    col = "NONE";
                    mydisplay_background(WHITE);
                    Motor(-1, -1);
                    delay(200);  // รอให้หุ่นนิ่ง เซ็นเซอร์อ่านค่าคงที่

                    // === Confirm สีเหลืองด้วยการตรวจ ratio ดิบโดยตรง (ไม่พึ่ง getColor()) ===
                    int yellow_count = 0;
                    for (int i = 0; i < 8; i++) 
                      {
                        if (color.readFast() ) 
                        {
                            if (color.getColor() == "YELLOW") 
                            {
                                yellow_count++;
                                bz(100);
                            }
                        }
                        delay(40);
                     }
                      
                    // === ตัดสินใจ ===
                  if (color.readFast() ) 
                        {
                    if (yellow_count >= 2 || color.getColor() == "YELLOW")  // เจออย่างน้อย 5/10 ครั้ง → ถือว่าเป็นพื้นเหลืองจริง
                    {
                        bz(200); bz(200);  // เสียงยืนยันว่ากำลังเก็บ

                        bw_chop(30, 30, 1.2, 10, "none_line");
                        Motor(1, 1);
                        delay(100);
                        servo_yellow();  // ปล่อยแขนเก็บกล่อง

                        bw_chop(30, 30, 1.2, 18, "none_line");
                        moveLR(70, -90);  // หมุนซ้าย 90° (ปรับทิศตาม layout สนามจริง)

                        yello_box = 1;    // สำคัญ: ตั้ง flag ว่ากเก็บเหลืองแล้ว
                        sett_f = false;
                        set_bb = false;

                       
                      }
                      else
                      {
                          // ไม่ใช่เหลืองจริง → แค่แตะขอบ → หมุนเลี่ยง
                          bz(500);  // เสียงเตือน false positive
                          moveLR(70, 90);   // หมุนขวา 90°
                          ch_poit++;
                          sett_f = false;
                          set_bb = false; // เช็คว่าครบ 4 สี + แต้มหรือยัง
                          if(red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
                              {
                                  servo(20, servo20-90);
                                  bz(300);bz(300);bz(300);
                                  runMenuSystem();  // เข้าเมนูทันที (จะบล็อกจนกว่าจะกด RUN)     
                              }
                              delay(200);
                      }
                    }

                    break;
                }
                   
          
           }
        
        // === ตรวจเส้นหน้า ===
        if (readSensor(1) < md_sensor(1)+100 || readSensor(2) < md_sensor(2)+100)
          {                    
            Motor(-spl, -spr);
            delay(30);
            Motor(-1, -1);
            delay(200);
            
            while(1)
                {
                   Motor(-15, -15); 
                   if (readSensor(1) > md_sensor(1)+100 && readSensor(2) > md_sensor(2)+100)
                    {
                        delay(60);
                        break;
                    }
                }
            
            set_f(2);
            do{ Motor(12, 12);} while(readSensor(2) > md_sensor(2)-30);
            Motor(-20, -20);
            delay(30);
            Motor(-1, -1);
            delay(500);
            if ( color.readFast() ) 
                {
                    String col = color.getColor();
                    //Serial.println(col);
                    if (col == "RED")
                        {
                            mydisplay_background(RED);
                            do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                            Motor(10, 10);
                            delay(20);
                            set_f(1);
                            
                            servo_red();
                            set_f(2);
                            bw_chop(30, 30, 1.2,18, "none_line");
                            moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                            sett_f = false;
                            set_bb = false;
                            
                            break;

                        }
                    else if (col == "GREEN")
                        {
                            mydisplay_background(GREEN);
                            do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                            Motor(10, 10);
                            delay(20);
                            set_f(1);
                            
                            servo_green();
                            set_f(2);
                            bw_chop(30, 30, 1.2,miss_distance, "none_line");
                            moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                            sett_f = false;
                            set_bb = false;                            
                            break;

                            }
                    else if (col == "BLUE")
                        {
                            mydisplay_background(BLUE);
                            do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                            Motor(10, 10);
                            delay(20);
                            set_f(1);
                            
                            servo_blue();
                            set_f(2);
                            bw_chop(30, 30, 1.2,miss_distance, "none_line");
                            moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
                            sett_f = false;
                            set_bb = false;
                                break;

                            }
                }
            moveLR(70, -90);      // หมุนซ้าย 90° (หรือปรับตามต้องการ)
            sett_f = false;
            set_bb = false;
            break;
        } 
    }

    Motor(-1, -1);
    delay(10);
}
