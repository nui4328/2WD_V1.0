// Flag global (ประกาศด้านนอกฟังก์ชัน เช่นในไฟล์หลัก)
bool just_collected_box = false;

void Moves_forward(int spl, int spr, float kps, int targetDistanceCm) 
{ 
    encoder.resetEncoders();  
    
    float targetPulses = targetDistanceCm * pulsesPerCm;

    Motor(-1, -1); delay(10);
    my.resetAngles();

    float yaw_offset = my.gyro('z');
    _integral = 0;
    _prevErr = 0;
    prevT = millis();    

    // === พารามิเตอร์ Ramp-up (ปรับให้ออกตัวเร็วกว่าเดิม) ===
    const int startSpeed = 20;                  // เพิ่มจาก 13 → 20
    const float rampUpPulses = targetPulses * 0.3;  // เร่งใน 30% แรก (เร็วขึ้น)
    int currentMaxSpeed = startSpeed;

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

        // === Ramp-up ===
        if (currentPulses < rampUpPulses) {
            float rampFactor = currentPulses / rampUpPulses;
            currentMaxSpeed = startSpeed + (int)((spl - startSpeed) * rampFactor);
            if (currentMaxSpeed > spl) currentMaxSpeed = spl;
        } else {
            currentMaxSpeed = spl;
        }

        int baseLeftSpeed = currentMaxSpeed;
        int baseRightSpeed = (spr * currentMaxSpeed) / spl;

        int leftSpeed = constrain(baseLeftSpeed - (int)corr, -100, 100);
        int rightSpeed = constrain(baseRightSpeed + (int)corr, -100, 100);

        Motor(leftSpeed, rightSpeed);

        // 1. เช็คถึงเป้าก่อนเลย
        if (currentPulses >= targetPulses) 
        {
            Motor(-1, -1);
            delay(100);
            moveLR(70, 90);      // หมุนขวา 90°
            sett_f = false;
            set_bb = false;
            just_collected_box = false;  // รีเซ็ต flag
            break;
        }

        // 2. เช็คตะเกียบ (เร็วที่สุด)
        if (digitalRead(0) == 0) 
        {
            Motor(-20, -20);
            delay(20);
            Motor(-1, -1);
            delay(150);  // ลดจาก 200

            tft.fillScreen(BLACK);
            tft.setTextSize(2);
            tft.setTextColor(GREEN);
            tft.setCursor(20, 25);
            tft.print("Chopsticks");

            Move_start(35, 35, 1.4, 60);

            sett_f = false;
            set_bb = false;
            just_collected_box = false;
            break;
        }

        // 3. เช็คเส้นหน้า
        if (readSensor(1) < md_sensor(1)+100 || readSensor(2) < md_sensor(2)+100)
        {                    
            Motor(-spl, -spr);
            delay(30);
            Motor(-1, -1);
            delay(150);  // ลดจาก 200
            
            while(1)
            {
               Motor(-15, -15); 
               if (readSensor(1) > md_sensor(1)+100 && readSensor(2) > md_sensor(2)+100)
                {
                    delay(60);
                    break;
                }
            }
            
            set_f(3);
            do{ Motor(12, 12);} while(readSensor(2) > md_sensor(2)-30);
            Motor(-10, -10);
            delay(30);
            Motor(-1, -1);
            delay(300);  // ลดจาก 500

            // ตรวจสีหลังปรับเส้นหน้า
            if (color.readFast()) 
            {
                String col = color.getColor();
                if (col == "RED" || col == "GREEN" || col == "BLUE") 
                {
                    // จัดการเหมือนเดิม (RED/GREEN/BLUE)
                    mydisplay_background(col == "RED" ? RED : (col == "GREEN" ? GREEN : BLUE));
                    do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                    Motor(10, 10); delay(20);
                    set_f(1);
                    
                    if (col == "RED") servo_red();
                    else if (col == "GREEN") servo_green();
                    else servo_blue();
                    
                    set_f(2);
                    bw_chop(30, 30, 1.2, 16, "none_line");
                    moveLR(70, -90);
                    sett_f = false;
                    set_bb = false;
                    just_collected_box = true;  // ตั้ง flag
                    break;
                }
            }
            moveLR(70, -90);
            sett_f = false;
            set_bb = false;
            just_collected_box = false;
            break;
        } 

        // 4. เช็คสี - ตรวจได้ตลอดเวลา ตั้งแต่เริ่มวิ่งเลย (เหมาะกับการวิ่งไปวางกล่อง)
        if (color.readFast()) 
        {
            String col = color.getColor();

            if (col == "RED" || col == "GREEN" || col == "BLUE")
            {
                Motor(-spl, -spr); delay(50);
                bz(50);
                Motor(-1, -1); delay(80);
                mydisplay_background(col == "RED" ? RED : (col == "GREEN" ? GREEN : BLUE));

                do{ Motor(-20, -20);} while(readSensor(1) < md_sensor(1)-30);
                Motor(10, 10); delay(20);
                set_f(1);
                
                if (col == "RED") servo_red();       // ปล่อยกล่องแดง
                else if (col == "GREEN") servo_green();
                else servo_blue();
                
                set_f(2);
                bw_chop(30, 30, 1.2, 16, "none_line");  // ถอยออกจากจุดวาง
                moveLR(70, -90);                       // หมุนตาม layout สนาม
                sett_f = false;
                set_bb = false;
                set_move_before_moveLR(retreat_distance);
                break;
            }
            else if (col == "YELLOW")
            {
                Motor(-spl, -spr); delay(20);
                bz(50); bz(50);
                Motor(-1, -1); delay(80);
                mydisplay_background(YELLOW);

                fw_chopsticks(25, 25, 1.4, 10, "none_line");  // เดินหน้าเข้าไปกลางจุดเหลือง
                Motor(-1, -1); delay(150);

                // Confirm เหลือง (ยังคงไว้เพราะเหลือง fluctuate มาก)
                int yellow_count = 0;
                for (int i = 0; i < 8; i++) 
                {
                    if (color.readFast() && color.c > 200) 
                    {
                        if (color.getColor() == "YELLOW") 
                        {
                            yellow_count++;
                            bz(100);
                        }
                    }
                    delay(40);
                }

                if (yellow_count >= 4) 
                {
                    bz(200); bz(200);

                    bw_chop(30, 30, 1.2, 10, "none_line");
                    Motor(1, 1); delay(100);
                    servo_yellow();  // ปล่อยกล่องเหลือง

                    bw_chop(30, 30, 1.2, 25, "none_line");
                    moveLR(70, -90);

                    yello_box = 1;
                    sett_f = false;
                    set_bb = false;
                }
                else 
                {
                    bz(500);
                    moveLR(70, 90);  // เลี่ยงจุดที่ไม่ใช่เหลืองจริง
                    ch_poit++;
                    sett_f = false;
                    set_bb = false;

                    if (red_box == 1 && green_box == 1 && blue_box == 1 && yello_box == 1 && ch_poit >= 2)
                    {
                        servo(20, servo20-90);
                        bz(300); bz(300); bz(300);
                        runMenuSystem();
                    }
                    delay(150);
                }
                break;
            }
        }
    }

    Motor(-1, -1);
    delay(10);
}