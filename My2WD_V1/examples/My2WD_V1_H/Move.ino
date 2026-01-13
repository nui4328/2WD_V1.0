


void bw_chop(int spl, int spr, float kps, int targetDistanceCm, String _line) 
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


void Move_start (int spl, int spr, float kps, int targetDistanceCm) 
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
    const int startSpeed = 13;              // ความเร็วเริ่มต้น (ต่ำเพื่อออกตัวนุ่ม)
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

        // === หยุดเมื่อถึงเป้า ===
        if (currentPulses >= targetPulses) 
        {
            Motor(-1, -1);
            delay(200);
            moveLR(70, 90);      // หมุนขวา 90°
            sett_f = false;
            set_bb = false;
            break;
        }
        
    }

    Motor(-1, -1);
    delay(10);
}
