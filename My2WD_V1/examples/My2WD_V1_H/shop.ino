
void Move_shop (int spl, int spr, float kps, int targetDistanceCm) 
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
