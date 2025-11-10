// PID parameters
float Kpm = 1.05;
float Kim = 0.0025;
float Kdm = 0.5;

float pid_integral = 0;
float pid_lastError = 0;

float pidCompute(float target, float current) {
  float error = target - current;
  pid_integral += error;
  float derivative = error - pid_lastError;
  pid_lastError = error;

  return (Kpm * error) + (Kim * pid_integral) + (Kdm * derivative);
}

float pidCompute_break(float target, float current) {
  float error = target - current;
  pid_integral += error;
  float derivative = error - pid_lastError;
  pid_lastError = error;

  return (1.5 * error) + (Kim * pid_integral) + (Kdm * derivative);
}

void goDistance(int speedPercent, float distanceMM) {
    long targetPulses = distanceMM * 1.5; // แปลง mm -> encoder pulses

    // รีเซ็ต gyro และ encoder
    my_GYRO::resetAngles();
    pid_integral = 0;
    pid_lastError = 0;
    encoder4set.resetEncoders();  

    int currentSpeed = 0; // สำหรับ Ramp-up

    while (true) {
        long leftPulses  = encoder4set.Poss_1A();
        long rightPulses = encoder4set.Poss_3A();
        long avgPulses   = (leftPulses + rightPulses) / 2;

        if (avgPulses >= targetPulses) break; // ถึงเป้าหมายแล้ว

        // Ramp-up: ค่อย ๆ เพิ่มความเร็ว
        if (currentSpeed < speedPercent) currentSpeed++;

        float angle = my.gyro('z');       // อ่านมุม gyro
        float correction = pidCompute(0, angle);

        // ขับหุ่นยนต์โดยปรับ PID
        Motor('B',  currentSpeed - correction); // FL
        Motor('A',  currentSpeed - correction); // RL
        Motor('C',  currentSpeed + correction); // FR
        Motor('D',  currentSpeed + correction); // RR

        delay(2);
    }

    // Ramp-down / เบรกนุ่ม
    for (int i = speedPercent; i > 0; i--) {
        float angle = my.gyro('z');
        float correction = pidCompute(0, angle);

        Motor('B',  i - correction);
        Motor('A',  i - correction);
        Motor('C',  i + correction);
        Motor('D',  i + correction);

        delay(2);
    }

    // เบรกเล็กน้อย
    Motor('B', -(currentSpeed+5) );
    Motor('A', -(currentSpeed+5) );
    Motor('C', -(currentSpeed+5) );
    Motor('D', -(currentSpeed+5) );
    delay(30);
   

    // หยุดมอเตอร์
    unsigned long startTime = millis();
    while (millis() - startTime < 200) 
      {
        float angle = my.gyro('z');       // อ่านมุม gyro
        float correction = pidCompute_break(0, angle);

        // ขับหุ่นยนต์โดยปรับ PID
        Motor('B',  0 - correction); // FL
        Motor('A',  0 - correction); // RL
        Motor('C',  0 + correction); // FR
        Motor('D',  0 + correction); // RR

        delay(2);
        String _gyro = String(my.gyro('z'));
        mydisplay_background(black);
        mydisplay("MY-MAKERS", 25, 10, 2, white);
        mydisplay("Gyro "+ _gyro, 10, 40, 2, white);
        if(int(my.gyro('z'))==0)
          {
            break;
          }
    }
     Motor('B', -5);
    Motor('A', -5);
    Motor('C', -5);
    Motor('D', -5);
    
    delay(500); 
     Motor('B', 2);
    Motor('A', 2);
    Motor('C', 2);
    Motor('D', 2);
    
    delay(200); 
}

void righ_distance(int speedPercent, float distanceMM) {
    long targetPulses = distanceMM * 2.2; // แปลง mm -> encoder pulses

    // รีเซ็ต gyro และ encoder
    //my_GYRO::resetAngles();
    pid_integral = 0;
    pid_lastError = 0;
    encoder4set.resetEncoders();  

    int currentSpeed = 0; // สำหรับ Ramp-up

    while (true) {
        long leftPulses  = abs(encoder4set.Poss_3A());
        long rightPulses = abs(encoder4set.Poss_4A());
        long avgPulses   = (leftPulses + rightPulses) / 2;

        if (avgPulses >= targetPulses) break; // ถึงเป้าหมายแล้ว

        // Ramp-up: ค่อย ๆ เพิ่มความเร็ว
        if (currentSpeed < speedPercent) currentSpeed++;

        float angle = my.gyro('z');       // อ่านมุม gyro
        float correction = pidCompute(0, angle);

          Motor('B',  speedPercent - correction);
          Motor('A', -(speedPercent + correction));

          // ขวา: FR(C), RR(D)
          Motor('C', speedPercent + correction);
          Motor('D', -(speedPercent - correction));
        }

        // หยุดมอเตอร์ทุกล้อ
        Motor('B',  -speedPercent);
        Motor('A', speedPercent);

          // ขวา: FR(C), RR(D)
        Motor('C', -speedPercent);
        Motor('D', speedPercent);
        delay(50);
         
    // หยุดมอเตอร์
     unsigned long startTime = millis();
     while (millis() - startTime < 200) 
      {
        float angle = my.gyro('z');       // อ่านมุม gyro
        float correction = pidCompute_break(0, angle);

        // ขับหุ่นยนต์โดยปรับ PID
        Motor('B',  0 - correction); // FL
        Motor('A',  0 - correction); // RL
        Motor('C',  0 + correction); // FR
        Motor('D',  0 + correction); // RR

        delay(2);
        String _gyro = String(my.gyro('z'));
        mydisplay_background(black);
        mydisplay("MY-MAKERS", 25, 10, 2, white);
        mydisplay("Gyro "+ _gyro, 10, 40, 2, white);
        if(int(my.gyro('z'))==0)
          {
            break;
          }
    }
    Motor('B', -1);
        Motor('A', -1);
        Motor('C', -1);
        Motor('D', -1);
        delay(200);
    delay(500); 
}

void blak_Distance(int speedPercent, float distanceMM) {
    long targetPulses = distanceMM * 1.5; // แปลง mm -> encoder pulses

    // รีเซ็ต gyro และ encoder
    my_GYRO::resetAngles();
    pid_integral = 0;
    pid_lastError = 0;
    encoder4set.resetEncoders();  

    int currentSpeed = 0; // สำหรับ Ramp-up

    while (true) {
        long leftPulses  = abs(encoder4set.Poss_1A());
        long rightPulses = abs(encoder4set.Poss_3A());
        long avgPulses   = (leftPulses + rightPulses) / 2;

        if (avgPulses >= targetPulses) break; // ถึงเป้าหมายแล้ว

        // Ramp-up: ค่อย ๆ เพิ่มความเร็ว
        if (currentSpeed < speedPercent) currentSpeed++;

        float angle = my.gyro('z');       // อ่านมุม gyro
        float correction = pidCompute(0, angle);

        // ขับหุ่นยนต์โดยปรับ PID
        Motor('B',  -(currentSpeed + correction)); // FL
        Motor('A',  -(currentSpeed + correction)); // RL
        Motor('C',  -(currentSpeed - correction)); // FR
        Motor('D',  -(currentSpeed - correction)); // RR

        delay(2);
    }

    // Ramp-down / เบรกนุ่ม
    for (int i = speedPercent; i > 0; i--) {
        float angle = my.gyro('z');
        float correction = pidCompute(0, angle);

        Motor('B',  -(i + correction)); // FL
        Motor('A',  -(i + correction)); // RL
        Motor('C',  -(i - correction)); // FR
        Motor('D',  -(i- correction)); // RR

        delay(2);
    }

    // เบรกเล็กน้อย
    Motor('B', currentSpeed+5 );
    Motor('A', currentSpeed+5 );
    Motor('C', currentSpeed+5 );
    Motor('D', currentSpeed+5 );
    delay(30);
   

    // หยุดมอเตอร์
    unsigned long startTime = millis();
    while (millis() - startTime < 200) 
      {
        float angle = my.gyro('z');       // อ่านมุม gyro
        float correction = pidCompute_break(0, angle);

        // ขับหุ่นยนต์โดยปรับ PID
        Motor('B',  0 + correction); // FL
        Motor('A',  0 + correction); // RL
        Motor('C',  0 - correction); // FR
        Motor('D',  0 - correction); // RR

        delay(2);
        String _gyro = String(my.gyro('z'));
        mydisplay_background(black);
        mydisplay("MY-MAKERS", 25, 10, 2, white);
        mydisplay("Gyro "+ _gyro, 10, 40, 2, white);
        if(int(my.gyro('z'))==0)
          {
            break;
          }
    }
     Motor('B', 5);
    Motor('A', 5);
    Motor('C', 5);
    Motor('D', 5);
    
    delay(500); 
     Motor('B', 2);
    Motor('A', 2);
    Motor('C', 2);
    Motor('D', 2);
    
    delay(100); 
}

void stop() {
  Motor('B', 0);   Motor('C',  0); // FR
  Motor('A',  0);   Motor('D', 0); // RR
}