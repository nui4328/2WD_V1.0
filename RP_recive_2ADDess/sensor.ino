
#define SLAVE_ADDRESS 0x10  // ที่อยู่ I2C ของ RP2350B
volatile int sensorValues1[8];  // ค่าจาก Nano 0x08
volatile int sensorValues2[8];  // ค่าจาก Nano 0x09
volatile uint8_t lastMaster = 0;

// อ่านค่าเซ็นเซอร์จาก Master 0x08 (A0–A7)
int read_sensorA(int sensor) {
  /*
  if (sensor < 0 || sensor > 7) {
    Serial.print("Error: Sensor index ");
    Serial.print(sensor);
    Serial.println(" out of range (0–7)");
    return -1;  // คืนค่า -1 หากอยู่นอกขอบเขต
  }
  */
  return sensorValues1[sensor];
}

// อ่านค่าเซ็นเซอร์จาก Master 0x09 (A0–A7)
int read_sensorB(int sensor) {
  /*
  if (sensor < 0 || sensor > 7) {
    Serial.print("Error: Sensor index ");
    Serial.print(sensor);
    Serial.println(" out of range (0–7)");
    return -1;  // คืนค่า -1 หากอยู่นอกขอบเขต
  }
  */
  return sensorValues2[sensor];
}

void _setup() {
  // ตั้งค่า I2C1 บน GPIO 14 (SDA), GPIO 15 (SCL)
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(32, OUTPUT);

  pinMode(32, OUTPUT);
  pinMode(33, INPUT_PULLUP);
   pinMode(3, OUTPUT);  // PWMA (มอเตอร์ซ้าย)
  pinMode(20, OUTPUT); // AIN1
  pinMode(21, OUTPUT); // AIN2
  pinMode(6, OUTPUT);  // PWMB (มอเตอร์ขวา)
  pinMode(22, OUTPUT); // BIN1
  pinMode(23, OUTPUT); // BIN2

   // ตั้งค่า PWM
  analogWriteResolution(12); // ความละเอียด 12 บิต (0-4095)
  analogWriteFreq(25000);    // ความถี่ PWM 2 kHz สำหรับ Coreless
  encoder.setupEncoder();    //-------------------->> เรียกฟังก์ชัน setupEncoder
  servo_set();
  
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  
  Wire1.begin(SLAVE_ADDRESS);    // เริ่ม I2C เป็น Slave
  Wire1.onReceive(receiveData);  // ตั้งฟังก์ชันรับข้อมูล
  
  Serial.begin(115200);
  Serial.println("RP2350B Slave Started (Address 0x10)");
  int rgb[] = {24, 25, 28};
  for(int i = 0; i<2; i++)
    {
      for(int i = 0; i<3; i++)
        {
          digitalWrite(rgb[i],1);
          delay(150);
          digitalWrite(rgb[i],0);
          delay(150);
        }
    }
  digitalWrite(rgb[1],1);
  
  
}

void receiveData(int byteCount) {
  if (byteCount == 17) {  // คาดหวัง 17 ไบต์ (header + 16 ไบต์)
    uint8_t masterId = Wire1.read();  // อ่าน header byte
    lastMaster = masterId;    
    volatile int* targetArray = (masterId == 0x08) ? sensorValues1 : sensorValues2;
    for (int i = 0; i < 8; i++) {
      int highByte = Wire1.read();
      int lowByte = Wire1.read();
      targetArray[i] = (highByte << 8) | lowByte;
    }
  }
}

void _loop() {
  // แสดงค่าจาก Nano 0x08
  Serial.print("From Master 0x08: ");
  for (int i = 0; i < 8; i++) {
    Serial.print("A");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(read_sensorA(i));  // ใช้ read_sensorA
    Serial.print(" ");
  }
  Serial.print("   ");
  
  // แสดงค่าจาก Nano 0x09
  Serial.print("From Master 0x09: ");
  for (int i = 0; i < 8; i++) {
    Serial.print("A");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(read_sensorB(i));  // ใช้ read_sensorB
    Serial.print(" ");
  }
  Serial.println();
  
  // ตัวอย่างการใช้งาน read_sensorA และ read_sensorB
  Serial.print("Example: Sensor A4 (0x08) = ");
  Serial.print(read_sensorA(4));
  Serial.print(", Sensor A4 (0x09) = ");
  Serial.println(read_sensorB(4));
  
  delayMicroseconds(50);   // อัปเดตทุก 100ms
}