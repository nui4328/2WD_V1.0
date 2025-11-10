/*
   get_maxmin_A();
   get_maxmin_B();
   get_maxmin_C(); 
   read_eepA();
   read_sensorA_program();
   read_eepB();
   read_sensorB_program();
   read_eepC();
   read_sensorC_program();
*/

int md_sensor(int sensor) 
  {      
     
     return (sensorMax_A[sensor-40]+sensorMin_A[sensor-40])/2;
  }
// ฟังก์ชันเขียนข้อมูลลง EEPROM
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte *data, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8)); // ส่ง MSB ของที่อยู่
  Wire.write((int)(eeAddress & 0xFF)); // ส่ง LSB ของที่อยู่
  for (int i = 0; i < dataLength; i++) {
    Wire.write(data[i]); // ส่งข้อมูลทีละไบต์
  }
  Wire.endTransmission();
  delay(5); // รอให้ EEPROM เขียนข้อมูลเสร็จ
}

// ฟังก์ชันอ่านข้อมูลจาก EEPROM
void readEEPROM(int deviceAddress, unsigned int eeAddress, byte *buffer, int dataLength) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8)); // ส่ง MSB ของที่อยู่
  Wire.write((int)(eeAddress & 0xFF)); // ส่ง LSB ของที่อยู่
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, dataLength); // ขอข้อมูล
  for (int i = 0; i < dataLength; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read(); // อ่านข้อมูลทีละไบต์
    }
  }
}



void get_maxmin_A() {
  // อ่านค่าและเก็บไว้สำหรับ read_sensorA
  for (int sample = 0; sample < numSamples; sample++) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      sensorValuesA[sensor][sample] =  analogRead(40 +sensor); // สมมติว่ามีฟังก์ชันนี้
      delay(1);
    }
  }

  // คำนวณ Max และ Min ของแต่ละเซนเซอร์
  for (int sensor = 0; sensor < numSensors; sensor++) {
    sensorMaxA[sensor] = sensorValuesA[sensor][0];
    sensorMinA[sensor] = sensorValuesA[sensor][0];
    
    for (int sample = 1; sample < numSamples; sample++) {
      int value = sensorValuesA[sensor][sample];
      if (value > sensorMaxA[sensor]) {
        sensorMaxA[sensor] = value;
      }
      if (value < sensorMinA[sensor]) {
        sensorMinA[sensor] = value;
      }
    }
  }

  // บันทึก sensorMaxA และ sensorMinA ลง EEPROM
  byte buffer[16];
  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMaxA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMaxA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 0, buffer, 16); // sensorMaxA ที่ที่อยู่ 0

  for (int i = 0; i < numSensors; i++) {
    buffer[i * 2] = highByte(sensorMinA[i]);
    buffer[i * 2 + 1] = lowByte(sensorMinA[i]);
  }
  writeEEPROM(EEPROM_ADDRESS, 16, buffer, 16); // sensorMinA ที่ที่อยู่ 16

  tone(32, 950, 100);
  delay(200);
  tone(32, 950, 200);
  delay(200);

  // แสดงผล
  Serial.println("Sensor A Results:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(sensorMaxA[sensor]);
    Serial.print(", Min: ");
    Serial.println(sensorMinA[sensor]);
  }

  // อ่านและตรวจสอบจาก EEPROM
  byte readBuffer[16];
  int readMaxA[numSensors], readMinA[numSensors];
  readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }
  readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
  for (int i = 0; i < numSensors; i++) {
    readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
  }

  Serial.println("Sensor A Values read from EEPROM:");
  for (int sensor = 0; sensor < numSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" => Max: ");
    Serial.print(readMaxA[sensor]);
    Serial.print(", Min: ");
    Serial.println(readMinA[sensor]);
  }
}

void read_eepA()
  {      
      // อ่านและตรวจสอบจาก EEPROM
      byte readBuffer[16];
      int readMaxA[numSensors], readMinA[numSensors];
      readEEPROM(EEPROM_ADDRESS, 0, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMaxA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }
      readEEPROM(EEPROM_ADDRESS, 16, readBuffer, 16);
      for (int i = 0; i < numSensors; i++) {
        readMinA[i] = (readBuffer[i * 2] << 8) | readBuffer[i * 2 + 1];
      }

      Serial.println("Sensor A Values read from EEPROM:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(readMaxA[sensor]);
        Serial.print(", Min: ");
        Serial.println(readMinA[sensor]);
        sensorMax_A[sensor] = readMaxA[sensor];
        sensorMin_A[sensor] = readMinA[sensor];
      }   
  }


void read_sensorA_program()    //-------->> อ่านค่า max min ที่เก้
  { 
      Serial.println("Sensor MAX A Values read from program:");
      for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(" => Max: ");
        Serial.print(sensorMax_A[sensor]);
        Serial.print(", Min: ");
        Serial.println(sensorMin_A[sensor]);
      }   
  }

void get_EEP_Program()
  {
    read_eepA();
    read_sensorA_program();
  }