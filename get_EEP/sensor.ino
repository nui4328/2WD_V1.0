#include <Arduino.h>
#include <Wire.h>

// อาร์เรย์สำหรับเก็บข้อมูลเซ็นเซอร์
#define SENSOR_COUNT 8
#define SAMPLE_COUNT 500
int sensor_data[SENSOR_COUNT][SAMPLE_COUNT];
#define SLAVE_ADDRESS 0x10  // ที่อยู่ I2C ของ RP2350B (Slave)
#define EEPROM_ADDRESS 0x50 // ที่อยู่ I2C ของ CAT24C256
#define BUTTON_PIN_0x09 23  // ปุ่มสำหรับ Master 0x09
#define BUTTON_PIN_0x08 24  // ปุ่มสำหรับ Master 0x08
volatile int sensorValues1[8];  // ค่าจาก Master 0x08
volatile int sensorValues2[8];  // ค่าจาก Master 0x09
volatile uint8_t lastMaster = 0;
volatile bool newDataReceived = false; // ตัวแปรตรวจสอบข้อมูลใหม่
volatile int receiveCount = 0;        // นับจำนวนครั้งที่ได้รับข้อมูล I2C

// ตัวแปรสำหรับเก็บค่าสูงสุดและต่ำสุด
int max_values_0x09[SENSOR_COUNT];
int min_values_0x09[SENSOR_COUNT];
int max_values_0x08[SENSOR_COUNT];
int min_values_0x08[SENSOR_COUNT];

// ตัวแปรสำหรับ debouncing ปุ่ม
bool lastButtonState_0x09 = HIGH; // สถานะปุ่ม GP23
bool lastButtonState_0x08 = HIGH; // สถานะปุ่ม GP24
unsigned long lastDebounceTime_0x09 = 0;
unsigned long lastDebounceTime_0x08 = 0;
const unsigned long debounceDelay = 50; // ระยะเวลา debounce 50ms

// อ่านค่าเซ็นเซอร์จาก Master 0x08 (A0–A7)
int read_sensorA(int sensor) {
  return sensorValues1[sensor];
}

// อ่านค่าเซ็นเซอร์จาก Master 0x09 (A0–A7)
int read_sensorB(int sensor) {
  return sensorValues2[sensor];
}

// ฟังก์ชันเพื่ออ่านค่าสูงสุด/ต่ำสุดจาก EEPROM
int read_eeprom_value(uint8_t masterId, int sensor, bool isMax) {
  if (sensor < 0 || sensor >= SENSOR_COUNT) {
    Serial.println("Error: Invalid sensor index");
    return -1;
  }
  uint16_t eeprom_address = (masterId == 0x08) ? 0x0080 : 0x0000;
  uint16_t value_offset = isMax ? (sensor * 4) : (sensor * 4 + 32);
  uint16_t checksum_offset = 64;

  // อ่านข้อมูลทั้งหมด (65 ไบต์) เพื่อตรวจสอบ checksum
  uint8_t data[65];
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((uint8_t)(eeprom_address >> 8)); // ที่อยู่หน่วยความจำ (high byte)
  Wire.write((uint8_t)(eeprom_address & 0xFF)); // ที่อยู่หน่วยความจำ (low byte)
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDRESS, 65);
  for (int i = 0; i < 65; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      Serial.print("Error: EEPROM read failed for Master 0x");
      Serial.println(masterId, HEX);
      return -1;
    }
  }

  // ตรวจสอบ checksum
  uint8_t stored_checksum = data[checksum_offset];
  uint8_t calc_checksum = 0;
  for (int i = 0; i < 64; i++) {
    calc_checksum ^= data[i];
  }
  if (stored_checksum != calc_checksum) {
    Serial.print("EEPROM data corrupted for Master 0x");
    Serial.println(masterId, HEX);
    return -1;
  }

  // แปลงข้อมูล 4 ไบต์เป็นค่า int
  int value = (data[value_offset] << 24) |
              (data[value_offset + 1] << 16) |
              (data[value_offset + 2] << 8) |
              data[value_offset + 3];
  return value;
}

// ฟังก์ชันสำหรับเรียกใช้งานค่าสูงสุด/ต่ำสุด
int max_09(int sensor) {
  return read_eeprom_value(0x09, sensor, true);
}

int min_09(int sensor) {
  return read_eeprom_value(0x09, sensor, false);
}

int max_08(int sensor) {
  return read_eeprom_value(0x08, sensor, true);
}

int min_08(int sensor) {
  return read_eeprom_value(0x08, sensor, false);
}

// เก็บข้อมูลเซ็นเซอร์
void collect_sensor_data(uint8_t masterId) {
  // เริ่มต้น sensor_data ด้วย -1
  for (int i = 0; i < SENSOR_COUNT; i++) {
    for (int s = 0; s < SAMPLE_COUNT; s++) {
      sensor_data[i][s] = -1;
    }
  }

  unsigned long startTime = millis();
  int sampleIndex = 0;
  receiveCount = 0; // รีเซ็ตตัวนับ

  while (sampleIndex < SAMPLE_COUNT && millis() - startTime < 6000) { // Timeout 6 วินาที
    if (newDataReceived && lastMaster == masterId) {
      for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_data[i][sampleIndex] = (masterId == 0x08) ? read_sensorA(i) : read_sensorB(i);
      }
      sampleIndex++;
      newDataReceived = false;
      delay(10); // รักษาการหน่วง 10ms เพื่อให้ได้ 500 ค่าใน 5 วินาที
    }
  }
  Serial.print("Collected samples for Master 0x");
  Serial.print(masterId, HEX);
  Serial.print(": ");
  Serial.print(sampleIndex);
  Serial.print("/500, I2C receive count: ");
  Serial.println(receiveCount);
  if (sampleIndex < SAMPLE_COUNT) {
    Serial.println("Warning: Data collection incomplete!");
  }
}

// หาค่าสูงสุดและต่ำสุด
void find_min_max(uint8_t masterId) {
  int *max_values = (masterId == 0x08) ? max_values_0x08 : max_values_0x09;
  int *min_values = (masterId == 0x08) ? min_values_0x08 : min_values_0x09;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    // เริ่มต้นด้วยค่าแรกที่ไม่ใช่ -1
    int validIndex = 0;
    while (validIndex < SAMPLE_COUNT && sensor_data[i][validIndex] == -1) {
      validIndex++;
    }
    if (validIndex >= SAMPLE_COUNT) {
      max_values[i] = 0;
      min_values[i] = 0;
      Serial.print("Warning: No valid data for Sensor ");
      Serial.print(i);
      Serial.print(" (Master 0x");
      Serial.print(masterId, HEX);
      Serial.println(")");
      continue;
    }
    max_values[i] = sensor_data[i][validIndex];
    min_values[i] = sensor_data[i][validIndex];
    for (int s = validIndex + 1; s < SAMPLE_COUNT; s++) {
      if (sensor_data[i][s] != -1) {
        if (sensor_data[i][s] > max_values[i]) {
          max_values[i] = sensor_data[i][s];
        }
        if (sensor_data[i][s] < min_values[i]) {
          min_values[i] = sensor_data[i][s];
        }
      }
    }
  }
}

// แสดงค่าสูงสุดและต่ำสุดก่อนบันทึก
void print_max_min(uint8_t masterId) {
  int *max_values = (masterId == 0x08) ? max_values_0x08 : max_values_0x09;
  int *min_values = (masterId == 0x08) ? min_values_0x08 : min_values_0x09;
  Serial.print("Max/Min Values before saving to EEPROM (Master 0x");
  Serial.print(masterId, HEX);
  Serial.println("):");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Max = ");
    Serial.print(max_values[i]);
    Serial.print(", Min = ");
    Serial.println(min_values[i]);
  }
}

// คำนวณ checksum (XOR ของทุกไบต์)
uint8_t calculate_checksum(uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// เขียนข้อมูลลง EEPROM CAT24C256 ผ่าน Wire
void write_to_eeprom(uint8_t masterId) {
  int *max_values = (masterId == 0x08) ? max_values_0x08 : max_values_0x09;
  int *min_values = (masterId == 0x08) ? min_values_0x08 : min_values_0x09;
  uint16_t eeprom_address = (masterId == 0x08) ? 0x0080 : 0x0000;

  uint8_t data[65]; // 64 ไบต์สำหรับข้อมูล + 1 ไบต์สำหรับ checksum
  for (int i = 0; i < SENSOR_COUNT; i++) {
    data[i * 4] = (max_values[i] >> 24) & 0xFF;
    data[i * 4 + 1] = (max_values[i] >> 16) & 0xFF;
    data[i * 4 + 2] = (max_values[i] >> 8) & 0xFF;
    data[i * 4 + 3] = max_values[i] & 0xFF;
    data[i * 4 + 32] = (min_values[i] >> 24) & 0xFF;
    data[i * 4 + 33] = (min_values[i] >> 16) & 0xFF;
    data[i * 4 + 34] = (min_values[i] >> 8) & 0xFF;
    data[i * 4 + 35] = min_values[i] & 0xFF;
  }
  data[64] = calculate_checksum(data, 64);

  // เขียนข้อมูลลง EEPROM
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((uint8_t)(eeprom_address >> 8)); // ที่อยู่หน่วยความจำ (high byte)
  Wire.write((uint8_t)(eeprom_address & 0xFF)); // ที่อยู่หน่วยความจำ (low byte)
  for (int i = 0; i < 65; i++) {
    Wire.write(data[i]);
    if ((i + 1) % 64 == 0 || i == 64) { // เขียนทุก 64 ไบต์ หรือสิ้นสุดข้อมูล
      Wire.endTransmission();
      delay(5); // รอ 5ms สำหรับ page write
      if (i < 64) { // เริ่ม transmission ใหม่สำหรับข้อมูลที่เหลือ
        Wire.beginTransmission(EEPROM_ADDRESS);
        Wire.write((uint8_t)((eeprom_address + i + 1) >> 8));
        Wire.write((uint8_t)((eeprom_address + i + 1) & 0xFF));
      }
    }
  }
  Wire.endTransmission();
  Serial.print("Data saved to EEPROM for Master 0x");
  Serial.println(masterId, HEX);
}

// อ่านข้อมูลจาก EEPROM CAT24C256 ผ่าน Wire
bool read_from_eeprom(uint8_t masterId, int *max_values_out, int *min_values_out) {
  uint16_t eeprom_address = (masterId == 0x08) ? 0x0080 : 0x0000;
  uint8_t data[65];

  // ตั้งค่าที่อยู่หน่วยความจำ
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((uint8_t)(eeprom_address >> 8)); // ที่อยู่หน่วยความจำ (high byte)
  Wire.write((uint8_t)(eeprom_address & 0xFF)); // ที่อยู่หน่วยความจำ (low byte)
  Wire.endTransmission();

  // อ่านข้อมูล 65 ไบต์
  Wire.requestFrom(EEPROM_ADDRESS, 65);
  for (int i = 0; i < 65; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      Serial.print("Error: EEPROM read failed for Master 0x");
      Serial.println(masterId, HEX);
      return false;
    }
  }

  // ตรวจสอบ checksum
  uint8_t stored_checksum = data[64];
  uint8_t calc_checksum = calculate_checksum(data, 64);
  if (stored_checksum != calc_checksum) {
    Serial.print("EEPROM data corrupted for Master 0x");
    Serial.println(masterId, HEX);
    return false;
  }

  // แปลงข้อมูลกลับเป็น max_values และ min_values
  for (int i = 0; i < SENSOR_COUNT; i++) {
    max_values_out[i] = (data[i * 4] << 24) |
                        (data[i * 4 + 1] << 16) |
                        (data[i * 4 + 2] << 8) |
                        data[i * 4 + 3];
    min_values_out[i] = (data[i * 4 + 32] << 24) |
                        (data[i * 4 + 33] << 16) |
                        (data[i * 4 + 34] << 8) |
                        data[i * 4 + 35];
  }
  return true;
}

// รับข้อมูล I2C ผ่าน Wire1
void receiveData(int byteCount) {
  if (byteCount == 17) {
    uint8_t masterId = Wire1.read();
    lastMaster = masterId;
    volatile int* targetArray = (masterId == 0x08) ? sensorValues1 : sensorValues2;
    for (int i = 0; i < 8; i++) {
      int highByte = Wire1.read();
      int lowByte = Wire1.read();
      targetArray[i] = (highByte << 8) | lowByte;
    }
    newDataReceived = true;
    receiveCount++; // เพิ่มตัวนับ
  }
}

void setup_sensor() {
  Serial.begin(115200);
  // รอ Serial พร้อม ด้วย timeout 2 วินาที
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000); // รอสูงสุด 2 วินาที

  // ตั้งค่าปุ่มกด
  pinMode(BUTTON_PIN_0x09, INPUT_PULLUP);
  pinMode(BUTTON_PIN_0x08, INPUT_PULLUP);

  // ตั้งค่า I2C สำหรับ EEPROM (Wire: SDA=GP4, SCL=GP5)
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin(); // เริ่ม Wire เป็น Master สำหรับ EEPROM

  // ตั้งค่า I2C สำหรับเซ็นเซอร์ (Wire1: SDA=GP14, SCL=GP15)
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin(SLAVE_ADDRESS); // เริ่ม Wire1 เป็น Slave
  Wire1.onReceive(receiveData);

  Serial.println("RP2350B Slave Started (Address 0x10) with EEPROM at 0x50");
  Serial.println("Press GP23 for Master 0x09, GP24 for Master 0x08");
}

void cal_sensor() {
  // ตรวจจับการกดปุ่ม GP23 (Master 0x09)
  bool buttonState_0x09 = digitalRead(BUTTON_PIN_0x09);
  if (buttonState_0x09 != lastButtonState_0x09) {
    lastDebounceTime_0x09 = millis();
  }
  if ((millis() - lastDebounceTime_0x09) > debounceDelay && buttonState_0x09 == LOW) {
    Serial.println("Button GP23 pressed! Collecting data for Master 0x09...");
    collect_sensor_data(0x09);
    find_min_max(0x09);
    print_max_min(0x09);
    write_to_eeprom(0x09);
  }
  lastButtonState_0x09 = buttonState_0x09;

  // ตรวจจับการกดปุ่ม GP24 (Master 0x08)
  bool buttonState_0x08 = digitalRead(BUTTON_PIN_0x08);
  if (buttonState_0x08 != lastButtonState_0x08) {
    lastDebounceTime_0x08 = millis();
  }
  if ((millis() - lastDebounceTime_0x08) > debounceDelay && buttonState_0x08 == LOW) {
    Serial.println("Button GP24 pressed! Collecting data for Master 0x08...");
    collect_sensor_data(0x08);
    find_min_max(0x08);
    print_max_min(0x08);
    write_to_eeprom(0x08);
  }
  lastButtonState_0x08 = buttonState_0x08;

  // แสดงค่าเซ็นเซอร์และค่าสูงสุด/ต่ำสุดจาก EEPROM ทุก 1 วินาที
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 10) {
    // แสดงค่าเซ็นเซอร์และค่าสูงสุด/ต่ำสุดสำหรับ Master 0x08
    Serial.print("Sensor_08 ");
    for (int i = 0; i < SENSOR_COUNT; i++) 
      {
        Serial.print(sensorValues1[i]);
        Serial.print(" ");
      }
    Serial.print("//Max--> ");
    for (int i = 0; i < SENSOR_COUNT; i++) 
      {
        Serial.print(max_08(i));
        Serial.print(" ");
      }
    Serial.print("//Min--> ");
    for (int i = 0; i < SENSOR_COUNT; i++) 
      {
        Serial.print(min_08(i));
        Serial.print(" ");
      }
    Serial.println(" ");
    /*
    Serial.println(" ");
    Serial.print("Sensor_09 ");
    for (int i = 0; i < SENSOR_COUNT; i++) 
      {
        Serial.print(sensorValues2[i]);
        Serial.print(" ");
      }
    Serial.print("//Max--> ");
    for (int i = 0; i < SENSOR_COUNT; i++) 
      {
        Serial.print(max_09(i));
        Serial.print(" ");
      }
    Serial.print("//Min--> ");
    for (int i = 0; i < SENSOR_COUNT; i++) 
      {
        Serial.print(min_09(i));
        Serial.print(" ");
      }
    Serial.println(" ");
    */
    lastPrint = millis();
  }
}