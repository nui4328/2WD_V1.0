#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BUTTON_PIN 24
#define LED_PIN 25
#define CAT24C256_ADDRESS 0x50 // ที่อยู่ I2C ของ CAT24C256
#define BNO055_ADDRESS 0x29    // ที่อยู่ I2C ของ BNO055
#define PAGE_SIZE 64           // ขนาดหน้าของ CAT24C256

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

bool calibrating = false;
float yawOffset = 0.0;
float yawHistory[5] = {0}; // Moving Average Filter ขนาด 5
int yawIndex = 0;
bool calibrationSaved = false;
unsigned long lastStableTime = 0;
float lastFilteredYaw = 0.0;

// ฟังก์ชันสแกน I2C
void scanI2C() {
  byte error, address;
  int devices = 0;

  Serial.println(F("I2C Scanning..."));
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.println(F(" !"));
      devices++;
    } else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
    }
  }
  if (devices == 0) {
    Serial.println(F("No I2C devices found\n"));
  } else {
    Serial.print(F("Found "));
    Serial.print(devices);
    Serial.println(F(" device(s)\n"));
  }
}

// ฟังก์ชันตรวจสอบการเชื่อมต่อ I2C
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  int error = Wire.endTransmission();
  return (error == 0);
}

// ฟังก์ชันกรองค่า yaw
float getFilteredYaw(float rawYaw) {
  yawHistory[yawIndex] = rawYaw;
  yawIndex = (yawIndex + 1) % 5;
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += yawHistory[i];
  }
  return sum / 5.0;
}

// ฟังก์ชันตรวจสอบความนิ่ง
bool isSensorStable() {
  imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float threshold = 0.1;
  return (abs(gyroData.x()) < threshold && abs(gyroData.y()) < threshold && abs(gyroData.z()) < threshold);
}

// ฟังก์ชันเขียนลง CAT24C256
bool writeEEPROM(uint16_t address, uint8_t* data, uint8_t length) {
  if (!checkI2CDevice(CAT24C256_ADDRESS)) return false;
  uint8_t bytesWritten = 0;
  while (bytesWritten < length) {
    uint8_t chunkSize = min(PAGE_SIZE - (address % PAGE_SIZE), length - bytesWritten);
    Wire.beginTransmission(CAT24C256_ADDRESS);
    Wire.write((uint8_t)(address >> 8));
    Wire.write((uint8_t)(address & 0xFF));
    for (uint8_t i = 0; i < chunkSize; i++) {
      Wire.write(data[bytesWritten + i]);
    }
    if (Wire.endTransmission() != 0) {
      Serial.println(F(">>> ข้อผิดพลาด: การเขียนลง CAT24C256 ล้มเหลว!"));
      return false;
    }
    address += chunkSize;
    bytesWritten += chunkSize;
    delay(5);
  }
  return true;
}

// ฟังก์ชันอ่านจาก CAT24C256
bool readEEPROM(uint16_t address, uint8_t* data, uint8_t length) {
  if (!checkI2CDevice(CAT24C256_ADDRESS)) return false;
  Wire.beginTransmission(CAT24C256_ADDRESS);
  Wire.write((uint8_t)(address >> 8));
  Wire.write((uint8_t)(address & 0xFF));
  if (Wire.endTransmission() != 0) {
    Serial.println(F(">>> ข้อผิดพลาด: การตั้งค่าที่อยู่ EEPROM ล้มเหลว!"));
    return false;
  }
  Wire.requestFrom(CAT24C256_ADDRESS, length);
  for (uint8_t i = 0; i < length; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      Serial.println(F(">>> ข้อผิดพลาด: อ่านข้อมูลจาก CAT24C256 ไม่ครบ!"));
      return false;
    }
  }
  return true;
}

// ฟังก์ชันทดสอบแมกนีโตมิเตอร์
void testMagnetometer() {
  Serial.println(F(">>> เริ่มทดสอบแมกนีโตมิเตอร์..."));
  bno.setMode(OPERATION_MODE_MAGONLY);
  delay(100);
  imu::Vector<3> magData = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print(F("Magnetometer: X="));
  Serial.print(magData.x());
  Serial.print(F(" Y="));
  Serial.print(magData.y());
  Serial.print(F(" Z="));
  Serial.println(magData.z());
  bno.setMode(OPERATION_MODE_NDOF);
  delay(100);
}

// ฟังก์ชันสอบเทียบ
void calibrateSensor() {
  uint8_t system, gyro, accel, mag;
  bool magCalibrated = false, gyroCalibrated = false;

  Serial.println(F(">>> เริ่มสอบเทียบเซ็นเซอร์..."));
  bno.setMode(OPERATION_MODE_NDOF);
  delay(100);

  testMagnetometer();

  unsigned long startTime = millis();
  const unsigned long timeout = 30000;

  while (gyro != 3 && millis() - startTime < timeout) {
    bno.getCalibration(&system, &gyro, &accel, &mag);

    Serial.print(F("สถานะการสอบเทียบ: ระบบ="));
    Serial.print(system);
    Serial.print(F(" ไจโร="));
    Serial.print(gyro);
    Serial.print(F(" แอคเซล="));
    Serial.print(accel);
    Serial.print(F(" แมก="));
    Serial.println(mag);

    if (!magCalibrated && mag < 1) {
      Serial.println(F(">>> หมุนหุ่นยนต์ในรูปแบบ '8' ในระนาบแนวนอน (XY) และแนวตั้ง (XZ, YZ) อย่างช้าๆ (1-2 รอบ/วินาที)"));
      Serial.println(F(">>> หรือหมุนในลักษณะทรงกลมเพื่อครอบคลุมทุกมุม"));
      Serial.println(F(">>> ทำในพื้นที่ห่างจากโลหะ, มอเตอร์, และอุปกรณ์ไฟฟ้าอย่างน้อย 2-3 เมตร!"));
      digitalWrite(LED_PIN, HIGH);
      testMagnetometer();
      delay(1000);
    } else if (mag >= 1 && !magCalibrated) {
      Serial.println(F("แมกนีโตมิเตอร์สอบเทียบถึงระดับเริ่มต้น!"));
      magCalibrated = true;
      digitalWrite(LED_PIN, LOW);
    }

    if (!gyroCalibrated && gyro < 3) {
      Serial.println(F(">>> หมุนหุ่นยนต์รอบทุกแกน (X, Y, Z) อย่างช้าๆ"));
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
    } else if (gyro == 3 && !gyroCalibrated) {
      Serial.println(F("ไจโรสโคปสอบเทียบสำเร็จ!"));
      gyroCalibrated = true;
      digitalWrite(LED_PIN, LOW);
    }

    if (gyro >= 3 && !calibrationSaved) {
      adafruit_bno055_offsets_t calData;
      if (bno.getSensorOffsets(calData)) {
        uint8_t* data = (uint8_t*)&calData;
        if (writeEEPROM(0, data, sizeof(calData))) {
          calibrationSaved = true;
          Serial.println(F("บันทึกข้อมูลการสอบเทียบลง CAT24C256 สำเร็จ!"));
        } else {
          calibrationSaved = false;
          Serial.println(F(">>> ข้อผิดพลาด: ไม่สามารถบันทึกข้อมูลการสอบเทียบ!"));
        }
      } else {
        calibrationSaved = false;
        Serial.println(F(">>> ข้อผิดพลาด: ไม่สามารถดึงข้อมูลการสอบเทียบจาก BNO055!"));
      }
    }

    delay(500);
  }

  if (gyro >= 3) {
    Serial.println(F("การสอบเทียบไจโรถึงระดับใช้งานได้!"));
  } else {
    Serial.println(F(">>> หมดเวลาการสอบเทียบ! ไจโรยังไม่ถึงระดับ 3"));
  }
}

// ฟังก์ชันโหลดข้อมูลการสอบเทียบ
bool loadCalibrationData() {
  if (!checkI2CDevice(CAT24C256_ADDRESS)) return false;
  adafruit_bno055_offsets_t calData;
  uint8_t* data = (uint8_t*)&calData;
  if (!readEEPROM(0, data, sizeof(calData))) {
    Serial.println(F(">>> ข้อผิดพลาด: อ่านข้อมูลจาก CAT24C256 ล้มเหลว!"));
    return false;
  }
  bno.setMode(OPERATION_MODE_NDOF);
  delay(100);
  bno.setSensorOffsets(calData);
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if (gyro >= 1) {
    Serial.println(F("โหลดข้อมูลการสอบเทียบจาก CAT24C256 สำเร็จ!"));
    Serial.print(F("สถานะหลังโหลด: ระบบ="));
    Serial.print(system);
    Serial.print(F(" ไจโร="));
    Serial.print(gyro);
    Serial.print(F(" แอคเซล="));
    Serial.print(accel);
    Serial.print(F(" แมก="));
    Serial.println(mag);
    calibrationSaved = true;
    return true;
  }
  Serial.println(F(">>> ข้อมูลการสอบเทียบจาก CAT24C256 ไม่สมบูรณ์!"));
  return false;
}
void displayYaw() {
  sensors_event_t event;
  bno.getEvent(&event);

  // --- อ่านค่า yaw จาก BNO055 fusion ---
  float rawYaw = event.orientation.x;
  if (rawYaw > 180) rawYaw -= 360;

  // --- อ่านค่า yaw จาก Magnetometer โดยตรง ---
  imu::Vector<3> magData = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  float magYaw = atan2(magData.y(), magData.x()) * 180.0 / PI;
  if (magYaw < -180) magYaw += 360;
  if (magYaw > 180)  magYaw -= 360;

  // --- รวมกันด้วย Complementary Filter ---
  float fusedYaw = 0.98 * rawYaw + 0.02 * magYaw;

  // --- ใช้ Moving Average Filter ---
  float filteredYaw = getFilteredYaw(fusedYaw);

  // --- หักค่า offset ---
  float adjustedYaw = filteredYaw - yawOffset;
  if (adjustedYaw > 180) adjustedYaw -= 360;
  if (adjustedYaw < -180) adjustedYaw += 360;

  int yaw = (int)adjustedYaw;

  Serial.print(F("Yaw: "));
  Serial.print(yaw);
  Serial.print(F(" | rawYaw="));
  Serial.print(rawYaw);
  Serial.print(F(" | magYaw="));
  Serial.println(magYaw);

  lastFilteredYaw = filteredYaw;
   // เก็บค่า yaw ล่าสุดสำหรับรอบถัดไป
  lastFilteredYaw = filteredYaw;
  String _gyro = String(yaw);
        mydisplay_background(black);
        mydisplay("MY-MAKERS", 25, 10, 2, white);
        mydisplay("Gyro "+ _gyro, 10, 40, 2, white);
}


void     setup_bno055() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();

  // สแกน I2C
  scanI2C();

  // ตรวจสอบ BNO055
  if (!checkI2CDevice(BNO055_ADDRESS)) {
    Serial.println(F(">>> ข้อผิดพลาด: ไม่พบเซ็นเซอร์ BNO055!"));
    while (1);
  }

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println(F(">>> ข้อผิดพลาด: เริ่มต้น BNO055 ล้มเหลว!"));
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // ลองโหลดข้อมูลการสอบเทียบ
  int retryCount = 0;
  const int maxRetries = 3;
  while (retryCount < maxRetries && !loadCalibrationData()) {
    Serial.print(F(">>> ลองโหลดข้อมูลการสอบเทียบครั้งที่ "));
    Serial.println(retryCount + 1);
    retryCount++;
    delay(500);
  }

  if (!calibrationSaved) {
    Serial.println(F("ไม่พบข้อมูลการสอบเทียบ เริ่มสอบเทียบใหม่..."));
    calibrateSensor();
  } else {
    Serial.println(F("ใช้ข้อมูลการสอบเทียบที่บันทึกไว้"));
    sensors_event_t event;
    bno.getEvent(&event);
    yawOffset = event.orientation.x;
    lastFilteredYaw = yawOffset;
  }
}

void loop_bon055() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if (gyro < 1) {
    Serial.println(F(">>> การสอบเทียบไจโรสูญหาย! เริ่มสอบเทียบใหม่..."));
    digitalWrite(LED_PIN, HIGH);
    calibrateSensor();
    digitalWrite(LED_PIN, LOW);
  }

  if (digitalRead(BUTTON_PIN) == LOW && !calibrating) {
    calibrating = true;
    Serial.println(F(">>> เริ่มสอบเทียบใหม่ตามคำสั่ง..."));
    digitalWrite(LED_PIN, HIGH);
    calibrateSensor();
    calibrating = false;
    digitalWrite(LED_PIN, LOW);
  }

  if (isSensorStable()) {
    unsigned long currentTime = millis();
    if (currentTime - lastStableTime > 2000) { // เพิ่มเป็น 2 วินาที
      sensors_event_t event;
      bno.getEvent(&event);
      yawOffset = event.orientation.x;
      lastFilteredYaw = yawOffset;
      Serial.println(F("รีเซ็ต yawOffset เมื่อหุ่นยนต์นิ่ง"));
      lastStableTime = currentTime;
    }
  } else {
    lastStableTime = millis();
  }

  displayYaw();
  delay(10);
}


