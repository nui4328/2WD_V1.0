#include <Wire.h>
#include <math.h>

// I2C Address
#define ADXL345_ADDR  0x53
#define ITG3205_ADDR  0x68
#define QMC5883L_ADDR 0x0D
#define EEPROM_ADDR   0x50 // CAT24C256 I2C address

// ขา GPIO
#define BUTTON_PIN    24 // GP24 สำหรับปุ่ม
#define LED_PIN       25 // GP25 สำหรับ LED

// ค่าคงที่สำหรับการสอบเทียบ QMC5883L
int16_t magXMin = 0, magXMax = 0;
int16_t magYMin = 0, magYMax = 0;
bool calibrationDone = false;
float yawOffset = 0.0; // ตัวแปรสำหรับเก็บ offset ของ yaw

// Function prototype สำหรับ calculateYaw
float calculateYaw(int16_t mx, int16_t my, int16_t mz, int16_t ax, int16_t ay, int16_t az, bool applyOffset = true);

void setup_gy85() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // ปุ่มเป็น INPUT_PULLUP
  pinMode(LED_PIN, OUTPUT);         // LED เป็น OUTPUT
  digitalWrite(LED_PIN, LOW);       // ปิด LED เริ่มต้น
  delay(1000);
  Serial.println("GY-85 Sensor Raw I2C Test with Yaw, Tilt Compensation, EEPROM, and Auto Yaw Reset on Startup");

  // ---- Initialize ADXL345 ----
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x2D); // POWER_CTL register
  Wire.write(0x08); // Measure mode
  Wire.endTransmission();

  // ---- Initialize ITG3205 ----
  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(0x3E); // PWR_MGM register
  Wire.write(0x00); // Use internal clock
  Wire.endTransmission();

  // ---- Initialize QMC5883L ----
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x0B); // Set register 0x0B
  Wire.write(0x01); // Define Set/Reset period
  Wire.endTransmission();
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09); // Control register 1
  Wire.write(0x19); // Continuous mode, 200Hz, 8G range
  Wire.endTransmission();

  // ---- โหลดค่าการสอบเทียบจาก EEPROM ----
  readCalibrationFromEEPROM();
  if (calibrationDone) {
    Serial.println("Loaded calibration from EEPROM:");
    Serial.print("X Min: "); Serial.print(magXMin); Serial.print(", X Max: "); Serial.println(magXMax);
    Serial.print("Y Min: "); Serial.print(magYMin); Serial.print(", Y Max: "); Serial.println(magYMax);
    if ((magXMax - magXMin < 100) || (magYMax - magYMin < 100)) {
      Serial.println("Warning: Calibration range too small! Press GP24 to recalibrate.");
      calibrationDone = false;
      digitalWrite(LED_PIN, LOW); // ปิด LED หากการสอบเทียบไม่ดี
    } else {
      digitalWrite(LED_PIN, HIGH); // ติด LED เมื่อโหลดการสอบเทียบสำเร็จ
      // ---- ตั้งค่า yaw เป็น 0 เมื่อเปิดเครื่อง ----
      Wire.beginTransmission(QMC5883L_ADDR);
      Wire.write(0x00); // Data Output X_LSB
      Wire.endTransmission();
      Wire.requestFrom(QMC5883L_ADDR, 6);
      int16_t mx = Wire.read() | (Wire.read() << 8);
      int16_t my = Wire.read() | (Wire.read() << 8);
      int16_t mz = Wire.read() | (Wire.read() << 8);

      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(0x32); // X0 register
      Wire.endTransmission();
      Wire.requestFrom(ADXL345_ADDR, 6);
      int16_t ax = Wire.read() | (Wire.read() << 8);
      int16_t ay = Wire.read() | (Wire.read() << 8);
      int16_t az = Wire.read() | (Wire.read() << 8);

      // คำนวณ yaw ดิบและตั้ง yawOffset
      yawOffset = calculateYaw(mx, my, mz, ax, ay, az, false);
      Serial.print("Yaw auto-reset on startup! Raw Yaw: ");
      Serial.print(yawOffset, 2);
      Serial.println(" degrees. Yaw will start at 0.");
    }
  } else {
    Serial.println("No valid calibration in EEPROM. Press GP24 to calibrate.");
  }
}

// ฟังก์ชันรีเซ็ต EEPROM
void resetEEPROM() {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(0x00); // High byte address
  Wire.write(0x00); // Low byte address
  for (int i = 0; i < 8; i++) {
    Wire.write(0xFF); // เขียน FF เพื่อรีเซ็ต
  }
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C write error in resetEEPROM: "); Serial.println(error);
  }
  delay(10);
  Serial.println("EEPROM reset to default values.");
}

// ฟังก์ชันอ่านการสอบเทียบจาก EEPROM
void readCalibrationFromEEPROM() {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(0x00); // High byte address
  Wire.write(0x00); // Low byte address
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C address error in readCalibrationFromEEPROM: "); Serial.println(error);
    return;
  }
  Wire.requestFrom(EEPROM_ADDR, 8); // อ่าน 8 ไบต์ (4 x int16_t)
  magXMin = Wire.read() | (Wire.read() << 8);
  magXMax = Wire.read() | (Wire.read() << 8);
  magYMin = Wire.read() | (Wire.read() << 8);
  magYMax = Wire.read() | (Wire.read() << 8);
  // ตรวจสอบว่ามีข้อมูลการสอบเทียบที่สมเหตุสมผล
  if (magXMin != -1 && magXMax != -1 && magYMin != -1 && magYMax != -1 &&
      magXMax > magXMin && magYMax > magYMin) {
    calibrationDone = true;
  } else {
    calibrationDone = false;
  }
}

// ฟังก์ชันบันทึกการสอบเทียบลง EEPROM
void writeCalibrationToEEPROM() {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(0x00); // High byte address
  Wire.write(0x00); // Low byte address
  Wire.write(lowByte(magXMin)); Wire.write(highByte(magXMin));
  Wire.write(lowByte(magXMax)); Wire.write(highByte(magXMax));
  Wire.write(lowByte(magYMin)); Wire.write(highByte(magYMin));
  Wire.write(lowByte(magYMax)); Wire.write(highByte(magYMax));
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C write error in writeCalibrationToEEPROM: "); Serial.println(error);
  }
  delay(10); // หน่วงเวลาให้ EEPROM เขียนข้อมูล
}

// ฟังก์ชันสอบเทียบ QMC5883L
void calibrateMagnetometer() {
  Serial.println("Calibrating QMC5883L...");
  Serial.println("Rotate sensor slowly in a figure-8 pattern or all directions for 20 seconds");
  Serial.println("Ensure sensor is away from metal objects or electronics");
  
  unsigned long startTime = millis();
  magXMin = magYMin = 32767; // ค่าเริ่มต้นสูงสุด
  magXMax = magYMax = -32768; // ค่าเริ่มต้นต่ำสุด

  while (millis() - startTime < 20000) { // สอบเทียบ 20 วินาที
    digitalWrite(LED_PIN, (millis() % 1000) < 500); // กระพริบ LED ทุก 500ms
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x00); // Data Output X_LSB
    Wire.endTransmission();
    Wire.requestFrom(QMC5883L_ADDR, 6);
    int16_t mx = Wire.read() | (Wire.read() << 8);
    int16_t my = Wire.read() | (Wire.read() << 8);
    int16_t mz = Wire.read() | (Wire.read() << 8);

    // อัปเดตค่า min/max
    if (mx < magXMin) magXMin = mx;
    if (mx > magXMax) magXMax = mx;
    if (my < magYMin) magYMin = my;
    if (my > magYMax) magYMax = my;

    // แสดงค่าเพื่อช่วยตรวจสอบ
    Serial.print("Calibrating - Mag: "); Serial.print(mx); Serial.print(", "); Serial.print(my); Serial.print(", "); Serial.println(mz);
    delay(50); // อ่านทุก 50ms
  }
  
  // ตรวจสอบช่วงการสอบเทียบ
  if ((magXMax - magXMin < 100) || (magYMax - magYMin < 100)) {
    Serial.println("Error: Calibration range too small! Calibration not saved.");
    calibrationDone = false;
    digitalWrite(LED_PIN, LOW); // ปิด LED
    return;
  }

  digitalWrite(LED_PIN, HIGH); // ติด LED เมื่อสอบเทียบสำเร็จ
  calibrationDone = true;
  
  // บันทึกค่าลง EEPROM
  writeCalibrationToEEPROM();
  Serial.println("Calibration done and saved to EEPROM!");
  Serial.print("X Min: "); Serial.print(magXMin); Serial.print(", X Max: "); Serial.println(magXMax);
  Serial.print("Y Min: "); Serial.print(magYMin); Serial.print(", Y Max: "); Serial.println(magYMax);

  // รีเซ็ต yaw หลังสอบเทียบ
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00); // Data Output X_LSB
  Wire.endTransmission();
  Wire.requestFrom(QMC5883L_ADDR, 6);
  int16_t mx = Wire.read() | (Wire.read() << 8);
  int16_t my = Wire.read() | (Wire.read() << 8);
  int16_t mz = Wire.read() | (Wire.read() << 8);

  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32); // X0 register
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDR, 6);
  int16_t ax = Wire.read() | (Wire.read() << 8);
  int16_t ay = Wire.read() | (Wire.read() << 8);
  int16_t az = Wire.read() | (Wire.read() << 8);

  yawOffset = calculateYaw(mx, my, mz, ax, ay, az, false);
  Serial.print("Yaw reset after calibration! Raw Yaw: ");
  Serial.print(yawOffset, 2);
  Serial.println(" degrees. Yaw will start at 0.");
}

// ฟังก์ชันคำนวณ yaw พร้อมชดเชยมุมเอียง
float calculateYaw(int16_t mx, int16_t my, int16_t mz, int16_t ax, int16_t ay, int16_t az, bool applyOffset) {
  // ตรวจสอบว่า offset และ scale สมเหตุสมผล
  float xRange = magXMax - magXMin;
  float yRange = magYMax - magYMin;
  if (xRange < 100 || yRange < 100) {
    Serial.println("Warning: Invalid calibration range in calculateYaw.");
    return 0.0; // คืนค่า 0 หากช่วงการสอบเทียบเล็กเกินไป
  }

  // ปรับค่าด้วย offset และ scale จากการสอบเทียบ
  float mxCalibrated = (float)(mx - (magXMax + magXMin) / 2) / (xRange / 2);
  float myCalibrated = (float)(my - (magYMax + magYMin) / 2) / (yRange / 2);
  float mzCalibrated = (float)(mz - (magXMax + magXMin) / 2) / (xRange / 2);

  // คำนวณมุม pitch และ roll จาก accelerometer
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // ชดเชยมุมเอียง (tilt compensation)
  float mxComp = mxCalibrated * cos(pitch) + mzCalibrated * sin(pitch);
  float myComp = mxCalibrated * sin(roll) * sin(pitch) + myCalibrated * cos(roll) - mzCalibrated * sin(roll) * cos(pitch);

  // คำนวณ yaw
  float yaw = atan2(myComp, mxComp) * 180.0 / PI;

  // ปรับให้อยู่ในช่วง -180 ถึง 180 องศา
  if (yaw > 180) {
    yaw -= 360;
  } else if (yaw < -180) {
    yaw += 360;
  }

  // ใช้ yawOffset เฉพาะเมื่อ applyOffset เป็น true
  if (applyOffset) {
    yaw -= yawOffset;
    // ปรับให้อยู่ในช่วง -180 ถึง 180 องศาอีกครั้งหลังลบ offset
    if (yaw > 180) {
      yaw -= 360;
    } else if (yaw < -180) {
      yaw += 360;
    }
  }

  return yaw;
}

void loop_gy85() {
  static unsigned long buttonPressStart = 0;
  static bool buttonPressed = false;

  // ตรวจสอบการกดปุ่ม
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressStart = millis();
      delay(50); // Debounce
    }
  } else if (buttonPressed) {
    unsigned long pressDuration = millis() - buttonPressStart;
    buttonPressed = false;
    if (pressDuration < 2000) { // กดสั้น (< 2 วินาที) สำหรับสอบเทียบ
      calibrateMagnetometer();
    } else { // กดยาว (>= 2 วินาที) สำหรับรีเซ็ต yaw
      if (calibrationDone) {
        // อ่านค่าเซ็นเซอร์ล่าสุด
        Wire.beginTransmission(QMC5883L_ADDR);
        Wire.write(0x00); // Data Output X_LSB
        Wire.endTransmission();
        Wire.requestFrom(QMC5883L_ADDR, 6);
        int16_t mx = Wire.read() | (Wire.read() << 8);
        int16_t my = Wire.read() | (Wire.read() << 8);
        int16_t mz = Wire.read() | (Wire.read() << 8);

        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(0x32); // X0 register
        Wire.endTransmission();
        Wire.requestFrom(ADXL345_ADDR, 6);
        int16_t ax = Wire.read() | (Wire.read() << 8);
        int16_t ay = Wire.read() | (Wire.read() << 8);
        int16_t az = Wire.read() | (Wire.read() << 8);

        // คำนวณ yaw ดิบ (ไม่ใช้ offset)
        float rawYaw = calculateYaw(mx, my, mz, ax, ay, az, false);
        yawOffset = rawYaw; // ตั้ง offset เป็น yaw ดิบ
        Serial.print("Yaw reset to 0 degrees! Raw Yaw: ");
        Serial.print(rawYaw, 2);
        Serial.print(", New yawOffset: ");
        Serial.println(yawOffset, 2);
        digitalWrite(LED_PIN, HIGH); // ติด LED เพื่อยืนยัน
        delay(500); // หน่วงเพื่อแสดงการยืนยัน
        digitalWrite(LED_PIN, calibrationDone ? HIGH : LOW); // คืนสถานะ LED
      } else {
        Serial.println("Cannot reset yaw: Magnetometer not calibrated.");
      }
    }
  }

  // ---- Read ADXL345 ----
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32); // X0 register
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDR, 6);
  int16_t ax = Wire.read() | (Wire.read() << 8);
  int16_t ay = Wire.read() | (Wire.read() << 8);
  int16_t az = Wire.read() | (Wire.read() << 8);

  // ---- Read ITG3205 ----
  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(0x1D); // GYRO_XOUT_H
  Wire.endTransmission();
  Wire.requestFrom(ITG3205_ADDR, 6);
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // ---- Read QMC5883L ----
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00); // Data Output X_LSB
  Wire.endTransmission();
  Wire.requestFrom(QMC5883L_ADDR, 6);
  int16_t mx = Wire.read() | (Wire.read() << 8);
  int16_t my = Wire.read() | (Wire.read() << 8);
  int16_t mz = Wire.read() | (Wire.read() << 8);

  // ---- คำนวณและแสดงผล Yaw (ถ้ามีการสอบเทียบ) ----
  if (calibrationDone) {
    float yaw = calculateYaw(mx, my, mz, ax, ay, az, true);
    Serial.print("Yaw: "); Serial.println(int(yaw)); // แสดง 2 ตำแหน่งทศนิยม
  } else {
    Serial.println("Yaw: Not calibrated. Press GP24 to calibrate.");
  }

  Serial.println("----------------------");
  delay(50);
}