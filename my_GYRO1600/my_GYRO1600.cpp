// ----------------------- my_GYRO1600.cpp (เวอร์ชันปรับปรุง – เพิ่มตรวจจับพื้นด้วยแกน Y) -----------------------
#include "my_GYRO1600.h"

#define _USE_MATH_DEFINES

#include <math.h>  // สำหรับ sqrt(), abs(), fabs()

const float my_GYRO1600::GYRO_THRESHOLD = 0.15;         // ลด noise
const float my_GYRO1600::ALPHA = 0.98;                   // เชื่อ gyro มาก แต่ยังให้ accel แก้ได้
const float my_GYRO1600::ACCEL_FILTER_ALPHA = 0.3;       // ตอบสนอง accel เร็วกว่าเดิม


unsigned long my_GYRO1600::_lastTime = 0;
float my_GYRO1600::_angleX = 0.0;
float my_GYRO1600::_angleY = 0.0;
float my_GYRO1600::_angleZ = 0.0;
float my_GYRO1600::_gyroOffsetX = 0.0;
float my_GYRO1600::_gyroOffsetY = 0.0;
float my_GYRO1600::_gyroOffsetZ = 0.0;
float my_GYRO1600::_accelX_prev = 0.0;
float my_GYRO1600::_accelY_prev = 0.0;
float my_GYRO1600::_accelZ_prev = 0.0;

// Constructor
my_GYRO1600::my_GYRO1600(uint8_t address) : _address(address) {}

// Initialize sensor
bool my_GYRO1600::begin() {
  Wire.begin();
  Wire.setClock(400000); // Fast mode 400kHz

  // Check chip ID
  Wire.beginTransmission(_address);
  Wire.write(my_GYRO1600_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(_address, (uint8_t)1);
  if (Wire.available()) {
    uint8_t chipID = Wire.read();
    if (chipID != 0xD1) { // BMI160 Chip ID = 0xD1
      return false;
    }
  } else {
    return false;
  }

  // Soft reset
  writeRegister(my_GYRO1600_CMD, 0xB6);
  delay(100);

  // Configure accelerometer (±2g, ODR 400Hz)
  writeRegister(my_GYRO1600_ACCEL_CONF, 0x2A);  // ODR = 400Hz, normal mode
  writeRegister(my_GYRO1600_ACCEL_RANGE, 0x03); // ±2g

  // Configure gyroscope (±2000 dps, ODR 400Hz)
  writeRegister(my_GYRO1600_GYRO_CONF, 0x2A);   // ODR = 400Hz
  writeRegister(my_GYRO1600_GYRO_RANGE, 0x00);  // ±2000 dps

  // Power on accelerometer and gyroscope
  writeRegister(my_GYRO1600_CMD, 0x11); // ACC normal mode
  delay(50);
  writeRegister(my_GYRO1600_CMD, 0x15); // GYRO normal mode
  delay(50);

  // Calibrate gyro offsets
  if (!calibrateGyro()) {
    return false;
  }

  _lastTime = micros();
  return true;
}

// อ่านและคำนวณมุม roll, pitch, yaw (เหมือนเดิมทุกประการ)
void my_GYRO1600::readAngles(float &roll, float &pitch, float &yaw) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Low-pass filter สำหรับ accelerometer (เฉพาะส่วนคำนวณมุม)
  accelX = ACCEL_FILTER_ALPHA * accelX + (1.0 - ACCEL_FILTER_ALPHA) * _accelX_prev;
  accelY = ACCEL_FILTER_ALPHA * accelY + (1.0 - ACCEL_FILTER_ALPHA) * _accelY_prev;
  accelZ = ACCEL_FILTER_ALPHA * accelZ + (1.0 - ACCEL_FILTER_ALPHA) * _accelZ_prev;
  _accelX_prev = accelX;
  _accelY_prev = accelY;
  _accelZ_prev = accelZ;

  // ชดเชย offset และแปลงหน่วย gyroscope
  float gyroX = (gx / 16.4) - _gyroOffsetX;
  float gyroY = (gy / 16.4) - _gyroOffsetY;
  float gyroZ = (gz / 16.4) - _gyroOffsetZ;

  // อัปเดต offset แบบออนไลน์เมื่อเซ็นเซอร์นิ่ง
  const float GYRO_CALIBRATION_ALPHA = 0.01;
  const float ONLINE_CAL_THRESHOLD = 0.5;
  if (abs(gx / 16.4) < ONLINE_CAL_THRESHOLD) _gyroOffsetX = GYRO_CALIBRATION_ALPHA * (gx / 16.4) + (1.0 - GYRO_CALIBRATION_ALPHA) * _gyroOffsetX;
  if (abs(gy / 16.4) < ONLINE_CAL_THRESHOLD) _gyroOffsetY = GYRO_CALIBRATION_ALPHA * (gy / 16.4) + (1.0 - GYRO_CALIBRATION_ALPHA) * _gyroOffsetY;
  if (abs(gz / 16.4) < ONLINE_CAL_THRESHOLD) _gyroOffsetZ = GYRO_CALIBRATION_ALPHA * (gz / 16.4) + (1.0 - GYRO_CALIBRATION_ALPHA) * _gyroOffsetZ;

  // ตัด noise เล็กน้อย (deadzone)
  if (abs(gyroX) < GYRO_THRESHOLD) gyroX = 0.0;
  if (abs(gyroY) < GYRO_THRESHOLD) gyroY = 0.0;
  if (abs(gyroZ) < GYRO_THRESHOLD) gyroZ = 0.0;

  // คำนวณมุมจาก accelerometer
  float accelRoll  = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  // คำนวณ delta time
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - _lastTime) / 1000000.0;
  if (deltaTime > 0.1) deltaTime = 0.1; // ป้องกัน spike
  _lastTime = currentTime;

  // Integrate gyroscope
  float gyroAngleX = _angleX + gyroX * deltaTime;
  float gyroAngleY = _angleY + gyroY * deltaTime;
  float gyroAngleZ = _angleZ + gyroZ * deltaTime;

  // Complementary filter
  _angleX = ALPHA * gyroAngleX + (1.0 - ALPHA) * accelRoll;
  _angleY = ALPHA * gyroAngleY + (1.0 - ALPHA) * accelPitch;
  _angleZ = gyroAngleZ; // Yaw จาก gyro เท่านั้น

  // Wrap yaw
  if (_angleZ > 180) _angleZ -= 360;
  else if (_angleZ < -180) _angleZ += 360;

  roll  = _angleX;
  pitch = _angleY;
  yaw   = _angleZ;
}

float my_GYRO1600::gyro(char axis) {
  float roll, pitch, yaw;
  readAngles(roll, pitch, yaw);
  switch (axis) {
    case 'x': case 'X': return roll;
    case 'y': case 'Y': return pitch;
    case 'z': case 'Z': return yaw;
    default: return 0.0;
  }
}

void my_GYRO1600::resetYaw() { _angleZ = 0.0; }
void my_GYRO1600::resetAngles() { _angleX = _angleY = _angleZ = 0.0; }

void my_GYRO1600::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void my_GYRO1600::readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(_address);
  Wire.write(my_GYRO1600_GYRO_DATA);
  Wire.endTransmission();
  Wire.requestFrom(_address, (uint8_t)12);

  if (Wire.available() == 12) {
    *gx = (Wire.read() | (Wire.read() << 8));
    *gy = (Wire.read() | (Wire.read() << 8));
    *gz = (Wire.read() | (Wire.read() << 8));
    *ax = (Wire.read() | (Wire.read() << 8));
    *ay = (Wire.read() | (Wire.read() << 8));
    *az = (Wire.read() | (Wire.read() << 8));
  }
}

bool my_GYRO1600::calibrateGyro() {
  const int CALIBRATION_SAMPLES = 800;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  float sumAX = 0.0, sumAY = 0.0, sumAZ = 0.0;
  float varX = 0.0, varY = 0.0, varZ = 0.0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    float gX = gx / 16.4;
    float gY = gy / 16.4;
    float gZ = gz / 16.4;
    float aX = ax / 16384.0;
    float aY = ay / 16384.0;
    float aZ = az / 16384.0;

    sumX += gX; sumY += gY; sumZ += gZ;
    sumAX += aX; sumAY += aY; sumAZ += aZ;
    varX += gX * gX; varY += gY * gY; varZ += gZ * gZ;
    delay(5);
  }

  float meanX = sumX / CALIBRATION_SAMPLES;
  float meanY = sumY / CALIBRATION_SAMPLES;
  float meanZ = sumZ / CALIBRATION_SAMPLES;
  varX = varX / CALIBRATION_SAMPLES - meanX * meanX;
  varY = varY / CALIBRATION_SAMPLES - meanY * meanY;
  varZ = varZ / CALIBRATION_SAMPLES - meanZ * meanZ;

  if (varX > 0.1 || varY > 0.1 || varZ > 0.1) return false;

  _gyroOffsetX = meanX;
  _gyroOffsetY = meanY;
  _gyroOffsetZ = meanZ;
  _accelX_prev = sumAX / CALIBRATION_SAMPLES;
  _accelY_prev = sumAY / CALIBRATION_SAMPLES;
  _accelZ_prev = sumAZ / CALIBRATION_SAMPLES;
  return true;
}

void my_GYRO1600::reCalibrateGyro() {
  const int CALIBRATION_SAMPLES = 100;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  float sumAX = 0.0, sumAY = 0.0, sumAZ = 0.0;
  float varX = 0.0, varY = 0.0, varZ = 0.0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
    float gX = gx / 16.4;
    float gY = gy / 16.4;
    float gZ = gz / 16.4;
    float aX = ax / 16384.0;
    float aY = ay / 16384.0;
    float aZ = az / 16384.0;

    sumX += gX; sumY += gY; sumZ += gZ;
    sumAX += aX; sumAY += aY; sumAZ += aZ;
    varX += gX * gX; varY += gY * gY; varZ += gZ * gZ;
    delay(5);
  }

  float meanX = sumX / CALIBRATION_SAMPLES;
  float meanY = sumY / CALIBRATION_SAMPLES;
  float meanZ = sumZ / CALIBRATION_SAMPLES;

  _gyroOffsetX = meanX;
  _gyroOffsetY = meanY;
  _gyroOffsetZ = meanZ;
  _accelX_prev = sumAX / CALIBRATION_SAMPLES;
  _accelY_prev = sumAY / CALIBRATION_SAMPLES;
  _accelZ_prev = sumAZ / CALIBRATION_SAMPLES;
}

// ==================== ฟังก์ชันตรวจจับพื้นด้วยแกน Y (ปรับสำหรับตะเกียบสูง 5 มม.) ====================
my_GYRO1600::SurfaceState my_GYRO1600::detectSurfaceY(float &bumpMagnitude) {
  int16_t ax, ay, az, gx, gy, gz;
  readAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);

  float accelY = ay / 16384.0; // ±2g range

  // Low-pass filter ที่ช้าพอที่จะกรอง vibration มอเตอร์ แต่ยังจับ bump สั้นจากตะเกียบได้
  static float accelY_filtered = 1.0;
  const float FILTER_ALPHA = 0.08;          // ปรับเพิ่มเพื่อกรอง noise มอเตอร์ดีขึ้น
  accelY_filtered = FILTER_ALPHA * accelY + (1.0 - FILTER_ALPHA) * accelY_filtered;

  // AC component = ส่วนกระแทก/สั่น
  float accelY_ac = accelY - accelY_filtered;
  bumpMagnitude = fabs(accelY_ac);

  // --- ค่าที่ปรับสำหรับตะเกียบเล็กมาก (5 mm) ---
  const float BUMP_THRESHOLD     = 0.10;    // ลดลงเพื่อ sensitivity สูง
  const float TILT_THRESHOLD     = 0.40;    // เพิ่มขึ้นเล็กน้อยเพื่อไม่สับสนกับพื้นเอียงเบา
  const float FLAT_TOLERANCE     = 0.12;    // ยอมรับความคลาดเคลื่อนมากขึ้น

  float dc_deviation = fabs(accelY_filtered - 1.0); // สมมติแกน Y ชี้ขึ้น

  if (bumpMagnitude > BUMP_THRESHOLD) {
    return SURFACE_BUMP;
  }
  else if (dc_deviation > TILT_THRESHOLD) {
    return SURFACE_TILTED;
  }
  else if (dc_deviation < FLAT_TOLERANCE && bumpMagnitude < 0.07) {
    return SURFACE_FLAT;
  }
  else {
    return SURFACE_FLAT;
  }
}

// Function ภายนอก
void reset_gyro160(my_GYRO1600& gyro) {
  gyro.resetAngles();
}