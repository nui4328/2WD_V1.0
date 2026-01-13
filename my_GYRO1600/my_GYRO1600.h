// ----------------------- my_GYRO1600.h -----------------------
#ifndef MY_GYRO1600_H
#define MY_GYRO1600_H

#include <Wire.h>
#ifndef PI
#define PI 3.141592653589793
#endif

class my_GYRO1600 {
public:
  static const uint8_t my_GYRO1600_CHIP_ID     = 0x00; // Register for chip ID
  static const uint8_t my_GYRO1600_CMD         = 0x7E; // Command register
  static const uint8_t my_GYRO1600_ACCEL_CONF  = 0x40; // Accelerometer config
  static const uint8_t my_GYRO1600_ACCEL_RANGE = 0x41; // Accelerometer range
  static const uint8_t my_GYRO1600_GYRO_CONF   = 0x42; // Gyroscope config
  static const uint8_t my_GYRO1600_GYRO_RANGE  = 0x43; // Gyroscope range
  static const uint8_t my_GYRO1600_GYRO_DATA   = 0x0C; // Data register start (gyro X LSB)

  // Constants for angle calculation
  static const float GYRO_THRESHOLD;        // Gyroscope threshold for deadzone
  static const float ALPHA;                 // Complementary filter constant (gyro weight)
  static const float ACCEL_FILTER_ALPHA;    // Accelerometer low-pass filter constant

  // *** ใหม่: สถานะพื้นผิวสำหรับตรวจจับตะเกียบ ***
  enum SurfaceState {
    SURFACE_FLAT,     // พื้นเรียบ ไม่มีสิ่งกีดขวาง
    SURFACE_BUMP,     // เจอการกระแทกหรือตะเกียบขวาง (vibration/bump)
    SURFACE_TILTED    // พื้นเอียงมาก (ขึ้น/ลงเนินอย่างชัดเจน)
  };

  // Constructor
  my_GYRO1600(uint8_t address = 0x69);

  // Methods เดิม (ไม่เปลี่ยนแปลง)
  bool begin();
  void readAngles(float &roll, float &pitch, float &yaw);
  float gyro(char axis);
  void resetYaw();
  void resetAngles();
  void reCalibrateGyro();
  void readAccelGyro(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

  // *** ใหม่: ฟังก์ชันตรวจจับพื้นด้วยแกน Y ***
  // คืนค่าประเภทพื้นผิว และส่งความรุนแรงของการกระแทกผ่านพารามิเตอร์อ้างอิง
  SurfaceState detectSurfaceY(float &bumpMagnitude);

private:
  uint8_t _address;

  // Static variables สำหรับคำนวณมุม (ไม่ถูกแตะต้องจากฟังก์ชันใหม่)
  static unsigned long _lastTime;
  static float _angleX;
  static float _angleY;
  static float _angleZ;
  static float _gyroOffsetX;
  static float _gyroOffsetY;
  static float _gyroOffsetZ;
  static float _accelX_prev;
  static float _accelY_prev;
  static float _accelZ_prev;

  // Private methods
  void writeRegister(uint8_t reg, uint8_t value);
  
  bool calibrateGyro();
};

// Function ภายนอกเพื่อ reset gyro (เดิม)
void reset_gyro160(my_GYRO1600& gyro);

#endif // MY_GYRO1600_H