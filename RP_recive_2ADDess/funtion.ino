void sw()
  {
    tone(32, 950, 100);
    delay(200); // รอ 1 วินาที
    tone(32, 950, 200);
    delay(200); // รอ 1 วินาที
    while(digitalRead(33) == 1)
      {
        Serial.print("From Master 0x08: ");
        
          // แสดงค่าจาก Nano 0x08
          Serial.print("From Master 0x08: ");
          for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorA(i));  // ใช้ read_sensorA
            Serial.print(" ");
          }
          Serial.print("   ");
          
          // แสดงค่าจาก Nano 0x09
          Serial.print("From Master 0x09: ");
          for (int i = 0; i < 8; i++) {
            Serial.print(read_sensorB(i));  // ใช้ read_sensorB
            Serial.print(" ");
          }
          Serial.println("  ");
        // Serial.println(my_tcs('r'));
          
          delay(10);  // อัปเดตทุก 100ms
          
      }
    tone(32, 950, 400);
  }