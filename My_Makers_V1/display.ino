


void mydisplay(String text, int x, int y, int size_text, uint16_t color) 
   {  
      tft.setCursor(x, y);
      tft.setTextSize(size_text);
      tft.setTextColor(color);
      tft.setTextWrap(true);
      tft.print(text);
      //delay(50);
   }

void mydisplay_background(uint16_t color_b)
   {
      tft.fillScreen(color_b);
   }

void testtext(char *text, uint16_t color) {
   tft.setTextSize(2);
   tft.setCursor(30, 30);
   tft.setTextColor(color);
   tft.setTextWrap(true);
   tft.print(text);
}

void display_gyro()
  {    
    int buttonState;
    unsigned long pressStartTime = 0;
    bool isPressed = false;  
    
    while(1)
      {
        String _gyro = String(my.gyro('z'));
        mydisplay_background(black);
        mydisplay("MY-MAKERS", 25, 10, 2, white);
        mydisplay("Gyro "+ _gyro, 10, 40, 2, white);
        buttonState = digitalRead(24);
        if (buttonState == LOW) 
              {  // ปุ่มถูกกด (LOW เพราะใช้ PULLUP)
                digitalWrite(rgb[1],0);
                
                if (!isPressed) \
                  {
                    pressStartTime = millis();  // บันทึกเวลาที่กดปุ่มครั้งแรก
                    isPressed = true;
                  } 
                else 
                  {
                    unsigned long pressDuration = millis() - pressStartTime;    
                    if (pressDuration >= 3000) 
                      {  // กดค้าง 3 วินาที
                        digitalWrite(rgb[1],1);
                        tone(32, 2500, 100);
                        delay(200); // รอ 1 วินาที
                        break;
                        while (digitalRead(24) == LOW);  // รอให้ปล่อยปุ่ม
                        delay(200);  // ป้องกันการเด้งของปุ่ม
                        
                      }
                  }
              } 
            else 
              {
                if (isPressed) 
                  {
                    unsigned long pressDuration = millis() - pressStartTime;
                    
                    if (pressDuration >= 50 && pressDuration < 3000) 
                      {  
                        my_GYRO::resetAngles(); delay(1000);
                      }
                    isPressed = false;
                  }
              }
      }

  }

void dis_BON055()
  {
    while(1)
      {
        // อ่านค่า yaw
        sensors_event_t event;
        bno.getEvent(&event);
        float rawYaw = event.orientation.x; // ค่า yaw ดิบ (0 ถึง 360)

        // ปรับค่า yaw โดยใช้ yawOffset ให้เริ่มต้นเป็น 0, หมุนซ้ายเป็นลบ, หมุนขวาเป็นบวก (-180 ถึง 180)
        float adjustedYaw = rawYaw - yawOffset;
        if (adjustedYaw > 180) adjustedYaw -= 360;
        if (adjustedYaw < -180) adjustedYaw += 360;

        // แสดงเฉพาะค่า yaw ที่ปรับแล้ว
        Serial.println(adjustedYaw, 2);
        String _gyro = String(adjustedYaw);
        mydisplay_background(black);
        mydisplay("MY-MAKERS", 25, 10, 2, white);
        mydisplay("Gyro "+ _gyro, 10, 40, 2, white);
      }
  }