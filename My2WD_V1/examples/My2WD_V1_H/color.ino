void color_begin()
  {       
      // เริ่มต้นเซ็นเซอร์ TCS34725
      if (!color.begin()) {
          Serial.println("WARNING: ไม่พบ TCS34725! หุ่นจะทำงานโดยไม่ใช้เซ็นเซอร์สี");
          Serial.println("ตรวจสอบสาย I2C (SDA/SCL) และไฟเลี้ยง 3.3V");
          // ไม่ต้อง while(1) → ให้หุ่นวิ่งต่อ แต่ปิดการใช้งานสี
          color_is_available = false;  // ตัวแปร global ที่เราสร้างเพิ่ม
      } else {
          Serial.println("พบ TCS34725 แล้ว! เซ็นเซอร์สีพร้อมใช้งาน");
          color_is_available = true;

          // โหลดค่า calibrate สีเก่า (ถ้ามี)
          color.loadColorCalibration();
          if (color.isCalibrated()) {
              Serial.println("โหลดค่า calibrate สีเก่าเรียบร้อย");
          }
      }
  }

void test_color()
  {
    while(1)
      {
        if (color.readFast()) 
            {
                
                  Serial.println(color.getColor());
                  if (color.getColor() == "GREEN") 
                    {
                      mydisplay_background(GREEN); 
                      Serial.println(color.getColor());
                    } 
                  else if (color.getColor() == "RED") 
                    {
                      mydisplay_background(RED); 
                      Serial.println(color.getColor());
                    }
                  else if (color.getColor() == "BLUE") 
                    {
                      mydisplay_background(BLUE); 
                      Serial.println(color.getColor());
                    }
                  else if (color.getColor() == "WHITE") 
                    {
                      mydisplay_background(WHITE); 
                      Serial.println(color.getColor());
                    }
                  else if (color.getColor() == "BLACK") 
                    {
                      mydisplay_background(BLACK); 
                      Serial.println(color.getColor());
                    }
                  else if (color.getColor() == "YELLOW") 
                    {
                      mydisplay_background(YELLOW); 
                      Serial.println(color.getColor());
                    }
                  // ... ฯลฯ
                
            }
      }
  }

// ====================== Splash Screen ======================
void showSplash() {
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(YELLOW);
  tft.setCursor(18, 22);
  tft.print("MYMAKERS");

  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(35, 55);
  tft.print("Robot System v1.0");
  delay(500);
}

void showMainMenu() {
  tft.fillScreen(BLACK);

  // เส้นขั้นสวย ๆ ด้านบนสุด
  tft.drawFastHLine(10, 8, 140, CYAN);
  tft.drawFastHLine(12, 10, 136, MAGENTA);

  String items[3] = {"RUN", "cal_ADC", "cal_COLOR"};

  for (int i = 0; i < 3; i++) {
    int y = 15 + i * 20;  // เริ่มจาก y=15 (ใกล้ขอบบนสุด) ระยะห่าง 20px

    if (i == mainMenu) {
      // ไฮไลต์เมนูที่เลือก - กล่องม่วง + ขอบขาว + ตัวหนังสือดำ
      tft.fillRoundRect(10, y - 2, 140, 22, 8, MAGENTA);
      tft.drawRoundRect(10, y - 2, 140, 22, 8, WHITE);
      tft.setTextColor(BLACK);
    } else {
      tft.setTextColor(WHITE);
    }

    tft.setTextSize(2);           // ตัวอักษรใหญ่ชัด
    tft.setCursor(25, y);         // จัดกึ่งกลางแนวนอน
    tft.print(items[i]);
  }

  // คำแนะนำด้านล่างสุด
  tft.setTextSize(1);
  tft.setTextColor(LIGHT_GRAY);
  tft.setCursor(20, 72);
  tft.print("Turn = Select | Press = Enter");
}


// ====================== เมนูย่อย calibrate สี ======================
void showColorCalSubMenu() {
  tft.fillScreen(BLACK);

  // ตัด "CAL COLOR" ออก → ใช้พื้นที่ให้เมนูเต็มจอ

  // เส้นขั้นสวย ๆ ด้านบนสุดแทนหัวข้อ
  tft.drawFastHLine(10, 6, 140, CYAN);
  tft.drawFastHLine(12, 8, 136, MAGENTA);

  String colors[7] = {
    "1. Red", "2. Green", "3. Blue",
    "4. Yellow", "5. White", "6. Black",
    "< DONE >"
  };

  for (int i = 0; i < 7; i++) {
    int y = 12 + i * 10;  // เริ่มจาก y=12 (แทบติดขอบบน) ระยะห่าง 10px เพื่อให้พอดีจอ 80px

    if (i == colorSubMenu) {
      // ไฮไลต์เมนูที่เลือก - กล่องเขียว + ขอบขาว + ตัวหนังสือดำ
      tft.fillRoundRect(8, y - 1, 144, 12, 4, GREEN);
      tft.drawRoundRect(8, y - 1, 144, 12, 4, WHITE);
      tft.setTextColor(BLACK);
    } else {
      tft.setTextColor(WHITE);
    }

    tft.setTextSize(1);
    tft.setCursor(18, y);
    tft.print(colors[i]);
  }

  // คำแนะนำด้านล่างสุด
  //tft.setTextSize(1);
  //tft.setTextColor(LIGHT_GRAY);
  //tft.setCursor(15, 72);
  //tft.print("Turn = Select | Press = Calibrate");
}

// ====================== อัปเดทเมนูจากปุ่มหมุน ======================
void updateMainMenu() {
  int pot = analogRead(potPin);
  int sel = map(pot, 0, 4095, 0, 2);
  sel = constrain(sel, 0, 2);
  if (sel != mainMenu) {
    mainMenu = sel;
    showMainMenu();
  }
}

void updateColorSubMenu() {
  int pot = analogRead(potPin);
  int sel = map(pot, 0, 4095, 0, 6);
  sel = constrain(sel, 0, 6);
  if (sel != colorSubMenu) {
    colorSubMenu = sel;
    showColorCalSubMenu();
  }
}

// ====================== ตรวจปุ่มกด (debounce) ======================
bool buttonPressed() {
  static unsigned long lastTime = 0;
  if (digitalRead(buttonPin) == LOW) {
    if (millis() - lastTime > 300) {
      lastTime = millis();
      return true;
    }
  }
  return false;
}

// ====================== ระบบเมนูทั้งหมด ======================
void runMenuSystem() {
  inColorCalMenu = false;
  mainMenu = 0;
  showMainMenu();

  while (true) {
    if (inColorCalMenu) {
      updateColorSubMenu();

      if (buttonPressed()) {
        // แสดง "OK!" ชั่วคราวเมื่อ calibrate สีสำเร็จ
        int okY = 32 + colorSubMenu * 9;
        tft.fillRect(90, okY, 60, 8, BLACK);
        tft.setTextColor(GREEN);
        tft.setCursor(95, okY);
        tft.print("OK!");

        switch (colorSubMenu) {
          case 0: color.calibrateRed();     Serial.println(">>> Calibrated RED");     break;
          case 1: color.calibrateGreen();   Serial.println(">>> Calibrated GREEN");   break;
          case 2: color.calibrateBlue();    Serial.println(">>> Calibrated BLUE");    break;
          case 3: color.calibrateYellow();  Serial.println(">>> Calibrated YELLOW");  break;
          case 4: color.calibrateWhite();   Serial.println(">>> Calibrated WHITE");   break;
          case 5: color.calibrateBlack();   Serial.println(">>> Calibrated BLACK");   break;
          case 6:  // DONE
            color.calibrateDone();
            color.saveColorCalibration();
            Serial.println(">>> ALL COLOR CALIBRATED & SAVED!");

            tft.fillScreen(BLACK);
            tft.setTextSize(2);
            tft.setTextColor(GREEN);
            tft.setCursor(20, 25);
            tft.print("COLOR CAL");
            tft.setCursor(45, 45);
            tft.print("DONE!");
            delay(2000);

            inColorCalMenu = false;
            showMainMenu();
            break;
        }
        delay(800);  // รอให้เห็น "OK!" ชัด ๆ
        showColorCalSubMenu();  // รีเฟรชหน้าจอ
      }
    }
    else {
      updateMainMenu();

      if (buttonPressed()) {
        switch (mainMenu) {
          case 0:  // RUN
            tft.fillScreen(BLACK);
            tft.setTextSize(3);
            tft.setTextColor(GREEN);
            tft.setCursor(40, 30);
            tft.print("RUN!");
            delay(10);
            bz(400); delay(100); 
            return;  // ออกจากเมนู เข้าโหมดวิ่งจริง

          case 1:  // cal_ADC
            add_sensor_F();
            showMainMenu();
            break;

          case 2:  // cal_COLOR → เข้าหน้าใหม่
            inColorCalMenu = true;
            colorSubMenu = 0;
            showColorCalSubMenu();
            Serial.println("\n=== Calibrate Color Mode ===\nวางเซ็นเซอร์บนสี → กดปุ่ม GP9");
            break;
        }
      }
    }
    delay(50);
  }
}