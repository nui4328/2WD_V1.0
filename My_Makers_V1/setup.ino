void _setup()
  {
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);
    
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(24, INPUT_PULLUP);
    analogReadResolution(12);
     // ตั้งค่าความถี่และความละเอียด PWM
    analogWriteFreq(20000); // ความถี่ PWM 25 kHz
    analogWriteResolution(12);
    //analogWriteRange(12); // ความละเอียด 12 บิต (0-4095)
    tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
    tft.setRotation(3);
    tft.fillScreen(ST77XX_WHITE);
    delay(50);

    void mydisplay_background(uint16_t color_b);
    void mydisplay(String text, int x, int y, int size_text, uint16_t color) ;
    void testtext(char *text, uint16_t color);
    my_GYRO::begin();
    my_GYRO::resetAngles();

    Wire.beginTransmission(MCP3421_ADDR);
    Wire.write(0b10011100);
    Wire.endTransmission();

    encoder4set.setupEncoders();    //-------------------->> เรียกฟังก์ชัน setupEncodersetupEncoders()
    encoder4set.resetEncoders();  //--------------------->> ฟังก์ชันรอก  
  }
void start_led()
  {
    pinMode(26, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(9, OUTPUT);
    tone(34, 3000, 100);
    delay(200); // รอ 1 วินาที
    tone(34, 3000, 200);
    for(int i = 0; i<2; i++)
    {
      for(int i = 0; i<3; i++)
        {
          digitalWrite(rgb[i],1);
          delay(50);
          digitalWrite(rgb[i],0);
          delay(50);
        }
    }
  digitalWrite(rgb[1],1);  
  }