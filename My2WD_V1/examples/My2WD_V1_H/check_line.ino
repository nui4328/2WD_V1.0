void check_lines()
  {
    while(1)
      {
        Motor(20, 20);
        if(readSensor(1) < md_sensor(1)-30 || readSensor(2) < md_sensor(2))
          {                    
            Motor(-20, -20);
            delay(50);
            set_f(2);
          } 
      
        if (color_is_available && color.readFast()) 
              {
                if (color.isCalibrated()) 
                  {
                  
                    Serial.println(color.getColor());
                    if (color.getColor() == "GREEN") 
                      {
                        mydisplay_background(GREEN); 
                        break;
                      } 
                    else if (color.getColor() == "RED") 
                      {
                        mydisplay_background(RED); 
                        break;
                      }
                    else if (color.getColor() == "BLUE") 
                      {
                        mydisplay_background(BLUE); 
                        break;
                      }
                    else if (color.getColor() == "YELLOW") 
                      {
                        mydisplay_background(YELLOW); 
                        break;
                      }
                    
                  }
              }
      }
  }