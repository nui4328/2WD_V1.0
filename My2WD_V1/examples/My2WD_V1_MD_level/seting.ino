

void sw_start()
  {
    bz(100);
    bz(100);    

    while(digitalRead(9)==1 )
      {
        if(analogRead(29) > 4000)
           {
            add_sensor_F();
           }
        String readSensor0 = String(readSensor(0));
        String readSensor1 = String(readSensor(1));
        String readSensor2 = String(readSensor(2));
        String readSensor3 = String(readSensor(3));
        String readSensor4 = String(readSensor(4));
        String readSensor5 = String(readSensor(5));
        String readSensor6 = String(readSensor(6));
        String readSensor7 = String(readSensor(7));
        String en_posL = String(encoder.Poss_L());
        String en_posR = String(encoder.Poss_R());
        String gyro = String(my.gyro('z'));
        String knob = String(analogRead(29));
        mydisplay_background(BLACK);
        mydisplay(readSensor0 +"  "+ readSensor1 +"  "+ readSensor2+"  "+ readSensor3, 5, 5 ,1, WHITE);
        mydisplay(knob, 120, 100 ,1, WHITE);
        mydisplay(readSensor7 +"  "+ readSensor6 +"  "+ readSensor5+"  "+ readSensor4, 5, 20 ,1, WHITE);
       
        mydisplay(gyro, 10, 35 ,1, WHITE);

        mydisplay(en_posL, 10, 45 ,1, WHITE);
        mydisplay(en_posR, 10, 60 ,1, WHITE);
        for(int i = 0; i<6; i++)
          {
            Serial.print(readSensor(i));
            Serial.print("  ");
            delay(10);
          }
        Serial.println(" ");
      }
    bz(500);
    mydisplay_background(BLACK);
    mydisplay("End", 5, 10 ,2, WHITE);

    
  }
 
