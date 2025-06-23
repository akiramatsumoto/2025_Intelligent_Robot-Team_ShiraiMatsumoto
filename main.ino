    case STATE_WAIT: {
      stopCenterLine();
      delay(30000);
      state = STATE_TO_BALL_AREA;
      break;
    }

    case STATE_TO_BALL_AREA: {
      // 白井ここ書いて
      //0616_白井追加
      pidControl(sensor_value_L, sensor_value_R);
      if (side_line == true && line_L == 1 && line_R == 1){
        linePos += 1;
        side_line = false;
      }
      if (linePos == 3)
        state = STATE_BALL_DETECT;
      break;
    }

    case STATE_BALL_DETECT: {
      stopAll();
      delay(2000);
      Serial.println("START");

      if (stringComplete) {
        processSerialData(inputString);
        inputString = "";
        stringComplete = false;
      }

      if (abs(angle) <= tolerance_angle) {
        Serial.println("STOP"); 
        stopAll();
        angle = tolerance_angle + 1.0;
        delay(5000);
        state = STATE_BALL_COLLECT;
      } else if (!stringComplete) {
        // 次のシリアル入力を待つ
        // rotateRobot() は processSerialData 内で呼び出されるためここでは何もしない
      }

      break;
    }

    case STATE_BALL_COLLECT: {
      stopAll();

      encoderRight.write(0);
      encoderLeft.write(0);

      int detectedCount = 0;  // 連続trueカウント
      int psdCount = 0;       // 検出回数カウント

      while (psdCount <= 1) {
        bool detected = isBallDetected();
        Serial.print("Detected: "); Serial.print(detected);
        Serial.print(" | detectedCount: "); Serial.print(detectedCount);
        Serial.print(" | psdCount: "); Serial.println(psdCount);

        if (detected) {
          detectedCount++;
          if (detectedCount >= 5) {
            psdCount++;
            detectedCount = 0;
          }
        } else {
          detectedCount = 0;
        }

        motorControl(255, 255);
        delay(5);  
        stopAll();
        delay(5);
      }
      stopAll();

      long distanceRight = abs(encoderRight.read());
      long distanceLeft  = abs(encoderLeft.read());
      long avgDistance = (distanceRight + distanceLeft) / 2;
      Serial.print("進んだ距離: "); Serial.println(avgDistance);

      delay(1000);

      rotateRobot(180, 1);
      delay(1000);

      stopAll();
      delay(1000);

      encoderRight.write(0);
      encoderLeft.write(0);

      linePos -= 1;

      while (true) {
        motorControl(255, 255);
        delay(5); 
        stopAll();
        delay(50);
        long posR = abs(encoderRight.read());
        long posL = abs(encoderLeft.read());
        long avgPos = (posR + posL) / 2;

        if (avgPos >= avgDistance) {
          stopAll();
          delay(2000);
          if (!isCenterLineDetected()){
            stopCenterLine();
          }
          stopAll();
          delay(10);
          if (color == 1) {
            state = STATE_TO_RED_GOAL;
            break;
          } else if (color == 2) {
            state = STATE_TO_YELLOW_GOAL;
            break;
          } else if (color == 3) {
            state = STATE_TO_BLUE_GOAL;
            break;
          } else {
            state = STATE_TO_RED_GOAL;
            break;
          }
        }
      }
      break;
    }

    case STATE_TO_RED_GOAL: {
      // 白井ここ書いて 
      //0616_白井追加_反時計回りを向いている物としている
      pidControl(sensor_value_L, sensor_value_R);
      if (side_line == true && line_L == 1 && line_R == 1){
        linePos -= 1;
        side_line = false;
      }
      if (linePos == 0)
        state = STATE_DROP_RED;
      break;
    }

    case STATE_TO_YELLOW_GOAL: {
      // 白井ここ書いて
      //0616_白井追加_反時計回りを向いている物としている
      pidControl(sensor_value_L, sensor_value_R);
      if (side_line == true && line_L == 1 && line_R == 1){
        linePos -= 1;
        side_line = false;
      }
      if (linePos == 1)
        state = STATE_DROP_YELLOW;
      break;
    }

    case STATE_TO_BLUE_GOAL: {
      // 白井ここ書いて 
      //0616_白井追加_反時計回りを向いている物としている
      pidControl(sensor_value_L, sensor_value_R);
      if (side_line == true && line_L == 1 && line_R == 1){
        linePos -= 1;
        side_line = false;
      }
      if (linePos == 2)
        state = STATE_DROP_BLUE;
      break;
    }

    case STATE_DROP_RED: {
      stopAll();
      delay(1000);
      motorControl(255, 255);
      delay(FORWORD_TIME);
      stopAll();
      delay(1000);

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      // サーボモータを下げる
      // しばらく待つ
      // サーボモータを上げる

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      state = STATE_TO_BALL_AREA;
      break;
    }

    case STATE_DROP_YELLOW: {
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      // サーボモータを下げる
      // しばらく待つ
      // サーボモータを上げる

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }

      state = STATE_TO_BALL_AREA;
      break;
    }

    case STATE_DROP_BLUE: {
      // 青色ゴールのモーション
      // 前に進む
      // 180°回転 あとで関数つくる
      // 前に進む
      state = STATE_TO_BALL_AREA;
      break;
    }

    default: {
      state = STATE_WAIT;
      break;
    }
