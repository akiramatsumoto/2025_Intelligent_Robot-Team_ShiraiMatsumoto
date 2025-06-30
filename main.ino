
#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>
Servo servo;

/* ピン割り当て */
// ライントレースセンサ (ch1～ch8)
#define LINE_CH1_PIN A0
#define LINE_CH2_PIN A1
#define LINE_CH3_PIN A2
#define LINE_CH4_PIN A3
#define LINE_CH5_PIN A4
#define LINE_CH6_PIN A5
#define LINE_CH7_PIN A6
#define LINE_CH8_PIN A7

// エンコーダ
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_RIGHT_FORWORD 4  // 右車輪入力 B (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪入力 A (PWM)
#define WHEEL_MD_LEFT_FORWORD  7  // 左車輪入力 A (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪入力 B (PWM)

// 吸引モータドライバ (AM2837)
#define SUCTION_MD_A   3  // 吸引用入力 A (PWM)
#define SUCTION_MD_B   2  // 吸引用入力 B (PWM)

// サーボ
#define SERVO_PIN      9  // PWM

// 測距センサ (I2C)
#define DIST_SDA_PIN  20
#define DIST_SCL_PIN  21

/* FSM 状態定義 */
#define STATE_WAIT           0
#define STATE_TO_BALL_AREA   1
#define STATE_BALL_DETECT    2
#define STATE_BALL_COLLECT   3
#define STATE_TO_RED_GOAL    4
#define STATE_TO_YELLOW_GOAL 5
#define STATE_TO_BLUE_GOAL   6
#define STATE_DROP_RED       7
#define STATE_DROP_YELLOW    8 
#define STATE_DROP_BLUE      9

#define PWM_RIGHT_MAX 77
#define PWM_LEFT_MAX  60
#define PWM_RIGHT_MIN 65
#define PWM_LEFT_MIN  55

// 関数rotateRobotで使用
#define ROTATE_C 6600
// エンコーダの値の許容誤差
#define TOLERANCE 10
#define KP 0.3
#define KI 0.02

// ゴールモーション
#define FORWORD_TIME 25
#define TO_BLUE_GOAL 2000

Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

/* グローバル変数 */

// ボールとの許容角度
const float tolerance_angle = 15;
// STATE_BALL_DETECTがスキップされないように予めtolerance_angleに1を足す
float angle = tolerance_angle + 1.0;

int state = 0; // 現在の状態
bool psd = false;       // 測距センサ検出フラグ
int color = 0;          // ボール色: 0=なし,1=赤,2=黄,3=青
// 0621_松本変更 0からスタート
int linePos = 0;        // ライン位置番号: 0~3

// シリアル通信用
String inputString = "";
bool stringComplete = false;

/* 0616_白井追加 */
/* ライントレース用PIDなど */
#define Kp 10.0 //ここ変える!
#define Ki 0.01 //ここ変える!
#define Kd 5.0  //ここ変える!
float I_diff = 0;
float past_diff = 0;
#define I_max 100 //ここ変えれる

bool side_line = true;

/* 0617_松本追加 */  
// その他
float base_speed = 50;  // 基本速度（0〜255）ここ変える!
float speed_l = 0;
float speed_r = 0;

void setup() {
  Serial.begin(9600);
  // ラインセンサ
  pinMode(LINE_CH1_PIN, INPUT);
  pinMode(LINE_CH2_PIN, INPUT);
  pinMode(LINE_CH3_PIN, INPUT);
  pinMode(LINE_CH4_PIN, INPUT);
  pinMode(LINE_CH5_PIN, INPUT);
  pinMode(LINE_CH6_PIN, INPUT);
  pinMode(LINE_CH7_PIN, INPUT);
  pinMode(LINE_CH8_PIN, INPUT);
  // モータドライバ
  pinMode(WHEEL_MD_RIGHT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);
  // エンコーダ
  encoderRight.write(0);
  encoderLeft.write(0);

  Wire.begin();
  if (!lox.begin()) {
    Serial.println("VL53L0X 初期化失敗");
  }
  Serial.println("VL53L0X 初期化完了");
}

void loop() {
  Serial.print(state);
  Serial.print(" ");
  Serial.println(linePos);
  //0616_白井追加_センサの値読み処理_どこに追加すべきかよくわかっていない
  //センサ値はbit下げて分解能下げが良さげな感じの雰囲気がした気がする
  int sensor_value_L = analogRead(LINE_CH4_PIN) >> 2;  // 左センサ値
  int sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2;  // 右センサ値 実際の環境で試さないと何とも言えぬ

  int line_L = digitalRead(LINE_CH2_PIN);
  int line_R = digitalRead(LINE_CH7_PIN);
  
  int center = (sensor_value_L + sensor_value_R) / 2;
  float diff = (sensor_value_R - center) - (sensor_value_L - center);

  if(side_line == false && line_L == 0 && line_R == 0){
    side_line = true;
  }

  switch (state) {
    case STATE_WAIT:{

      delay(20000);
      state = STATE_TO_BALL_AREA;
      break;
    }
    case STATE_TO_BALL_AREA:
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

    case STATE_BALL_DETECT:
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

    case STATE_BALL_COLLECT:{
      stopAll();

      encoderRight.write(0);
      encoderLeft.write(0);

      int detectedCount = 0;  // 連続trueカウント
      int psdCount = 0;           // 検出回数カウント

      controlServo(false);
      controlSuction(true);

      while (psdCount <= 1) {
        bool detected = isBallDetected();
        Serial.print("Detected: "); Serial.print(detected);
        Serial.print(" | detectedCount: "); Serial.print(detectedCount);
        Serial.print(" | psdCount: "); Serial.println(psdCount);

        if (detected) {
          detectedCount++;
          if (detectedCount >= 5) {  // 5回連続でtrueなら1カウント
            psdCount++;
            detectedCount = 0;  // リセットして次の検出を待つ
          }
        } else {
          detectedCount = 0;  // 連続が途切れたらリセット
        }

        motorControl(255, 255);
        delay(5);  
        stopAll();
        delay(5);
      }
      stopAll();

      // サーボを持ち上げる
      controlServo(true);
      delay(500);
      controlSuction(false);

      // 走行距離を記録
      long distanceRight = abs(encoderRight.read());
      long distanceLeft  = abs(encoderLeft.read());
      long avgDistance = (distanceRight + distanceLeft) / 2;
      Serial.print("進んだ距離: "); Serial.println(avgDistance);

      delay(1000);

      rotateRobot(180, 1);
      delay(1000);

      stopAll();
      delay(1000);

      // 記録した距離だけ戻る
      encoderRight.write(0);
      encoderLeft.write(0);

      // 3本目のラインは越えている状態
      // これいらない気がする
      // linePos -= 1; 

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
            state = STATE_TO_RED_GOAL;  // 未知の色
            break;
          }
        }
      }
    }

    case STATE_TO_RED_GOAL:
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

    case STATE_TO_YELLOW_GOAL:
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

    case STATE_TO_BLUE_GOAL:
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
    
    case STATE_DROP_RED:
      // 赤・黄色ゴール用
      stopAll();
      delay(1000);
      motorControl(255, 255);
      // 少し前に進む
      delay(FORWORD_TIME);
      stopAll();
      delay(1000);
      // もし今黒を踏んでいたら少し回転
      //sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      //if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      //}
      // センサの値を更新
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      // もう一度黒を踏むまで回転
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }
      // サーボモータを下げる
      stopAll();
      delay(2000);
      // しばらく待つ
      // サーボモータを上げる
      // もし今黒を踏んでいたら少し回転
      //sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      //if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(20);
        stopAll();
        delay(100);
      //}
      // センサの値を更新
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      stopAll();
      delay(2000);
      // もう一度黒を踏むまで回転
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }
      state = STATE_TO_BALL_AREA;
      break;

    case STATE_DROP_YELLOW:
      // 赤・黄色ゴール用
      // もし今黒を踏んでいたら少し回転
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }
      // センサの値を更新
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      // もう一度黒を踏むまで回転
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
      // もし今黒を踏んでいたら少し回転
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      if(sensor_value_R < 100){
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }
      // センサの値を更新
      sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
      // もう一度黒を踏むまで回転
      while(sensor_value_R < 100){
        sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2; 
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }
      state = STATE_TO_BALL_AREA;
      break;

    case STATE_DROP_BLUE:
      // 青色ゴールのモーション
      // 前に進む
      // 180°回転 あとで関数つくる
      // 前に進む
      state = STATE_TO_BALL_AREA;
      break;

    default:
      state = STATE_WAIT;
      break;
  }
  delay(50);
}

//0616_白井追加
void pidControl(int sensor_value_L, int sensor_value_R) {
  int center = (sensor_value_L + sensor_value_R) / 2;
  float diff = (sensor_value_R - center) - (sensor_value_L - center);

  float rotate = Kp * diff + Ki * I_diff + Kd * (diff - past_diff);

  past_diff = diff;
  I_diff += diff;
  I_diff = constrain(I_diff, -I_max, I_max);

  // 左右の速度調整
  speed_l = constrain(base_speed + rotate, 0, 80); //ここ変える!
  speed_r = constrain(base_speed - rotate, 0, 100); //ここ変える!

  /* 0617_松本追加 */  
  // モーター制御
  motorControl(speed_l, speed_r);
}

/* 0617_松本追加 */
void motorControl(float left, float right) {
  // 安全な範囲に制限（0〜255）
  //left = constrain(left, -100, 100);
  //right = constrain(right, -100, 100);

  // 左モーター制御
  if (left >= 0) {
    analogWrite(WHEEL_MD_LEFT_FORWORD, left);
    analogWrite(WHEEL_MD_LEFT_BACK, 0);
  } else {
    analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
    analogWrite(WHEEL_MD_LEFT_BACK, -left);
  }

  // 右モーター制御
  if (right >= 0) {
    analogWrite(WHEEL_MD_RIGHT_BACK, 0);
    analogWrite(WHEEL_MD_RIGHT_FORWORD, right);
  } else {
    analogWrite(WHEEL_MD_RIGHT_BACK, -right);
    analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  }
}

void driveStraight() {
  digitalWrite(WHEEL_MD_RIGHT_FORWORD, HIGH);
  digitalWrite(WHEEL_MD_LEFT_FORWORD, HIGH);
}

void rotateRobot(float degree, int repeat) {
  for (int i = 0; i < repeat; i++) {
    encoderRight.write(0);
    encoderLeft.write(0);

    long targetCount = (long)(ROTATE_C * (abs(degree) / 360.0));
    bool clockwise = (degree > 0);

    long errorSum = 0;
    
    while (true) {
      long posRight = encoderRight.read();
      long posLeft  = encoderLeft.read();
      long averagePos = (abs(posRight) + abs(posLeft)) / 2;
      long error = targetCount - averagePos;
      errorSum += error;

      Serial.print("Target:");
      Serial.print(targetCount);
      Serial.print(" Current:");
      Serial.print(averagePos);
      Serial.print(" Error:");
      Serial.println(error);

      // PI制御で速度決定
      float control = KP * error + KI * errorSum;
      control = constrain(control, 0.0, 1.0); // 正規化（0〜1）

      uint8_t pwmRight = max(PWM_RIGHT_MIN, (uint8_t)(PWM_RIGHT_MAX * control));
      uint8_t pwmLeft  = max(PWM_LEFT_MIN,  (uint8_t)(PWM_LEFT_MAX  * control));

      // 出力
      if (clockwise) {
        analogWrite(WHEEL_MD_RIGHT_FORWORD, pwmRight);
        analogWrite(WHEEL_MD_RIGHT_BACK, 0);
        analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
        analogWrite(WHEEL_MD_LEFT_BACK, pwmLeft);
      } else {
        analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
        analogWrite(WHEEL_MD_RIGHT_BACK, pwmRight);
        analogWrite(WHEEL_MD_LEFT_FORWORD, pwmLeft);
        analogWrite(WHEEL_MD_LEFT_BACK, 0);
      }

      if (error <= TOLERANCE) {
        stopAll();
        break;
      }

      delay(20);
    }

    delay(500); // 各回転の間に少し待つ
  }
}

void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 0);
}

bool isBallDetected() {
  VL53L0X_RangingMeasurementData_t meas;
  lox.rangingTest(&meas, false);
  long distance = (meas.RangeStatus != 4) ? meas.RangeMilliMeter : -1;
  return (distance >= 0 && distance <= 80);
}

// カメラ用シリアル通信関数

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processSerialData(String data) {
  // カンマで3分割
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0) return;

  String colorStr = data.substring(0, firstComma);
  String areaStr  = data.substring(firstComma + 1, secondComma);
  String angleStr = data.substring(secondComma + 1);

  // 色名から色番号に変換
  if (colorStr == "Red") {
    color = 1;
  } else if (colorStr == "Yellow") {
    color = 2;
  } else if (colorStr == "Blue") {
    color = 3;
  } else {
    color = 0;  // 未知の色
    return;
  }

  angle = angleStr.toFloat();

  // 意味ないと思うけど消して動かなくなったら嫌だから残してる
  // もし15度以内なら何もしない
  if (abs(angle) <= tolerance_angle) {
    stopAll();
    state = STATE_BALL_COLLECT;
    return;
  }

  // 15度より大きければ10度回転
  int commandAngle = (angle > 0) ? 5 : -5;
  rotateRobot(-commandAngle, 1);  // 向きを反転

  stopAll();
  delay(5000);  // 一時停止して次の測定を待つ
}

// 0622松本追加
// 中央2つのフォトリフレクタがラインを踏んでいるか
bool isCenterLineDetected() {
  bool detected = false;

  int  sensor_value_L = analogRead(LINE_CH4_PIN) >> 2;
  int sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2;

  if (sensor_value_L > 150 || sensor_value_R > 100) {
    detected = true;
    return detected;
  }
}

// 真ん中2つのフォトリフレクタが反応するまで回転する
void stopCenterLine() {  
  bool detected = isCenterLineDetected();
  int attempts = 0;

  while (!detected) {
    if (attempts % 2 == 0) {
      // 90
      for (int i = 0; i < 15; i++) {
        detected = isCenterLineDetected();
        if(detected){
          break;
        }
        motorControl(255, -255);
        delay(5);
        stopAll();
        delay(100);
      }
      // -90
      for (int i = 0; i < 15; i++) {
        detected = isCenterLineDetected();
        if(detected){
          break;
        }
        motorControl(-255, 255);
        delay(5);
        stopAll();
        delay(100);
      }
      // 下がっちゃうので戻す
      driveStraight();
      delay(20);
    } else {
        // -90
        for (int i = 0; i < 15; i++) {
        detected = isCenterLineDetected();
        if(detected){
          break;
        }
          motorControl(-255, 255);
          delay(5);
          stopAll();
          delay(100);
        }
        // 90
        for (int i = 0; i < 15; i++) {
          detected = isCenterLineDetected();
          if(detected){
            break;
          }
          motorControl(255, -255);
          delay(5);
          stopAll();
          delay(100);
        }
        // 下がっちゃうので戻す
        driveStraight();
        delay(20);
    }
    stopAll();
    delay(500);
    attempts++;
  }
}

// サーボ昇降 (true: 上げる, false: 下げる)
void controlServo(bool up) {
  servo.attach(SERVO_PIN);
  // 0と90は適宜変更
  servo.write(up ? 60 : 5);
  delay(500);
  servo.detach();
}

// 吸引オンオフ (true: ON, false: OFF)
void controlSuction(bool on) {
  if (on) {
    analogWrite(SUCTION_MD_A, 150);
    analogWrite(SUCTION_MD_B, 0);
  } else {
    analogWrite(SUCTION_MD_A, 0);
    analogWrite(SUCTION_MD_B, 0);
  }
}
