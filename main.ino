#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

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
#define STATE_WAIT           1
#define STATE_FORWARD        2
#define STATE_TO_BALL_AREA   3
#define STATE_BALL_DETECT    4
#define STATE_BALL_COLLECT   5
#define STATE_TO_RED_GOAL    6
#define STATE_TO_YELLOW_GOAL 7
#define STATE_TO_BLUE_GOAL   8
#define STATE_DROP_RED       9
#define STATE_DROP_YELLOW   10
#define STATE_DROP_BLUE     11
#define STATE_FUNCTION_TEST 12

#define ROTATE_C 5000

#define PWM_RIGHT_MAX 77
#define PWM_LEFT_MAX  55
#define PWM_RIGHT_MIN 65
#define PWM_LEFT_MIN  50

#define TOLERANCE 10
#define KP 0.3
#define KI 0.02

#define START_TO_RINE 1000
// ラインからゴールまで直進を待機する時間
#define TO_RED_GOAL 1000
#define TO_YELLOW_GOAL 1000
#define TO_BLUE_GOAL 2000

Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

/* グローバル変数 */
int state = STATE_WAIT; // 現在の状態
bool psd = false;       // 測距センサ検出フラグ
int color = 0;          // ボール色: 0=なし,1=赤,2=黄,3=青
// 0617_松本変更
// 扱い考えると1からスタートしたほうがいい
int linePos = 1;        // ライン位置番号: 1～4

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

/* 0617_松本追加 */  
// その他
float base_speed = 50;  // 基本速度（0〜255）ここ変える!
float speed_l = 0;
float speed_r = 0;

void setup() {
  Serial.begin(115200);
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
    while (1);
  }
  Serial.println("VL53L0X 初期化完了");
}

void loop() {
  //0616_白井追加_センサの値読み処理_どこに追加すべきかよくわかっていない
  //センサ値はbit下げて分解能下げが良さげな感じの雰囲気がした気がする
  int sensor_value_L = analogRead(LINE_CH4_PIN) >> 2;  // 左センサ値
  int sensor_value_R = (analogRead(LINE_CH5_PIN) >> 2) * 1.2;  // 右センサ値 実際の環境で試さないと何とも言えぬ

  int line_L = digitalRead(LINE_CH1_PIN);
  int line_R = digitalRead(LINE_CH8_PIN);
  
  int center = (sensor_value_L + sensor_value_R) / 2;
  float diff = (sensor_value_R - center) - (sensor_value_L - center);

  switch (state) {
    case STATE_WAIT:
      stopAll();
      delay(15000);
      state = STATE_FORWARD;
      break;

    case STATE_FORWARD:
      driveStraight();
      if (line_L == 0 && line_R == 0)
        state = STATE_TO_BALL_AREA;
      break;

    case STATE_TO_BALL_AREA:
    // 白井ここ書いて
    //0616_白井追加
    pidControl(sensor_value_L, sensor_value_R);
      if (line_L == 0 && line_R == 0)
        linePos += 1;
      if (linePos == 3)
        state = STATE_BALL_COLLECT;
      break;  

    case STATE_BALL_DETECT:
      if (stringComplete) {
        processSerialData(inputString);
        inputString = "";
        stringComplete = false;
      }

    case STATE_BALL_COLLECT:
      encoderRight.write(0);
      encoderLeft.write(0);

      // ボール検出まで直進
      driveStraight();
      while (!isBallDetected()) {
        delay(10);
      }
      stopAll();
//
      // 走行距離を記録
      long distanceRight = abs(encoderRight.read());
      long distanceLeft  = abs(encoderLeft.read());
      long avgDistance = (distanceRight + distanceLeft) / 2;
      Serial.print("進んだ距離: "); Serial.println(avgDistance);

      delay(1000);

      rotateRobot(10, 18);
      delay(1000);

      stopAll();
      delay(1000);

      // 記録した距離だけ戻る
      encoderRight.write(0);
      encoderLeft.write(0);
      driveStraight();
      while (true) {
        long posR = abs(encoderRight.read());
        long posL = abs(encoderLeft.read());
        long avgPos = (posR + posL) / 2;

        if (avgPos >= avgDistance) {
          stopAll();
          break;
        }
        delay(10);
      }
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
      
    
    case STATE_TO_RED_GOAL:
    // 白井ここ書いて 
    //0616_白井追加_反時計回りを向いている物としている
      pidControl(sensor_value_L, sensor_value_R);
      if (line_L == 0 && line_R == 0)
         linePos -= 1;
      if (linePos == 1)
        state = STATE_DROP_RED;
      break; 

    case STATE_TO_YELLOW_GOAL:
    // 白井ここ書いて
    //0616_白井追加_反時計回りを向いている物としている
      pidControl(sensor_value_L, sensor_value_R);
      if (line_L == 0 && line_R == 0)
         linePos -= 1;
      if (linePos == 2)
        state = STATE_DROP_YELLOW;
      break;  

    case STATE_TO_BLUE_GOAL:
    // 白井ここ書いて 
    //0616_白井追加_反時計回りを向いている物としている
      pidControl(sensor_value_L, sensor_value_R);
      if (line_L == 0 && line_R == 0)
         linePos -= 1;
      if (linePos == 3)
        state = STATE_DROP_BLUE;
      break;  
    
    case STATE_DROP_RED:
      // 赤色ゴールのモーション
      stopAll();
      delay(1000);

      rotateRobot(10, 9);

      stopAll();
      delay(1000);

      driveStraight();
      delay(TO_RED_GOAL);

      stopAll();
      delay(1000);

      // サーボモータを下げる

      rotateRobot(10, 18);

      stopAll();
      delay(1000);

      driveStraight();
      delay(TO_RED_GOAL);

      rotateRobot(10, 9);

      stopAll();
      delay(1000);

      state = STATE_TO_BALL_AREA;
      break;

    case STATE_DROP_YELLOW:
      // 黄色ゴールのモーション
      stopAll();
      delay(1000);

      rotateRobot(10, 9);

      stopAll();
      delay(1000);

      driveStraight();
      delay(TO_YELLOW_GOAL);

      stopAll();
      delay(1000);

      // サーボモータを下げる

      rotateRobot(10, 18);

      stopAll();
      delay(1000);

      driveStraight();
      delay(TO_YELLOW_GOAL);

      rotateRobot(10, 9);

      stopAll();
      delay(1000);

      state = STATE_TO_BALL_AREA;
      break;

    case STATE_DROP_BLUE:
      // 青色ゴールのモーション
      stopAll();
      delay(1000);

      driveStraight();
      delay(TO_BLUE_GOAL);

      stopAll();
      delay(1000);

      // サーボモータを下げる

      rotateRobot(10, 18);

      stopAll();
      delay(1000);

      driveStraight();
      delay(TO_BLUE_GOAL);

      stopAll();
      delay(1000);

      state = STATE_TO_BALL_AREA;
      break;

    case STATE_FUNCTION_TEST:
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
  speed_l = constrain(base_speed + rotate, 0, 50); //ここ変える!
  speed_r = constrain(base_speed - rotate, 0, 100) * 1.5; //ここ変える!

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

    delay(1000); // 各回転の間に少し待つ
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

  float angle = angleStr.toFloat();

  // 色名から色番号に変換
  if (colorStr == "Red") {
    color = 1;
  } else if (colorStr == "Yellow") {
    color = 2;
  } else if (colorStr == "Blue") {
    color = 3;
  } else {
    color = 0;  // 未知の色
  }

  // もし15度以内なら何もしない
  if (abs(angle) <= 15.0) {
    stopAll();
    state = STATE_BALL_COLLECT;
    return;
  }

  // 15度より大きければ10度回転
  int commandAngle = (angle > 0) ? 10 : -10;
  rotateRobot(-commandAngle, 1);  // 向きを反転

  stopAll();
  delay(5000);  // 一時停止して次の測定を待つ
}
