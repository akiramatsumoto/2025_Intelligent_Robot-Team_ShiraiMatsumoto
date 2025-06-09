/* メインプログラム: FSM によるロボット動作制御 (Arduino Mega2560 Rev3 ピン接続) */

#include <Arduino.h>
#include <Wire.h> // I2C 用
#include <Adafruit_VL53L0X.h>
#include <Servo.h>
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
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
#define WHEEL_MD_RIGHT_A 5  // 右車輪入力 A (PWM)
#define WHEEL_MD_RIGHT_B 4  // 右車輪入力 B (PWM)
#define WHEEL_MD_LEFT_A  7  // 左車輪入力 A (PWM)
#define WHEEL_MD_LEFT_B  6  // 左車輪入力 B (PWM)

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
#define STATE_DETECT_COLLECT 4
#define STATE_TO_RED_GOAL    5
#define STATE_TO_YELLOW_GOAL 6
#define STATE_TO_BLUE_GOAL   7
#define STATE_DROP_RED       8
#define STATE_DROP_YELLOW    9
#define STATE_DROP_BLUE     10
#define STATE_FUNCTION_TEST 11

/* グローバル変数 */
int state = STATE_WAIT; // 現在の状態
bool psd = false;       // 測距センサ検出フラグ
int color = 0;          // ボール色: 0=なし,1=赤,2=黄,3=青
int linePos = 0;        // ライン位置番号: 1～8

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=20, SCL=21 の場合は別途 Wire.setPins() が必要な環境もあり
  if (!lox.begin()) {
    Serial.println("ERROR: VL53L0X 初期化失敗");
    while (1) delay(10);
  }
  Serial.println("VL53L0X 初期化完了");
  // ラインセンサ
  pinMode(LINE_CH1_PIN, INPUT);
  pinMode(LINE_CH2_PIN, INPUT);
  pinMode(LINE_CH3_PIN, INPUT);
  pinMode(LINE_CH4_PIN, INPUT);
  pinMode(LINE_CH5_PIN, INPUT);
  pinMode(LINE_CH6_PIN, INPUT);
  pinMode(LINE_CH7_PIN, INPUT);
  pinMode(LINE_CH8_PIN, INPUT);
  // エンコーダ
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  // モータドライバ
  pinMode(WHEEL_MD_RIGHT_A, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_B, OUTPUT);
  pinMode(WHEEL_MD_LEFT_A, OUTPUT);
  pinMode(WHEEL_MD_LEFT_B, OUTPUT);
  pinMode(SUCTION_MD_A, OUTPUT);
  pinMode(SUCTION_MD_B, OUTPUT);
  // サーボ
  pinMode(SERVO_PIN, OUTPUT);
  // 初期ヘルパー関数テスト用状態
  state = STATE_FUNCTION_TEST;
}

void loop() {
  // センサ更新
  psd = readDistanceSensor();
  color = readColor();
  linePos = readLinePosition();

  switch (state) {
    case STATE_WAIT:
      stopAll();
      if (!psd && color == 0 && linePos == 0) state = STATE_FORWARD;
      break;

    // ... 他ステート実装 ...

    case STATE_FUNCTION_TEST:
      // ヘルパー関数の動作確認例
      controlServo(true);
      delay(500);
      controlServo(false);
      controlSuction(true);
      delay(500);
      controlSuction(false);
      driveStraight(200);
      delay(1000);
      stopAll();
      rotateRobot(90);
      stopAll();
      state = STATE_WAIT;
      break;

    default:
      state = STATE_WAIT;
      break;
  }
  delay(50);
}

/* --- ヘルパー関数 --- */

// サーボ昇降 (true: 上げる, false: 下げる)
void controlServo(bool up) {
  servo.attach(SERVO_PIN);
  servo.write(up ? 0 : 90);
  delay(500);
  servo.detach();
}

// 吸引オンオフ (true: ON, false: OFF)
void controlSuction(bool on) {
  if (on) {
    analogWrite(SUCTION_MD_A, 255);
    analogWrite(SUCTION_MD_B, 0);
  } else {
    analogWrite(SUCTION_MD_A, 0);
    analogWrite(SUCTION_MD_B, 0);
  }
}


// 前後移動: mm/s (正:前進, 負:後退)
void driveStraight(int speed_mm_s) {
  int pwm = constrain(map(abs(speed_mm_s), 0, 500, 0, 255), 0, 255);
  if (speed_mm_s >= 0) {
    analogWrite(WHEEL_MD_RIGHT_A, pwm);
    analogWrite(WHEEL_MD_RIGHT_B, 0);
    analogWrite(WHEEL_MD_LEFT_A, pwm);
    analogWrite(WHEEL_MD_LEFT_B, 0);
  } else {
    analogWrite(WHEEL_MD_RIGHT_A, 0);
    analogWrite(WHEEL_MD_RIGHT_B, pwm);
    analogWrite(WHEEL_MD_LEFT_A, 0);
    analogWrite(WHEEL_MD_LEFT_B, pwm);
  }
}

// 旋回: 度 (正:右回転, 負:左回転)
void rotateRobot(int degrees) {
  int pwm = 150; // 要調整
  unsigned long duration = abs(degrees) * 10UL; // 要調整
  if (degrees >= 0) {
    analogWrite(WHEEL_MD_RIGHT_A, 0);
    analogWrite(WHEEL_MD_RIGHT_B, pwm);
    analogWrite(WHEEL_MD_LEFT_A, pwm);
    analogWrite(WHEEL_MD_LEFT_B, 0);
  } else {
    analogWrite(WHEEL_MD_RIGHT_A, pwm);
    analogWrite(WHEEL_MD_RIGHT_B, 0);
    analogWrite(WHEEL_MD_LEFT_A, 0);
    analogWrite(WHEEL_MD_LEFT_B, pwm);
  }
  delay(duration);
  stopAll();
}

// 全モーター停止
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_A, 0);
  analogWrite(WHEEL_MD_RIGHT_B, 0);
  analogWrite(WHEEL_MD_LEFT_A, 0);
  analogWrite(WHEEL_MD_LEFT_B, 0);
  analogWrite(SUCTION_MD_A, 0);
  analogWrite(SUCTION_MD_B, 0);
}

bool readDistanceSensor() {
  VL53L0X_RangingMeasurementData_t meas;
  // false: デバッグ用シリアル出力なし
  lox.rangingTest(&meas, false);

  // RangeStatus が 4 以外なら有効値
  if (meas.RangeStatus != 4) {
    // meas.RangeMilliMeter は距離[mm]
    return (meas.RangeMilliMeter <= 50);
  } else {
    // エラー時は「遠い」とみなす
    return false;
  }
}
